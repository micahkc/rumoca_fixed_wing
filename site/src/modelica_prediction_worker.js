import initRumoca, * as rumoca from "../public/wasm/rumoca_bind_wasm.js";

let rumocaReady = null;

async function loadRumoca() {
  if (!rumocaReady) {
    rumocaReady = initRumoca().then(() => {
      if (typeof rumoca.wasm_init === "function") rumoca.wasm_init(0);
      if (typeof rumoca.init === "function") rumoca.init();
      return rumoca;
    });
  }
  return rumocaReady;
}

function finiteNumber(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normQuat(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function quatMulWxyz(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function quatFromEuler(phi, theta, psi) {
  const cr = Math.cos(0.5 * phi), sr = Math.sin(0.5 * phi);
  const cp = Math.cos(0.5 * theta), sp = Math.sin(0.5 * theta);
  const cy = Math.cos(0.5 * psi), sy = Math.sin(0.5 * psi);
  return normQuat([
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ]);
}

function read(stepper, names, fallback = 0) {
  for (const name of names) {
    const value = stepper.get(name);
    if (Number.isFinite(value)) return value;
  }
  return fallback;
}

function stateFromStepper(stepper) {
  const pN = read(stepper, ["p_n", "airframe.pos[1]", "pos[1]", "ground.p_n"]);
  const pE = read(stepper, ["p_e", "airframe.pos[2]", "pos[2]", "ground.p_e"]);
  const pD = read(stepper, ["p_d", "airframe.pos[3]", "pos[3]"]);
  const u = read(stepper, ["u", "airframe.vel[1]", "vel[1]", "ground.V"]);
  const v = read(stepper, ["v", "airframe.vel[2]", "vel[2]"]);
  const w = read(stepper, ["w", "airframe.vel[3]", "vel[3]"]);
  const qW = stepper.get("q_w");
  const qX = stepper.get("q_x");
  const qY = stepper.get("q_y");
  const qZ = stepper.get("q_z");
  let quat;
  if ([qW, qX, qY, qZ].every(Number.isFinite)) {
    quat = normQuat([qW, qX, qY, qZ]);
  } else {
    const phi = read(stepper, ["phi", "airframe.att[1]", "att[1]"]);
    const theta = read(stepper, ["theta", "airframe.att[2]", "att[2]"]);
    const psi = read(stepper, ["psi", "airframe.att[3]", "att[3]", "ground.psi"]);
    quat = quatFromEuler(phi, theta, psi);
  }
  return [
    pN, pE, pD, u, v, w,
    quat[0], quat[1], quat[2], quat[3],
    read(stepper, ["p", "rate_p", "airframe.rates[1]", "rates[1]"]),
    read(stepper, ["q", "rate_q", "airframe.rates[2]", "rates[2]"]),
    read(stepper, ["r", "rate_r", "airframe.rates[3]", "rates[3]"]),
  ];
}

function sourceWithInitialState(source, initialState) {
  if (!initialState) return source;
  const params = {
    p0_n: initialState[0], p0_e: initialState[1], p0_d: initialState[2],
    u0: initialState[3], v0: initialState[4], w0: initialState[5],
    q0_w: initialState[6], q0_x: initialState[7], q0_y: initialState[8], q0_z: initialState[9],
    rate0_p: initialState[10], rate0_q: initialState[11], rate0_r: initialState[12],
  };
  let out = source;
  for (const [name, value] of Object.entries(params)) {
    const re = new RegExp(`(parameter\\s+Real\\s+${name}\\s*=\\s*)[-+0-9.eE]+`);
    out = out.replace(re, `$1${finiteNumber(value).toPrecision(12)}`);
  }
  return out;
}

function nedStateToEnuPosition(x) {
  return [x[1], x[0], -x[2]];
}

function nedStateToEnuQuat(x) {
  const qNedBody = [x[6], x[7], x[8], x[9]];
  const qEnuFromNed = [0, Math.SQRT1_2, Math.SQRT1_2, 0];
  return normQuat(quatMulWxyz(qEnuFromNed, qNedBody));
}

function stepPrediction(stepper, inputNames, entry, stick, dt, options = {}) {
  const set = (name, value) => {
    if (inputNames.has(name)) stepper.set_input(name, value);
  };
  const throttle = clamp(finiteNumber(stick?.[0]), 0, 1);
  const elevator = clamp(finiteNumber(entry?.elevatorSign ?? 1) * finiteNumber(stick?.[1]), -1, 1);
  const aileron = clamp(finiteNumber(stick?.[2]), -1, 1);
  const rudder = clamp(finiteNumber(stick?.[3]), -1, 1);
  set("enable_safe", options.safeEnabled === false ? 0 : 1);
  set("mode", finiteNumber(options.mode) ? options.mode : 0);
  set("throttle", throttle);
  set("elevator", elevator);
  set("aileron", aileron);
  set("rudder", rudder);
  set("stick_elevator", elevator);
  set("stick_aileron", aileron);
  set("stick_rudder", rudder);
  stepper.step(dt);
  return stateFromStepper(stepper);
}

self.onmessage = async (event) => {
  const msg = event.data || {};
  if (msg.type !== "predict") return;
  const id = msg.id;
  const t0 = performance.now();
  let stepper = null;
  try {
    self.postMessage({ type: "progress", id, phase: "loading Rumoca" });
    const wasm = await loadRumoca();
    const compileStart = performance.now();
    self.postMessage({ type: "progress", id, phase: "building stepper" });
    const simulationSource = sourceWithInitialState(msg.source || "", msg.initialState);
    stepper = new wasm.WasmStepper(simulationSource, msg.modelName);
    const compileMs = performance.now() - compileStart;
    const inputNames = new Set(JSON.parse(stepper.input_names() || "[]"));
    const resetStart = performance.now();
    stepper.reset();
    let x = stateFromStepper(stepper);
    if (!x.every(Number.isFinite)) throw new Error(`${msg.modelName} produced a non-finite initial state.`);
    const resetMs = performance.now() - resetStart;
    const points = [];
    const times = [];
    const quats = [];
    const sticks = Array.isArray(msg.sticks) ? msg.sticks : [];
    const modes = Array.isArray(msg.modes) ? msg.modes : [];
    const dt = finiteNumber(msg.dt, 1 / 240);
    const stride = Math.max(1, Math.trunc(finiteNumber(msg.stride, 24)));
    const startTimeS = finiteNumber(msg.startTimeS, 0);
    const stepStart = performance.now();
    for (let index = 0; index < sticks.length; index += 1) {
      if (index % stride === 0) {
        times.push(startTimeS + index * dt);
        points.push(nedStateToEnuPosition(x));
        quats.push(nedStateToEnuQuat(x));
      }
      x = stepPrediction(stepper, inputNames, msg.entry || {}, sticks[index], dt, { mode: modes[index], safeEnabled: false });
      if (!x.every(Number.isFinite)) {
        throw new Error(`${msg.modelName} prediction became non-finite at t=${(startTimeS + index * dt).toFixed(2)}s.`);
      }
      if ((index + 1) % 240 === 0) {
        self.postMessage({ type: "progress", id, phase: "stepping", simulatedS: (index + 1) * dt });
      }
    }
    if (!points.length) throw new Error(`${msg.modelName} prediction produced no drawable samples.`);
    const stepMs = performance.now() - stepStart;
    self.postMessage({
      type: "done",
      id,
      method: msg.method,
      color: msg.color,
      points,
      times,
      quats,
      timing: { compileMs, resetMs, stepMs, totalMs: performance.now() - t0, steps: sticks.length },
    });
  } catch (error) {
    self.postMessage({ type: "error", id, message: error?.message || String(error) });
  } finally {
    if (stepper && typeof stepper.free === "function") stepper.free();
  }
};
