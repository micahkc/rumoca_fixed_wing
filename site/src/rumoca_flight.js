import initRumoca, * as rumoca from "@cognipilot/rumoca";
import { FIXEDWING_GA_SOURCE } from "./fixedwing_ga_model.js";

let rumocaReady = null;

const RUMOCA_VERSION = "v0.9.8";
const CMM_VERSION = "v0.0.2";
const RUMOCA_RAW_BASE = `https://raw.githubusercontent.com/CogniPilot/rumoca/${RUMOCA_VERSION}`;
const CMM_RAW_BASE = `https://raw.githubusercontent.com/CogniPilot/modelica_models/${CMM_VERSION}`;
let externalFlightEntriesReady = null;

function finiteNumber(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function wrapPi(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
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

function eulerFromQuatWxyz(q) {
  const [w, x, y, z] = normQuat(q);
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  };
}

function fmtModelicaArray(values) {
  return `{${values.map((value) => finiteNumber(value).toPrecision(12)).join(", ")}}`;
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

async function loadRumoca() {
  if (!rumocaReady) {
    rumocaReady = initRumoca()
      .then(() => {
        if (typeof rumoca.wasm_init === "function") rumoca.wasm_init(0);
        if (typeof rumoca.init === "function") rumoca.init();
        return rumoca;
      })
      .catch((error) => {
        rumocaReady = null;
        throw error;
      });
  }
  return rumocaReady;
}

function read(stepper, names, fallback = 0) {
  for (const name of names) {
    const value = stepper.get(name);
    if (Number.isFinite(value)) return value;
  }
  return fallback;
}

function genericAircraftWrapper(baseName, wrapperName, airframeMods = "") {
  return `
model ${wrapperName}
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  ${baseName} airframe${airframeMods};

  Real p_n;
  Real p_e;
  Real p_d;
  Real u;
  Real v;
  Real w;
  Real phi;
  Real theta;
  Real psi;
  Real p;
  Real q;
  Real r;

equation
  airframe.throttle = throttle;
  airframe.elevator = elevator;
  airframe.aileron = aileron;
  airframe.rudder = rudder;

  p_n = airframe.pos[1];
  p_e = airframe.pos[2];
  p_d = airframe.pos[3];
  u = airframe.vel[1];
  v = airframe.vel[2];
  w = airframe.vel[3];
  phi = airframe.att[1];
  theta = airframe.att[2];
  psi = airframe.att[3];
  p = airframe.rates[1];
  q = airframe.rates[2];
  r = airframe.rates[3];
end ${wrapperName};
`;
}

async function fetchText(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`failed to fetch ${url}: ${response.status}`);
  return response.text();
}

function rumocaFixedWingWrapper() {
  return `
model RumocaFixedWingFlight
  input Real throttle(start = 0.0);
  input Real elevator(start = 0.0);
  input Real aileron(start = 0.0);
  input Real rudder(start = 0.0);

  FixedWing fixedwing;

  Real p_n;
  Real p_e;
  Real p_d;
  Real u;
  Real v;
  Real w;
  Real q_w;
  Real q_x;
  Real q_y;
  Real q_z;
  Real rate_p;
  Real rate_q;
  Real rate_r;

equation
  fixedwing.stick_throttle = throttle;
  fixedwing.stick_pitch = elevator;
  fixedwing.stick_roll = aileron;
  fixedwing.stick_yaw = rudder;
  fixedwing.armed = 1.0;

  // Rumoca's interactive fixed-wing uses NWU/FLU. The playback harness expects
  // NED/FRD, so flip the lateral/vertical axes for position, attitude, rates.
  p_n = fixedwing.position[1];
  p_e = -fixedwing.position[2];
  p_d = -fixedwing.position[3];
  u = fixedwing.airspeed;
  v = 0.0;
  w = 0.0;
  q_w = fixedwing.quat[1];
  q_x = fixedwing.quat[2];
  q_y = -fixedwing.quat[3];
  q_z = -fixedwing.quat[4];
  rate_p = fixedwing.gyro[1];
  rate_q = -fixedwing.gyro[2];
  rate_r = -fixedwing.gyro[3];
end RumocaFixedWingFlight;
`;
}

function sourceWithInitialState(entry, source, initialState) {
  if (!initialState) return source;
  const x = initialState;  // measured NED/FRD 13-state [x_n,y_e,z_d,u,v,w,qw,qx,qy,qz,p,q,r]
  // Preferred: the GA harness exposes the start state as NED/FRD parameters
  // (p0_*, u0/v0/w0, q0_*, rate0_*) and converts to the plant's NWU/FLU frame
  // internally -- so we substitute the RAW measured numbers, no frame logic here
  // (keeps the page convention-agnostic; the model owns every sign/frame).
  const params = {
    p0_n: x[0], p0_e: x[1], p0_d: x[2], u0: x[3], v0: x[4], w0: x[5],
    q0_w: x[6], q0_x: x[7], q0_y: x[8], q0_z: x[9],
    rate0_p: x[10], rate0_q: x[11], rate0_r: x[12],
  };
  let out = source;
  let substituted = false;
  for (const [name, val] of Object.entries(params)) {
    const re = new RegExp(`(parameter\\s+Real\\s+${name}\\s*=\\s*)[-+0-9.eE]+`);
    if (re.test(out)) { out = out.replace(re, `$1${val}`); substituted = true; }
  }
  if (substituted) return out;
  // Fallback for the old external baseline: inject into FixedWing(vehicle(...))
  // in NWU/FLU (conversion done here because that model has no p0_* params).
  if (entry?.label?.startsWith("RumocaFixedWing") && source.includes("FixedWing fixedwing;")) {
    const positionNwu = [x[0], -x[1], -x[2]];
    const velocityFlu = [x[3], -x[4], -x[5]];
    const quatNwuFlu = normQuat([x[6], x[7], -x[8], -x[9]]);
    const omegaFlu = [x[10], -x[11], -x[12]];
    const fixedwing = `FixedWing fixedwing(vehicle(p_start = ${fmtModelicaArray(positionNwu)}, v_b_start = ${fmtModelicaArray(velocityFlu)}, q_start = ${fmtModelicaArray(quatNwuFlu)}, omega_start = ${fmtModelicaArray(omegaFlu)}));`;
    return source.replace("FixedWing fixedwing;", fixedwing);
  }
  return source;
}

async function rumocaFixedWingSource() {
  const [lieGroup, rigidBody, fixedWing] = await Promise.all([
    fetchText(`${CMM_RAW_BASE}/LieGroup/package.mo`),
    fetchText(`${CMM_RAW_BASE}/RigidBody/package.mo`),
    fetchText(`${RUMOCA_RAW_BASE}/examples/interactive/fixedwing/FixedWingSIL.mo`),
  ]);
  return [
    `// External baseline sources pinned to @cognipilot/rumoca ${RUMOCA_VERSION} and CMM ${CMM_VERSION}.`,
    `// CMM release package: https://github.com/CogniPilot/modelica_models/releases/download/${CMM_VERSION}/CMM_${CMM_VERSION}.zip`,
    lieGroup,
    rigidBody,
    fixedWing,
    rumocaFixedWingWrapper(),
  ].join("\n\n");
}

export async function loadExternalFlightModelEntries() {
  if (!externalFlightEntriesReady) {
    externalFlightEntriesReady = Promise.resolve([{
      label: "RumocaFixedWing",
      group: "baseline",
      modelName: "RumocaFixedWingFlight",
      source: FIXEDWING_GA_SOURCE,
      output: "generic",
      elevatorSign: 1,
    }]);
  }
  return externalFlightEntriesReady;
}

function groundedGreyboxSource(greyboxSource, modelName = "SportCubGreyboxGrounded") {
  let source = String(greyboxSource || "");
  source = source.replace(/model\s+SportCubGreyboxIdentified\b/, `model ${modelName}`);
  source = source.replace(/end\s+SportCubGreyboxIdentified\s*;/, `end ${modelName};`);
  if (source.includes("wing_incidence") && source.includes("thr_max") && source.includes("ground_k")) {
    return source;
  }
  if (source.includes("use_ground")) {
    return source
      .replace("constant Real use_ground = 0.0", "constant Real use_ground = 1.0")
      .replace("Real pos[3](start = {0, 0, 0});", "Real pos[3](start = {0, 0, -0.055});")
      .replace("Real vel[3](start = {16, 0, 0});", "Real vel[3](start = {0, 0, 0});");
  }
  source = source.replace("Real pos[3](start = {0, 0, 0});", "Real pos[3](start = {0, 0, -0.055});");
  source = source.replace("Real vel[3](start = {16, 0, 0});", "Real vel[3](start = {0, 0, 0});");
  source = source.replace(
    "  Real r00, r01, r02, r10, r11, r12, r20, r21, r22;\n",
    `  Real r00, r01, r02, r10, r11, r12, r20, r21, r22;

  // Tricycle ground-contact model adapted from Rumoca's interactive fixed-wing
  // example. This greybox is NED/FRD, so positive world-down penetration creates
  // an upward body force along -R[3,:], with light rolling and lateral friction.
  parameter Real ground_d = 0.0 "Runway height in NED down coordinate [m]";
  parameter Real ground_k = 140.0 "Gear stiffness per wheel [N/m]";
  parameter Real ground_c = 7.0 "Gear normal damping per wheel [N*s/m]";
  parameter Real roll_fric = 0.02 "Rolling resistance [N/(m/s)]";
  parameter Real side_fric = 1.2 "Lateral tire grip [N/(m/s)]";
  parameter Real ground_contact_eps = 1e-4 "Contact transition penetration [m]";
  parameter Real wheel_x[3] = {0.10, -0.08, -0.08} "Wheel forward offsets nose,R,L [m]";
  parameter Real wheel_y[3] = {0.0, 0.10, -0.10} "Wheel right offsets [m]";
  parameter Real wheel_z[3] = {0.055, 0.055, 0.055} "Wheel down offsets [m]";
  Real wh_h[3], wh_pen[3], wh_contact[3], wh_vbx[3], wh_vby[3], wh_vbz[3], wh_vwd[3], wh_Fn[3];
  Real wh_F[3, 3], wh_M[3, 3];
  Real F_ground[3], M_ground[3];
`
  );
  source = source.replace(
    "  der(vel[1]) = fx/m - g*s_th + rates[3]*vel[2] - rates[2]*vel[3];\n  der(vel[2]) = fy/m + g*s_phi*c_th + rates[1]*vel[3] - rates[3]*vel[1];\n  der(vel[3]) = fz/m + g*c_phi*c_th + rates[2]*vel[1] - rates[1]*vel[2];",
    "  der(vel[1]) = (fx + F_ground[1])/m - g*s_th + rates[3]*vel[2] - rates[2]*vel[3];\n  der(vel[2]) = (fy + F_ground[2])/m + g*s_phi*c_th + rates[1]*vel[3] - rates[3]*vel[1];\n  der(vel[3]) = (fz + F_ground[3])/m + g*c_phi*c_th + rates[2]*vel[1] - rates[1]*vel[2];"
  );
  source = source.replace(
    "  der(rates[1]) = roll_accel + ((Iyy - Izz)/Ixx)*rates[2]*rates[3] + (Ixz/Ixx)*rates[1]*rates[2];\n  der(rates[2]) = pitch_accel + ((Izz - Ixx)/Iyy)*rates[1]*rates[3] + (Ixz/Iyy)*(rates[3]^2 - rates[1]^2);\n  der(rates[3]) = yaw_accel + ((Ixx - Iyy)/Izz)*rates[1]*rates[2] + (Ixz/Izz)*rates[2]*rates[3];",
    "  der(rates[1]) = roll_accel + M_ground[1]/Ixx + ((Iyy - Izz)/Ixx)*rates[2]*rates[3] + (Ixz/Ixx)*rates[1]*rates[2];\n  der(rates[2]) = pitch_accel + M_ground[2]/Iyy + ((Izz - Ixx)/Iyy)*rates[1]*rates[3] + (Ixz/Iyy)*(rates[3]^2 - rates[1]^2);\n  der(rates[3]) = yaw_accel + M_ground[3]/Izz + ((Ixx - Iyy)/Izz)*rates[1]*rates[2] + (Ixz/Izz)*rates[2]*rates[3];"
  );
  source = source.replace(
    "  // body translational dynamics\n",
    `  // landing gear contact: wheel world-down position/velocity and body force
  for i in 1:3 loop
    wh_h[i] = pos[3] + r20*wheel_x[i] + r21*wheel_y[i] + r22*wheel_z[i] - ground_d;
    wh_pen[i] = 0.5*(wh_h[i] + sqrt(wh_h[i]*wh_h[i] + ground_contact_eps*ground_contact_eps));
    wh_contact[i] = wh_pen[i]/(wh_pen[i] + ground_contact_eps);
    wh_vbx[i] = vel[1] + rates[2]*wheel_z[i] - rates[3]*wheel_y[i];
    wh_vby[i] = vel[2] + rates[3]*wheel_x[i] - rates[1]*wheel_z[i];
    wh_vbz[i] = vel[3] + rates[1]*wheel_y[i] - rates[2]*wheel_x[i];
    wh_vwd[i] = r20*wh_vbx[i] + r21*wh_vby[i] + r22*wh_vbz[i];
    wh_Fn[i] = max(0, ground_k*wh_pen[i] + ground_c*max(0, wh_vwd[i])*wh_contact[i]);
    wh_F[1, i] = -wh_Fn[i]*r20 - roll_fric*wh_vbx[i]*wh_contact[i];
    wh_F[2, i] = -wh_Fn[i]*r21 - side_fric*wh_vby[i]*wh_contact[i];
    wh_F[3, i] = -wh_Fn[i]*r22;
    wh_M[1, i] = wheel_y[i]*wh_F[3, i] - wheel_z[i]*wh_F[2, i];
    wh_M[2, i] = wheel_z[i]*wh_F[1, i] - wheel_x[i]*wh_F[3, i];
    wh_M[3, i] = wheel_x[i]*wh_F[2, i] - wheel_y[i]*wh_F[1, i];
  end for;
  F_ground = {wh_F[1, 1] + wh_F[1, 2] + wh_F[1, 3],
              wh_F[2, 1] + wh_F[2, 2] + wh_F[2, 3],
              wh_F[3, 1] + wh_F[3, 2] + wh_F[3, 3]};
  M_ground = {wh_M[1, 1] + wh_M[1, 2] + wh_M[1, 3],
              wh_M[2, 1] + wh_M[2, 2] + wh_M[2, 3],
              wh_M[3, 1] + wh_M[3, 2] + wh_M[3, 3]};

  // body translational dynamics
`
  );
  return source;
}

function safeGreyboxWrapper(greyboxName, controllerName, airframeMods = "") {
  return `
model SafeControllerGreyboxFlight
  input Real enable_safe(start = 1.0);
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  ${greyboxName} airframe${airframeMods};
  ${controllerName} safe;

  Real p_n;
  Real p_e;
  Real p_d;
  Real u;
  Real v;
  Real w;
  Real phi;
  Real theta;
  Real psi;
  Real p;
  Real q;
  Real r;

equation
  safe.stick_elevator = elevator;
  safe.stick_aileron = aileron;
  safe.stick_rudder = rudder;
  safe.phi = airframe.att[1];
  safe.theta = airframe.att[2];
  safe.p = airframe.rates[1];
  safe.q = airframe.rates[2];
  safe.r = airframe.rates[3];

  airframe.throttle = throttle;
  airframe.elevator = if enable_safe > 0.5 then safe.delta_e else elevator;
  airframe.aileron = if enable_safe > 0.5 then safe.delta_a else aileron;
  airframe.rudder = if enable_safe > 0.5 then safe.delta_r else rudder;

  p_n = airframe.pos[1];
  p_e = airframe.pos[2];
  p_d = airframe.pos[3];
  u = airframe.vel[1];
  v = airframe.vel[2];
  w = airframe.vel[3];
  phi = airframe.att[1];
  theta = airframe.att[2];
  psi = airframe.att[3];
  p = airframe.rates[1];
  q = airframe.rates[2];
  r = airframe.rates[3];
end SafeControllerGreyboxFlight;
`;
}

function groundWrapper(groundName) {
  return `
model GroundRollFlight
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  ${groundName} ground;

  Real p_n;
  Real p_e;
  Real p_d;
  Real u;
  Real v;
  Real w;
  Real phi;
  Real theta;
  Real psi;
  Real p;
  Real q;
  Real r;

equation
  ground.throttle = throttle;
  ground.rudder = rudder;
  p_n = ground.p_n;
  p_e = ground.p_e;
  p_d = 0.0;
  u = ground.V;
  v = 0.0;
  w = 0.0;
  phi = 0.0;
  theta = 0.0;
  psi = ground.psi;
  p = 0.0;
  q = 0.0;
  r = (ground.ks*rudder + ground.k0)*ground.V;
end GroundRollFlight;
`;
}

function generatedEntry(label, mo, group = "Modelica") {
  if (!mo?.generated_source || !mo?.generated_name) return null;
  return {
    label,
    group,
    modelName: mo.generated_name,
    source: mo.generated_source,
    output: "generic",
  };
}

function identifiedEntry(label, mo, group = "Modelica") {
  if (!mo?.identified_source || !mo?.identified_name) return null;
  return {
    label,
    group,
    modelName: mo.identified_name,
    source: mo.identified_source,
    output: "generic",
  };
}

export function buildModelicaFlightCatalog(models, externalEntries = []) {
  return externalEntries.filter((entry) => entry?.label === "RumocaFixedWing");
}

function rawNedStateFromStepper(stepper) {
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

function enuStateFromStepper(stepper) {
  const rawNed = rawNedStateFromStepper(stepper);
  const qNedBody = rawNed.slice(6, 10);
  const quatEnu = normQuat(quatMulWxyz([0, Math.SQRT1_2, Math.SQRT1_2, 0], qNedBody));
  const eulerNed = eulerFromQuatWxyz(qNedBody);
  const eulerEnu = {
    roll: eulerNed.roll,
    pitch: eulerNed.pitch,
    yaw: wrapPi(Math.PI / 2 - eulerNed.yaw),
    yawNed: eulerNed.yaw,
  };
  const state = [
    rawNed[1], rawNed[0], -rawNed[2],
    rawNed[4], rawNed[3], -rawNed[5],
    quatEnu[0], quatEnu[1], quatEnu[2], quatEnu[3],
    rawNed[10], rawNed[11], rawNed[12],
  ];
  state.positionEnu = state.slice(0, 3);
  state.quatEnu = quatEnu;
  state.eulerEnu = eulerEnu;
  state.rawNed = rawNed;
  return state;
}

export async function createModelicaFlightRunner(entry, source, options = {}) {
  const wasm = await loadRumoca();
  const modelName = entry?.modelName;
  if (!modelName) throw new Error("No Modelica model selected.");
  const simulationSource = sourceWithInitialState(entry, source || entry.source, options.initialState);
  const stepper = new wasm.WasmStepper(simulationSource, modelName);
  const inputNames = new Set(JSON.parse(stepper.input_names() || "[]"));
  const set = (name, value) => {
    if (inputNames.has(name)) stepper.set_input(name, value);
  };
  return {
    modelName,
    supportsSafeToggle: inputNames.has("enable_safe") || inputNames.has("mode"),
    reset() {
      stepper.reset();
      return enuStateFromStepper(stepper);
    },
    step(stick, dt, options = {}) {
      const throttle = clamp(finiteNumber(stick[0]), 0, 1);
      const elevator = clamp((entry?.elevatorSign ?? 1) * finiteNumber(stick[1]), -1, 1);
      const aileron = clamp(finiteNumber(stick[2]), -1, 1);
      const rudder = clamp(finiteNumber(stick[3]), -1, 1);
      set("enable_safe", options.safeEnabled === false ? 0 : 1);
      // Per-sample flight mode (0 = manual, 1 = SAFE) for the unified harness;
      // the model self-selects manual surfaces vs the learned SAFE controller.
      set("mode", Number.isFinite(Number(options.mode)) ? Number(options.mode) : (options.safeEnabled ? 1 : 0));
      set("throttle", throttle);
      set("elevator", elevator);
      set("aileron", aileron);
      set("rudder", rudder);
      set("stick_elevator", elevator);
      set("stick_aileron", aileron);
      set("stick_rudder", rudder);
      stepper.step(dt);
      return enuStateFromStepper(stepper);
    },
  };
}

export async function createModelicaAutopilotRunner(source, options = {}) {
  const wasm = await loadRumoca();
  const modelName = options.modelName || "CubControl.FixedWingOuterLoop";
  const dt = clamp(finiteNumber(options.dt, 0.02), 0.005, 0.2);
  const simulationSource = source.replace(/(parameter\s+Real\s+dt\s*=\s*)[-+0-9.eE]+/, `$1${dt.toPrecision(12)}`);
  const stepper = new wasm.WasmStepper(simulationSource, modelName);
  const inputNames = new Set(JSON.parse(stepper.input_names() || "[]"));
  const set = (name, value) => {
    if (inputNames.has(name)) stepper.set_input(name, finiteNumber(value));
  };
  const get = (name, fallback = 0) => {
    const value = stepper.get(name);
    return Number.isFinite(value) ? value : fallback;
  };
  const getOptional = (name) => {
    const value = stepper.get(name);
    return Number.isFinite(value) ? value : null;
  };
  const outputStick = () => [
    clamp(get("throttle", 0.7), 0, 1),
    clamp(get("elevator", 0), -1, 1),
    clamp(get("aileron", 0), -1, 1),
    clamp(get("rudder", 0), -1, 1),
  ];
  return {
    modelName,
    dt,
    reset() {
      stepper.reset();
      return outputStick();
    },
    step(state) {
      const positionEnu = state?.positionEnu || state?.slice?.(0, 3) || [0, 0, 0];
      const eulerEnu = state?.eulerEnu || { roll: 0, pitch: 0, yaw: 0, yawNed: NaN };
      set("x", positionEnu[0]);
      set("east", positionEnu[0]);
      set("y", positionEnu[1]);
      set("north", positionEnu[1]);
      set("z", positionEnu[2]);
      set("up", positionEnu[2]);
      set("roll", eulerEnu.roll);
      set("pitch", eulerEnu.pitch);
      set("yaw", eulerEnu.yaw);
      set("yaw_enu", eulerEnu.yaw);
      stepper.step(dt);
      const targetX = getOptional("target_x");
      const targetY = getOptional("target_y");
      const targetZ = getOptional("target_z");
      const targetEnu = [targetX, targetY, targetZ].every(Number.isFinite) ? [targetX, targetY, targetZ] : null;
      return {
        stick: outputStick(),
        telemetry: {
          waypoint: Math.max(1, Math.round(get("current_wp", 1))),
          waypointCount: Math.max(1, Math.round(get("waypoint_count", get("current_wp", 1)))),
          targetEnu,
          desiredSpeed: get("des_v", 0),
          desiredHeading: get("des_heading", 0),
          headingError: get("chi_err", 0),
          yawNed: eulerEnu.yawNed,
          yawEnu: eulerEnu.yaw,
          airborne: get("airborne", 0) > 0.5,
        },
      };
    },
  };
}

export async function modelicaDiagnostics(source) {
  const wasm = await loadRumoca();
  if (typeof wasm.lsp_diagnostics !== "function") return [];
  try {
    const raw = wasm.lsp_diagnostics(source);
    const parsed = JSON.parse(raw || "[]");
    if (Array.isArray(parsed)) return parsed;
    if (Array.isArray(parsed?.diagnostics)) return parsed.diagnostics;
    if (Array.isArray(parsed?.items)) return parsed.items;
    return [];
  } catch (_error) {
    return [];
  }
}

export async function modelicaCompletions(source, line, character) {
  const wasm = await loadRumoca();
  if (typeof wasm.lsp_completion !== "function") return [];
  try {
    const raw = wasm.lsp_completion(source, line, character);
    const parsed = JSON.parse(raw || "[]");
    return Array.isArray(parsed) ? parsed : (parsed.items || []);
  } catch (_error) {
    return [];
  }
}

export async function modelicaHover(source, line, character) {
  const wasm = await loadRumoca();
  if (typeof wasm.lsp_hover !== "function") return null;
  try {
    const raw = wasm.lsp_hover(source, line, character);
    return JSON.parse(raw || "null");
  } catch (_error) {
    return null;
  }
}
