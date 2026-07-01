// Flight explorer: full-flight segmentation viewer with on-the-fly model rollouts.
//
// Loads flight_explorer.json (truth, per-sample segmentation labels, full-rate
// sticks, fitted model parameters) and integrates method predictions in the
// browser from any clicked segment or time: manual segments use the recorded
// sticks re-referenced by the per-flight trim bias; stabilized segments close
// the loop with the identified SAFE inner-loop controller model, because the
// bare airframe alone cannot represent stabilized flight. Computing rollouts
// client-side avoids exporting a trace per (segment, method) pair.

const DATA_URLS = {
  "sportcub_mocap_5_22_26": "./public/data/flight_explorer.json",
  "sportcub_mocap_4_17_26": "./public/data/flight_explorer_4_17.json",
};
const DEFAULT_DATASET = "sportcub_mocap_5_22_26";
const LABEL_COLORS = { ground: "#8d6e63", ground_effect: "#26a69a", stabilized: "#5c7cfa", manual: "#f08c00" };
const METHOD_COLORS = { "6DOF-NominalGreyBox": "#d62728", "6DOF-LinearSS": "#2ca02c", "6DOF-RidgeResidual": "#9467bd", "6DOF-GreyBoxOEM": "#e8a838", "Modelica:RumocaFixedWing": "#7dd3fc", "6DOF-EquationError-LS": "#17becf", "6DOF-SINDy": "#e377c2", "6DOF-Koopman-EDMD": "#bcbd22", "6DOF-Symbolic-Stepwise": "#8c564b", "6DOF-Subspace-Hankel": "#1f77b4", "6DOF-GP-RBF": "#f7b6d2" };
const MIN_SPEED = 2.5;
const MAX_SPEED = 12.0;

const ex = {
  data: null,
  flightIndex: 0,
  anchorTimeS: null,
  selectedMethods: new Set(),
  predictions: {},
  // Whether the playback is showing this module's dataset. While another
  // dataset (synthetic 6DOF) is displayed the explorer must stay
  // silent: publishing an overlay would hijack the view back to the Sport
  // Cub flights.
  active: true,
};

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function normQuat(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function eulerFromQuat(q) {
  const [q0, q1, q2, q3] = normQuat(q);
  return [
    Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)),
    Math.asin(clamp(2 * (q0 * q2 - q3 * q1), -1, 1)),
    Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)),
  ];
}

function rotationBodyToInertial(q) {
  const [q0, q1, q2, q3] = q;
  return [
    [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
    [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
    [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
  ];
}

function matVec(m, v) {
  return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2], m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]];
}

function matTVec(m, v) {
  return [m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2], m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2], m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2]];
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

// Attached-flow nominal 6DOF dynamics ported from models/aircraft6dof/model.py
// (the nonlinear=False path: no stall gate, no hidden residual terms).
function nominalRhs(x, u, cfg) {
  const vel = [x[3], x[4], x[5]];
  const quat = normQuat([x[6], x[7], x[8], x[9]]);
  const rates = [x[10], x[11], x[12]];
  const speed = Math.max(Math.hypot(...vel), 1e-6);
  const alpha = Math.atan2(vel[2], Math.max(vel[0], 1e-6));
  const beta = Math.asin(clamp(vel[1] / speed, -0.98, 0.98));
  const throttle = clamp(u[0], 0, 1);
  const [, elevator, aileron, rudder] = u;
  const rateScale = Math.max(2 * speed, 1e-6);
  const pHat = (cfg.wing_span * rates[0]) / rateScale;
  const qHat = (cfg.mean_chord * rates[1]) / rateScale;
  const rHat = (cfg.wing_span * rates[2]) / rateScale;

  const cl = 0.27 + 5.2 * alpha + 0.36 * elevator + 3.1 * qHat;
  const cd = 0.045 + 0.075 * cl * cl + 0.42 * beta * beta + 0.018 * (aileron * aileron + rudder * rudder) + 0.01 * elevator * elevator;
  const cm = 0.03 - 1.05 * alpha - 1.15 * elevator - 9.0 * qHat;
  const cy = -0.82 * beta + 0.3 * rudder + 0.12 * aileron - 0.35 * rHat;
  const clRoll = -0.12 * beta + 0.42 * aileron - 0.5 * pHat + 0.1 * rHat;
  const cn = 0.18 * beta - 0.26 * rudder - 0.08 * aileron - 0.42 * rHat;
  const cx = -cd * Math.cos(alpha) + cl * Math.sin(alpha);
  const cz = -cd * Math.sin(alpha) - cl * Math.cos(alpha);

  const qbar = 0.5 * cfg.rho * speed * speed;
  const propWash = 1 + cfg.prop_wash_gain * throttle;
  const thrust = cfg.max_thrust * Math.pow(throttle, 1.45);
  const force = [
    propWash * qbar * cfg.wing_area * cx + thrust,
    propWash * qbar * cfg.wing_area * cy,
    propWash * qbar * cfg.wing_area * cz,
  ];
  const moment = [
    propWash * qbar * cfg.wing_area * cfg.wing_span * clRoll,
    propWash * qbar * cfg.wing_area * cfg.mean_chord * cm + cfg.prop_arm * thrust,
    propWash * qbar * cfg.wing_area * cfg.wing_span * cn,
  ];

  const rot = rotationBodyToInertial(quat);
  const posDot = matVec(rot, vel);
  const gravBody = matTVec(rot, [0, 0, cfg.gravity]);
  const velDot = [
    force[0] / cfg.mass + gravBody[0] - (rates[1] * vel[2] - rates[2] * vel[1]),
    force[1] / cfg.mass + gravBody[1] - (rates[2] * vel[0] - rates[0] * vel[2]),
    force[2] / cfg.mass + gravBody[2] - (rates[0] * vel[1] - rates[1] * vel[0]),
  ];
  const [ix, iy, iz] = cfg.inertia;
  const ixz = cfg.inertia_xz;
  const h = [ix * rates[0] - ixz * rates[2], iy * rates[1], iz * rates[2] - ixz * rates[0]];
  const torque = [moment[0] - (rates[1] * h[2] - rates[2] * h[1]), moment[1] - (rates[2] * h[0] - rates[0] * h[2]), moment[2] - (rates[0] * h[1] - rates[1] * h[0])];
  // Solve [ix,0,-ixz;0,iy,0;-ixz,0,iz] wdot = torque (2x2 block + scalar).
  const det = ix * iz - ixz * ixz;
  const ratesDot = [(iz * torque[0] + ixz * torque[2]) / det, torque[1] / iy, (ixz * torque[0] + ix * torque[2]) / det];
  const [q0, q1, q2, q3] = quat;
  const [p, qr, r] = rates;
  const quatDot = [
    0.5 * (-q1 * p - q2 * qr - q3 * r),
    0.5 * (q0 * p + q2 * r - q3 * qr),
    0.5 * (q0 * qr - q1 * r + q3 * p),
    0.5 * (q0 * r + q1 * qr - q2 * p),
  ];
  return [...posDot, ...velDot, ...quatDot, ...ratesDot];
}

function quatFromEuler(roll, pitch, yaw) {
  const cr = Math.cos(0.5 * roll), sr = Math.sin(0.5 * roll);
  const cp = Math.cos(0.5 * pitch), sp = Math.sin(0.5 * pitch);
  const cy = Math.cos(0.5 * yaw), sy = Math.sin(0.5 * yaw);
  return normQuat([
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ]);
}

// Sport Cub grey-box OEM dynamics ported from modelica/SportCubGreybox.mo.
// Twelve Euler states (pos NED, body FRD uvw, roll/pitch/yaw, body FRD rates);
// u is the flight-control vector in (throttle, elevator, aileron, rudder)
// order; gb carries the fitted+fixed parameters by name.
function greyboxRhs(s, u, gb) {
  const p = gb.p;
  const [uB, vB, wB] = [s[3], s[4], s[5]];
  const [phi, theta, psi] = [s[6], s[7], s[8]];
  const [pR, qR, rR] = [s[9], s[10], s[11]];
  const eps = 1e-6;

  const cPhi = Math.cos(phi), sPhi = Math.sin(phi);
  const cTh = Math.cos(theta), sTh = Math.sin(theta);
  const cPsi = Math.cos(psi), sPsi = Math.sin(psi);
  const r00 = cTh * cPsi;
  const r01 = sPhi * sTh * cPsi - cPhi * sPsi;
  const r02 = cPhi * sTh * cPsi + sPhi * sPsi;
  const r10 = cTh * sPsi;
  const r11 = sPhi * sTh * sPsi + cPhi * cPsi;
  const r12 = cPhi * sTh * sPsi - sPhi * cPsi;
  const r20 = -sTh;
  const r21 = sPhi * cTh;
  const r22 = cPhi * cTh;

  const vt = Math.sqrt(uB * uB + vB * vB + wB * wB) + eps;
  const vxz = Math.sqrt(uB * uB + wB * wB) + eps;
  const alpha = Math.atan2(wB, uB) + p.wing_incidence;
  const beta = Math.atan2(vB, vxz);
  const qbar = 0.5 * p.rho * vt * vt;
  const sigma = (1 + Math.tanh((alpha - p.alpha_stall) / p.blend_width)) / 2;

  const wx = [uB / vt, vB / vt, wB / vt];
  const ref = Math.abs(wx[2]) < Math.abs(wx[0]) ? [0, 0, 1] : [1, 0, 0];
  const rdot = ref[0] * wx[0] + ref[2] * wx[2];
  const wzt = [ref[0] - rdot * wx[0], -rdot * wx[1], ref[2] - rdot * wx[2]];
  const nz = Math.hypot(...wzt) + eps;
  const wz = [wzt[0] / nz, wzt[1] / nz, wzt[2] / nz];
  const wy = [wz[1] * wx[2] - wz[2] * wx[1], wz[2] * wx[0] - wz[0] * wx[2], wz[0] * wx[1] - wz[1] * wx[0]];

  const thr = clamp(u[0], 0, 1);
  const elevRad = p.max_defl_elev * clamp(u[1], -1, 1);
  const ailRad = p.max_defl_ail * clamp(u[2], -1, 1);
  const rudRad = -p.max_defl_rud * clamp(u[3], -1, 1);

  const clLin = p.CL0 + p.CLa * alpha;
  const clFp = 2 * Math.sin(alpha) * Math.cos(alpha);
  const cl = (1 - sigma) * clLin + sigma * clFp;
  const cdLin = p.CD0 + p.k_ind * clLin * clLin;
  const cdFp = p.CD0_fp + 2 * Math.sin(alpha) * Math.sin(alpha);
  const cd = (1 - sigma) * cdLin + sigma * cdFp;
  const cyLin = p.CYb * beta + p.CYda * ailRad + p.CYdr * rudRad + p.CYp * (p.b / (2 * vt)) * pR + p.CYr * (p.b / (2 * vt)) * rR;
  const cyFp = p.CY_fp_coef * Math.sin(beta) * Math.cos(alpha);
  const cy = (1 - sigma) * cyLin + sigma * cyFp;
  const clAero = p.Clda * ailRad + p.Cldr * rudRad + p.Clb * beta + p.Clp * (p.b / (2 * vt)) * pR + p.Clr * (p.b / (2 * vt)) * rR;
  const cmAero = p.Cm0 + p.Cma * alpha + p.Cmde * elevRad + p.Cmq * (p.cbar / (2 * vt)) * qR;
  const cnAero = p.Cnb * beta + p.Cndr * rudRad + p.Cnda * ailRad + p.Cnp * (p.b / (2 * vt)) * pR + p.Cnr * (p.b / (2 * vt)) * rR;

  const fa = [
    qbar * p.S * (wx[0] * (-cd) + wy[0] * cy + wz[0] * (-cl)),
    qbar * p.S * (wx[1] * (-cd) + wy[1] * cy + wz[1] * (-cl)),
    qbar * p.S * (wx[2] * (-cd) + wy[2] * cy + wz[2] * (-cl)),
  ];
  const ma = [qbar * p.S * p.b * clAero, qbar * p.S * p.cbar * cmAero, qbar * p.S * p.b * cnAero];

  const wheelX = [0.10, -0.08, -0.08];
  const wheelY = [0.0, 0.10, -0.10];
  const wheelZ = [0.055, 0.055, 0.055];
  const fg = [0, 0, 0];
  const mg = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    const whH = s[2] + r20 * wheelX[i] + r21 * wheelY[i] + r22 * wheelZ[i];
    const whVbx = uB + qR * wheelZ[i] - rR * wheelY[i];
    const whVby = vB + rR * wheelX[i] - pR * wheelZ[i];
    const whVbz = wB + pR * wheelY[i] - qR * wheelX[i];
    const whVwd = r20 * whVbx + r21 * whVby + r22 * whVbz;
    const contactEps = p.ground_contact_eps ?? 1e-4;
    const whPen = 0.5 * (whH + Math.sqrt(whH * whH + contactEps * contactEps));
    const whContact = whPen / (whPen + contactEps);
    const fn = Math.max(0, p.ground_k * whPen + p.ground_c * Math.max(0, whVwd) * whContact);
    const wf = [
      -fn * r20 - p.roll_fric * whVbx * whContact,
      -fn * r21 - p.side_fric * whVby * whContact,
      -fn * r22,
    ];
    fg[0] += wf[0]; fg[1] += wf[1]; fg[2] += wf[2];
    mg[0] += wheelY[i] * wf[2] - wheelZ[i] * wf[1];
    mg[1] += wheelZ[i] * wf[0] - wheelX[i] * wf[2];
    mg[2] += wheelX[i] * wf[1] - wheelY[i] * wf[0];
  }

  const fx = fa[0] + p.thr_max * thr + fg[0];
  const fy = fa[1] + fg[1];
  const fz = fa[2] + fg[2];
  const mx = ma[0] + mg[0];
  const my = ma[1] + mg[1];
  const mz = ma[2] + mg[2];

  const uDot = fx / p.m - p.g * sTh + rR * vB - qR * wB;
  const vDot = fy / p.m + p.g * sPhi * cTh + pR * wB - rR * uB;
  const wDot = fz / p.m + p.g * cPhi * cTh + qR * uB - pR * vB;

  const hx = p.Ixx * pR - p.Ixz * rR;
  const hy = p.Iyy * qR;
  const hz = p.Izz * rR - p.Ixz * pR;
  const tx = mx - (qR * hz - rR * hy);
  const ty = my - (rR * hx - pR * hz);
  const tz = mz - (pR * hy - qR * hx);
  const detI = p.Ixx * p.Izz - p.Ixz * p.Ixz;
  const pDot = (p.Izz * tx + p.Ixz * tz) / detI;
  const qDot = ty / p.Iyy;
  const rDot = (p.Ixz * tx + p.Ixx * tz) / detI;

  const cThSafe = (cTh >= 0 ? 1 : -1) * Math.max(Math.abs(cTh), 1e-3);
  const common = qR * sPhi + rR * cPhi;
  const phiDot = pR + (sTh / cThSafe) * common;
  const thetaDot = qR * cPhi - rR * sPhi;
  const psiDot = common / cThSafe;

  return [
    r00 * uB + r01 * vB + r02 * wB,
    r10 * uB + r11 * vB + r12 * wB,
    r20 * uB + r21 * vB + r22 * wB,
    uDot, vDot, wDot,
    phiDot, thetaDot, psiDot,
    pDot, qDot, rDot,
  ];
}

export function makeGreyboxStepper(greybox, dt) {
  const p = { ...greybox.fixed_parameters, ...(greybox.default_parameters || {}) };
  greybox.parameter_names.forEach((name, i) => { p[name] = greybox.parameters[i]; });
  const gb = { p };
  return (x, u) => {
    const e = eulerFromQuat([x[6], x[7], x[8], x[9]]);
    let s = [x[0], x[1], x[2], x[3], x[4], x[5], e[0], e[1], e[2], x[10], x[11], x[12]];
    const k1 = greyboxRhs(s, u, gb);
    const k2 = greyboxRhs(s.map((v, i) => v + 0.5 * dt * k1[i]), u, gb);
    const k3 = greyboxRhs(s.map((v, i) => v + 0.5 * dt * k2[i]), u, gb);
    const k4 = greyboxRhs(s.map((v, i) => v + dt * k3[i]), u, gb);
    s = s.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    s[8] = Math.atan2(Math.sin(s[8]), Math.cos(s[8]));
    const q = quatFromEuler(s[6], s[7], s[8]);
    return [s[0], s[1], s[2], s[3], s[4], s[5], q[0], q[1], q[2], q[3], s[9], s[10], s[11]];
  };
}

// Heading/position-invariant closed-loop SAFE model: the regression predicts
// the next body velocities, roll/pitch, rates, and the heading increment from
// [u,v,w,phi,theta,p,q,r, stick, 1]; heading integrates the increment and
// position integrates the rotated body velocity exactly, so free runs can fly
// whole laps instead of wandering off the affine fit's operating point.
export function makeSafeStepper(W, dt) {
  return (x, stick) => {
    const quat = normQuat([x[6], x[7], x[8], x[9]]);
    const e = eulerFromQuat(quat);
    const rot = rotationBodyToInertial(quat);
    const step = matVec(rot, [x[3], x[4], x[5]]);
    const z = [x[3], x[4], x[5], e[0], e[1], x[10], x[11], x[12], stick[0], stick[1], stick[2], stick[3], 1];
    const out = new Array(9).fill(0);
    for (let i = 0; i < z.length; i++) {
      const row = W[i];
      for (let j = 0; j < 9; j++) out[j] += z[i] * row[j];
    }
    const psi = Math.atan2(Math.sin(e[2] + out[8]), Math.cos(e[2] + out[8]));
    const q = quatFromEuler(out[3], out[4], psi);
    return [
      x[0] + step[0] * dt, x[1] + step[1] * dt, x[2] + step[2] * dt,
      out[0], out[1], out[2],
      q[0], q[1], q[2], q[3],
      out[5], out[6], out[7],
    ];
  };
}

function postStep(x) {
  const out = x.slice();
  const q = normQuat([out[6], out[7], out[8], out[9]]);
  out[6] = q[0]; out[7] = q[1]; out[8] = q[2]; out[9] = q[3];
  const speed = Math.hypot(out[3], out[4], out[5]);
  if (speed > MAX_SPEED) {
    for (let i = 3; i < 6; i++) out[i] *= MAX_SPEED / speed;
  } else if (speed > 1e-9 && speed < MIN_SPEED) {
    for (let i = 3; i < 6; i++) out[i] *= MIN_SPEED / speed;
  }
  for (let i = 10; i < 13; i++) out[i] = clamp(out[i], -8, 8);
  return out;
}

function nominalStep(x, u, dt, cfg) {
  const k1 = nominalRhs(x, u, cfg);
  const x2 = x.map((v, i) => v + 0.5 * dt * k1[i]);
  const k2 = nominalRhs(x2, u, cfg);
  const x3 = x.map((v, i) => v + 0.5 * dt * k2[i]);
  const k3 = nominalRhs(x3, u, cfg);
  const x4 = x.map((v, i) => v + dt * k3[i]);
  const k4 = nominalRhs(x4, u, cfg);
  return postStep(x.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])));
}

function affinePredict(x, u, weights) {
  const phi = [...x, ...u, 1];
  const out = new Array(13).fill(0);
  for (let i = 0; i < phi.length; i++) {
    const row = weights[i];
    const value = phi[i];
    for (let j = 0; j < 13; j++) out[j] += value * row[j];
  }
  return out;
}

// Generic neural-network evaluator for browser-side prediction. NumPy-trained
// nets (random-feature readouts, MLPs, RBF expansions) deploy as JSON weight
// specs and run through this forward pass; torch-trained models deploy as
// ONNX and run through onnxruntime-web instead, with the same integration
// loop calling the session per step.
function evalNet(spec, input) {
  if (spec.type === "rbf") {
    const z = input.map((v, i) => (v - spec.x_mean[i]) / spec.x_scale[i]);
    const phi = [1];
    for (const center of spec.centers) {
      let d2 = 0;
      for (let i = 0; i < z.length; i++) d2 += (z[i] - center[i]) ** 2;
      phi.push(Math.exp(-0.5 * d2 / (spec.length_scale * spec.length_scale)));
    }
    return spec.weights[0].map((_, j) => {
      let acc = 0;
      for (let i = 0; i < phi.length; i++) acc += phi[i] * spec.weights[i][j];
      return acc * spec.y_scale[j] + spec.y_mean[j];
    });
  }
  let h = input;
  for (const layer of spec.layers) {
    const out = layer.b.slice();
    for (let i = 0; i < h.length; i++) {
      const row = layer.w[i];
      for (let j = 0; j < out.length; j++) out[j] += h[i] * row[j];
    }
    if (layer.act === "tanh") h = out.map(Math.tanh);
    else if (layer.act === "relu") h = out.map((v) => Math.max(0, v));
    else h = out;
  }
  return h;
}

// Heading/position-invariant feature maps mirroring the suite: body
// velocities, body-frame gravity direction (attitude enters the dynamics
// only through it), body rates, sticks.
function invariantStateJS(x) {
  const q = normQuat([x[6], x[7], x[8], x[9]]);
  const [q0, q1, q2, q3] = q;
  return [x[3], x[4], x[5],
    2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2),
    x[10], x[11], x[12]];
}

function linearFeaturesJS(x, u) {
  return [...invariantStateJS(x), ...u, 1];
}

function polyFeaturesJS(x, u) {
  const z = [...invariantStateJS(x), ...u];
  const out = [1, ...z];
  for (let i = 0; i < z.length; i++) {
    for (let j = i; j < z.length; j++) out.push(z[i] * z[j]);
  }
  return out;
}

function applyStandardizedJS(phi, spec) {
  const out = new Array(spec.weights[0].length).fill(0);
  for (let i = 0; i < phi.length; i++) {
    const v = (phi[i] - spec.mean[i]) / spec.scale[i];
    if (v === 0) continue;
    const row = spec.weights[i];
    for (let j = 0; j < out.length; j++) out[j] += v * row[j];
  }
  return out;
}

// Generic surrogates predict only the ten dynamic states; position advances
// kinematically so travel is bounded by the (clamped) velocity state.
function kinematicStepJS(x, dyn, dt) {
  // dyn = predicted [u, v, w, p, q, r]; position and quaternion advance by
  // exact kinematics from the current state (axis-angle attitude step).
  let quat = normQuat([x[6], x[7], x[8], x[9]]);
  const step = matVec(rotationBodyToInertial(quat), [x[3], x[4], x[5]]);
  const wMag = Math.hypot(x[10], x[11], x[12]);
  if (wMag * dt > 1e-12) {
    const a = 0.5 * wMag * dt;
    const sc = Math.sin(a) / wMag;
    quat = quatMul(quat, [Math.cos(a), x[10] * sc, x[11] * sc, x[12] * sc]);
  }
  return postStep([
    x[0] + step[0] * dt, x[1] + step[1] * dt, x[2] + step[2] * dt,
    dyn[0], dyn[1], dyn[2],
    quat[0], quat[1], quat[2], quat[3],
    dyn[3], dyn[4], dyn[5],
  ]);
}

function makeSurrogateStepper(spec, dt, cfg) {
  if (spec.kind === "hankel") {
    // Lagged ARX on position-free state history, increment-integrated. The
    // window re-seeds whenever the incoming state is not our own last output
    // (fresh anchor or SAFE-model handoff).
    let hist = null;
    let last = null;
    return (x, u) => {
      if (last !== x) hist = new Array(spec.lag).fill(x);
      const phi = [];
      for (const h of hist) phi.push(...invariantStateJS(h));
      phi.push(...u, 1);
      const delta = new Array(6).fill(0);
      for (let i = 0; i < phi.length; i++) {
        if (phi[i] === 0) continue;
        const row = spec.weights[i];
        for (let j = 0; j < delta.length; j++) delta[j] += phi[i] * row[j];
      }
      const dynRows = [3, 4, 5, 10, 11, 12];
      const next = kinematicStepJS(x, delta.map((d, j) => x[dynRows[j]] + d), dt);
      hist = [...hist.slice(1), next];
      last = next;
      return next;
    };
  }
  if (spec.kind === "rbf_residual") {
    return (x, u) => {
      const base = nominalStep(x, u, dt, cfg);
      const z = [...invariantStateJS(x), ...u];
      const phi = spec.centers.map((center) => {
        let d2 = 0;
        for (let i = 0; i < z.length; i++) {
          const d = (z[i] - center[i]) / spec.length_scale[i];
          d2 += d * d;
        }
        return Math.exp(-0.5 * d2);
      });
      phi.push(1);
      const out = new Array(6).fill(0);
      for (let i = 0; i < phi.length; i++) {
        const row = spec.weights[i];
        for (let j = 0; j < out.length; j++) out[j] += phi[i] * row[j];
      }
      const dynRows = [3, 4, 5, 10, 11, 12];
      return kinematicStepJS(x, out.map((d, j) => base[dynRows[j]] + d), dt);
    };
  }
  return (x, u) => {
    const phi = spec.degree === 1 ? linearFeaturesJS(x, u) : polyFeaturesJS(x, u);
    const delta = applyStandardizedJS(phi, spec);
    const gain = spec.kind === "derivative" ? dt : 1;
    const dynRows = [3, 4, 5, 10, 11, 12];
    return kinematicStepJS(x, delta.map((d, j) => x[dynRows[j]] + gain * d), dt);
  };
}

export function makeStepper(method, models, dt) {
  const cfg = models.config;
  if (models.surrogates && models.surrogates[method]) {
    return makeSurrogateStepper(models.surrogates[method], dt, cfg);
  }
  if (models.nets && models.nets[method]) {
    const spec = models.nets[method];
    return (x, u) => {
      const base = spec.residual ? nominalStep(x, u, dt, cfg) : new Array(13).fill(0);
      const out = evalNet(spec, [...x, ...u]);
      return postStep(base.map((v, i) => v + out[i]));
    };
  }
  if (method === "6DOF-NominalGreyBox") return (x, u) => nominalStep(x, u, dt, cfg);
  if (method === "6DOF-GreyBoxOEM" && models.greybox) return makeGreyboxStepper(models.greybox, dt);
  if (method === "6DOF-LinearSS") return (x, u) => postStep(affinePredict(x, u, models.linear_weights));
  return (x, u) => {
    const base = nominalStep(x, u, dt, cfg);
    const res = affinePredict(x, u, models.residual_weights);
    return postStep(base.map((v, i) => v + res[i]));
  };
}

export function safeController(gains) {
  // SAFE self-level: the stick commands attitude through the envelope clip
  // ([Kp, cmd_scale, envelope_limit, Kd, offset] per attitude axis), the loop
  // closes on attitude error with rate damping, and surfaces saturate.
  return (stick, x) => {
    const euler = eulerFromQuat([x[6], x[7], x[8], x[9]]);
    const ge = gains.elevator;
    const ga = gains.aileron;
    const gr = gains.rudder;
    const thetaCmd = clamp(ge[1] * stick[1], -Math.abs(ge[2]), Math.abs(ge[2]));
    const phiCmd = clamp(ga[1] * stick[2], -Math.abs(ga[2]), Math.abs(ga[2]));
    return [
      stick[0],
      clamp(ge[0] * (thetaCmd - euler[1]) - ge[3] * x[11] + ge[4], -0.65, 0.65),
      clamp(ga[0] * (phiCmd - euler[0]) - ga[3] * x[10] + ga[4], -0.75, 0.75),
      clamp(gr[0] * stick[3] + gr[1] * x[12] + gr[2], -0.65, 0.65),
    ];
  };
}

export function makeKeyboardFlightSimulation(method, timeS) {
  if (!ex.data?.models) return null;
  const f = flight();
  const models = ex.data.models;
  const dt = f.dt_full || 1 / 240;
  const startS = firstFlyableTime(f, timeS ?? ex.anchorTimeS ?? 0);
  const x0 = estimateInitialState(f, startS ?? 0);
  let step;
  if (method === "SAFE closed loop") {
    if (!models.safe_invariant_weights) return null;
    step = makeSafeStepper(models.safe_invariant_weights, dt);
  } else if (method === "SAFE controller + GreyBox") {
    if (!models.safe_gains || !models.greybox) return null;
    const airframeStep = makeStepper("6DOF-GreyBoxOEM", models, dt);
    const controller = safeController(models.safe_gains);
    step = (x, stick) => airframeStep(x, controller(stick, x));
  } else {
    step = makeStepper(method, models, dt);
  }
  return { x: x0, dt, startS: startS ?? 0, step };
}

function estimateInitialState(flight, timeS) {
  // Interpolate the measured state at timeS between its bracketing samples.
  // (A wide local-fit window biases the position toward the inside of turns,
  // so the free run would start visibly off the measured track; the converter
  // already smooths velocities, leaving nothing for a fit to clean up.)
  const t = flight.time;
  let hi = t.findIndex((v) => v >= timeS);
  if (hi < 0) hi = t.length - 1;
  const lo = Math.max(0, hi - 1);
  const span = t[hi] - t[lo];
  const w = span > 1e-9 ? clamp((timeS - t[lo]) / span, 0, 1) : 0;
  const a = flight.state[lo];
  const b = flight.state[hi];
  const x0 = a.map((v, j) => v + (b[j] - v) * w);
  if (a[6] * b[6] + a[7] * b[7] + a[8] * b[8] + a[9] * b[9] < 0) {
    // Antipodal quaternions: lerp through the short way before normalizing.
    for (let j = 6; j < 10; j++) x0[j] = a[j] + (-b[j] - a[j]) * w;
  }
  const q = normQuat([x0[6], x0[7], x0[8], x0[9]]);
  x0[6] = q[0]; x0[7] = q[1]; x0[8] = q[2]; x0[9] = q[3];
  return x0;
}

const Q_NED_TO_ENU = [0, Math.SQRT1_2, Math.SQRT1_2, 0]; // 180 deg about (1,1,0)/sqrt(2)

function quatMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function makeGroundStepper(ground, dt) {
  // Planar rolling model on the 13-state: position/heading integrate the
  // fitted (kT, mu, cv, ks, k0) law, altitude pins to the flight's ground
  // plane, attitude is level at the rolled heading.
  const p = ground.parameters;
  const f = ground.fixed;
  return (x, stick, groundZ) => {
    const psi0 = eulerFromQuat([x[6], x[7], x[8], x[9]])[2];
    let V = Math.hypot(x[3], x[4]);
    const thr = Math.max(stick[0], 0);
    const acc = p.kT * (f.max_thrust_n / f.mass) * Math.pow(thr, f.thrust_exponent) - p.mu * f.g - p.cv * V * V;
    V = Math.max(V + dt * acc, 0);
    const psi = psi0 + dt * (p.ks * stick[3] + p.k0) * V;
    const out = x.slice();
    out[0] = x[0] + dt * V * Math.cos(psi);
    out[1] = x[1] + dt * V * Math.sin(psi);
    out[2] = groundZ != null ? groundZ : x[2];
    out[3] = V; out[4] = 0; out[5] = 0;
    out[6] = Math.cos(psi / 2); out[7] = 0; out[8] = 0; out[9] = Math.sin(psi / 2);
    out[10] = 0; out[11] = 0; out[12] = (p.ks * stick[3] + p.k0) * V;
    return out;
  };
}

function rolloutFrom(flight, models, method, timeS) {
  const dt = flight.dt_full;
  const startIdx = Math.max(0, Math.round(timeS / dt));
  const sticks = flight.stick_full;
  const labels = flight.labels_full;
  const modes = flight.mode_full || labels.map((label) => (label === 2 ? 1 : 0));
  const groundStep = models.ground ? makeGroundStepper(models.ground, dt) : null;
  // Without a ground model, stop at the next ground contact (no gear physics).
  let endIdx = sticks.length;
  if (!groundStep) {
    for (let k = startIdx + Math.round(1 / dt); k < labels.length; k++) {
      if (labels[k] === 0) { endIdx = k; break; }
    }
  }
  const stepper = makeStepper(method, models, dt);
  const safeStep = models.safe_invariant_weights ? makeSafeStepper(models.safe_invariant_weights, dt) : null;
  const ctrl = models.safe_gains ? safeController(models.safe_gains) : null;
  const bias = flight.bias;
  let x = estimateInitialState(flight, timeS);
  const stride = Math.max(1, Math.round(0.1 / dt));
  const times = [];
  const altitude = [];
  const pitch = [];
  const posEnu = [];
  const quatEnu = [];
  for (let k = startIdx; k < endIdx - 1; k++) {
    if ((k - startIdx) % stride === 0) {
      times.push(k * dt);
      altitude.push(-x[2]);
      pitch.push(eulerFromQuat([x[6], x[7], x[8], x[9]])[1]);
      posEnu.push([x[1], x[0], -x[2]]);
      quatEnu.push(quatMul(Q_NED_TO_ENU, normQuat([x[6], x[7], x[8], x[9]])));
    }
    const stick = sticks[k];
    if (labels[k] === 0 && groundStep) {
      // On the ground (per the recorded segmentation) the planar rolling
      // model drives the state; at liftoff the airborne model takes over
      // from the rolled position/heading/speed.
      x = groundStep(x, stick, flight.ground_z);
    } else if (modes[k] === 1 && safeStep) {
      // SAFE engaged: the directly identified closed-loop model replaces the
      // bare airframe + provisional controller decomposition. The handoff
      // keys off the recorded mode channel (a low SAFE pass is still
      // closed-loop flight) and the state carries over continuously.
      x = safeStep(x, stick);
    } else {
      const u = modes[k] === 1 && ctrl ? ctrl(stick, x) : stick.map((v, i) => v - bias[i]);
      x = stepper(x, u);
    }
    if (!x.every(Number.isFinite)) break;
  }
  return { times, altitude, pitch, posEnu, quatEnu };
}

function flight() {
  return ex.data.flights[ex.flightIndex];
}

function predictionMethods() {
  // Temporarily focus the browser free-run on the Rumoca/CasADi greybox path.
  const usable = Array.from(ex.selectedMethods).filter((m) => ex.data.methods.includes(m));
  if (usable.length) return usable;
  return ex.selectedMethods.size ? [] : ["6DOF-GreyBoxOEM"];
}

function recomputePredictions() {
  ex.predictions = {};
  if (ex.anchorTimeS == null) return;
  for (const method of predictionMethods()) {
    ex.predictions[method] = rolloutFrom(flight(), ex.data.models, method, ex.anchorTimeS);
  }
}

const datasetCache = {};

async function loadExplorerDataset(scenario) {
  if (!datasetCache[scenario]) {
    const response = await fetch(DATA_URLS[scenario]);
    if (!response.ok) throw new Error(`${response.status}`);
    datasetCache[scenario] = await response.json();
  }
  return datasetCache[scenario];
}

function safeScoreNote() {
  const scores = ex.data?.models?.safe_scores;
  if (!scores || scores.validation_pos_err_5s_m == null) return "";
  return ` (currently ${scores.validation_pos_err_5s_m.toFixed(1)} m mean over ${scores.validation_windows} held-out windows)`;
}

function groundScoreNote() {
  const sc = ex.data?.models?.ground?.scores;
  if (!sc || sc.ground_pos_err_5s_m == null) return "";
  return `; currently ${sc.ground_pos_err_5s_m.toFixed(1)} m mean over ${sc.validation_windows} held-out windows`;
}

const FEATURE_STATE_NAMES = ["u", "v", "w", "g\u2093", "g\u1d67", "g\u1d22", "p", "q", "r"];
const STICK_NAMES = ["thr", "elev", "ail", "rud"];
const DYN_TARGET_NAMES = ["u\u0307", "v\u0307", "w\u0307", "p\u0307", "q\u0307", "r\u0307"];
const STATE13_NAMES = ["x", "y", "z", "u", "v", "w", "qw", "qx", "qy", "qz", "p", "q", "r"];

function polyFeatureNames() {
  const z = [...FEATURE_STATE_NAMES, ...STICK_NAMES];
  const names = ["1", ...z];
  for (let i = 0; i < z.length; i++) for (let j = i; j < z.length; j++) names.push(`${z[i]}\u00b7${z[j]}`);
  return names;
}

// Full sorted equations from a weight matrix: one line per output listing
// every coefficient above the cutoff.
function weightEquations(W, featureNames, outputNames, { cutoff = 1e-4, maxTerms = 12, transform = null } = {}) {
  const lines = [];
  for (let j = 0; j < outputNames.length; j++) {
    const terms = [];
    for (let i = 0; i < W.length; i++) {
      let v = W[i][j];
      if (transform) v = transform(v, i, j);
      if (Math.abs(v) > cutoff) terms.push({ name: featureNames[i], v });
    }
    terms.sort((a, b) => Math.abs(b.v) - Math.abs(a.v));
    const shown = terms.slice(0, maxTerms);
    const rhs = shown.map((t, idx) => `${t.v < 0 ? "\u2212" : idx ? "+" : ""} ${Math.abs(t.v).toPrecision(3)}\u00b7${t.name}`).join(" ");
    const more = terms.length > maxTerms ? ` \u2026 (+${terms.length - maxTerms} terms)` : "";
    lines.push(`<b>${outputNames[j]}</b> = ${rhs || "0"}${more}`);
  }
  return `<p class="model-terms">${lines.join("<br>")}</p>`;
}

function surrogateDetail(name, spec) {
  if (spec.kind === "hankel") {
    const featNames = [];
    for (let lagIdx = spec.lag - 1; lagIdx >= 0; lagIdx--) {
      for (const n of FEATURE_STATE_NAMES) featNames.push(lagIdx ? `${n}[k\u2212${lagIdx}]` : `${n}[k]`);
    }
    featNames.push(...STICK_NAMES, "1");
    return `<p class="model-note">lag-${spec.lag} ARX on the heading/position-invariant state history; predicts dynamic-state increments, position and attitude integrate kinematically. Weights ${spec.weights.length}\u00d7${spec.weights[0].length}.</p>`
      + weightEquations(spec.weights, featNames, DYN_TARGET_NAMES.map((n) => `\u0394${n.replace("\u0307", "")}`), { cutoff: 1e-3 });
  }
  if (spec.kind === "rbf_residual") {
    const ls = spec.length_scale.map((v, i) => `${[...FEATURE_STATE_NAMES, ...STICK_NAMES][i]}=${v.toPrecision(3)}`).join(", ");
    return `<p class="model-note">${spec.centers.length} radial basis centers over the invariant state + sticks; the kernel residual corrects the attached-flow nominal model's dynamic states. Weights ${spec.weights.length}\u00d7${spec.weights[0].length}.</p>
      <p class="model-note">per-dimension length scales: ${ls}</p>`;
  }
  const names = spec.degree === 1 ? [...FEATURE_STATE_NAMES, ...STICK_NAMES, "1"] : polyFeatureNames();
  const nnz = spec.weights.flat().filter((v) => v !== 0).length;
  const outs = spec.kind === "derivative" ? DYN_TARGET_NAMES : DYN_TARGET_NAMES.map((n) => `\u0394${n.replace("\u0307", "")}`);
  return `<p class="model-note">${spec.kind === "derivative" ? "continuous-time derivative" : "one-step increment"} model on ${spec.degree === 1 ? "linear" : "quadratic"} invariant features \u00b7 ${nnz} nonzero of ${spec.weights.length * spec.weights[0].length} standardized coefficients. Position and attitude integrate kinematically.</p>`
    + weightEquations(spec.weights, names, outs, { cutoff: 1e-3 })
    + modelicaSection(spec.modelica);
}

function affineDetail(weights, dt, residual) {
  const featNames = [...STATE13_NAMES, ...["thr", "elev", "ail", "rud"], "1"];
  const note = residual
    ? "one-step residual added to the attached-flow nominal RK4 step"
    : "one-step affine map x[k+1] = W\u1d40[x, u, 1]; shown as continuous-time rates ((W \u2212 I)/dt)";
  const transform = residual
    ? (v) => v / dt
    : (v, i, j) => (i === j ? v - 1 : v) / dt;
  return `<p class="model-note">${note}; weights ${weights.length}\u00d7${weights[0].length}, raw state/stick features (positions included by the affine contract).</p>`
    + weightEquations(weights, featNames, STATE13_NAMES.map((n) => `${n}\u0307`), { cutoff: 0.05, transform });
}

function escapeHtml(text) {
  return String(text).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

function moDownloadLink(filename, text) {
  const uri = "data:text/x-modelica;charset=utf-8," + encodeURIComponent(text);
  return `<a class="mo-download" href="${uri}" download="${filename}">download ${filename}</a>`;
}

// Show the actual Modelica source: the single source of truth for the physics
// (compiled to CasADi/JAX via Rumoca) plus the identified model with the fitted
// parameters baked in.
// Per-method "what it is / inputs / outputs", shown at the top of every Model
// Inspector card. Keyed by the card title. In the flight explorer every method
// is rolled out open-loop from the state at the chosen start time, driven by
// the recorded pilot sticks, so inputs/outputs are described in those terms.
const METHOD_INFO = {
  "GreyBoxOEM": {
    what: "Physical lumped-parameter grey-box airframe using the Rumoca fixed-wing plant structure; its aero, thrust, ground-contact, and control-surface coefficients are fitted by output-error against the real flights.",
    in: "initial flight state + recorded pilot commands (throttle, elevator, aileron, rudder)",
    out: "open-loop predicted trajectory — position, attitude, body velocity & rates",
  },
  "SAFE closed loop (shared)": {
    what: "Direct closed-loop linear fit of the SAFE-stabilized dynamics: one heading/position-invariant linear map (sticks + state → next state), with no controller/airframe split. Deployed model for stabilized segments.",
    in: "invariant state + pilot sticks (attitude commands while SAFE is engaged)",
    out: "next state, integrated to a closed-loop trajectory",
  },
  "SAFE controller (grey-box)": {
    what: "The identified SAFE PD attitude controller (sat·stick − attitude PD + rate damping, surface lag) composed with the grey-box airframe to close the loop. Interpretable structure diagnostic.",
    in: "pilot sticks (commanding attitude); the controller computes surfaces, the airframe integrates",
    out: "closed-loop trajectory of the composed controller + airframe",
  },
  "Ground roll & ground effect": {
    what: "Planar runway ground-roll: thrust minus rolling/quadratic drag drives speed, rudder steers the heading rate, position integrates along heading. (Ground-effect is a reported ΔCL/ΔCD diagnostic, not a model.)",
    in: "throttle, rudder",
    out: "ground track — north, east, heading ψ, ground speed V",
  },
  "LinearSS": {
    what: "A single global affine discrete state-space, x[k+1] = A·x[k] + B·u + c, fit by ridge regression.",
    in: "full state + pilot commands",
    out: "next state, integrated to a trajectory",
  },
  "RidgeResidual": {
    what: "The attached-flow nominal physics plus a ridge-fitted one-step residual correction on the dynamic states.",
    in: "state + pilot commands",
    out: "nominal RK4 step + learned residual → trajectory",
  },
  "SINDy": {
    what: "Sparse regression over a quadratic feature library to discover a parsimonious symbolic ODE (the equations shown below), via sequentially-thresholded least squares (STLSQ).",
    in: "heading/position-invariant features (body velocity, gravity direction, body rates) + pilot sticks",
    out: "state derivative ẋ, integrated open-loop to a trajectory",
  },
  "EquationError-LS": {
    what: "Affine regression of the state derivatives on the invariant features (equation error), then integrated open-loop.",
    in: "invariant features + pilot sticks",
    out: "state derivative ẋ → trajectory",
  },
  "Koopman-EDMD": {
    what: "Koopman / extended DMD: lift the state into quadratic features and fit a single linear one-step operator, rolled out back in the original coordinates.",
    in: "lifted (quadratic) state features + pilot sticks",
    out: "next-state increment → trajectory",
  },
  "Symbolic-Stepwise": {
    what: "Sparse quadratic one-step predictor — the discrete-time cousin of SINDy.",
    in: "invariant features + pilot sticks",
    out: "next-state increment → trajectory",
  },
  "Subspace-Hankel": {
    what: "Lagged ARX / Hankel linear predictor using a short window of past state samples.",
    in: "invariant state history (a few lags) + pilot sticks",
    out: "next-state increment → trajectory",
  },
  "GP-RBF": {
    what: "The attached-flow nominal physics plus a radial-basis (Gaussian-process-style) kernel residual on the dynamic states.",
    in: "invariant state + pilot sticks",
    out: "nominal step + kernel residual → trajectory",
  },
};

function methodInfo(title) {
  const i = METHOD_INFO[title];
  if (!i) return "";
  return `<div class="method-info">
      <p class="model-note"><strong>What it is.</strong> ${i.what}</p>
      <p class="model-note"><strong>Inputs:</strong> ${i.in}. &nbsp;<strong>Output:</strong> ${i.out}.</p>
    </div>`;
}

function modelicaSection(mo) {
  if (!mo) return "";
  if (mo.generated_source) {
    return `<details class="modelica-source">
      <summary>Modelica Model</summary>
      <p class="model-note"><code>${mo.generated_name}.mo</code> &middot; ${moDownloadLink(mo.generated_name + ".mo", mo.generated_source)}</p>
      <pre class="modelica-code">${escapeHtml(mo.generated_source)}</pre>
    </details>`;
  }
  return `<details class="modelica-source">
      <summary>Modelica Model</summary>
      <p class="model-note"><strong>Identified</strong> &mdash; <code>${mo.identified_name}.mo</code> &middot; ${moDownloadLink(mo.identified_name + ".mo", mo.identified_source)}</p>
      <pre class="modelica-code">${escapeHtml(mo.identified_source)}</pre>
      <p class="model-note"><strong>Baseline</strong> &mdash; <code>${mo.baseline_name}.mo</code> &middot; ${moDownloadLink(mo.baseline_name + ".mo", mo.baseline_source)}</p>
      <pre class="modelica-code">${escapeHtml(mo.baseline_source)}</pre>
    </details>`;
}

function renderModelInspector() {
  const grid = document.querySelector("#models-grid");
  if (!grid || !ex.data) return;
  const m = ex.data.models;
  const dt = ex.data.flights[0]?.dt_full || 1 / 240;
  const catalog = [];
  if (m.greybox) {
    catalog.push(["GreyBoxOEM", () => {
      const gb = m.greybox;
      const groups = [
        ["setup / ground", 0, 6],
        ["lift / drag / pitch", 6, 15],
        ["side force", 15, 21],
        ["roll", 21, 27],
        ["yaw", 27, 32],
        ["stall / surfaces", 32, gb.parameter_names.length],
      ];
      const tables = groups.map(([label, a, b]) => {
        const rows = gb.parameter_names.slice(a, b).map((n, i) => {
          const v = gb.parameters[a + i];
          const sd = gb.cr_std ? gb.cr_std[a + i] : null;
          const rel = sd != null && Math.abs(v) > 1e-9 ? (100 * sd) / Math.abs(v) : null;
          const flag = rel != null && rel > 100 ? " \u26a0 unidentifiable" : rel != null && rel > 25 ? " \u26a0 weak" : "";
          const pm = sd != null ? ` \u00b1 ${sd.toPrecision(2)}` : "";
          return `<tr><td>${n}${flag}</td><td>${v.toPrecision(4)}${pm}</td></tr>`;
        }).join("");
        return `<div><p class="model-note">${label}</p><table class="model-table"><tbody>${rows}</tbody></table></div>`;
      }).join("");
      const couplings = (gb.couplings || []).map((c) => `${c.a}\u2194${c.b} (r=${c.r.toFixed(2)})`).join(", ");
      const fixed = Object.entries(gb.fixed_parameters).map(([k, v]) => `${k}=${v}`).join(", ");
      return `<p class="model-note">Rumoca fixed-wing plant coefficients identified by segment-wise output error on the manual training chunks; the same parameters integrate in the browser and compile as Modelica.</p>
        <div class="model-columns">${tables}</div>
        <p class="model-note">fixed: ${fixed}</p>
        ${couplings ? `<p class="model-note">strongly coupled pairs (|r|&gt;0.9): ${couplings}</p>` : ""}
        ${gb.uncertainty_note ? `<p class="model-note">${gb.uncertainty_note}</p>` : ""}
        ${modelicaSection(gb.modelica)}`;
    }]);
  }
  if (m.safe_invariant_weights) {
    catalog.push(["SAFE closed loop (shared)", () => {
      const sc = m.safe_scores || {};
      const featNames = ["u", "v", "w", "\u03c6", "\u03b8", "p", "q", "r", ...STICK_NAMES, "1"];
      const outNames = ["u\u0307", "v\u0307", "w\u0307", "\u03c6\u0307", "\u03b8\u0307", "p\u0307", "q\u0307", "r\u0307", "\u03c8\u0307"];
      const transform = (v, i, j) => ((j < 8 && i === j) ? v - 1 : v) / dt;
      return `<p class="model-note">stabilized segments always use this single heading/position-invariant linear ridge fit (not a neural network, not per-method); the method picker only changes the manual-segment airframe. Direct closed-loop identification beats composing the identified controller with each airframe (2.9 m vs 13.6 m at 5 s).</p>`
        + weightEquations(m.safe_invariant_weights, featNames, outNames, { cutoff: 0.05, transform })
        + `<p class="model-note">${sc.train_samples ?? "?"} train samples \u00b7 held-out 5 s position error ${sc.validation_pos_err_5s_m ?? "?"} m over ${sc.validation_windows ?? "?"} windows</p>`
        + modelicaSection(m.safe_closed_loop_modelica);
    }]);
  }
  if (m.safe_gains) {
    catalog.push(["SAFE controller (grey-box)", () => {
      const g = m.safe_gains;
      const c = m.safe_controller || {};
      const lag = c.surface_lag_s || {};
      const sc = c.scores || {};
      const corr = Object.entries(c.airframe_corrections || {});
      const row = (axis) => `<tr><td>${axis}</td>${(g[axis] || []).map((v) => `<td>${v.toPrecision(3)}</td>`).join("")}${lag[axis] != null ? `<td>${lag[axis]}</td>` : ""}</tr>`;
      const corrRows = corr.map(([n, v]) => `<tr><td>${n}</td><td>${v.manual}</td><td>${v.refined}</td><td>${v.sigma > 0 ? "+" : ""}${v.sigma}\u03c3</td></tr>`).join("");
      const implied = c.implied_law ? `<p class="model-note">structure check: feedback through the grey-box's rate-row effectiveness attributes ${(100 * c.implied_law.sensed_feedback_explained).toFixed(0)}% of the closed-loop change to sensed attitude/rate states; the remainder is stabilized-regime airframe mismatch (the SAFE unit has no airspeed sensor).</p>` : "";
      return `<p class="model-note">attitude-command law identified jointly with Cram\u00e9r\u2013Rao-regularized airframe corrections by closed-loop simulation error through the <b>fitted</b> grey-box: \u03b4 = K\u209a\u00b7(sat(scale\u00b7stick, \u00b1envelope) \u2212 attitude) \u2212 K\u1d48\u00b7rate + bias, then a first-order surface lag. Rudder row is (stick gain, yaw-rate gain, bias).</p>
        <table class="model-table"><tbody>
        <tr><td></td><td>K\u209a</td><td>scale</td><td>envelope (rad)</td><td>K\u1d48</td><td>bias</td><td>lag (s)</td></tr>
        ${row("elevator")}${row("aileron")}${row("rudder")}
        </tbody></table>
        <p class="model-note">composed grey-box+controller held-out 5 s position error ${sc.composed_pos_err_5s_m ?? "?"} m (staged init ${sc.staged_init_pos_err_5s_m ?? "?"} m) over ${sc.validation_windows ?? "?"} windows \u2014 the direct closed-loop fit above remains the stabilized-segment prediction model.</p>
        ${implied}
        ${corrRows ? `<p class="model-note">airframe parameters pulled &gt; 0.5\u03c3 from the manual fit by the stabilized regime:</p><table class="model-table"><tbody><tr><td></td><td>manual</td><td>refined</td><td>shift</td></tr>${corrRows}</tbody></table>` : ""}
        ${modelicaSection(c.modelica)}`;
    }]);
  }
  if (m.ground) {
    catalog.push(["Ground roll & ground effect", () => {
      const p = m.ground.parameters;
      const sc = m.ground.scores || {};
      const ge = m.ground_effect || {};
      return `<p class="model-note">planar rolling model fitted by simulation error on the tracked ground windows: dV/dt = kT\u00b7(T\u2098\u2090\u2093/m)\u00b7thr<sup>1.45</sup> \u2212 \u03bc\u00b7g \u2212 c\u1d65\u00b7V\u00b2, d\u03c8/dt = (k\u209b\u00b7rudder + k\u2080)\u00b7V. Free runs anchored on the ground use it until the recorded liftoff, then hand the rolled state to the selected airframe method.</p>
        <table class="model-table"><tbody>
        <tr><td>thrust scale kT</td><td>${p.kT}</td></tr>
        <tr><td>rolling resistance \u03bc</td><td>${p.mu}</td></tr>
        <tr><td>quadratic drag c\u1d65</td><td>${p.cv}</td></tr>
        <tr><td>steering gain k\u209b (rad/m per cmd)</td><td>${p.ks}</td></tr>
        <tr><td>steering trim k\u2080 (rad/m)</td><td>${p.k0}</td></tr>
        </tbody></table>
        <p class="model-note">held-out 5 s position error ${sc.ground_pos_err_5s_m ?? "?"} m over ${sc.validation_windows ?? "?"} windows (hold-position baseline ${sc.hold_position_baseline_m ?? "?"} m, constant-velocity ${sc.constant_velocity_baseline_m ?? "?"} m).</p>
        ${ge.dCL != null ? `<p class="model-note">ground-effect band (${ge.band_seconds} s of rotation/flare transition): equation-error force-coefficient increments \u0394C\u2097 = ${ge.dCL} \u00b1 ${ge.dCL_sem}, \u0394C\u1d05 = ${ge.dCD} \u00b1 ${ge.dCD_sem} relative to the airborne reference. ${ge.note}</p>` : ""}
        ${modelicaSection(m.ground.modelica)}`;
    }]);
  }
  if (m.linear_weights) catalog.push(["LinearSS", () => affineDetail(m.linear_weights, dt, false) + modelicaSection(m.linear_modelica)]);
  if (m.residual_weights) catalog.push(["RidgeResidual", () => affineDetail(m.residual_weights, dt, true)]);
  for (const [name, spec] of Object.entries(m.surrogates || {})) {
    catalog.push([name.replace("6DOF-", ""), () => surrogateDetail(name, spec)]);
  }

  grid.innerHTML = "";
  const picker = document.createElement("select");
  picker.id = "model-inspect-select";
  for (const [title] of catalog) {
    const option = document.createElement("option");
    option.value = title;
    option.textContent = title;
    picker.append(option);
  }
  const detail = document.createElement("div");
  detail.className = "model-detail";
  const show = (title) => {
    const entry = catalog.find(([t]) => t === title) || catalog[0];
    ex.modelInspectorChoice = entry[0];
    detail.innerHTML = `<h3>${entry[0]}</h3>${methodInfo(entry[0])}${entry[1]()}`;
  };
  picker.addEventListener("change", () => show(picker.value));
  if (ex.modelInspectorChoice && catalog.some(([t]) => t === ex.modelInspectorChoice)) picker.value = ex.modelInspectorChoice;
  grid.append(picker, detail);
  show(picker.value);
}

function renderSplitsView() {
  // "Data Splits" tab: every flight as a timeline colored by segmentation
  // class, with the manual maneuver windows marked by their train/validation
  // membership, so the provenance of every fitted model is visible.
  const chart = document.querySelector("#splits-chart");
  const notes = document.querySelector("#splits-notes");
  if (!chart || !ex.data) return;
  chart.innerHTML = "";
  const maxDuration = Math.max(...ex.data.flights.map((f) => f.time[f.time.length - 1] || 1));
  const colorAt = (f, k) => (f.tracked && !f.tracked[k] ? "#4a5159" : LABEL_COLORS[ex.data.labels[f.labels[k]]] || "#666");
  for (const f of ex.data.flights) {
    const row = document.createElement("div");
    row.className = "splits-row";
    const name = document.createElement("div");
    name.className = "splits-name";
    name.textContent = f.name + (f.autonomous ? " (autonomous)" : "");
    row.append(name);
    const lane = document.createElement("div");
    lane.className = "splits-lane";
    const duration = f.time[f.time.length - 1] || 1;
    lane.style.width = `${(100 * duration) / maxDuration}%`;
    // Segmentation gradient, like the playback time bar.
    const stops = [];
    let runStart = 0;
    for (let k = 1; k <= f.labels.length; k += 1) {
      if (k === f.labels.length || colorAt(f, k) !== colorAt(f, runStart)) {
        const a = ((f.time[runStart] / duration) * 100).toFixed(2);
        const b = ((f.time[Math.min(k, f.time.length - 1)] / duration) * 100).toFixed(2);
        stops.push(`${colorAt(f, runStart)} ${a}% ${b}%`);
        runStart = k;
      }
    }
    const bar = document.createElement("div");
    bar.className = "splits-bar";
    bar.style.background = `linear-gradient(to right, ${stops.join(", ")})`;
    lane.append(bar);
    // Train/validation membership strips: manual maneuver windows (bare
    // airframe methods) above the bar, stabilized windows (closed-loop SAFE
    // model) below it.
    const addStrip = (start, stop, split, title, below) => {
      const strip = document.createElement("div");
      strip.className = `splits-window splits-${split}${below ? " splits-below" : ""}`;
      strip.style.left = `${(100 * start) / duration}%`;
      strip.style.width = `${(100 * (stop - start)) / duration}%`;
      strip.title = title;
      lane.append(strip);
    };
    for (const segment of f.segments) {
      if (!segment.split) continue;
      addStrip(segment.start_s, segment.stop_s, segment.split,
        `${segment.kind} ${segment.start_s}-${segment.stop_s} s -> ${segment.split} (airframe methods)`, false);
    }
    for (const window of f.stabilized_splits || []) {
      addStrip(window.start_s, window.stop_s, window.split,
        `stabilized ${window.start_s}-${window.stop_s} s -> ${window.split} (closed-loop SAFE model)`, true);
    }
    for (const window of f.ground_splits || []) {
      addStrip(window.start_s, window.stop_s, window.split,
        `ground ${window.start_s}-${window.stop_s} s -> ${window.split} (planar rolling model)`, true);
    }
    row.append(lane);
    chart.append(row);
  }
  const legend = document.createElement("div");
  legend.className = "splits-legend";
  legend.innerHTML = [
    ...Object.entries(LABEL_COLORS).map(([label, color]) => `<span><i style="background:${color}"></i>${label.replace("_", " ")}</span>`),
    '<span><i style="background:#4a5159"></i>mocap dropout</span>',
    '<span><i class="splits-train-key"></i>train window (above: airframe methods, below: SAFE / ground models)</span>',
    '<span><i class="splits-validation-key"></i>validation window</span>',
  ].join("");
  chart.append(legend);
  if (notes) {
    notes.innerHTML = `
      <p>Manual maneuver windows (orange) are detected from the transmitter mode channel. Each window is split
      into a quasi-steady <em>lead-in</em> — pooled per flight to estimate the stick trim bias — and the
      <em>control actuation</em> portion, which is cut into 0.6&ndash;1.2&nbsp;s gap-free chunks whose initial states are
      estimated at each chunk start. Windows are assigned round-robin within each flight (every third manual
      window becomes validation), so both splits span multiple flights and battery states; models are fitted on
      the train chunks only and scored on held-out validation chunks.</p>
      <p>Stabilized segments (blue) never train the bare-airframe methods: the SAFE inner loop adds hidden surface
      corrections. They train the separate <em>closed-loop SAFE model</em> instead, with the same discipline as the
      manual windows: the tracked stabilized spans are cut into ~10&nbsp;s windows (strips below the bar), every third
      window per flight is held out, the model fits on the train windows only, and the held-out windows score it by
      5&nbsp;s free-run position error${safeScoreNote()}. The autonomous flight is excluded: its lateral commands
      bypass the recorded sticks.</p>
      <p>Ground (brown) spans train the <em>planar rolling model</em> with the same discipline (strips below the
      bar, every third window held out${groundScoreNote()}); free runs anchored on the ground roll with it until the
      recorded liftoff and then hand off to the selected airframe method. Ground effect (teal) is kept out of all
      airframe fits — its few seconds of rotation/flare transition only support the equation-error lift/drag
      increments reported in the Model Inspector — and mocap dropouts (gray) are never trained on or scored.</p>`;
  }
}

function svgEl(tag, attrs) {
  const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  return el;
}

function renderAll() {
  const status = document.querySelector("#explorer-status");
  if (ex.anchorTimeS == null) {
    status.textContent = "Scrub the colored timeline and press Predict here to set the prediction initial condition.";
  } else {
    const usedKeys = Object.keys(ex.predictions).length ? Object.keys(ex.predictions) : Array.from(ex.selectedMethods);
    const used = usedKeys.map((m) => m.replace("6DOF-", "").replace("Modelica:", "")).join(", ");
    const fallback = ex.selectedMethods.size ? "" : " (using GreyBoxOEM)";
    const note = ex.anchorNote ? ` [${ex.anchorNote}]` : "";
    const dead = flight().autonomous
      ? " Warning: this autonomous flight was flown by an offboard autopilot whose lateral commands are not in the recorded sticks, so free runs cannot anticipate its turns."
      : "";
    status.textContent = `Free run from t = ${ex.anchorTimeS.toFixed(2)} s with ${used}${fallback}${note}; SAFE controller closes the loop through stabilized segments.${dead}`;
  }
}

const METHOD_COLORS_HEX = { "6DOF-NominalGreyBox": 0xd62728, "6DOF-LinearSS": 0x2ca02c, "6DOF-RidgeResidual": 0x9467bd, "6DOF-GreyBoxOEM": 0xe8a838, "Modelica:RumocaFixedWing": 0x7dd3fc, "6DOF-EquationError-LS": 0x17becf, "6DOF-SINDy": 0xe377c2, "6DOF-Koopman-EDMD": 0xbcbd22, "6DOF-Symbolic-Stepwise": 0x8c564b, "6DOF-Subspace-Hankel": 0x1f77b4, "6DOF-GP-RBF": 0xf7b6d2 };

function publishOverlay(timeS) {
  // Publish the segmentation-colored full-flight track and the free-run
  // predictions so the 3D playback can draw them. Selecting a flight (before
  // any click) publishes the colored track alone.
  const f = flight();
  const overlay = {
    stamp: `${f.name}@${timeS == null ? "none" : timeS.toFixed(2)}#${Array.from(ex.selectedMethods).join("|")}`,
    track: f.pos,
    origin: f.pos[0],
    dtFull: f.dt_full,
    anchored: timeS != null && ex.anchorTimeS != null,
    labels: f.labels,
    tracked: f.tracked || f.labels.map(() => 1),
    predictions: Object.entries(ex.predictions).map(([method, pred]) => ({
      method,
      color: METHOD_COLORS_HEX[method] ?? 0x444444,
      points: pred.posEnu,
      times: pred.times,
      quats: pred.quatEnu,
    })),
  };
  if (!ex.playbackTrack) ex.playbackTrack = buildPlaybackTrack();
  window.dispatchEvent(
    new CustomEvent("explorer-set-ic", {
      detail: {
        flight: f.name,
        scenario: ex.data.dataset,
        flightIndex: ex.flightIndex,
        timeS: timeS == null ? 0 : timeS,
        overlay,
        // Browser-runnable methods + colors, so the playback can offer a
        // color-coded picker matching the prediction lines.
        methods: ex.data.methods.filter((method) => method === "6DOF-GreyBoxOEM"),
        methodColors: METHOD_COLORS,
        models: ex.data.models,
        predictionFlight: {
          time: f.time,
          dtFull: f.dt_full,
          state: f.state,
          stick: f.stick_full,
          mode: f.mode_full,
        },
        // Carry the full-flight track with the event so registration can
        // never be lost to module load order.
        track: ex.playbackTrack,
      },
    }),
  );
}

function buildPlaybackTrack() {
  // Full flights as first-class playback tracks: the 3D animation flies the
  // whole record (positions re-zeroed per flight; the overlay carries the
  // matching origin), instead of only the comparison chunk windows.
  const five22 = ex.data.dataset === "sportcub_mocap_5_22_26";
  return {
    id: `explorer_${ex.data.dataset}`,
    model_family: "aircraft6dof",
    source: five22 ? "mocap full record" : "mocap maneuver windows",
    title: five22 ? "Sport Cub full flights (2026-05-22)" : "Sport Cub maneuver windows (2026-04-17)",
    segments: ex.data.flights.map((f) => ({
      name: f.name,
      time_s: f.time,
      position_enu_m: f.pos.map((p) => [p[0] - f.pos[0][0], p[1] - f.pos[0][1], p[2] - f.pos[0][2]]),
      quaternion_wxyz: f.quat,
      labels: f.labels,
      tracked: f.tracked,
      mode: f.mode,
      control_meas: f.stick_full
        .filter((_, index) => index % Math.max(1, Math.round(0.1 / f.dt_full)) === 0)
        .map((u) => [u[0], u[2], u[1], u[3]]),
    })),
  };
}

function firstFlyableTime(f, timeS) {
  // Mocap dropouts have no state, so anchors require tracked samples. Ground
  // anchors are allowed when the planar rolling model is available;
  // otherwise free-runs anchor at the first airborne sample.
  const dtFull = f.dt_full;
  let k = Math.max(0, Math.round(timeS / dtFull));
  const tracked = f.tracked_full || null;
  const groundOk = Boolean(ex.data?.models?.ground);
  const okAt = (index) => (groundOk || f.labels_full[index] !== 0) && (!tracked || tracked[index]);
  // Require a short run of clean samples so the anchor never sits on a
  // tracking-reacquisition edge where smoothed attitude is contaminated.
  const margin = Math.round(0.3 / dtFull);
  while (k < f.labels_full.length) {
    let run = 0;
    while (k + run < f.labels_full.length && okAt(k + run) && run < margin) run += 1;
    if (run >= margin) return (k + margin) * dtFull;
    k += run + 1;
  }
  return null;
}

function setAnchor(timeS) {
  const snapped = firstFlyableTime(flight(), timeS);
  if (snapped == null) {
    ex.anchorTimeS = null;
    ex.predictions = {};
    ex.anchorNote = "no airborne data after the requested time";
    renderAll();
    publishOverlay(null);
    return;
  }
  ex.anchorNote = snapped - timeS > 0.1 ? "anchor moved past a dropout to the next tracked sample" : "";
  ex.anchorTimeS = snapped;
  recomputePredictions();
  renderAll();
  publishOverlay(snapped);
}

function bind() {
  const wrap = document.querySelector("#explorer-flight-wrap");
  if (wrap) wrap.hidden = false;
  const select = document.querySelector("#explorer-flight");
  select.innerHTML = ex.data.flights.map((f, i) => `<option value="${i}">${f.name}</option>`).join("");
  select.value = String(ex.flightIndex);
  select.addEventListener("change", (event) => {
    ex.flightIndex = parseInt(event.target.value, 10);
    ex.anchorTimeS = null;
    ex.predictions = {};
    renderAll();
    publishOverlay(null);
  });
}

export async function initExplorer() {
  const host = document.querySelector("#explorer-flight");
  if (!host) return;
  try {
    ex.data = await loadExplorerDataset(DEFAULT_DATASET);
  } catch (error) {
    const status = document.querySelector("#explorer-status");
    if (status) status.textContent = `Flight data unavailable (${error.message}).`;
    return;
  }
  // Default to the flight with the cleanest full trajectory.
  const preferred = ex.data.flights.findIndex((f) => f.name.startsWith("elev3211_2026_05"));
  if (preferred >= 0) ex.flightIndex = preferred;
  bind();
  renderAll();
  renderSplitsView();
  renderModelInspector();
  // Handshake with the playback module: announce the full-flight view and
  // retry until acknowledged, so no module load order or transient error can
  // leave the 3D viewer on the comparison-window view.
  let playbackLinked = false;
  window.addEventListener("playback-ack", () => {
    playbackLinked = true;
    console.debug("explorer: 3D playback linked");
  });
  const announce = () => {
    window.dispatchEvent(new CustomEvent("explorer-flights-ready", { detail: { track: buildPlaybackTrack() } }));
    publishOverlay(ex.anchorTimeS);
  };
  const tryAnnounce = (remaining) => {
    if (playbackLinked || remaining <= 0) {
      if (!playbackLinked) {
        document.querySelector("#explorer-status").textContent =
          "3D playback link failed; check the browser console for errors.";
      }
      return;
    }
    announce();
    setTimeout(() => tryAnnounce(remaining - 1), 500);
  };
  tryAnnounce(40);
  window.addEventListener("playback-ready", () => tryAnnounce(40));
  // The leaderboard owns method selection; free-run the selected methods the
  // browser has model parameters for.
  window.addEventListener("playback-context-changed", async (event) => {
    const wasActive = ex.active;
    const scenario = event.detail.scenario;
    ex.active = Boolean(DATA_URLS[scenario]);
    const wrap = document.querySelector("#explorer-flight-wrap");
    if (wrap) wrap.hidden = !ex.active;
    if (!ex.active) return;
    const switching = ex.data?.dataset !== scenario;
    if (switching) {
      try {
        ex.data = await loadExplorerDataset(scenario);
      } catch (error) {
        console.warn("explorer dataset unavailable", scenario, error);
        ex.active = false;
        return;
      }
      ex.flightIndex = 0;
      ex.anchorTimeS = null;
      ex.predictions = {};
      ex.playbackTrack = buildPlaybackTrack();
      bind();
      renderAll();
      renderSplitsView();
      renderModelInspector();
    }
    if (switching || !wasActive) publishOverlay(ex.anchorTimeS);
  });
  window.addEventListener("explorer-anchor-request", (event) => {
    if (!ex.active) return;
    setAnchor(event.detail.timeS);
  });
  window.addEventListener("methods-changed", (event) => {
    const available = new Set(ex.data.methods.filter((method) => method === "6DOF-GreyBoxOEM"));
    ex.selectedMethods = new Set((event.detail.methods || []).filter((m) => available.has(m) || String(m).startsWith("Modelica:")));
    if (!ex.active) return;
    recomputePredictions();
    renderAll();
    if (ex.anchorTimeS != null) publishOverlay(ex.anchorTimeS);
  });
  window.addEventListener("resize", renderAll);
}

initExplorer();
