// FixedWingPlant model generated from FixedWingPlant.mo, then hand-retuned for
// the measured micro-airframe: 0.3 N max thrust, V_md ~= 4 m/s, half-throttle
// level cruise ~= 4.25 m/s, and full-stick 5 m/s turn radius ~= 4 m.
// External baseline sources pinned to @cognipilot/rumoca v0.9.8 and CMM v0.0.2.

// CMM release package: https://github.com/CogniPilot/modelica_models/releases/download/v0.0.2/CMM_v0.0.2.zip

package LieGroup
  package SO3
    function quaternionNormError
      input Real q[4] "Quaternion w,x,y,z";
      output Real err "Quaternion norm error";

    algorithm
      err := q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4] - 1;
    end quaternionNormError;

    function quaternionDerivative
      input Real q[4] "Quaternion w,x,y,z";
      input Real omega[3] "Body angular velocity [rad/s]";
      input Real qnorm_gain = 1.0 "Quaternion renormalization gain";
      output Real q_dot[4] "Quaternion derivative";
    protected
      Real err "Quaternion norm error";

    algorithm
      err := q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4] - 1;
      q_dot[1] := 0.5 * (-q[2] * omega[1] - q[3] * omega[2] - q[4] * omega[3]) - qnorm_gain * err * q[1];
      q_dot[2] := 0.5 * (q[1] * omega[1] - q[4] * omega[2] + q[3] * omega[3]) - qnorm_gain * err * q[2];
      q_dot[3] := 0.5 * (q[4] * omega[1] + q[1] * omega[2] - q[2] * omega[3]) - qnorm_gain * err * q[3];
      q_dot[4] := 0.5 * (-q[3] * omega[1] + q[2] * omega[2] + q[1] * omega[3]) - qnorm_gain * err * q[4];
    end quaternionDerivative;

    function rotationMatrix
      input Real q[4] "Quaternion w,x,y,z";
      output Real R[3, 3] "Direction cosine matrix, body to world";

    algorithm
      R[1, 1] := 1 - 2 * (q[3] * q[3] + q[4] * q[4]);
      R[1, 2] := 2 * (q[2] * q[3] - q[1] * q[4]);
      R[1, 3] := 2 * (q[2] * q[4] + q[1] * q[3]);
      R[2, 1] := 2 * (q[2] * q[3] + q[1] * q[4]);
      R[2, 2] := 1 - 2 * (q[2] * q[2] + q[4] * q[4]);
      R[2, 3] := 2 * (q[3] * q[4] - q[1] * q[2]);
      R[3, 1] := 2 * (q[2] * q[4] - q[1] * q[3]);
      R[3, 2] := 2 * (q[3] * q[4] + q[1] * q[2]);
      R[3, 3] := 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    end rotationMatrix;

    model Quaternion
      parameter Real q_start[4] = {1, 0, 0, 0} "Initial quaternion w,x,y,z";
      parameter Real qnorm_gain = 1.0 "Quaternion renormalization gain";

      input Real omega[3] "Body angular velocity [rad/s]";
      output Real q[4](start = q_start, each fixed = true) "Quaternion w,x,y,z";
      output Real R[3, 3](start = [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
      ]) "Direction cosine matrix, body to world";
      output Real q_norm_err(start = 0) "Quaternion norm error";

    equation
      q_norm_err = quaternionNormError(q);
      der(q) = quaternionDerivative(q, omega, qnorm_gain);
      R = rotationMatrix(q);
    end Quaternion;
  end SO3;
end LieGroup;


package RigidBody
  partial model RigidBody6DOF
    parameter Real mass = 1.0 "Mass [kg]";
    parameter Real g = 9.8 "Gravity [m/s^2]";
    parameter Real ixx = 1.0 "Body inertia matrix xx entry [kg*m^2]";
    parameter Real iyy = 1.0 "Body inertia matrix yy entry [kg*m^2]";
    parameter Real izz = 1.0 "Body inertia matrix zz entry [kg*m^2]";
    parameter Real ixy = 0.0 "Body inertia matrix xy entry [kg*m^2]";
    parameter Real ixz = 0.0 "Body inertia matrix xz entry [kg*m^2]";
    parameter Real iyz = 0.0 "Body inertia matrix yz entry [kg*m^2]";
    parameter Real J[3, 3] = [
      ixx, ixy, ixz;
      ixy, iyy, iyz;
      ixz, iyz, izz
    ] "Body inertia matrix [kg*m^2]";
    parameter Real p_start[3] = {0, 0, 0} "Initial world position";
    parameter Real v_b_start[3] = {0, 0, 0} "Initial body velocity";
    parameter Real q_start[4] = {1, 0, 0, 0} "Initial quaternion w,x,y,z";
    parameter Real omega_start[3] = {0, 0, 0} "Initial body angular velocity";
    parameter Real qnorm_gain = 1.0 "Quaternion renormalization gain";

    Real F_b[3] "Total non-gravity force in body frame [N]";
    Real M_b[3] "Total moment in body frame [N*m]";
    output Real p[3](start = p_start, each fixed = true) "World position [m]";
    output Real v_b[3](start = v_b_start, each fixed = true) "Body velocity [m/s]";
    output Real q[4](start = q_start) "Quaternion w,x,y,z";
    output Real omega[3](start = omega_start, each fixed = true) "Body angular velocity [rad/s]";
    output Real R[3, 3](start = [
      1, 0, 0;
      0, 1, 0;
      0, 0, 1
    ]) "Direction cosine matrix, body to world";
    output Real v_w[3](start = v_b_start) "World velocity [m/s]";
    output Real a_b[3](start = {0, 0, 0}) "Body specific force [m/s^2]";

  protected
    LieGroup.SO3.Quaternion attitude(q_start = q_start, qnorm_gain = qnorm_gain);
    Real gravity_w[3] "Gravity in world frame [m/s^2]";
    Real gravity_b[3] "Gravity in body frame [m/s^2]";
    Real H_b[3] "Angular momentum in body frame [kg*m^2/s]";
    Real M_gyro[3] "Gyroscopic inertia moment in body frame [N*m]";
    Real M_body[3] "Rigid-body angular acceleration moment [N*m]";

  equation
    attitude.omega = omega;
    q = attitude.q;
    R = attitude.R;

    v_w = R * v_b;
    gravity_w = {0, 0, -g};
    gravity_b = transpose(R) * gravity_w;
    a_b = F_b / mass;

    der(p) = v_w;
    der(v_b) = a_b + gravity_b - cross(omega, v_b);

    H_b = J * omega;
    M_gyro = cross(omega, H_b);
    M_body = M_b - M_gyro;
    J * der(omega) = M_body;
  end RigidBody6DOF;
end RigidBody;


// 6-DOF fixed-wing SIL plant + stabilized controller.
//
// Built on the shared CMM RigidBody / LieGroup packages, exactly like the
// quadrotor example, so the two share the validated rigid-body integrator.
//
//   World frame : Z-up
//   Body frame  : FLU (x forward / nose, y left, z up)
//   Quaternion  : {w, x, y, z} scalar-first, body-to-world
//   Gravity     : handled inside RigidBody6DOF (F_b is the NON-gravity force)
//
// Aerodynamics are built in the conventional FRD / stability axes and then
// rotated into the library's FLU body frame (negate the y, z components).
//
//   FixedWingPlant : aero + thrust -> RigidBody6DOF
//   RumocaFixedWingFlight : plant + stabilizing attitude controller (playback harness)

model FixedWingPlant "6-DOF fixed-wing aero plant on RigidBody6DOF"
  // Mass / inertia (HobbyZone Sport Cub S 2: ~63 g, 0.617 m span; published
  // weight/span + grey-box S/cbar/inertia)
  parameter Real vehicle_mass = 0.063 "Vehicle mass [kg]";
  parameter Real vehicle_ixx = 0.00069 "Body inertia xx (roll) [kg*m^2]";
  parameter Real vehicle_iyy = 0.0006 "Body inertia yy (pitch) [kg*m^2]";
  parameter Real vehicle_izz = 0.00125 "Body inertia zz (yaw) [kg*m^2]";

  // Start at rest on the ground (wheels just touching), ready to take off.
  extends RigidBody.RigidBody6DOF(
    mass = vehicle_mass,
    g = 9.81,
    ixx = vehicle_ixx,
    iyy = vehicle_iyy,
    izz = vehicle_izz,
    p_start = {0, 0, 0.149},
    v_b_start = {0, 0, 0},
    qnorm_gain = 1.0
  );

  // Reference geometry
  parameter Real rho = 1.225 "Air density [kg/m^3]";
  parameter Real S = 0.0555 "Wing reference area [m^2]";
  parameter Real span = 0.617 "Wing span [m]";
  parameter Real cbar = 0.09 "Mean aerodynamic chord [m]";
  parameter Real wing_incidence = 0.14999999999999847 "Wing incidence angle [rad] (6 deg, cubs2)";
  parameter Real thr_max = 0.3 "Maximum thrust [N]";

  // Tricycle landing gear — contact points in body FLU, matched to the GLB
  // wheels at the visual scale (1 unit = 1 m). Per-wheel spring-damper normal
  // force, light rolling resistance (forward) and firmer lateral grip so it
  // tracks straight on the ground and absorbs landings.
  parameter Real ground_k = 3000 "Gear stiffness per wheel [N/m]";
  parameter Real ground_c = 150 "Gear normal damping per wheel [N*s/m]";
  parameter Real roll_fric = 0.1 "Rolling resistance [N/(m/s)]";
  parameter Real side_fric = 25 "Lateral grip [N/(m/s)]";
  parameter Real wheel_x[3] = {0.164, -0.173, -0.173} "Wheel fwd offsets nose,L,R [m]";
  parameter Real wheel_y[3] = {0.0, 0.185, -0.185} "Wheel left offsets [m]";
  parameter Real wheel_z[3] = {-0.151, -0.151, -0.151} "Wheel down offsets [m]";

  // Longitudinal coefficients (retuned micro-airframe, cubs2)
  parameter Real CL0 = 0.755 "Lift at zero AoA";
  parameter Real CLa = 4.20 "Lift slope [1/rad]";
  parameter Real CD0 = 0.1380 "Parasitic drag";
  parameter Real k_ind = 0.1050 "Induced drag factor";
  parameter Real CD0_fp = 0.60 "Flat-plate drag (post-stall)";
  parameter Real Cm0 = 0.0128 "Pitch moment at alpha=0";
  parameter Real Cma = -0.212 "Pitch stiffness (static stability) [1/rad]";
  parameter Real Cmq = -13.66 "Pitch damping";
  parameter Real Cmde = 0.24 "Elevator pitch effectiveness";

  // Lateral / directional coefficients (retuned micro-airframe, cubs2)
  parameter Real CYb = -0.50 "Sideslip side force [1/rad]";
  parameter Real CYda = 0.01 "Aileron side force";
  parameter Real CYdr = -0.015 "Rudder side force";
  parameter Real CYp = -0.15 "Roll-rate side force";
  parameter Real CYr = 0.20 "Yaw-rate side force";
  parameter Real CY_fp_coef = 0.50 "Flat-plate side force (post-stall)";
  parameter Real Clb = -0.05 "Dihedral effect (roll due to sideslip)";
  parameter Real Clp = -0.50 "Roll damping";
  parameter Real Clr = 0.15 "Yaw-roll coupling";
  parameter Real Clda = 0.05 "Aileron roll effectiveness";
  parameter Real Cldr = 0.006 "Rudder roll";
  parameter Real Cnb = 0.06 "Weathercock stability";
  parameter Real Cnp = 0.010 "Roll-yaw coupling";
  parameter Real Cnr = -0.25 "Yaw damping";
  parameter Real Cndr = 0.015 "Rudder yaw effectiveness";
  parameter Real Cnda = 0.006 "Aileron adverse yaw";

  // Stall blending (flat-plate beyond alpha_stall, cubs2)
  parameter Real alpha_stall = 0.2500000000000005 "Stall angle [rad] (20 deg)";
  parameter Real blend_width = 0.03251400845244633 "Stall blend width [rad] (5 deg)";

  // Control-surface travel (cubs2)
  parameter Real max_defl_ail = 0.5236 "Aileron travel [rad] (30 deg)";
  parameter Real max_defl_elev = 0.4189 "Elevator travel [rad] (24 deg)";
  parameter Real max_defl_rud = 0.349 "Rudder travel [rad] (20 deg)";

  parameter Real eps = 1e-6;

  // Normalized pilot/controller commands
  input Real ail "Aileron command [-1..1]";
  input Real elev "Elevator command [-1..1]";
  input Real rud "Rudder command [-1..1]";
  input Real thr "Throttle command [0..1]";

  // Viewer / controller outputs
  output Real position[3](start = p_start) "World position [m]";
  output Real velocity[3](start = v_b_start) "World velocity [m/s]";
  output Real quat[4](start = q_start) "Quaternion w,x,y,z";
  output Real gyro[3](start = {0, 0, 0}) "Body angular rate FLU [rad/s]";
  output Real up_body[3](start = {0, 0, 1}) "World up expressed in body FLU";
  output Real airspeed(start = 15) "True airspeed [m/s]";
  output Real alpha_deg(start = 0) "Angle of attack [deg]";
  output Real ail_rad(start = 0) "Aileron deflection [rad]";
  output Real elev_rad(start = 0) "Elevator deflection [rad]";
  output Real rud_rad(start = 0) "Rudder deflection [rad]";
  output Real thr_out(start = 0) "Throttle fraction [0..1]";

protected
  Real U, V_frd, W_frd "Body velocity in FRD axes [m/s]";
  Real Vt, Vxz, alpha_body, alpha, beta, qbar, sigma;
  Real P_frd, Q_frd, R_frd "Body rates in FRD axes [rad/s]";
  Real wx1, wx2, wx3, wy1, wy2, wy3, wz1, wz2, wz3 "Wind-frame axes in body FRD";
  Real refx, refz, rdot, wzt1, wzt2, wzt3, nz "Wind-frame Gram-Schmidt temporaries";
  Real CL_lin, CL_fp, CL, CD_lin, CD_fp, CD, CY_lin, CY_fp, CY;
  Real Cl_aero, Cm_aero, Cn_aero;
  Real FA_frd[3] "Aero force in body FRD [N]";
  Real MA_frd[3] "Aero moment in body FRD [N*m]";
  Real F_aero[3], F_thrust[3], M_aero[3] "In body FLU";
  Real wh_h[3], wh_vbx[3], wh_vby[3], wh_vbz[3], wh_vwz[3], wh_Fn[3], wh_on[3];
  Real wh_F[3, 3], wh_M[3, 3] "Per-wheel force / moment in body FLU";
  Real F_ground[3], M_ground[3] "Total ground wrench in body FLU";

equation
  // --- FLU body velocity / rates -> FRD aero convention ---
  U = v_b[1];
  V_frd = -v_b[2];
  W_frd = -v_b[3];
  Vt = sqrt(U*U + V_frd*V_frd + W_frd*W_frd) + eps;
  Vxz = sqrt(U*U + W_frd*W_frd) + eps;
  alpha_body = atan2(W_frd, U);
  alpha = alpha_body + wing_incidence;
  beta = atan2(V_frd, Vxz);
  qbar = 0.5*rho*Vt*Vt;
  sigma = (1 + tanh((alpha - alpha_stall)/blend_width))/2;
  P_frd = omega[1];
  Q_frd = -omega[2];
  R_frd = -omega[3];

  // --- wind-frame axes from velocity (branch-free Gram-Schmidt, cubs2) ---
  wx1 = U/Vt; wx2 = V_frd/Vt; wx3 = W_frd/Vt;
  // reference = body axis most perpendicular to velocity (avoids singularity).
  // NB: written branchless via sign() rather than `if .. then 0 else 1`.
  // Rumoca v0.9.8 constant-folds an `if <state-expr> then <literal> else
  // <literal>` to its initialization branch (here refx=1, refz=0 frozen),
  // which defeats the singularity guard and corrupts the wind-frame force
  // projection at any AoA/sideslip. sign() of a state expression is not folded.
  // refz=1 (use body-z) when |wx1|>|wx3|, else refx=1 (use body-x).
  refz = (1 + sign(abs(wx1) - abs(wx3)))/2;
  refx = 1 - refz;
  rdot = refx*wx1 + refz*wx3;
  wzt1 = refx - rdot*wx1;
  wzt2 = -rdot*wx2;
  wzt3 = refz - rdot*wx3;
  nz = sqrt(wzt1*wzt1 + wzt2*wzt2 + wzt3*wzt3) + eps;
  wz1 = wzt1/nz; wz2 = wzt2/nz; wz3 = wzt3/nz;
  wy1 = wz2*wx3 - wz3*wx2;
  wy2 = wz3*wx1 - wz1*wx3;
  wy3 = wz1*wx2 - wz2*wx1;

  // --- surface deflections ---
  ail_rad = max_defl_ail*min(1, max(-1, ail));
  elev_rad = max_defl_elev*min(1, max(-1, elev));
  rud_rad = -max_defl_rud*min(1, max(-1, rud));  // cubs2 polarity (positive cmd -> +yaw)
  thr_out = min(1, max(0, thr));

  // --- aerodynamic coefficients (stability axes, smooth stall blend) ---
  CL_lin = CL0 + CLa*alpha;
  CL_fp = 2*sin(alpha)*cos(alpha);
  CL = (1 - sigma)*CL_lin + sigma*CL_fp;
  CD_lin = CD0 + k_ind*CL_lin*CL_lin;
  CD_fp = CD0_fp + 2*sin(alpha)*sin(alpha);
  CD = (1 - sigma)*CD_lin + sigma*CD_fp;
  CY_lin = CYb*beta + CYda*ail_rad + CYdr*rud_rad + CYp*(span/(2*Vt))*P_frd + CYr*(span/(2*Vt))*R_frd;
  CY_fp = CY_fp_coef*sin(beta)*cos(alpha);
  CY = (1 - sigma)*CY_lin + sigma*CY_fp;
  Cl_aero = Clda*ail_rad + Cldr*rud_rad + Clb*beta + Clp*(span/(2*Vt))*P_frd + Clr*(span/(2*Vt))*R_frd;
  Cm_aero = Cm0 + Cma*alpha + Cmde*elev_rad + Cmq*(cbar/(2*Vt))*Q_frd;
  Cn_aero = Cnb*beta + Cndr*rud_rad + Cnda*ail_rad + Cnp*(span/(2*Vt))*P_frd + Cnr*(span/(2*Vt))*R_frd;

  // --- wind axes -> body FRD via R_b_wind (columns wx, wy, wz),
  //     FA_wind = qbar*S*{-CD, CY, -CL} ---
  FA_frd[1] = qbar*S*(wx1*(-CD) + wy1*CY + wz1*(-CL));
  FA_frd[2] = qbar*S*(wx2*(-CD) + wy2*CY + wz2*(-CL));
  FA_frd[3] = qbar*S*(wx3*(-CD) + wy3*CY + wz3*(-CL));
  MA_frd[1] = qbar*S*span*Cl_aero;
  MA_frd[2] = qbar*S*cbar*Cm_aero;
  MA_frd[3] = qbar*S*span*Cn_aero;

  // --- body FRD -> body FLU (negate y, z) ---
  F_aero = {FA_frd[1], -FA_frd[2], -FA_frd[3]};
  M_aero = {MA_frd[1], -MA_frd[2], -MA_frd[3]};
  F_thrust = {thr_max*thr_out, 0, 0};

  // --- tricycle landing-gear contact (spring-damper per wheel) ---
  // Contact height uses world-up (R[3,:]); normal force acts along world-up
  // expressed in body (= R[3,:]); friction is applied in the body tangential
  // plane (low forward, firm lateral). Moments are r x F about the CG.
  for i in 1:3 loop
    wh_h[i] = p[3] + R[3, 1]*wheel_x[i] + R[3, 2]*wheel_y[i] + R[3, 3]*wheel_z[i];
    wh_vbx[i] = v_b[1] + omega[2]*wheel_z[i] - omega[3]*wheel_y[i];
    wh_vby[i] = v_b[2] + omega[3]*wheel_x[i] - omega[1]*wheel_z[i];
    wh_vbz[i] = v_b[3] + omega[1]*wheel_y[i] - omega[2]*wheel_x[i];
    wh_vwz[i] = R[3, 1]*wh_vbx[i] + R[3, 2]*wh_vby[i] + R[3, 3]*wh_vbz[i];
    // Branchless contact gate (1 when wh_h<0, else 0). See the refx/refz note:
    // Rumoca v0.9.8 constant-folds `if <state> then .. else <literal>` to the
    // literal branch, which would silently zero the gear; sign() avoids that.
    wh_on[i] = (1 + sign(-wh_h[i]))/2;
    wh_Fn[i] = wh_on[i]*max(0, ground_k*(-wh_h[i]) - ground_c*wh_vwz[i]);
    wh_F[1, i] = wh_Fn[i]*R[3, 1] - wh_on[i]*roll_fric*wh_vbx[i];
    wh_F[2, i] = wh_Fn[i]*R[3, 2] - wh_on[i]*side_fric*wh_vby[i];
    wh_F[3, i] = wh_Fn[i]*R[3, 3];
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

  // --- hand non-gravity wrench to the rigid-body integrator ---
  F_b = F_aero + F_thrust + F_ground;
  M_b = M_aero + M_ground;

  // --- viewer outputs ---
  gyro = omega;
  up_body = R[3, :];
  airspeed = Vt;
  alpha_deg = alpha*57.29577951308232;
  position = p;
  velocity = v_w;
  quat = q;
end FixedWingPlant;

// === Unified playback harness for the website Predict-here page ===
// One model, both flight modes, selected by the `mode` input (the recorded
// flight_mode channel): 0 = MANUAL, 1 = STABILIZED. The five inputs map 1:1 to
// the five recorded data channels. Per-axis surface source is an arithmetic
// blend (NOT an `if` -- Rumoca folds `if` on inputs/states to their start value):
//   surface = mode*stabilized + (1 - mode)*manual
// MANUAL: recorded channels are raw surfaces, sign-flipped to the model
//   convention (the dataset is wired opposite on all three axes).
// STABILIZED: recorded channels are attitude sticks; the LEARNED stabilizing
//   controller (stage-2 GA gains below) turns them into surfaces. Throttle is
//   direct in both modes.
model RumocaFixedWingFlight
  input Real throttle(start = 0.0);
  input Real elevator(start = 0.0);
  input Real aileron(start = 0.0);
  input Real rudder(start = 0.0);
  input Real mode(start = 0.0) "0 = manual (raw surfaces), 1 = stabilized (attitude sticks)";

  // stabilizing attitude-controller gains (stage-2 GA) -- substituted by bake.
  // Cascaded attitude hold about a cruise-trim reference (theta_trim, e_off), with
  // a two-part turn rudder: r_ff is an OPEN-LOOP feedforward from the roll stick
  // (crisp turn entry / tightens the turn), r_coord is CLOSED-LOOP coordination on
  // the ACTUAL bank angle phi (keeps any held bank coordinated).
  parameter Real e_Kp = 4.0;
  parameter Real e_scale = 0.32;
  parameter Real e_env = 0.32;
  parameter Real e_Kd = 0.25;
  parameter Real e_off = 0.0;
  parameter Real theta_trim = -0.09 "Level-cruise pitch attitude [rad] (~-5 deg in up_body frame)";
  parameter Real a_Kp = 2.5;
  parameter Real a_scale = 0.567;
  parameter Real a_env = 0.567;
  parameter Real a_Kd = 0.25;
  parameter Real a_off = 0.0;
  parameter Real r_Kp = 0.5;
  parameter Real r_Kd = 0.10;
  parameter Real r_off = 0.0;
  parameter Real r_coord = -0.14 "Bank-angle (phi) -> rudder coordination gain (coordinated turn)";
  parameter Real r_ff = -0.10 "Roll-stick -> rudder open-loop feedforward (turn entry / tightening)";

  // Initial state in the dataset's NED/FRD convention -- the playback page bakes
  // the measured state at the chosen "predict here" start into these and
  // recompiles. Mapped to the plant's NWU/FLU start values (negate y, z).
  parameter Real p0_n = 0.0;
  parameter Real p0_e = 0.0;
  parameter Real p0_d = -20.0;
  parameter Real u0 = 4.5;
  parameter Real v0 = 0.0;
  parameter Real w0 = 0.0;
  parameter Real q0_w = 1.0;
  parameter Real q0_x = 0.0;
  parameter Real q0_y = 0.0;
  parameter Real q0_z = 0.0;
  parameter Real rate0_p = 0.0;
  parameter Real rate0_q = 0.0;
  parameter Real rate0_r = 0.0;

  FixedWingPlant vehicle(
    p_start = {p0_n, -p0_e, -p0_d},
    v_b_start = {u0, -v0, -w0},
    q_start = {q0_w, q0_x, -q0_y, -q0_z},
    omega_start = {rate0_p, -rate0_q, -rate0_r}
  );

  Real phi;
  Real theta;
  Real theta_cmd;
  Real phi_cmd;
  Real stabilized_elev;
  Real stabilized_ail;
  Real stabilized_rud;
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
  // stabilizing attitude controller (cruise-trim attitude hold + turn rudder).
  // theta_cmd is biased by theta_trim so zero pitch stick holds level cruise.
  // stabilized_rud combines an open-loop roll-stick feedforward (r_ff*aileron, crisp
  // entry / tightening) with closed-loop coordination on the ACTUAL bank angle
  // (r_coord*phi, holds any bank coordinated), plus the rudder stick and yaw damping.
  phi = atan2(vehicle.up_body[2], vehicle.up_body[3]);
  theta = atan2(vehicle.up_body[1], vehicle.up_body[3]);
  theta_cmd = theta_trim + min(e_env, max(-e_env, e_scale*elevator));
  phi_cmd = min(a_env, max(-a_env, a_scale*aileron));
  stabilized_elev = e_Kp*(theta_cmd - theta) - e_Kd*vehicle.gyro[2] + e_off;
  stabilized_ail = a_Kp*(phi_cmd - phi) - a_Kd*vehicle.gyro[1] + a_off;
  stabilized_rud = r_Kp*rudder + r_ff*aileron + r_coord*phi - r_Kd*vehicle.gyro[3] + r_off;

  // mode blend: stabilizing controller (mode=1) or manual sign-flipped surfaces (mode=0)
  vehicle.elev = mode*stabilized_elev + (1 - mode)*(-elevator);
  vehicle.ail = mode*stabilized_ail + (1 - mode)*(-aileron);
  vehicle.rud = mode*stabilized_rud + (1 - mode)*(-rudder);
  vehicle.thr = throttle;

  // NWU/FLU -> NED/FRD for the playback harness
  p_n = vehicle.position[1];
  p_e = -vehicle.position[2];
  p_d = -vehicle.position[3];
  u = vehicle.v_b[1];
  v = -vehicle.v_b[2];
  w = -vehicle.v_b[3];
  q_w = vehicle.quat[1];
  q_x = vehicle.quat[2];
  q_y = -vehicle.quat[3];
  q_z = -vehicle.quat[4];
  rate_p = vehicle.gyro[1];
  rate_q = -vehicle.gyro[2];
  rate_r = -vehicle.gyro[3];
end RumocaFixedWingFlight;
