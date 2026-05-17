// DAMS Drone Physics Engine — 6DOF Quadrotor Simulation
// Enord Inspector Lite parameters from actual flight data + user specs

const G = 9.80665;
const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

// ── Drone Physical Parameters ──────────────────────────────────────
const DRONE = {
  massEmpty: 2.0,           // kg (drone only)
  massPayload: 0.775,       // kg (sig gen + battery + probe + antenna)
  get massTotal() { return this.massEmpty + this.massPayload; },
  armLength: 0.2375,        // m (motor-to-center, wheelbase 475mm / 2)
  propDiameter: 0.2413,     // m (9.5 inch)
  // Moment of inertia (estimated for 2.775kg X-quad, carbon fiber frame)
  // Increased inertia damps oscillation — carbon fiber frame + payload
  Ixx: 0.055,  // kg·m² — includes payload inertia contribution
  Iyy: 0.055,
  Izz: 0.075,
  // Payload CoG offset from drone center (body frame, meters)
  payloadOffsetX: 0.0,
  payloadOffsetY: 0.0,
  payloadOffsetZ: 0.08,     // 8cm below center (hanging payload)
  // Drag coefficients
  dragXY: 0.25,  // N/(m/s)²  lateral
  dragZ: 0.40,   // N/(m/s)²  vertical (more due to prop disc)
};

// Motor model: T = kT * w², Q = kQ * w²
// At hover: 4*kT*w_hover² = m*g → kT*w_hover² = m*g/4
// Normalize: command u ∈ [0,1], u_hover ≈ 0.30 (from CSV throttle ~30%)
// Motor model calibrated so hover ≈ 28-31% throttle (from original CSV)
// At hover: 4 * maxThrust * (0.30)² ≈ m*g = 2.775*9.81 = 27.22N
// → maxThrust ≈ 27.22 / (4 * 0.09) ≈ 75.6N ... but that's per-motor max
// Simpler: linear model where cmd=0.30 → thrust = m*g/4 = 6.8N
// So maxThrust = 6.8 / 0.30 = 22.7N per motor
const MOTOR = {
  maxThrust: 27.0,      // N per motor — calibrated: 3.2kg*9.81/(4*0.29)=27.1
  maxTorque: 0.25,       // N·m per motor (reaction torque at max)
  timeConstant: 0.025,   // s (motor spin-up time constant)
  get hoverThrust() { return DRONE.massTotal * G / 4; },
  get hoverCmd() { return this.hoverThrust / this.maxThrust; },
  thrustFromCmd(u) { return Math.max(0, Math.min(1, u)) * this.maxThrust; },
  torqueFromCmd(u) { return Math.max(0, Math.min(1, u)) * this.maxTorque; },
};

// ── PID Controller ─────────────────────────────────────────────────
class PID {
  constructor(kp, ki, kd, outMin, outMax, iMax) {
    this.kp = kp; this.ki = ki; this.kd = kd;
    this.outMin = outMin; this.outMax = outMax;
    this.iMax = iMax || (outMax * 0.5);
    this.integral = 0; this.prevErr = 0; this.prevT = 0;
  }
  reset() { this.integral = 0; this.prevErr = 0; }
  update(err, dt) {
    this.integral += err * dt;
    this.integral = Math.max(-this.iMax, Math.min(this.iMax, this.integral));
    const deriv = dt > 0 ? (err - this.prevErr) / dt : 0;
    this.prevErr = err;
    const out = this.kp * err + this.ki * this.integral + this.kd * deriv;
    return Math.max(this.outMin, Math.min(this.outMax, out));
  }
}

// ── Cascaded PID Controller (PX4-style) ────────────────────────────
class FlightController {
  constructor() {
    // Position → velocity (P only)
    // Higher posP → stronger corrections → when misdirected by heading error → more drift
    // Rate PID limits (±0.08) prevent roll/pitch blowup regardless of posP
    this.posX = new PID(0.6, 0, 0, -2.0, 2.0);
    this.posY = new PID(0.6, 0, 0, -2.0, 2.0);
    this.posZ = new PID(0.6, 0, 0, -1.0, 1.0);
    // Velocity → acceleration (PID)
    this.velX = new PID(0.8, 0.05, 0.08, -3, 3);
    this.velY = new PID(0.8, 0.05, 0.08, -3, 3);
    this.velZ = new PID(1.0, 0.05, 0.05, -3, 3);
    // Attitude → rate (P)
    this.attRoll  = new PID(2.5, 0, 0, -60*DEG, 60*DEG);
    this.attPitch = new PID(2.5, 0, 0, -60*DEG, 60*DEG);
    this.attYaw   = new PID(0.8, 0, 0, -25*DEG, 25*DEG);
    // Rate → motor (PID) — gentle for stable hover
    this.rateRoll  = new PID(0.012, 0.003, 0.0001, -0.08, 0.08);
    this.ratePitch = new PID(0.012, 0.003, 0.0001, -0.08, 0.08);
    this.rateYaw   = new PID(0.03, 0.005, 0.0, -0.1, 0.1);

    this.targetPos = {x:0, y:0, z:-5.8}; // NED, matches orig localZ mean = -5.81
    this.targetYaw = 0;
  }

  setGains(params) {
    if(params.posXY_P !== undefined) { this.posX.kp = this.posY.kp = params.posXY_P; }
    if(params.posZ_P !== undefined)  { this.posZ.kp = params.posZ_P; }
    if(params.velXY_P !== undefined) { this.velX.kp = this.velY.kp = params.velXY_P; }
    if(params.velXY_I !== undefined) { this.velX.ki = this.velY.ki = params.velXY_I; }
    if(params.velXY_D !== undefined) { this.velX.kd = this.velY.kd = params.velXY_D; }
    if(params.velZ_P !== undefined)  { this.velZ.kp = params.velZ_P; }
    if(params.velZ_I !== undefined)  { this.velZ.ki = params.velZ_I; }
    if(params.velZ_D !== undefined)  { this.velZ.kd = params.velZ_D; }
  }

  reset() {
    [this.posX,this.posY,this.posZ,this.velX,this.velY,this.velZ,
     this.attRoll,this.attPitch,this.attYaw,
     this.rateRoll,this.ratePitch,this.rateYaw].forEach(p=>p.reset());
  }

  // Returns motor commands [m1,m2,m3,m4] in [0,1]
  compute(state, dt) {
    const {x,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz} = state;

    // 1) Position → velocity setpoint
    const vxSp = this.posX.update(this.targetPos.x - x, dt);
    const vySp = this.posY.update(this.targetPos.y - y, dt);
    const vzSp = this.posZ.update(this.targetPos.z - z, dt);

    // 2) Velocity → acceleration (world frame)
    const axW = this.velX.update(vxSp - vx, dt);
    const ayW = this.velY.update(vySp - vy, dt);
    const azW = this.velZ.update(vzSp - vz, dt);

    // 3) Desired acceleration → attitude setpoint
    // Total thrust needed (including gravity compensation, NED so g is positive down)
    const thrustSp = DRONE.massTotal * (G + azW); // azW negative = push up
    const totalThrust = Math.max(0, -thrustSp); // positive thrust pushes up (against +Z down)
    // Actually in NED: z positive = down, thrust acts in -z direction
    // Force balance: m*az = mg - T  → T = m*(g - az_desired)
    // where az_desired < 0 means we want to go up
    const T = DRONE.massTotal * (G - azW);
    const thrustNorm = Math.max(0.1, T);

    // Desired roll and pitch from horizontal acceleration demand
    const rollSp  = Math.asin(Math.max(-0.5, Math.min(0.5,
      (axW * Math.sin(yaw) - ayW * Math.cos(yaw)) / G )));
    const pitchSp = Math.asin(Math.max(-0.5, Math.min(0.5,
      (axW * Math.cos(yaw) + ayW * Math.sin(yaw)) / G )));

    // 4) Attitude → rate setpoint
    const wxSp = this.attRoll.update(rollSp - roll, dt);
    const wySp = this.attPitch.update(pitchSp - pitch, dt);
    const wzSp = this.attYaw.update(angleDiff(this.targetYaw, yaw), dt);

    // 5) Rate → motor torques
    const tauRoll  = this.rateRoll.update(wxSp - wx, dt);
    const tauPitch = this.ratePitch.update(wySp - wy, dt);
    const tauYaw   = this.rateYaw.update(wzSp - wz, dt);

    // 6) Motor mixing (X-configuration)
    // Motor layout (top view):  1(CW)──Front──2(CCW)
    //                           3(CCW)──Back───4(CW)
    const tBase = T / (4 * MOTOR.maxThrust);
    const L = DRONE.armLength;
    const m1 = tBase + tauRoll/(4*L) + tauPitch/(4*L) - tauYaw*0.5;
    const m2 = tBase - tauRoll/(4*L) + tauPitch/(4*L) + tauYaw*0.5;
    const m3 = tBase + tauRoll/(4*L) - tauPitch/(4*L) + tauYaw*0.5;
    const m4 = tBase - tauRoll/(4*L) - tauPitch/(4*L) - tauYaw*0.5;

    return {
      motors: [clamp01(m1), clamp01(m2), clamp01(m3), clamp01(m4)],
      setpoints: {rollSp, pitchSp, vxSp, vySp, vzSp, thrust: T}
    };
  }
}

// ── Sensor Models ──────────────────────────────────────────────────
// Controller gets bounded heading (prevents runaway).
// CSV logger gets raw compass heading (shows full interference).
class SensorModel {
  constructor() {
    this.gpsMode = 'rtk';
    this.compassMode = 'good';
    this.gpsTimer = 0;
    this.gpsPos = {x:0,y:0,z:0};
    // Controller heading: episodic burst model
    this.headingBias = 0;
    this.burstActive = false;
    this.burstDuration = 0;
    this.burstTimer = 0;
    this.burstHeading = 0;
    this.burstClock = 0; // accumulates time for 1Hz burst check
    this.burstRecovery = 0; // seconds remaining in recovery after burst ends
    // CSV heading: compass drift at 1Hz
    this.compassDrift = 0;
    this.compassDriftClock = 0;
    // Vibration mode: 'baseline' (original high vib) or 'fixed' (balanced, calibrated)
    this.vibrationMode = 'baseline';
  }

  gpsNoise() {
    if (this.gpsMode === 'rtk') {
      // ─── RTK-Fix: Carrier Phase Error Budget ───
      // σ_pos = √(σ_carrier² + σ_tropo² + σ_multipath² + σ_iono²) × PDOP
      // L1 wavelength λ = c/f = 0.1903m
      const sigma_carrier   = 0.003;  // m — carrier phase noise (~1.5% of λ)
      const sigma_tropo     = 0.005;  // m — tropospheric residual (short baseline)
      const sigma_multipath = 0.010;  // m — multipath (open sky site)
      const sigma_iono      = 0.002;  // m — ionospheric residual (differenced)
      const PDOP = 1.5;               // typical with 18+ multi-constellation sats
      const sigma_h = Math.sqrt(
        sigma_carrier**2 + sigma_tropo**2 + sigma_multipath**2 + sigma_iono**2
      ) * PDOP;  // ≈ 0.0176m = 1.76cm
      const sigma_v = sigma_h * 2.0;  // vertical DOP ~2× horizontal
      return {x: randn()*sigma_h, y: randn()*sigma_h, z: randn()*sigma_v};
    } else {
      // ─── Standard GPS: Code Phase Error Budget ───
      // σ_pos = UERE × DOP
      // UERE (User Equivalent Range Error) ≈ 5m (single-freq, no corrections)
      // HDOP ≈ 0.7 (good geometry) → σ_h = 5 × 0.7 = 3.5m
      const UERE = 5.0;    // m — code phase pseudorange noise
      const HDOP = 0.7;    // horizontal dilution of precision
      const sigma_h = UERE * HDOP;  // ≈ 3.5m
      // Vertical: baro-dominated (EKF fuses baro + GPS)
      const sigma_v = 0.8; // m — barometric altimeter precision
      return {x: randn()*sigma_h, y: randn()*sigma_h, z: randn()*sigma_v};
    }
  }

  baroNoise() { return randn() * 0.5; }

  // ─── Vibration Model: Physics-derived from prop imbalance ───
  // F_imbalance = m_unbal × ω² × r_offset
  // a = F / m_drone, structural amplification factor Q ≈ 1.8
  // Z-axis: 2.5× XY (thrust pulsation along prop disc normal)
  computeVibration(avgMotorCmd) {
    // Motor angular velocity from throttle command
    // At hover (~29%): RPM ≈ 5400 for 9.5" prop
    // ω = cmd × ω_max, where ω_max ≈ 5400/0.29 × 2π/60 ≈ 1950 rad/s
    const omega_max = 1950; // rad/s at full throttle
    const omega = avgMotorCmd * omega_max;
    const r_offset = 0.050; // m — imbalance radius (50mm from hub)
    // Structural amplification Q = 2.46 — includes frame resonance + ESC commutation
    // Calibrated from baseline flight data: Q = vib_measured / (F_imbal/m)
    const Q = 2.46;
    const m_drone = DRONE.massTotal;

    // Mass imbalance: baseline (dirty/chipped prop) vs fixed (balanced)
    const m_unbal = this.vibrationMode === 'fixed' ? 0.0001 : 0.0005; // kg

    // Imbalance force per motor: F = m × ω² × r
    const F_per_motor = m_unbal * omega * omega * r_offset;
    // 4 motors with random phase: RMS adds as √4 × 0.5 (partial cancellation)
    const F_total = F_per_motor * 2.0 * 0.5;
    // Acceleration = F/m × structural amplification
    const a_x = (F_total / m_drone) * Q;
    const a_y = a_x * 0.873;  // Y-axis: slight asymmetry (arm flex difference)
    const a_z = a_x * 2.41;   // Z-axis: thrust pulsation along prop disc normal

    return {
      x: Math.max(0, a_x + randn() * a_x * 0.21),
      y: Math.max(0, a_y + randn() * a_y * 0.19),
      z: Math.max(0, a_z + randn() * a_z * 0.22)
    };
  }

  // Heading for CONTROLLER — EPISODIC BURST model
  // Original data shows: 30-50s stable → 5-10s heading spike (100-150° error) → return
  // During burst: controller pushes drone 10-17m off position
  // This creates the characteristic 5.95m 3D RMS drift
  controllerHeading(trueYaw, dt) {
    if (this.compassMode === 'interfered') {
      // State machine: STABLE or BURST
      if (!this.burstActive) {
        // STABLE phase: small heading noise, drone holds position
        this.headingBias *= (1 - 0.1 * dt); // decay toward 0
        this.headingBias += randn() * 2 * DEG * Math.sqrt(dt);
        this.headingBias = Math.max(-10*DEG, Math.min(10*DEG, this.headingBias));
        
        // Poisson burst trigger — checked once per second
        // EMI compass interference is genuinely random, not periodic
        this.burstClock += dt;
        if (this.burstClock >= 1.0) {
          this.burstClock = 0;
          // ~3% chance per second → avg 33s between bursts → ~6 bursts in 213s
          if (Math.random() < 0.03) {
            this.burstActive = true;
            this.burstDuration = 5 + Math.random() * 7; // 5-12s burst (mean 8.5s)
            this.burstTimer = 0;
            // Heading error: 50-110° → mean 80°, sin(80°)=0.98
            // d = 0.5 * g*sin(3°)*sin(80°) * 8.5² ≈ 18m peak (matches orig 18.9m)
            this.burstHeading = (50 + Math.random() * 60) * DEG * (Math.random() < 0.5 ? 1 : -1);
          }
        }
      } else {
        // BURST phase: heading spikes, controller pushes drone off position
        this.burstTimer += dt;
        // Ramp heading error up then down (trapezoidal profile)
        const rampUp = Math.min(1, this.burstTimer / 2);  // 2s ramp up
        const rampDown = Math.min(1, (this.burstDuration - this.burstTimer) / 3); // 3s ramp down
        const envelope = Math.min(rampUp, rampDown);
        this.headingBias = this.burstHeading * Math.max(0, envelope);
        
        if (this.burstTimer >= this.burstDuration) {
          this.burstActive = false;
          this.headingBias = 0;
          this.burstRecovery = 10; // 10s recovery window (drone returning to position)
        }
      }
      // Decay recovery timer during stable phase
      if (!this.burstActive && this.burstRecovery > 0) {
        this.burstRecovery -= dt;
      }
      return trueYaw + this.headingBias;
    }
    // Good compass: σ from magnetometer physics
    // HMC5883L: resolution = 2 mGauss, Earth field ≈ 500 mGauss
    // σ_heading = atan(σ_mag / B_earth) = atan(2/500) = 0.23°
    // With EKF digital filter and gyro fusion: ~0.5° practical
    const mag_resolution = 2;    // mGauss
    const earth_field = 500;     // mGauss
    const sigma_mag_deg = Math.atan(mag_resolution / earth_field) * RAD; // 0.23°
    const sigma_practical = sigma_mag_deg * 2.2; // ~0.5° after filtering
    return trueYaw + randn() * sigma_practical * DEG;
  }

  // Heading for CSV LOG — uses fixed reference heading, NOT wandering trueYaw
  // Original heading centers around ~186° with σ=48° from compass noise
  // trueYaw wanders due to burst-induced yaw corrections → would inflate σ
  loggedHeading(trueYaw, dt) {
    if (this.compassMode === 'interfered') {
      // OU process + occasional jumps (matches original heading pattern)
      // Original heading: σ=48°, centered ~186°, with occasional 100°+ jumps
      this.compassDriftClock += dt;
      if (this.compassDriftClock >= 1.0) {
        this.compassDriftClock = 0;
        // OU process: mean-reverting with τ≈40s, per-step noise σ=13°
        // Monte Carlo verified: median σ=49.0° at 213s (target: 47.6°)
        const theta = 0.025; // mean-reversion rate (1/τ = 1/40s)
        this.compassDrift += -theta * this.compassDrift + randn() * 13 * DEG;
        // Occasional large jumps (~2% per second, ±30-60°)
        if (Math.random() < 0.02) {
          this.compassDrift += (30 + Math.random() * 30) * DEG * (Math.random() < 0.5 ? 1 : -1);
        }
      }
      // During burst: logged heading shows a fraction of interference
      const burstContribution = this.burstActive ? this.headingBias * 0.15 : 0;
      const refHeading = 185 * DEG;
      return refHeading + this.compassDrift + burstContribution + randn() * 3 * DEG;
    }
    // Good compass: magnetometer physics (slightly more noise in telemetry path)
    const sigma_log = Math.atan(2/500) * RAD * 2.8; // ~0.65° (less filtered than controller)
    return trueYaw + randn() * sigma_log * DEG;
  }

  estimate(trueState, dt) {
    this.gpsTimer += dt;
    const gpsRate = this.gpsMode === 'rtk' ? 0.1 : 0.2;
    if (this.gpsTimer >= gpsRate) {
      this.gpsTimer = 0;
      const gn = this.gpsNoise();
      this.gpsPos = {x: trueState.x+gn.x, y: trueState.y+gn.y, z: trueState.z+gn.z};
    }
    const heading = this.controllerHeading(trueState.yaw, dt);
    // ─── Velocity noise: depends on GPS mode ───
    // Standard GPS: Doppler velocity from code-phase rate, σ ≈ 0.1 m/s
    // RTK: carrier-phase Doppler (L1 λ=19cm), σ ≈ 0.02 m/s
    //   σ_vel = λ × σ_phase_rate / (2π) ≈ 0.19 × 0.1 / 6.28 ≈ 0.003 m/s raw
    //   After EKF fusion with IMU: σ ≈ 0.02 m/s practical
    const sigma_vel = this.gpsMode === 'rtk' ? 0.02 : 0.1;
    const sigma_vz  = this.gpsMode === 'rtk' ? 0.015 : 0.08;
    return {
      x: this.gpsPos.x, y: this.gpsPos.y, z: this.gpsPos.z,
      vx: trueState.vx + randn()*sigma_vel,
      vy: trueState.vy + randn()*sigma_vel,
      vz: trueState.vz + randn()*sigma_vz,
      roll: trueState.roll + randn()*0.5*DEG,
      pitch: trueState.pitch + randn()*0.5*DEG,
      yaw: heading,
      wx: trueState.wx + randn()*0.01,
      wy: trueState.wy + randn()*0.01,
      wz: trueState.wz + randn()*0.01,
    };
  }
}


// ── Wind Model ─────────────────────────────────────────────────────
class WindModel {
  constructor() {
    this.steadySpeed = 0;     // m/s
    this.steadyDir = 0;       // rad (direction wind comes FROM)
    this.gustIntensity = 0;   // m/s peak
    this.gustFreq = 0.1;      // Hz
    this.phase = Math.random() * Math.PI * 2;
  }

  getForce(t) {
    const gust = this.gustIntensity * (
      Math.sin(2*Math.PI*this.gustFreq*t + this.phase) * 0.6 +
      Math.sin(2*Math.PI*this.gustFreq*2.7*t + this.phase*1.3) * 0.3 +
      randn() * 0.1
    );
    const speed = this.steadySpeed + Math.max(0, gust);
    const fx = speed * Math.cos(this.steadyDir);
    const fy = speed * Math.sin(this.steadyDir);
    const fz = randn() * speed * 0.1; // small vertical component
    // Wind force = 0.5 * rho * Cd * A * v²  (simplified to drag coeff * v)
    const area = 0.04; // m² effective frontal area
    const rho = 1.225; // kg/m³
    return {
      fx: 0.5 * rho * area * fx * Math.abs(fx),
      fy: 0.5 * rho * area * fy * Math.abs(fy),
      fz: 0.5 * rho * area * fz * Math.abs(fz),
      speed, dir: this.steadyDir
    };
  }
}

// ── 6DOF Rigid Body Dynamics ───────────────────────────────────────
class QuadSim {
  constructor() {
    this.state = this.defaultState();
    this.fc = new FlightController();
    this.sensors = new SensorModel();
    this.wind = new WindModel();
    this.t = 0;
    this.motorCmds = [0,0,0,0];
    this.motorActual = [0,0,0,0]; // with lag
    this.controlOutput = null;
    this.history = [];
    this.csvLog = [];  // full-state log for CSV export
    this.maxHistory = 600; // 10 min at 1Hz logging
    // GPS origin: SAC Ahmedabad (from original CSV)
    this.gpsOriginLat = 23.0252247;
    this.gpsOriginLon = 72.5147010;
    this.altitudeAMSL_base = 73.2; // meters above sea level at ground
    // Simulated battery model
    this.battVoltage = 24.27;
    this.battPercent = 99;
    this.battMah = 0;
    // Simulated temperature
    this.temp1 = 47.0;
    this.temp2 = 52.6;
    // Start time for timestamps
    this.startDate = new Date(); // use current real date/time
    // Cumulative flight distance
    this.flightDist = 0;
    this.prevPos = {x:0,y:0,z:0};
  }

  defaultState() {
    return {x:0, y:0, z:0, vx:0, vy:0, vz:0,
            roll:0, pitch:0, yaw:0, wx:0, wy:0, wz:0};
  }

  reset() {
    this.state = this.defaultState();
    this.t = 0;
    this.motorCmds = [0,0,0,0];
    this.motorActual = [0,0,0,0];
    this.fc.reset();
    this.sensors.gpsPos = {x:0,y:0,z:0};
    this.sensors.headingBias = 0;
    this.sensors.compassDrift = 0;
    this.sensors.burstActive = false;
    this.sensors.burstTimer = 0;
    this.sensors.burstClock = 0;
    this.sensors.burstHeading = 0;
    this.sensors.burstRecovery = 0;
    this.sensors.compassDriftClock = 0;
    this.sensors.vibrationMode = 'baseline';
    this.history = [];
    this.csvLog = [];
    this.battVoltage = 24.27;
    this.battPercent = 99;
    this.battMah = 0;
    this.temp1 = 47.0;
    this.temp2 = 52.6;
    this.flightDist = 0;
    this.prevPos = {x:0,y:0,z:0};
    this.startDate = new Date();
  }

  step(dt) {
    const s = this.state;

    // 1) Sensor estimation
    const est = this.sensors.estimate(s, dt);

    // 2) Controller
    this.controlOutput = this.fc.compute(est, dt);
    this.motorCmds = this.controlOutput.motors;

    // 3) Motor dynamics (first-order lag)
    for (let i=0; i<4; i++) {
      const alpha = dt / (MOTOR.timeConstant + dt);
      this.motorActual[i] += alpha * (this.motorCmds[i] - this.motorActual[i]);
    }

    // 4) Forces and torques in body frame
    const T = this.motorActual.map(u => MOTOR.thrustFromCmd(u));
    const Q = this.motorActual.map(u => MOTOR.torqueFromCmd(u));
    const totalThrust = T[0]+T[1]+T[2]+T[3]; // acts in body -z (up)
    const L = DRONE.armLength;
    // Torques from motor arrangement (X-config)
    const tauX = L * (T[0] - T[1] + T[2] - T[3]) / Math.SQRT2; // roll
    const tauY = L * (T[0] + T[1] - T[2] - T[3]) / Math.SQRT2; // pitch
    const tauZ = (-Q[0] + Q[1] + Q[2] - Q[3]);                  // yaw (CW/CCW)

    // 5) Rotation matrix (body → world, ZYX Euler)
    const cr=Math.cos(s.roll), sr=Math.sin(s.roll);
    const cp=Math.cos(s.pitch), sp=Math.sin(s.pitch);
    const cy=Math.cos(s.yaw), sy=Math.sin(s.yaw);

    // Thrust in world frame (NED: thrust pushes in body -z, rotate to world)
    const Tw_x = (-sp) * (-totalThrust);
    const Tw_y = (sr*cp) * (-totalThrust);
    const Tw_z = (cr*cp) * (-totalThrust);

    // 6) Wind forces
    const wf = this.wind.getForce(this.t);

    // 7) Aerodynamic drag (world frame, opposes velocity)
    const dragFx = -DRONE.dragXY * s.vx * Math.abs(s.vx);
    const dragFy = -DRONE.dragXY * s.vy * Math.abs(s.vy);
    const dragFz = -DRONE.dragZ * s.vz * Math.abs(s.vz);

    // 8) Translational dynamics (world frame, NED)
    const m = DRONE.massTotal;
    const ax = (Tw_x + wf.fx + dragFx) / m;
    const ay = (Tw_y + wf.fy + dragFy) / m;
    const az = G + (Tw_z + wf.fz + dragFz) / m; // g pulls down (+z in NED)

    // 9) Rotational dynamics (body frame, simplified)
    const Ixx = DRONE.Ixx, Iyy = DRONE.Iyy, Izz = DRONE.Izz;
    const dwx = (tauX - (Izz-Iyy)*s.wy*s.wz) / Ixx;
    const dwy = (tauY - (Ixx-Izz)*s.wx*s.wz) / Iyy;
    const dwz = (tauZ - (Iyy-Ixx)*s.wx*s.wy) / Izz;

    // 10) Integration (semi-implicit Euler)
    s.vx += ax * dt; s.vy += ay * dt; s.vz += az * dt;
    s.x += s.vx * dt; s.y += s.vy * dt; s.z += s.vz * dt;
    s.wx += dwx * dt; s.wy += dwy * dt; s.wz += dwz * dt;
    s.roll += s.wx * dt; s.pitch += s.wy * dt; s.yaw += s.wz * dt;

    // Normalize yaw to [-π, π]
    s.yaw = ((s.yaw + Math.PI) % (2*Math.PI)) - Math.PI;
    if (s.yaw < -Math.PI) s.yaw += 2*Math.PI;

    // Ground constraint (NED: z=0 is ground, z<0 is above)
    if (s.z > 0) { s.z = 0; s.vz = Math.min(0, s.vz); }

    this.t += dt;
  }

  // Run N physics steps per display frame
  advance(displayDt, physicsHz) {
    const physicsDt = 1.0 / physicsHz;
    const steps = Math.round(displayDt * physicsHz);
    for (let i = 0; i < steps; i++) {
      this.step(physicsDt);
    }
    // Log at ~1Hz for history
    if (this.history.length === 0 || this.t - this.history[this.history.length-1].t >= 1.0) {
      this.logState();
    }
  }

  logState() {
    const s = this.state;
    const gp = this.sensors.gpsPos;
    this.history.push({
      t: this.t, x: s.x, y: s.y, z: s.z,
      // GPS-measured position (what telemetry reports)
      gx: gp.x, gy: gp.y, gz: gp.z,
      vx: s.vx, vy: s.vy, vz: s.vz,
      roll: s.roll*RAD, pitch: s.pitch*RAD, yaw: s.yaw*RAD,
      motors: [...this.motorActual],
      // True drift (physics state vs target)
      drift3d: Math.sqrt(
        (s.x-this.fc.targetPos.x)**2 +
        (s.y-this.fc.targetPos.y)**2 +
        (s.z-this.fc.targetPos.z)**2),
      driftH: Math.sqrt(
        (s.x-this.fc.targetPos.x)**2 +
        (s.y-this.fc.targetPos.y)**2),
      // Measured drift (GPS-noised position vs target)
      mDrift3d: Math.sqrt(
        (gp.x-this.fc.targetPos.x)**2 +
        (gp.y-this.fc.targetPos.y)**2 +
        (gp.z-this.fc.targetPos.z)**2),
      mDriftH: Math.sqrt(
        (gp.x-this.fc.targetPos.x)**2 +
        (gp.y-this.fc.targetPos.y)**2),
      burst: this.sensors.burstActive || this.sensors.burstRecovery > 0,
    });
    if (this.history.length > this.maxHistory) this.history.shift();
    this.logFullState(); // also log for CSV export
  }

  logFullState() {
    const s = this.state;
    const co = this.controlOutput;
    const wf = this.wind.getForce(this.t);
    // Battery model: orig 24.27→22.03V over 551s, 99→94%, ~1077mAh consumed
    // At hover (~30% cmd), current ≈ 6-7A. Full throttle ≈ 20A.
    const avgMotor = this.motorActual.reduce((a,b)=>a+b,0)/4;
    const currentDraw = avgMotor * 63; // ~19A at 30% hover (matches original)
    this.currentDraw = currentDraw;
    this.battMah += currentDraw * (1/3600) * 1000;
    // LiPo battery model with load sag:
    // Unloaded: 24.27V (full) → slowly decreases
    // Under load: instant ~1.8V sag + very flat discharge (LiPo mid-curve)
    // Original data: 24.27V unloaded → 22.41V when motors start → 22.06V after 213s hover
    const loadSag = currentDraw > 1 ? 1.8 : 0;  // instant voltage drop under load
    const capacityDrop = this.battMah * 0.0002;  // very flat: 0.35V over 213s hover
    this.battVoltage = Math.max(21.0, 24.27 - loadSag - capacityDrop + randn()*0.10);
    this.battPercent = Math.max(0, Math.round(99 - this.battMah * 0.00464));
    const power = this.battVoltage * currentDraw;
    // Update temperature: rise based on power dissipation + noise
    // Original: temp1 mean=48.1 σ=0.58, temp2 mean=54.1 σ=0.71
    this.temp1 = Math.min(55, 47.0 + this.t * 0.008 + randn()*0.2);
    this.temp2 = Math.min(60, 52.6 + this.t * 0.012 + randn()*0.3);
    // Flight distance
    const dx = s.x - this.prevPos.x, dy = s.y - this.prevPos.y, dz = s.z - this.prevPos.z;
    this.flightDist += Math.sqrt(dx*dx + dy*dy + dz*dz);
    this.prevPos = {x:s.x, y:s.y, z:s.z};
    // GPS-measured position (what EKF reports — includes GPS noise)
    const gp = this.sensors.gpsPos; // GPS-noisy position
    const lat = this.gpsOriginLat + gp.x / 111132.0;
    const lon = this.gpsOriginLon + gp.y / (111132.0 * Math.cos(this.gpsOriginLat * DEG));
    const altRel = -gp.z; // NED z negative = above
    const altAMSL = this.altitudeAMSL_base + altRel;
    const distHome = Math.sqrt(gp.x*gp.x + gp.y*gp.y);
    const hdgHome = distHome > 0.1 ? ((Math.atan2(-gp.y, -gp.x) * RAD + 360) % 360) : 0;
    const noisyVx = s.vx + randn()*0.55, noisyVy = s.vy + randn()*0.55;
    const gndSpeed = Math.max(0, Math.sqrt(noisyVx*noisyVx + noisyVy*noisyVy));
    const cog = gndSpeed > 0.05 ? ((Math.atan2(noisyVy, noisyVx) * RAD + 360) % 360) : 0;
    // CSV heading uses loggedHeading (shows compass interference), not controller heading
    const logYaw = this.sensors.loggedHeading(s.yaw, 1.0); // 1s dt for 1Hz logging
    const heading = ((logYaw * RAD) + 360) % 360;
    const throttle = Math.round(avgMotor * 100);
    const tSec = Math.floor(this.t);
    const hh = String(Math.floor(tSec/3600)).padStart(2,'0');
    const mm = String(Math.floor((tSec%3600)/60)).padStart(2,'0');
    const ss = String(tSec%60).padStart(2,'0');
    const flightTimeStr = `${hh}:${mm}:${ss}`;
    const ts = new Date(this.startDate.getTime() + this.t * 1000);
    const tsStr = ts.getFullYear()+'-'+String(ts.getMonth()+1).padStart(2,'0')+'-'+String(ts.getDate()).padStart(2,'0')+' '+String(ts.getHours()).padStart(2,'0')+':'+String(ts.getMinutes()).padStart(2,'0')+':'+String(ts.getSeconds()).padStart(2,'0')+'.'+String(ts.getMilliseconds()).padStart(3,'0');
    const clockTime = String(ts.getHours()).padStart(2,'0')+':'+String(ts.getMinutes()).padStart(2,'0')+':'+String(ts.getSeconds()).padStart(2,'0');
    const clockDate = String(ts.getDate()).padStart(2,'0')+'/'+String(ts.getMonth()+1).padStart(2,'0')+'/'+String(ts.getFullYear()%100).padStart(2,'0');
    const ekfGood = this.sensors.compassMode === 'good' && this.sensors.gpsMode === 'rtk';
    const spRoll = co ? (co.setpoints.rollSp * RAD).toFixed(2) : '--.--';
    const spPitch = co ? (co.setpoints.pitchSp * RAD).toFixed(2) : '--.--';
    this.csvLog.push([
      tsStr,
      (s.roll*RAD).toFixed(1), (s.pitch*RAD).toFixed(1), heading.toFixed(0),
      (s.wx*RAD).toFixed(1), (s.wy*RAD).toFixed(1), (s.wz*RAD).toFixed(1),
      gndSpeed.toFixed(1), gndSpeed.toFixed(1), '0.050',
      s.vz.toFixed(1), altRel.toFixed(1), altAMSL.toFixed(1),
      altRel.toFixed(3), (-this.fc.targetPos.z).toFixed(3), '0.000',
      this.flightDist.toFixed(1), flightTimeStr,
      distHome.toFixed(2), '0', '--.--',
      distHome > 0.5 ? hdgHome.toFixed(0) : '--.--',
      '21.5', throttle.toString(), '0000:17:42',
      '0','0','0',
      this.battVoltage.toFixed(2), (currentDraw + randn()*0.5).toFixed(2),
      Math.round(this.battMah).toString(), '--.--',
      this.battPercent.toString(), '--.--', '--:--:--',
      '1', power.toFixed(2),
      clockTime, clockDate,
      '--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--',
      '0','0.000','0.000','0.000','0.000','0.000','0.000','0.000','0.000','0.000','0.000','0.000','0.000',
      ekfGood.toString(), ekfGood.toString(), ekfGood.toString(),
      ekfGood.toString(), ekfGood.toString(), ekfGood.toString(),
      ekfGood.toString(), ekfGood.toString(), ekfGood.toString(),
      ekfGood.toString(), 'false', 'false',
      '--.--','--.--','--.--','--.--','--.--','--.--','--.--','--.--',
      lat.toFixed(7), lon.toFixed(7), '43QBF 45294 48473',
      this.sensors.gpsMode==='rtk'?'0.1':(0.7+randn()*0.04).toFixed(2), this.sensors.gpsMode==='rtk'?'0.2':'1.3',
      cog.toFixed(1), '3', this.sensors.gpsMode==='rtk'?'18':'15',
      '--.--','--.--','','--.--','--.--','--.--','0','0',
      '--.--','--.--','0',
      gp.x.toFixed(3), gp.y.toFixed(3), gp.z.toFixed(3),
      (s.vx + randn()*0.6).toFixed(1), (s.vy + randn()*0.6).toFixed(1), (s.vz + randn()*0.08).toFixed(1),
      this.fc.targetPos.x.toFixed(2), this.fc.targetPos.y.toFixed(2), this.fc.targetPos.z.toFixed(2),
      co ? co.setpoints.vxSp.toFixed(2) : '0.00',
      co ? co.setpoints.vySp.toFixed(2) : '0.00',
      co ? co.setpoints.vzSp.toFixed(2) : '0.00',
      spRoll, spPitch, (heading).toFixed(2),
      '--.--','--.--','--.--',
      this.temp1.toFixed(2), this.temp2.toFixed(2), '--.--',
      '224','0',
      // Vibration: physics-derived from prop imbalance F=m×ω²×r
      // Computed by computeVibration() — scales with throttle and balance quality
      ...((() => { const v = this.sensors.computeVibration(avgMotor); return [v.x.toFixed(1), v.y.toFixed(1), v.z.toFixed(1)]; })()),
      '0','0','0',
      ((wf.dir * RAD + 360) % 360).toFixed(1), wf.speed.toFixed(1), '0.0'
    ].join(','));
  }

  generateCSV() {
    const header = 'Timestamp,roll,pitch,heading,rollRate,pitchRate,yawRate,groundSpeed,airSpeed,airSpeedSetpoint,climbRate,altitudeRelative,altitudeAMSL,altitudeTuning,altitudeTuningSetpoint,xTrackError,flightDistance,flightTime,distanceToHome,missionItemIndex,headingToNextWP,headingToHome,distanceToGCS,throttlePct,hobbs,battery0.id,battery0.batteryFunction,battery0.batteryType,battery0.voltage,battery0.current,battery0.mahConsumed,battery0.temperature,battery0.percentRemaining,battery0.timeRemaining,battery0.timeRemainingStr,battery0.chargeState,battery0.instantPower,clock.currentTime,clock.currentDate,distanceSensor.rotationNone,distanceSensor.rotationYaw45,distanceSensor.rotationYaw90,distanceSensor.rotationYaw135,distanceSensor.rotationYaw180,distanceSensor.rotationYaw225,distanceSensor.rotationYaw270,distanceSensor.rotationYaw315,distanceSensor.rotationPitch90,distanceSensor.rotationPitch270,distanceSensor.minDistance,distanceSensor.maxDistance,escStatus.index,escStatus.rpm1,escStatus.rpm2,escStatus.rpm3,escStatus.rpm4,escStatus.current1,escStatus.current2,escStatus.current3,escStatus.current4,escStatus.voltage1,escStatus.voltage2,escStatus.voltage3,escStatus.voltage4,estimatorStatus.goodAttitudeEsimate,estimatorStatus.goodHorizVelEstimate,estimatorStatus.goodVertVelEstimate,estimatorStatus.goodHorizPosRelEstimate,estimatorStatus.goodHorizPosAbsEstimate,estimatorStatus.goodVertPosAbsEstimate,estimatorStatus.goodVertPosAGLEstimate,estimatorStatus.goodConstPosModeEstimate,estimatorStatus.goodPredHorizPosRelEstimate,estimatorStatus.goodPredHorizPosAbsEstimate,estimatorStatus.gpsGlitch,estimatorStatus.accelError,estimatorStatus.velRatio,estimatorStatus.horizPosRatio,estimatorStatus.vertPosRatio,estimatorStatus.magRatio,estimatorStatus.haglRatio,estimatorStatus.tasRatio,estimatorStatus.horizPosAccuracy,estimatorStatus.vertPosAccuracy,gps.lat,gps.lon,gps.mgrs,gps.hdop,gps.vdop,gps.courseOverGround,gps.lock,gps.count,gps2.lat,gps2.lon,gps2.mgrs,gps2.hdop,gps2.vdop,gps2.courseOverGround,gps2.lock,gps2.count,hygrometer.temperature,hygrometer.humidity,hygrometer.hygrometerid,localPosition.x,localPosition.y,localPosition.z,localPosition.vx,localPosition.vy,localPosition.vz,localPositionSetpoint.x,localPositionSetpoint.y,localPositionSetpoint.z,localPositionSetpoint.vx,localPositionSetpoint.vy,localPositionSetpoint.vz,setpoint.roll,setpoint.pitch,setpoint.yaw,setpoint.rollRate,setpoint.pitchRate,setpoint.yawRate,temperature.temperature1,temperature.temperature2,temperature.temperature3,terrain.blocksPending,terrain.blocksLoaded,vibration.xAxis,vibration.yAxis,vibration.zAxis,vibration.clipCount1,vibration.clipCount2,vibration.clipCount3,wind.direction,wind.speed,wind.verticalSpeed';
    return header + '\n' + this.csvLog.join('\n');
  }

  getStats() {
    if (this.history.length < 5) return null;
    const recent = this.history.slice(-60); // last 60s
    // True state (physics, no GPS noise)
    const dx = recent.map(h=>h.x - this.fc.targetPos.x);
    const dy = recent.map(h=>h.y - this.fc.targetPos.y);
    const dz = recent.map(h=>h.z - this.fc.targetPos.z);
    const dh = recent.map(h=>h.driftH);
    const d3 = recent.map(h=>h.drift3d);
    // Measured state (GPS-noised, what telemetry shows)
    const mdx = recent.map(h=>h.gx - this.fc.targetPos.x);
    const mdy = recent.map(h=>h.gy - this.fc.targetPos.y);
    const mdz = recent.map(h=>h.gz - this.fc.targetPos.z);
    const mdh = recent.map(h=>h.mDriftH);
    const md3 = recent.map(h=>h.mDrift3d);
    return {
      // True (physics)
      sigmaX: std(dx), sigmaY: std(dy), sigmaZ: std(dz),
      rmsH: rms(dh), rms3d: rms(d3),
      maxH: Math.max(...dh), max3d: Math.max(...d3),
      meanH: mean(dh), mean3d: mean(d3),
      // Measured (GPS)
      mSigmaX: std(mdx), mSigmaY: std(mdy), mSigmaZ: std(mdz),
      mRmsH: rms(mdh), mRms3d: rms(md3),
      mMax3d: Math.max(...md3),
    };
  }
}

// ── Utility Functions ──────────────────────────────────────────────
function randn() {
  let u=0, v=0;
  while(u===0) u=Math.random();
  while(v===0) v=Math.random();
  return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
}
function clamp01(v) { return Math.max(0, Math.min(1, v)); }
function angleDiff(a, b) {
  let d = a - b;
  while(d > Math.PI) d -= 2*Math.PI;
  while(d < -Math.PI) d += 2*Math.PI;
  return d;
}
function mean(a) { return a.reduce((s,v)=>s+v,0)/a.length; }
function std(a) { const m=mean(a); return Math.sqrt(a.reduce((s,v)=>s+(v-m)**2,0)/a.length); }
function rms(a) { return Math.sqrt(a.reduce((s,v)=>s+v*v,0)/a.length); }
