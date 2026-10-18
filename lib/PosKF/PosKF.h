// #pragma once

// // ─────────────────────────────────────────────────────────────────────────────
// //  PosKF  —  discrete-time linear Kalman filter for position hold
// //
// //  State vector   x = [ pos_z,  vel_z,  b_az ]ᵀ
// //    pos_z   — NED depth below home position  (m, positive = descending)
// //    vel_z   — vertical velocity              (m/s, positive = descending)
// //    b_az    — vertical accel bias            (m/s²) absorbs IMU DC offset, tilt leakage,
// //                                              and vibration-induced DC shift
// //
// //  Inputs
// //    predict()   world-frame accel (g, NED) — caller rotates body→world   500 Hz
// //    update()    barometric depth below home (m, positive = descending)   ~30 Hz
// //
// //  Coordinate conventions
// //    World frame : NED for all outputs.
// //                  x = North, y = East, z = Down (positive = descending).
// //
// //  Call order in the main loop
// //    500 Hz accel block  → accel_world = amekf.rotateBodyToWorld(accel_data)
// //                          PosKF.predict(accel_world, dtAccel)
// //
// //  Prerequisite hardware configuration
// //    MPU6500 DLPF_CFG = 4 (20 Hz cutoff) must be set in the IMU driver.
// //    Attenuates propeller vibration before it enters the prediction step.
// // ─────────────────────────────────────────────────────────────────────────────

// class PosKF
// {
// public:
//     // ── Tuning parameters ─────────────────────────────────────────────────
//     struct Config
//     {
//         // Vertical accelerometer noise std-dev (m/s²).
//         // Drives the top-left 2×2 block of Q.
//         // Raise if the velocity estimate jitters badly under prop vibration.
//         float sigmaA = 0.05f;

//         // Bias random-walk rate (m/s² · s⁻¹ᐟ²).
//         // Keep small — a converged bias estimate should change slowly.
//         // Increase (to 1e-3) if you observe persistent, growing velocity drift.
//         float sigmaB = 1.0e-3f;

//         // Barometer measurement noise variance (m²).
//         // BMP180 OSS-2 floor ≈ 0.16 m² (0.4 m RMS) per datasheet.
//         // With props spinning and an open baro port, expect 0.5 – 1.5 m².
//         // Tune empirically with the vehicle on the bench, motors running.
//         float R = 0.15f;

//         // Mahalanobis gate threshold (chi-_squared, DOF = 1).
//         // 9.0 ≈ 3 σ — rejects ~1 % of nominal samples.
//         // Tighten to 6.25 (2.5 σ) for aggressive propwash rejection.
//         // Loosen during initial convergence testing if updates are sparse.
//         float mahaGate = 9.0f;

//         // Hard clamp on |b_az| (m/s²).
//         // MPU6500 in-run bias + 10° tilt leakage ≈ 0.15 – 0.30 m/s².
//         // 1.0 m/s² is generous — prevents pathological wind-up without
//         // ever activating under normal conditions.
//         float biasClamp = 1.0f;

//     };

//     // ── Lifecycle ─────────────────────────────────────────────────────────

//     PosKF();

//     // Call once in setup(), after homeBaroAlt has been written by Welford
//     // calibration.  initialDepth is the initial NED depth in metres —
//     // normally 0.0 (the filter starts at home/launch position).
//     void begin(float initialDepth = 0.0f);

//     void setConfig(const Config &cfg);
//     Config getConfig() const;

//     // Reinitialise state and covariance.  Safe to call mid-flight for
//     // a hard reset if the filter diverges, but prefer re-tuning R / sigmaA.
//     void reset(float initialDepth = 0.0f);

//     // ── Prediction step — call at 500 Hz alongside MEKF accel update ──────
//     //
//     // accelZ  gravity-removed vertical acceleration, world NED frame (m/s²),
//     //         positive = descending. Caller computes this externally, e.g.
//     //         via getLinearAccelWorldNED(...).z
//     // dt      seconds since previous predict() call.
//     void predict(float accelZ, float dt);

//     // ── Correction step — call when a fresh baro sample is dequeued (~30 Hz)
//     //
//     //  depth   barometric depth below home (m, positive = below home = descending).
//     //          Compute as:  homeBaroAlt - baro_position_unfilt
//     //
//     //  Apply the isfinite + BARO_SANE_WINDOW sanity gate in the caller
//     //  before invoking this function.  This function handles only the
//     //  internal Mahalanobis consistency gate.
//     //
//     //  Returns true if the measurement was accepted and the state updated.
//     bool update(float depth);

//     // ── Primary state outputs ─────────────────────────────────────────────

//     float getDepth() const;    // pos_z — NED depth (m, positive = descending)
//     float getVelocity() const; // vel_z — NED vertical velocity (m/s, positive = descending)
//     float getBias() const;     // b_az — estimated vertical accel bias (m/s²)

//     // ── 3-D world-frame telemetry (NED convention) ────────────────────────
//     // Both vectors are refreshed every predict() call.

//     // ── Diagnostics ───────────────────────────────────────────────────────

//     float getLastInnovation() const; // innov = depth_meas − z_predicted
//     float getLastInnovCov() const;   // S = P[0][0] + R
//     bool wasLastUpdateAccepted() const;

//     float getCovZ() const;    // P[0][0] — depth variance      (m²)
//     float getCovVz() const;   // P[1][1] — velocity variance   (m²/s²)
//     float getCovBias() const; // P[2][2] — bias variance       (m²/s⁴)

//     // True once P[0][0] < 0.05 m² and P[2][2] < 0.001 m²/s⁴.
//     // Gate position-hold engagement on this.  Expect 2 – 5 s to converge.
//     bool isConverged() const;

// private:
//     Config config;

//     float x[3];    // state: [z (m, NED depth), vz (m/s, NED), b_az (m/s²)]
//     float P[3][3]; // 3×3 error covariance — always symmetric, positive definite

//     float lastInnov;
//     float lastS;
//     bool lastUpdateAccepted;

//     void symmetrizeP();
// };


#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  PosKF  —  5-state augmented discrete-time Kalman filter
//  Drop-in replacement for the 3-state version.
//
//  STATE VECTOR  x = [ pos_z,  vel_z,  b_az,  b_baro1,  b_baro2 ]ᵀ
//
//    pos_z    — NED depth below home (m, positive = descending)
//    vel_z    — vertical velocity    (m/s, positive = descending)
//    b_az     — vertical accel bias  (m/s²)  IMU DC + tilt leakage
//    b_baro1  — slow baro GM bias    (m)      thermal / pressure drift
//                                             τ ≈ 354 s,  σ ≈ 0.35 m
//    b_baro2  — fast baro GM bias    (m)      propwash / fast noise
//                                             τ ≈ 0.61 s, σ ≈ 0.26 m
//
//  AUGMENTED MEASUREMENT MODEL
//    z_baro[k] = pos_z[k]  +  b_baro1[k]  +  b_baro2[k]  +  v[k]
//    H_baro    = [1,  0,  0,  1,  1]
//    v[k]      ~ N(0, R)    R ≈ 0.05 m²  (white noise floor, static bench)
//                            → inflate to 0.15–0.50 m² in-flight with props
//
//  AUGMENTED DYNAMICS (F, 5×5)
//    [1   dt  −½dt²   0    0  ]  pos_z   (kinematic — driven by IMU)
//    [0    1   −dt    0    0  ]  vel_z
//    [0    0    1     0    0  ]  b_az    (random walk)
//    [0    0    0    φ₁    0  ]  b_baro1 (GM: φ₁ = 1 − dt/τ₁)
//    [0    0    0     0   φ₂ ]  b_baro2 (GM: φ₂ = exp(−dt/τ₂))
//
//  PROCESS NOISE (Q, additive diagonal)
//    [¼dt⁴·σa²   ½dt³·σa²   0        0          0      ]
//    [½dt³·σa²    dt²·σa²   0        0          0      ]
//    [0           0         σb²·dt   0          0      ]
//    [0           0         0       Q_b1        0      ]
//    [0           0         0        0         Q_b2    ]
//
//    Q_bi = 2·σi²·dt/τi   (continuous-time limit, valid when dt ≪ τi)
//
//  PARAMETER VALUES (from 36-min static baro log, 25 Hz, N=2 GM model)
//    R       = 5.006e-2 m²   (white noise floor, static)
//    tau_b1  = 353.677 s     sigma_b1 = 0.35137 m
//    tau_b2  =   0.608 s     sigma_b2 = 0.25745 m
//
//  FUNDAMENTAL LIMITATION (baro-only)
//    Without GPS/LiDAR the filter cannot distinguish true altitude change
//    from baro drift slower than 1/(2π·τ₁) ≈ 0.45 mHz.  Expect ≈ ±0.35 m
//    altitude hold error from thermal drift in steady hover.
//
//  CALL ORDER  (same as original)
//    500 Hz:  PosKF.predict(accel_world_z, dt)
//    ~30 Hz:  PosKF.update(homeBaroAlt − baro_unfilt)
// ─────────────────────────────────────────────────────────────────────────────

class PosKF
{
public:
    // ── Tuning parameters ─────────────────────────────────────────────────
    struct Config
    {
        // ── Kinematic noise ───────────────────────────────────────────────
        // sigmaA: vertical accelerometer noise std-dev (m/s²).
        //         Drives the top-left 2×2 block of Q.
        //         Raise if velocity jitters badly under propeller vibration.
        float sigmaA = 0.05f;

        // sigmaB: accel bias random-walk rate (m/s²·s⁻¹ᐟ²).
        //         Keep small — a converged bias changes slowly.
        //         Increase to 1e-3 if you see persistent velocity drift.
        float sigmaB = 1.0e-3f;

        // ── Baro measurement white noise ──────────────────────────────────
        // R: baro white noise variance (m²).
        //    Static bench (no props):       ≈ 0.05  m²  (from log analysis)
        //    Bench, props spinning:         ≈ 0.15  m²
        //    In-flight, open baro port:     ≈ 0.50  m²
        //    Start at 0.05 and inflate until innovation sequence looks white.
        float R = 5.006e-2f;

        // ── Slow baro GM bias  (b_baro1) ─────────────────────────────────
        // Thermal / pressure drift component.
        // tau_b1 and sigma_b1 directly from ACF model-fit to your log.
        // Increase sigma_b1 only if you observe persistent uncorrected
        // altitude drift over many minutes that b_baro1 is not tracking.
        float tau_b1   = 353.677f; // time constant (s)
        float sigma_b1 = 0.35137f; // steady-state σ (m)

        // ── Fast baro GM bias  (b_baro2) ─────────────────────────────────
        // Propwash / fast pressure noise component.
        // With props spinning aggressively, sigma_b2 may need to increase
        // to 0.35–0.50 m to prevent propwash transients from being
        // misinterpreted as fast altitude changes.
        float tau_b2   = 0.608f;   // time constant (s)
        float sigma_b2 = 0.25745f; // steady-state σ (m)

        // ── Gates and clamps ──────────────────────────────────────────────
        // mahaGate: Mahalanobis gate (χ², DOF=1).  9.0 ≈ 3σ.
        float mahaGate = 9.0f;

        // biasClamp: hard |b_az| limit (m/s²). Prevents accel bias wind-up.
        float biasClamp = 1.0f;

        // baroBiasClamp: hard limit on each baro bias state (m).
        // Physical baro offset rarely exceeds 2–3 m. Clamp prevents
        // runaway if a sustained propwash sequence drives a bias state
        // to a physically unrealistic value.
        float baroBiasClamp = 3.0f;
    };

    // ── Lifecycle ─────────────────────────────────────────────────────────

    PosKF();

    // Call once in setup() after homeBaroAlt has been written by Welford
    // calibration.  initialDepth is normally 0.0 (starts at launch position).
    void begin(float initialDepth = 0.0f);

    void   setConfig(const Config &cfg);
    Config getConfig() const;

    // Hard reset — safe to call mid-flight for divergence recovery.
    // Prefer re-tuning R / sigmaA before resorting to this.
    void reset(float initialDepth = 0.0f);

    // ── 500 Hz predict ────────────────────────────────────────────────────
    //
    // accelZ  gravity-removed vertical accel, world NED frame (m/s²),
    //         positive = descending.
    // dt      seconds since previous predict() call.
    void predict(float accelZ, float dt);

    // ── ~30 Hz baro correction ────────────────────────────────────────────
    //
    // depth   barometric depth below home (m, positive = descending).
    //         Compute as:  homeBaroAlt − baro_position_unfilt
    //
    // Applies isfinite guard and internal Mahalanobis gate.
    // Returns true if measurement was accepted and state was updated.
    bool update(float depth);

    // ── Primary state outputs ─────────────────────────────────────────────

    float getDepth()         const;  // pos_z   NED depth (m)
    float getVelocity()      const;  // vel_z   NED vertical velocity (m/s)
    float getBias()          const;  // b_az    vertical accel bias (m/s²)
    float getBaroBias1()     const;  // b_baro1 slow GM baro bias (m)
    float getBaroBias2()     const;  // b_baro2 fast GM baro bias (m)
    float getTotalBaroBias() const;  // b_baro1 + b_baro2 — net baro offset (m)

    // ── Diagnostics ───────────────────────────────────────────────────────

    float getLastInnovation()      const; // ν = depth_meas − (z + b1 + b2)
    float getLastInnovCov()        const; // S = H·P·Hᵀ + R
    bool  wasLastUpdateAccepted()  const;

    float getCovZ()    const;   // P[0][0] — depth variance      (m²)
    float getCovVz()   const;   // P[1][1] — velocity variance   (m²/s²)
    float getCovBias() const;   // P[2][2] — accel bias variance (m²/s⁴)

    // True once P[0][0] < 0.05 m² and P[2][2] < 0.001 m²/s⁴.
    // Gate position-hold engagement on this.
    bool isConverged() const;

private:
    static constexpr int N = 5;

    Config config;
    float  x[N];       // [pos_z, vel_z, b_az, b_baro1, b_baro2]
    float  P[N][N];    // 5×5 error covariance — symmetric, positive definite

    float lastInnov;
    float lastS;
    bool  lastUpdateAccepted;

    void symmetrizeP();
    static inline float _sq(float v) { return v * v; }
};