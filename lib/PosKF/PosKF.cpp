// #include "PosKF.h"
// #include <math.h>
// #include <string.h>

// // ── File-local helpers ───────────────────────────────────────────────────────

// static inline float _sq(float v) { return v * v; }

// // ── Constructor / lifecycle ──────────────────────────────────────────────────

// PosKF::PosKF()
// {
//     memset(x, 0, sizeof(x));
//     memset(P, 0, sizeof(P));

//     lastInnov = 0.0f;
//     lastS = 0.0f;
//     lastUpdateAccepted = false;
// }

// void PosKF::begin(float initialDepth)
// {
//     reset(initialDepth);
// }

// void PosKF::setConfig(const Config &cfg)
// {
//     config = cfg;
// }

// PosKF::Config PosKF::getConfig() const
// {
//     return config;
// }

// void PosKF::reset(float initialDepth)
// {
//     x[0] = initialDepth; // z   — NED depth at start (0 = at home/launch altitude)
//     x[1] = 0.0f;         // vz  — vehicle is stationary
//     x[2] = 0.0f;         // b   — assume zero; filter converges from here

//     memset(P, 0, sizeof(P));
//     P[0][0] = 1.0f;  // 1.0  m   std-dev initial depth uncertainty
//     P[1][1] = 1.0f;  // 1.0  m/s std-dev initial velocity uncertainty
//     P[2][2] = 0.01f; // 0.1  m/s² std-dev initial bias uncertainty

//     lastInnov = 0.0f;
//     lastS = 0.0f;
//     lastUpdateAccepted = false;
// }

// // ── Prediction step ──────────────────────────────────────────────────────────
// //
// //  Process model
// //
// //    x(k+1) = F·x(k) + G·a_ned_z(k)  +  process noise
// //
// //         [1   dt  −½dt²]       [½dt²]
// //    F  = [0    1   −dt ]   G = [ dt ]
// //         [0    0    1  ]       [  0 ]
// //
// //  The bias b enters F, not G — it is part of the state, not the input.
// //  a_ned_z is the gravity-removed vertical acceleration from the IMU
// //  (m/s², NED / down-positive).
// //
// //  Process noise
// //    Q = σ_a²·G·Gᵀ  +  diag(0, 0, σ_b²·dt)
// //
// //       [¼dt⁴  ½dt³  0]           [0  0      0  ]
// //  σ_a²·[½dt³   dt²  0]  + σ_b²dt·[0  0      0  ]
// //       [ 0     0    0]           [0  0      1  ]

// void PosKF::predict(float accelZ, float dt)
// {
//     if (dt <= 0.0f || dt > 0.1f || !isfinite(dt))
//         return;

//     // ── 4. State propagation ──────────────────────────────────────────────
//     // Bias-corrected vertical acceleration — this is the actual KF input:
//     //   a_eff = a_ned_z − b
//     // The sign convention: positive b means the sensor reads too high
//     // (overestimates downward acceleration), so we subtract it.
//     const float dt2 = dt * dt;
//     const float hdt2 = 0.5f * dt2;
//     const float a_eff = accelZ - x[2];

//     x[0] += x[1] * dt + hdt2 * a_eff; // z(k+1)  = z + vz·dt + ½a_eff·dt²
//     x[1] += a_eff * dt;               // vz(k+1) = vz + a_eff·dt
//     // x[2] (bias) is unchanged in the prediction step; it is a random walk
//     // driven only by process noise σ_b, corrected by baro updates.

//     // ── 5. Covariance propagation  P ← F·P·Fᵀ + Q ───────────────────────
//     //
//     // Fully expanded — no heap allocation, no inner loops — because this
//     // runs at 500 Hz on the same core as the attitude filter.
//     //
//     // Step 5a: compute F·P row by row.
//     //   F row 0 = [1,  dt,  −½dt²]
//     //   F row 1 = [0,   1,   −dt ]
//     //   F row 2 = [0,   0,    1  ]

//     const float FP00 = P[0][0] + dt * P[1][0] - hdt2 * P[2][0];
//     const float FP01 = P[0][1] + dt * P[1][1] - hdt2 * P[2][1];
//     const float FP02 = P[0][2] + dt * P[1][2] - hdt2 * P[2][2];

//     const float FP10 = P[1][0] - dt * P[2][0];
//     const float FP11 = P[1][1] - dt * P[2][1];
//     const float FP12 = P[1][2] - dt * P[2][2];

//     // Row 2 of F is [0, 0, 1], so F·P row 2 = P row 2 unchanged.
//     const float FP20 = P[2][0];
//     const float FP21 = P[2][1];
//     const float FP22 = P[2][2];

//     // Step 5b: compute (F·P)·Fᵀ.
//     // Element [i][j] = FP[i] · F[j]  where F[j] is the j-th row of F.
//     //   F[0] = [1, dt, −½dt²]
//     //   F[1] = [0,  1,  −dt ]
//     //   F[2] = [0,  0,   1  ]

//     P[0][0] = FP00 + dt * FP01 - hdt2 * FP02;
//     P[0][1] = FP01 - dt * FP02;
//     P[0][2] = FP02;

//     P[1][0] = FP10 + dt * FP11 - hdt2 * FP12;
//     P[1][1] = FP11 - dt * FP12;
//     P[1][2] = FP12;

//     P[2][0] = FP20 + dt * FP21 - hdt2 * FP22;
//     P[2][1] = FP21 - dt * FP22;
//     P[2][2] = FP22;

//     // Step 5c: add process noise Q.
//     const float sa2 = _sq(config.sigmaA);
//     const float sb2 = _sq(config.sigmaB);
//     const float dt3 = dt2 * dt;
//     const float dt4 = dt3 * dt;

//     // σ_a²·G·Gᵀ  — symmetric, only top-left 2×2 is non-zero.
//     P[0][0] += sa2 * 0.25f * dt4;
//     P[0][1] += sa2 * 0.5f * dt3;
//     // P[0][2] += 0
//     P[1][0] += sa2 * 0.5f * dt3;
//     P[1][1] += sa2 * dt2;
//     // P[1][2], P[2][0], P[2][1] += 0

//     // σ_b²·dt — bias random walk, bottom-right element only.
//     P[2][2] += sb2 * dt;

//     symmetrizeP();
// }

// // ── Correction step ──────────────────────────────────────────────────────────
// //
// //  Measurement model
// //    H = [1, 0, 0]   (barometer observes z/depth directly)
// //
// //  Because H is a row vector, all matrix-vector products reduce to scalars
// //  or column-picks, avoiding any heap allocation.
// //
// //  Covariance update uses the Joseph form
// //    P ← (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ
// //  which guarantees symmetry and positive semi-definiteness even under
// //  finite-precision arithmetic, unlike the simpler (I − K·H)·P form.

// bool PosKF::update(float depth)
// {
//     // Caller should have applied isfinite + BARO_SANE_WINDOW before calling.
//     // Guard here catches any NaN/Inf that slips through.
//     if (!isfinite(depth))
//         return false;

//     // ── Innovation and innovation covariance ─────────────────────────────
//     // depth and x[0] are both NED depth (positive = below home = descending).
//     // With H = [1, 0, 0]:  innov = y − H·x = depth − x[0]
//     //                       S     = H·P·Hᵀ + R = P[0][0] + R
//     const float innov = depth - x[0];
//     const float S = P[0][0] + config.R;

//     lastInnov = innov;
//     lastS = S;

//     if (S <= 0.0f || !isfinite(S))
//         return false;

//     // ── Mahalanobis gate  (χ², DOF = 1) ──────────────────────────────────
//     // Rejects baro outliers from propwash pressure spikes, dynamic pressure
//     // during aggressive manoeuvres, and transient sensor glitches.
//     // The gate naturally widens at startup (large P[0][0]) to accept the
//     // large corrections needed for initial convergence, then tightens
//     // automatically as the filter gains confidence.
//     if (_sq(innov) / S > config.mahaGate)
//     {
//         lastUpdateAccepted = false;
//         return false;
//     }

//     // ── Kalman gain  K = P·Hᵀ / S ────────────────────────────────────────
//     // H = [1, 0, 0] → P·Hᵀ = first column of P.
//     const float K0 = P[0][0] / S;
//     const float K1 = P[1][0] / S;
//     const float K2 = P[2][0] / S;

//     // ── State update  x ← x + K·innov ────────────────────────────────────
//     x[0] += K0 * innov;
//     x[1] += K1 * innov;
//     x[2] += K2 * innov;

//     // Hard clamp on bias — prevents pathological wind-up if a bad baro
//     // sequence drives b to a physically unrealistic value.
//     if (x[2] > config.biasClamp)
//         x[2] = config.biasClamp;
//     else if (x[2] < -config.biasClamp)
//         x[2] = -config.biasClamp;

//     // ── Joseph-form covariance update ─────────────────────────────────────
//     //   P ← A·P·Aᵀ + K·R·Kᵀ    where  A = I − K·H
//     //
//     //           [1−K0   0   0]
//     //   A     = [−K1    1   0]
//     //           [−K2    0   1]

//     const float a0 = 1.0f - K0; // A[0][0]; A[0][1]=A[0][2]=0

//     // Step 1: A·P  (left-multiply P by A)
//     // Row 0 of A·P = a0 · P row 0
//     const float AP00 = a0 * P[0][0];
//     const float AP01 = a0 * P[0][1];
//     const float AP02 = a0 * P[0][2];

//     // Row 1 of A·P = −K1·P row 0  +  P row 1
//     const float AP10 = -K1 * P[0][0] + P[1][0];
//     const float AP11 = -K1 * P[0][1] + P[1][1];
//     const float AP12 = -K1 * P[0][2] + P[1][2];

//     // Row 2 of A·P = −K2·P row 0  +  P row 2
//     const float AP20 = -K2 * P[0][0] + P[2][0];
//     const float AP21 = -K2 * P[0][1] + P[2][1];
//     const float AP22 = -K2 * P[0][2] + P[2][2];

//     // Step 2: (A·P)·Aᵀ  —  element [i][j] = AP[i] · A[j]
//     // A rows: [a0, 0, 0], [−K1, 1, 0], [−K2, 0, 1]
//     float Pnew[3][3];

//     Pnew[0][0] = AP00 * a0;
//     Pnew[0][1] = -AP00 * K1 + AP01;
//     Pnew[0][2] = -AP00 * K2 + AP02;

//     Pnew[1][0] = AP10 * a0;
//     Pnew[1][1] = -AP10 * K1 + AP11;
//     Pnew[1][2] = -AP10 * K2 + AP12;

//     Pnew[2][0] = AP20 * a0;
//     Pnew[2][1] = -AP20 * K1 + AP21;
//     Pnew[2][2] = -AP20 * K2 + AP22;

//     // Step 3: add K·R·Kᵀ  (symmetric rank-1 matrix, scaled by R)
//     const float R = config.R;
//     Pnew[0][0] += R * K0 * K0;
//     Pnew[0][1] += R * K0 * K1;
//     Pnew[0][2] += R * K0 * K2;
//     Pnew[1][0] += R * K1 * K0;
//     Pnew[1][1] += R * K1 * K1;
//     Pnew[1][2] += R * K1 * K2;
//     Pnew[2][0] += R * K2 * K0;
//     Pnew[2][1] += R * K2 * K1;
//     Pnew[2][2] += R * K2 * K2;

//     memcpy(P, Pnew, sizeof(P));
//     symmetrizeP();

//     lastUpdateAccepted = true;
//     return true;
// }

// // ── State accessors ──────────────────────────────────────────────────────────

// float PosKF::getDepth() const { return x[0]; }
// float PosKF::getVelocity() const { return x[1]; }
// float PosKF::getBias() const { return x[2]; }

// // ── Diagnostics ──────────────────────────────────────────────────────────────

// float PosKF::getLastInnovation() const { return lastInnov; }
// float PosKF::getLastInnovCov() const { return lastS; }
// bool PosKF::wasLastUpdateAccepted() const { return lastUpdateAccepted; }

// float PosKF::getCovZ() const { return P[0][0]; }
// float PosKF::getCovVz() const { return P[1][1]; }
// float PosKF::getCovBias() const { return P[2][2]; }

// bool PosKF::isConverged() const
// {
//     return (P[0][0] < 0.05f) && (P[2][2] < 0.001f);
// }

// // ── Internal ─────────────────────────────────────────────────────────────────

// void PosKF::symmetrizeP()
// {
//     for (int r = 0; r < 3; r++)
//     {
//         for (int c = r + 1; c < 3; c++)
//         {
//             const float v = 0.5f * (P[r][c] + P[c][r]);
//             P[r][c] = v;
//             P[c][r] = v;
//         }

//         // Enforce strict positive-definiteness floor on the diagonal.
//         // Prevents covariance collapse from numerical underflow.
//         if (P[r][r] < 1.0e-12f)
//             P[r][r] = 1.0e-12f;
//     }
// }

#include "PosKF.h"
#include <math.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor / lifecycle
// ─────────────────────────────────────────────────────────────────────────────

PosKF::PosKF()
{
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    lastInnov         = 0.0f;
    lastS             = 0.0f;
    lastUpdateAccepted = false;
}

void PosKF::begin(float initialDepth) { reset(initialDepth); }

void PosKF::setConfig(const Config &cfg) { config = cfg; }
PosKF::Config PosKF::getConfig() const  { return config; }

void PosKF::reset(float initialDepth)
{
    x[0] = initialDepth; // pos_z   — start at specified depth (normally 0)
    x[1] = 0.0f;         // vel_z   — stationary
    x[2] = 0.0f;         // b_az    — assume no accel bias; filter converges
    x[3] = 0.0f;         // b_baro1 — assume no slow baro bias
    x[4] = 0.0f;         // b_baro2 — assume no fast baro bias

    memset(P, 0, sizeof(P));

    // Initial uncertainties — generous enough for cold start.
    // Baro bias states initialised at their steady-state variances (σ²)
    // so the filter starts with the correct prior scale for each component.
    P[0][0] = 1.0f;                          // 1.0  m   depth σ
    P[1][1] = 1.0f;                          // 1.0  m/s velocity σ
    P[2][2] = 0.01f;                         // 0.1  m/s² accel bias σ
    P[3][3] = _sq(config.sigma_b1);           // 0.123 m²  slow baro bias prior
    P[4][4] = _sq(config.sigma_b2);           // 0.066 m²  fast baro bias prior

    lastInnov          = 0.0f;
    lastS              = 0.0f;
    lastUpdateAccepted = false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  predict()  — call at 500 Hz
//
//  STATE PROPAGATION
//    a_eff = accelZ − b_az          (bias-corrected vertical accel)
//    pos_z  ← pos_z + vel_z·dt + ½·a_eff·dt²
//    vel_z  ← vel_z + a_eff·dt
//    b_az   unchanged (random walk, corrected by baro updates)
//    b_baro1 ← φ₁ · b_baro1        φ₁ = 1 − dt/τ₁  (linear approx, valid ∵ dt≪τ₁)
//    b_baro2 ← φ₂ · b_baro2        φ₂ = exp(−dt/τ₂)
//
//  COVARIANCE PROPAGATION  P ← F·P·Fᵀ + Q
//
//    F (5×5):
//      [1   dt  −½dt²   0    0  ]  row 0
//      [0    1   −dt    0    0  ]  row 1
//      [0    0    1     0    0  ]  row 2
//      [0    0    0    φ₁    0  ]  row 3
//      [0    0    0     0   φ₂ ]  row 4
//
//    Implemented as two sequential passes:
//      FP[i][j]  = F[i]·P[:,j]        (row-times-column = left-multiply)
//      P[i][j]   = FP[i,:]·F[j]ᵀ     (right-multiply by Fᵀ)
//
//    Q additions (diagonal only):
//      P[0][0] += ¼dt⁴·σa²,   P[0][1] += ½dt³·σa²
//      P[1][0] += ½dt³·σa²,   P[1][1] += dt²·σa²
//      P[2][2] += dt·σb²
//      P[3][3] += Q_b1 = 2·σ_b1²·dt/τ₁
//      P[4][4] += Q_b2 = 2·σ_b2²·dt/τ₂
//
//  Stack usage: 25 floats (FP array) + scalars ≈ 140 bytes — safe on Cortex-M4.
//  Computational cost: 2 expf + ~80 fmac per call at 500 Hz → < 0.1% CPU at 168 MHz.
// ─────────────────────────────────────────────────────────────────────────────

void PosKF::predict(float accelZ, float dt)
{
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt)) return;

    const float dt2  = dt * dt;
    const float hdt2 = 0.5f * dt2;
    const float dt3  = dt2 * dt;
    const float dt4  = dt3 * dt;

    // ── GM decay coefficients for this dt ────────────────────────────────
    //
    // phi1: linear approximation  φ₁ ≈ 1 − dt/τ₁
    //   Exact formula is expf(−dt/τ₁).  For τ₁=354 s, dt≈0.002 s:
    //   dt/τ₁ ≈ 5.65e-6.  Linear error ≈ (dt/τ₁)²/2 ≈ 1.6e-11 — negligible.
    //   This avoids computing expf(−5.65e-6) whose argument is near the float
    //   precision boundary when subtracted from 1.0.
    //
    // phi2: expf used directly (τ₂=0.608 s → dt/τ₂ ≈ 0.0033, well-conditioned).
    //
    // Q_bi: continuous-time limit Q ≈ 2σ²dt/τ.
    //   Exact: σ²(1−φᵢ²).  Relative error vs exact < 0.5% for τ₂, negligible for τ₁.
    const float phi1 = 1.0f - dt / config.tau_b1;
    const float phi2 = expf(-dt / config.tau_b2);
    const float Q_b1 = 2.0f * _sq(config.sigma_b1) * dt / config.tau_b1;
    const float Q_b2 = 2.0f * _sq(config.sigma_b2) * dt / config.tau_b2;

    // ── State propagation ─────────────────────────────────────────────────
    const float a_eff = accelZ - x[2];            // bias-corrected accel
    x[0] += x[1] * dt + hdt2 * a_eff;            // kinematic: pos
    x[1] += a_eff * dt;                           // kinematic: vel
    // x[2] (b_az) — unchanged; random walk driven only by process noise Q[2][2]
    x[3] *= phi1;                                  // b_baro1 decays toward 0
    x[4] *= phi2;                                  // b_baro2 decays toward 0

    // ── Covariance propagation: step 1  FP = F · P ───────────────────────
    // F row 0 = [1, dt, −hdt2, 0, 0]
    // F row 1 = [0, 1,  −dt,   0, 0]
    // F row 2 = [0, 0,   1,    0, 0]
    // F row 3 = [0, 0,   0,   φ₁, 0]
    // F row 4 = [0, 0,   0,   0, φ₂]
    float FP[N][N];
    for (int j = 0; j < N; ++j) {
        FP[0][j] =  P[0][j] + dt  * P[1][j] - hdt2 * P[2][j];
        FP[1][j] =  P[1][j] - dt  * P[2][j];
        FP[2][j] =  P[2][j];
        FP[3][j] =  phi1 * P[3][j];
        FP[4][j] =  phi2 * P[4][j];
    }

    // ── Covariance propagation: step 2  P = FP · Fᵀ ─────────────────────
    // Fᵀ column j = F row j (so right-multiplying mirrors left-multiplying).
    // Element P[i][j] = FP[i,:] dot F[j]
    // F[j] for j=0: [1, dt, −hdt2, 0, 0]    → P[i][0] = FP[i][0]+dt·FP[i][1]−hdt2·FP[i][2]
    // F[j] for j=1: [0, 1,  −dt,   0, 0]    → P[i][1] = FP[i][1]−dt·FP[i][2]
    // F[j] for j=2: [0, 0,   1,    0, 0]    → P[i][2] = FP[i][2]
    // F[j] for j=3: [0, 0,   0,   φ₁, 0]   → P[i][3] = φ₁·FP[i][3]
    // F[j] for j=4: [0, 0,   0,   0, φ₂]   → P[i][4] = φ₂·FP[i][4]
    for (int i = 0; i < N; ++i) {
        P[i][0] = FP[i][0] + dt  * FP[i][1] - hdt2 * FP[i][2];
        P[i][1] = FP[i][1] - dt  * FP[i][2];
        P[i][2] = FP[i][2];
        P[i][3] = phi1 * FP[i][3];
        P[i][4] = phi2 * FP[i][4];
    }

    // ── Add process noise Q (sparse diagonal + kinematic cross term) ──────
    const float sa2 = _sq(config.sigmaA);
    const float sb2 = _sq(config.sigmaB);

    P[0][0] += sa2 * 0.25f * dt4;  // (G·Gᵀ)[0][0]
    P[0][1] += sa2 * 0.5f  * dt3;  // (G·Gᵀ)[0][1]
    P[1][0] += sa2 * 0.5f  * dt3;  // (G·Gᵀ)[1][0]  — symmetry
    P[1][1] += sa2 * dt2;           // (G·Gᵀ)[1][1]
    P[2][2] += sb2 * dt;            // accel bias random walk
    P[3][3] += Q_b1;                // slow GM process noise
    P[4][4] += Q_b2;                // fast GM process noise

    symmetrizeP();
}

// ─────────────────────────────────────────────────────────────────────────────
//  update()  — call at ~30 Hz when a fresh baro sample is ready
//
//  MEASUREMENT MODEL
//    H = [1, 0, 0, 1, 1]   (baro sees pos_z + b_baro1 + b_baro2)
//    ν = depth − (x[0] + x[3] + x[4])
//    S = H·P·Hᵀ + R  =  Σ P[i][j]  for i,j ∈ {0,3,4}  +  R
//    K = P·Hᵀ / S    where  (P·Hᵀ)[i] = P[i][0] + P[i][3] + P[i][4]
//
//  COVARIANCE UPDATE — Joseph form for numerical symmetry / PD guarantee
//    A   = I − K·H
//    P ← A·P·Aᵀ + K·R·Kᵀ
//
//    Computed as:
//      c[j]     = (H·P)[j] = P[0][j] + P[3][j] + P[4][j]   (H-selected row sum)
//      AP[i][j] = P[i][j] − K[i]·c[j]                       (A·P)
//      d[i]     = AP[i][0] + AP[i][3] + AP[i][4]            (H·AP row, = K[i]·R for sym P)
//      P[i][j]  = AP[i][j] − K[j]·d[i] + R·K[i]·K[j]       (APAᵀ + KRKᵀ)
//
//  Stack usage: 2 × 25 floats (c, AP arrays) + scalars ≈ 240 bytes — safe.
//  Computational cost: ~150 fmac per call at 30 Hz → < 0.001% CPU at 168 MHz.
// ─────────────────────────────────────────────────────────────────────────────

bool PosKF::update(float depth)
{
    if (!isfinite(depth)) return false;

    // ── Innovation  ν = z − H·x ───────────────────────────────────────────
    // H·x = x[0] + x[3] + x[4]   (H = [1,0,0,1,1])
    const float innov = depth - (x[0] + x[3] + x[4]);

    // ── Innovation covariance  S = H·P·Hᵀ + R ─────────────────────────────
    // H picks rows/cols {0,3,4}:
    //   S = P[0][0]+P[0][3]+P[0][4] + P[3][0]+P[3][3]+P[3][4] + P[4][0]+P[4][3]+P[4][4] + R
    const float S = P[0][0] + P[0][3] + P[0][4]
                  + P[3][0] + P[3][3] + P[3][4]
                  + P[4][0] + P[4][3] + P[4][4]
                  + config.R;

    lastInnov = innov;
    lastS     = S;

    if (S <= 0.0f || !isfinite(S)) return false;

    // ── Mahalanobis gate  (χ², DOF=1) ─────────────────────────────────────
    // Automatically wide at startup (large P), tightens as filter converges.
    if (_sq(innov) / S > config.mahaGate) {
        lastUpdateAccepted = false;
        return false;
    }

    // ── Kalman gain  K = P·Hᵀ / S ─────────────────────────────────────────
    // H = [1,0,0,1,1]  →  P·Hᵀ picks columns {0,3,4} of P.
    float K[N];
    for (int i = 0; i < N; ++i) {
        K[i] = (P[i][0] + P[i][3] + P[i][4]) / S;
    }

    // ── State update  x ← x + K·ν ────────────────────────────────────────
    for (int i = 0; i < N; ++i) {
        x[i] += K[i] * innov;
    }

    // Hard clamps — prevent pathological wind-up from sustained bad baro
    if      (x[2] >  config.biasClamp)     x[2] =  config.biasClamp;
    else if (x[2] < -config.biasClamp)     x[2] = -config.biasClamp;
    if      (x[3] >  config.baroBiasClamp) x[3] =  config.baroBiasClamp;
    else if (x[3] < -config.baroBiasClamp) x[3] = -config.baroBiasClamp;
    if      (x[4] >  config.baroBiasClamp) x[4] =  config.baroBiasClamp;
    else if (x[4] < -config.baroBiasClamp) x[4] = -config.baroBiasClamp;

    // ── Joseph-form covariance update ─────────────────────────────────────

    // Step 1 — c[j] = (H·P)[j] = P[0][j] + P[3][j] + P[4][j]
    //          The H-selected row sum of P (= S·K[j] for symmetric P).
    float c[N];
    for (int j = 0; j < N; ++j) {
        c[j] = P[0][j] + P[3][j] + P[4][j];
    }

    // Step 2 — AP[i][j] = P[i][j] − K[i]·c[j]   =  (I − KH)·P
    float AP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            AP[i][j] = P[i][j] - K[i] * c[j];
        }
    }

    // Step 3 — P[i][j] = AP·Aᵀ + K·R·Kᵀ
    //   (AP·Aᵀ)[i][j] = AP[i][j] − K[j]·d[i]
    //   where d[i] = AP[i][0]+AP[i][3]+AP[i][4]  (H·AP row i)
    //   For exactly symmetric P and optimal K: d[i] = K[i]·R,
    //   so the K·R·Kᵀ term cancels the −K[j]·d[i] term.
    //   We keep both for full numerical stability (Joseph form).
    for (int i = 0; i < N; ++i) {
        const float d_i = AP[i][0] + AP[i][3] + AP[i][4];
        for (int j = 0; j < N; ++j) {
            P[i][j] = AP[i][j] - K[j] * d_i + config.R * K[i] * K[j];
        }
    }

    symmetrizeP();
    lastUpdateAccepted = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  State accessors
// ─────────────────────────────────────────────────────────────────────────────

float PosKF::getDepth()         const { return x[0]; }
float PosKF::getVelocity()      const { return x[1]; }
float PosKF::getBias()          const { return x[2]; }
float PosKF::getBaroBias1()     const { return x[3]; }
float PosKF::getBaroBias2()     const { return x[4]; }
float PosKF::getTotalBaroBias() const { return x[3] + x[4]; }

// ─────────────────────────────────────────────────────────────────────────────
//  Diagnostics
// ─────────────────────────────────────────────────────────────────────────────

float PosKF::getLastInnovation()      const { return lastInnov; }
float PosKF::getLastInnovCov()        const { return lastS; }
bool  PosKF::wasLastUpdateAccepted()  const { return lastUpdateAccepted; }

float PosKF::getCovZ()    const { return P[0][0]; }
float PosKF::getCovVz()   const { return P[1][1]; }
float PosKF::getCovBias() const { return P[2][2]; }

bool PosKF::isConverged() const
{
    // Same criterion as 3-state version — kinematic states ready for hold.
    // Note: P[3][3] and P[4][4] will settle near but not at zero because
    // baro biases are weakly observable from baro alone; do not gate on them.
    return (P[0][0] < 0.05f) && (P[2][2] < 0.001f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  symmetrizeP  — enforce P symmetric + PD diagonal floor
// ─────────────────────────────────────────────────────────────────────────────

void PosKF::symmetrizeP()
{
    for (int r = 0; r < N; ++r) {
        for (int c = r + 1; c < N; ++c) {
            const float v = 0.5f * (P[r][c] + P[c][r]);
            P[r][c] = v;
            P[c][r] = v;
        }
        if (P[r][r] < 1.0e-12f) P[r][r] = 1.0e-12f;
    }
}