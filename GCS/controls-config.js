// // ═══════════════════════════════════════════════════════════════════════
// //  controls-config.js  —  THE ONLY FILE YOU NEED TO EDIT
// //
// //  Each object in CONTROL_GROUPS becomes one panel section in the UI.
// //  Each control inside it becomes one labelled input row.
// //
// //  Fields per control:
// //    key      {string}  — MUST exactly match wifiBridge_addControl("key", ...)
// //                         on the ESP32. This is what gets sent over the wire.
// //    label    {string}  — Short display name shown on screen
// //    step     {number}  — Increment for the number input +/- arrows
// //    default  {number}  — Value pre-filled when the page loads
// //
// //  To add a control:   add a line to the relevant group's controls array.
// //  To add a group:     append a new { label, controls: [...] } object.
// //  To remove either:   delete the line/block.
// //  Nothing else needs changing.
// //
// //  KEY RULE: every key here must match wifiBridge_addControl("key", ...)
// //  on the ESP32 byte-for-byte — copy-paste, never retype.
// // ═══════════════════════════════════════════════════════════════════════

// const CONTROL_GROUPS = [

//   // ── Rate loop PIDs ──────────────────────────────────────────────────
//   // ESP32: wifiBridge_addControl("roll_rate_p/i/d", ...)
//   {
//     label: 'Roll Rate PID',
//     controls: [
//       { key: 'roll_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
//       { key: 'roll_rate_i', label: 'ki', step: 0.001, default: 0.000 },
//       { key: 'roll_rate_d', label: 'kd', step: 0.001, default: 0.100 },
//     ],
//   },
//   // ESP32: wifiBridge_addControl("pitch_rate_p/i/d", ...)
//   {
//     label: 'Pitch Rate PID',
//     controls: [
//       { key: 'pitch_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
//       { key: 'pitch_rate_i', label: 'ki', step: 0.001, default: 0.000 },
//       { key: 'pitch_rate_d', label: 'kd', step: 0.001, default: 0.100 },
//     ],
//   },
//   // ESP32: wifiBridge_addControl("yaw_rate_p/i/d", ...)
//   {
//     label: 'Yaw Rate PID',
//     controls: [
//       { key: 'yaw_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
//       { key: 'yaw_rate_i', label: 'ki', step: 0.001, default: 0.000 },
//       { key: 'yaw_rate_d', label: 'kd', step: 0.001, default: 0.100 },
//     ],
//   },

//   // ── Angle loop (outer P loop, no I or D) ────────────────────────────
//   // ESP32: wifiBridge_addControl("roll_angle_p", ...)
//   //        wifiBridge_addControl("pitch_angle_p", ...)
//   {
//     label: 'Angle Loop  (P only)',
//     controls: [
//       { key: 'roll_angle_p',  label: 'kp roll',  step: 0.1, default: 5.0 },
//       { key: 'pitch_angle_p', label: 'kp pitch', step: 0.1, default: 5.0 },
//     ],
//   },

//   // ── State-estimation filter tuning ──────────────────────────────────
//   // ESP32: wifiBridge_addControl("filt_gnoise", &amekf_cfg.gyroNoiseRad)
//   //        wifiBridge_addControl("filt_gbias",  &amekf_cfg.gyroBiasNoiseRad)
//   //        wifiBridge_addControl("filt_anoise", &amekf_cfg.accelNoise)
//   {
//     label: 'Filter (AMEKF)',
//     controls: [
//       { key: 'filt_gnoise', label: 'gyro noise', step: 0.0001, default: 0.0010 },
//       { key: 'filt_gbias',  label: 'gyro bias',  step: 0.0001, default: 0.0010 },
//       { key: 'filt_anoise', label: 'accel noise', step: 0.001, default: 0.0100 },
//     ],
//   },

// ];

const CONTROL_GROUPS = [

  {
    label: 'Roll Rate PID',
    controls: [
      { key: 'roll_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
      { key: 'roll_rate_i', label: 'ki', step: 0.001, default: 0.000 },
      { key: 'roll_rate_d', label: 'kd', step: 0.001, default: 0.100 },
    ],
  },

  {
    label: 'Pitch Rate PID',
    controls: [
      { key: 'pitch_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
      { key: 'pitch_rate_i', label: 'ki', step: 0.001, default: 0.000 },
      { key: 'pitch_rate_d', label: 'kd', step: 0.001, default: 0.100 },
    ],
  },

  {
    label: 'Yaw Rate PID',
    controls: [
      { key: 'yaw_rate_p', label: 'kp', step: 0.01,  default: 1.000 },
      { key: 'yaw_rate_i', label: 'ki', step: 0.001, default: 0.000 },
      { key: 'yaw_rate_d', label: 'kd', step: 0.001, default: 0.100 },
    ],
  },

];