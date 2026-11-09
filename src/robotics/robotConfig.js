/**
 * Robot configuration: DH parameters, joint limits, and link lengths.
 *
 * 6-DOF articulated industrial robot arm:
 *   J1 - Base rotation (about Z)
 *   J2 - Shoulder pitch (about Y)
 *   J3 - Elbow pitch (about Y)
 *   J4 - Wrist roll (about Z)
 *   J5 - Wrist pitch (about Y)
 *   J6 - End effector / gripper rotation (about Z)
 *
 * All angles in radians. Lengths in meters.
 */

export const RAD = Math.PI / 180;
export const DEG = 180 / Math.PI;

// Denavit-Hartenberg parameters: [a, alpha, d]
// theta (q) is the joint variable.
export const DH_LINKS = [
  { a: 0.0, alpha: -Math.PI / 2, d: 0.15 }, // J1 Base: offset height
  { a: 0.45, alpha: 0.0, d: 0.0 },           // J2 Shoulder -> upper arm
  { a: 0.35, alpha: 0.0, d: 0.0 },           // J3 Elbow -> forearm
  { a: 0.0, alpha: -Math.PI / 2, d: 0.0 },   // J4 Wrist roll
  { a: 0.0, alpha: Math.PI / 2, d: 0.0 },    // J5 Wrist pitch
  { a: 0.10, alpha: 0.0, d: 0.0 },           // J6 Gripper
];

// Joint limits in degrees -> converted to radians below.
export const JOINT_LIMITS_DEG = {
  j1: { min: -180, max: 180 },   // Base rotation
  j2: { min: -90, max: 90 },     // Shoulder pitch
  j3: { min: -140, max: 10 },    // Elbow pitch
  j4: { min: -180, max: 180 },   // Wrist roll
  j5: { min: -90, max: 90 },     // Wrist pitch
  j6: { min: -90, max: 90 },     // Gripper rotation
};

// Convert limits to radians.
export const JOINT_LIMITS = Object.fromEntries(
  Object.entries(JOINT_LIMITS_DEG).map(([key, { min, max }]) => [
    key,
    { min: min * RAD, max: max * RAD },
  ])
);

// Home pose (all joints at 0 except elbow slightly down for a natural rest).
export const HOME_POSE = [0, 0, 0, 0, 0, 0];

// Clamp an angle to its joint limits.
export function clampAngle(jointKey, angle) {
  const { min, max } = JOINT_LIMITS[jointKey];
  return Math.min(Math.max(angle, min), max);
}

// Clamp all angles to their respective limits.
export function clampAllAngles(angles) {
  const keys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
  return angles.map((a, i) => clampAngle(keys[i], a));
}

// Total reach of the arm (shoulder + elbow + gripper offset).
export const MAX_REACH = 0.45 + 0.35 + 0.10;
// Minimum reach (elbow fully folded).
export const MIN_REACH = Math.abs(0.45 - 0.35) - 0.10;

// Joint labels for the UI.
export const JOINT_LABELS = {
  j1: 'J1 – Base Rotation',
  j2: 'J2 – Shoulder',
  j3: 'J3 – Elbow',
  j4: 'J4 – Wrist Roll',
  j5: 'J5 – Wrist Pitch',
  j6: 'J6 – Gripper Rotation',
};

// Default target position (within workspace).
export const DEFAULT_TARGET = { x: 0.5, y: 0.0, z: 0.4 };
