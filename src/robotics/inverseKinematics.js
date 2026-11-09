/**
 * Inverse kinematics for the 6-DOF robot arm.
 *
 * Strategy (geometric / decomposed):
 *   1. J1 (base yaw) points the arm toward the target's XY projection.
 *   2. J2 (shoulder) and J3 (elbow) are solved in the vertical radial plane
 *      using the law of cosines (2-link IK).
 *   3. J4, J5 (wrist) are set to keep the end effector upright.
 *   4. J6 (gripper) is set to 0.
 *
 * Returns { q: [6 angles in radians], reachable: boolean }.
 */

import { DH_LINKS, JOINT_LIMITS, MAX_REACH, MIN_REACH, clampAngle } from './robotConfig.js';
import { forwardKinematics } from './kinematics.js';

const L2 = 0.45; // upper arm
const L3 = 0.35; // forearm
const L6 = 0.10; // gripper offset

/**
 * Clamp a value between min and max.
 */
function clamp(v, min, max) {
  return Math.min(Math.max(v, min), max);
}

/**
 * Solve IK for a target position [x, y, z].
 *
 * @param {number[]} target - [x, y, z] in meters relative to base.
 * @param {number[]} seed - optional seed joint angles (unused here, kept for API).
 * @returns {{ q: number[], reachable: boolean }}
 */
export function inverseKinematics(target, seed = [0, 0, 0, 0, 0, 0]) {
  const [tx, ty, tz] = target;
  const q = [0, 0, 0, 0, 0, 0];

  // Distance from base center to target (ignoring base height offset for reach check).
  const baseH = DH_LINKS[0].d; // 0.15
  const r = Math.sqrt(tx * tx + ty * ty); // radial distance in XY
  const dz = tz - baseH; // vertical difference from shoulder

  // Reachability: the reachable sphere for the shoulder-elbow chain.
  const reach = Math.sqrt(r * r + dz * dz);

  let reachable = true;
  if (reach > MAX_REACH + 0.02 || reach < Math.max(0.01, MIN_REACH - 0.02)) {
    reachable = false;
  }

  // ---- J1: Base yaw ----
  // Rotate base so the arm plane faces the target.
  q[0] = Math.atan2(ty, tx);
  q[0] = clamp(q[0], JOINT_LIMITS.j1.min, JOINT_LIMITS.j1.max);

  // If unreachable, still produce a best-effort pose (clamped) so nothing breaks.
  if (!reachable) {
    // Point toward target anyway but scale down to max reach.
    const scale = (MAX_REACH - 0.02) / Math.max(reach, 0.001);
    const rx = r * scale;
    const dzClamped = dz * scale;
    return { q: clampAllAngles(q), reachable: false };
  }

  // ---- J2 & J3: 2-link IK in the radial-vertical plane ----
  // We solve for the shoulder and elbow such that the wrist ends up at
  // (r, dz) adjusted for the gripper offset.
  const effectiveR = r - L6;
  const effectiveZ = dz;

  const d2 = effectiveR * effectiveR + effectiveZ * effectiveZ;

  // Law of cosines: cos(q3) = (d2 - L2^2 - L3^2) / (2 * L2 * L3)
  const cosQ3 = clamp(
    (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3),
    -1,
    1
  );
  const q3Elbow = Math.acos(cosQ3); // elbow bent "down" (positive)

  // Choose elbow-down configuration.
  let q3 = q3Elbow;
  q3 = clamp(q3, JOINT_LIMITS.j3.min, JOINT_LIMITS.j3.max);

  // k1 = L2 + L3*cos(q3), k2 = L3*sin(q3)
  const k1 = L2 + L3 * Math.cos(q3);
  const k2 = L3 * Math.sin(q3);

  // q2 = atan2(effectiveZ, effectiveR) - atan2(k2, k1)
  let q2 = Math.atan2(effectiveZ, effectiveR) - Math.atan2(k2, k1);
  q2 = clamp(q2, JOINT_LIMITS.j2.min, JOINT_LIMITS.j2.max);

  q[1] = q2;
  q[2] = q3;

  // ---- J4, J5: Wrist ----
  // Keep the end effector pointing roughly downward (gripper vertical).
  q[3] = 0; // wrist roll
  q[4] = -(q2 + q3); // wrist pitch compensates shoulder+elbow to stay level
  q[4] = clamp(q[4], JOINT_LIMITS.j5.min, JOINT_LIMITS.j5.max);

  // ---- J6: Gripper ----
  q[5] = 0;

  return { q: clampAllAngles(q), reachable: true };
}

/**
 * Clamp all joint angles to their limits.
 */
function clampAllAngles(angles) {
  const keys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
  return angles.map((a, i) => clampAngle(keys[i], a));
}

/**
 * Check if a target is reachable without solving full IK.
 */
export function isReachable(target) {
  const [tx, ty, tz] = target;
  const baseH = DH_LINKS[0].d;
  const r = Math.sqrt(tx * tx + ty * ty);
  const dz = tz - baseH;
  const reach = Math.sqrt(r * r + dz * dz);
  return reach >= Math.max(0.01, MIN_REACH - 0.02) && reach <= MAX_REACH + 0.02;
}
