/**
 * Forward kinematics using Denavit-Hartenberg homogeneous transforms.
 * Returns the 4x4 transform of each frame and the end effector position.
 */

import { DH_LINKS } from './robotConfig.js';

/**
 * Build a 4x4 DH transformation matrix.
 */
export function dhMatrix(a, alpha, d, theta) {
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);

  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

/**
 * Multiply two 4x4 matrices.
 */
export function mul4(A, B) {
  const C = [];
  for (let i = 0; i < 4; i++) {
    C[i] = [];
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
  return C;
}

/**
 * Compute forward kinematics for a given joint angle vector q (length 6).
 *
 * Returns:
 *   { transforms: [T0, T1, ..., T6], p: [x, y, z] }
 * where transforms[i] is the frame of joint i (base = 0) and p is the
 * end effector position.
 */
export function forwardKinematics(q) {
  let T = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
  const transforms = [T];
  for (let i = 0; i < DH_LINKS.length; i++) {
    const { a, alpha, d } = DH_LINKS[i];
    const theta = q[i] !== undefined ? q[i] : 0;
    T = mul4(T, dhMatrix(a, alpha, d, theta));
    transforms.push(T);
  }
  const p = [T[0][3], T[1][3], T[2][3]];
  return { transforms, p };
}

/**
 * Extract position [x, y, z] from a 4x4 transform.
 */
export function positionFromTransform(T) {
  return [T[0][3], T[1][3], T[2][3]];
}

/**
 * Extract the 3x3 rotation part from a 4x4 transform.
 */
export function rotationFromTransform(T) {
  return [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ];
}

/**
 * Compute the 3x6 Jacobian (linear velocity only) for q.
 * Column i: z_{i-1} x (p_n - p_{i-1})
 */
export function jacobian(q) {
  const { transforms, p } = forwardKinematics(q);
  const n = DH_LINKS.length;
  const J = Array.from({ length: 3 }, () => Array(n).fill(0));
  for (let i = 0; i < n; i++) {
    const Ti = transforms[i];
    const zi = [Ti[0][2], Ti[1][2], Ti[2][2]]; // z-axis of frame i-1
    const pi = [Ti[0][3], Ti[1][3], Ti[2][3]]; // origin of frame i-1
    J[0][i] = zi[1] * (p[2] - pi[2]) - zi[2] * (p[1] - pi[1]);
    J[1][i] = zi[2] * (p[0] - pi[0]) - zi[0] * (p[2] - pi[2]);
    J[2][i] = zi[0] * (p[1] - pi[1]) - zi[1] * (p[0] - pi[0]);
  }
  return J;
}
