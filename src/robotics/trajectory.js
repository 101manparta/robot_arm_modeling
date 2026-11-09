/**
 * Smooth trajectory generation for the pick & place demo.
 *
 * Provides easing functions and a waypoint sequencer that yields joint
 * targets over time. All motion is smooth (no teleporting).
 */

import { HOME_POSE } from './robotConfig.js';
import { inverseKinematics } from './inverseKinematics.js';

/**
 * Smoothstep easing: t^2 * (3 - 2t). Smooth start and end.
 */
export function smoothstep(t) {
  return t * t * (3 - 2 * t);
}

/**
 * Smootherstep easing: 6t^5 - 15t^4 + 10t^3. Even smoother.
 */
export function smootherstep(t) {
  return t * t * t * (t * (t * 6 - 15) + 10);
}

/**
 * Interpolate between two joint vectors with an easing function.
 */
export function lerpJoints(a, b, t, ease = smoothstep) {
  const e = ease(t);
  return a.map((v, i) => v + (b[i] - v) * e);
}

/**
 * A trajectory step: { duration (seconds), target (joint array), ease }.
 */
export class Trajectory {
  constructor() {
    this.steps = [];
    this.totalDuration = 0;
    this._rebuildTable();
  }

  addStep(duration, target, ease = smoothstep) {
    this.steps.push({ duration, target: [...target], ease });
    this.totalDuration += duration;
    this._rebuildTable();
    return this;
  }

  _rebuildTable() {
    // Table not strictly needed for linear scan, but kept for future binary search.
  }

  /**
   * Get the interpolated joint target at time `elapsed` (seconds).
   * Returns null when the trajectory is finished.
   */
  getAt(elapsed) {
    if (this.steps.length === 0) return HOME_POSE.slice();
    if (elapsed >= this.totalDuration) return null;

    let t0 = 0;
    for (let i = 0; i < this.steps.length; i++) {
      const step = this.steps[i];
      const t1 = t0 + step.duration;
      if (elapsed < t1) {
        const localT = (elapsed - t0) / step.duration;
        const from = i === 0 ? HOME_POSE : this.steps[i - 1].target;
        return lerpJoints(from, step.target, localT, step.ease);
      }
      t0 = t1;
    }
    return this.steps[this.steps.length - 1].target;
  }

  isFinished(elapsed) {
    return elapsed >= this.totalDuration;
  }
}

/**
 * Build a pick & place trajectory as a list of { duration, targetQ } steps.
 *
 * @param {number[]} objectPos  - [x, y, z] position of the object to pick.
 * @param {number[]} placePos   - [x, y, z] position where to place it.
 * @param {number[]} homePose   - joint angles for home (radians).
 * @returns {Array<{duration: number, targetQ: number[], label: string}>}
 */
export function buildPickAndPlaceTrajectory(objectPos, placePos, homePose = HOME_POSE) {
  const [ox, oy, oz] = objectPos;
  const [px, py, pz] = placePos;

  const aboveObject = [ox, oy, oz + 0.28];
  const abovePlace = [px, py, pz + 0.28];

  // Solve IK for each position-based waypoint.
  const solve = (pos) => {
    const result = inverseKinematics(pos, homePose);
    return result.q;
  };

  const waypoints = [
    { label: 'home', pos: null, q: homePose, gripper: 1 },
    { label: 'approach-object', pos: aboveObject, gripper: 1 },
    { label: 'lower-to-object', pos: objectPos, gripper: 1 },
    { label: 'grasp', pos: objectPos, gripper: 0 },
    { label: 'lift', pos: aboveObject, gripper: 0 },
    { label: 'move-to-place', pos: abovePlace, gripper: 0 },
    { label: 'lower-to-place', pos: placePos, gripper: 0 },
    { label: 'release', pos: placePos, gripper: 1 },
    { label: 'lift-after-place', pos: abovePlace, gripper: 1 },
    { label: 'return-home', pos: null, q: homePose, gripper: 1 },
  ];

  // Resolve joint targets for each waypoint.
  const resolved = waypoints.map((w) =>
    w.pos ? { ...w, q: solve(w.pos) } : { ...w, q: w.q.slice() }
  );

  const steps = [];
  const stepDurations = [
    1.2, // home -> approach-object
    0.8, // approach-object -> lower-to-object
    0.4, // lower: close gripper (pause-ish)
    0.8, // lift
    1.2, // move to place
    0.8, // lower to place
    0.4, // release (pause-ish)
    0.8, // lift after place
    1.2, // return home
  ];

  for (let i = 0; i < resolved.length - 1; i++) {
    const from = resolved[i];
    const to = resolved[i + 1];
    steps.push({
      duration: stepDurations[i] || 1.0,
      targetQ: to.q,
      label: to.label,
    });
  }

  return steps;
}

/**
 * Build a simple move-to-target trajectory from current pose to a target position.
 */
export function buildMoveToTargetTrajectory(targetPos, currentQ, homePose = HOME_POSE) {
  const result = inverseKinematics(targetPos, currentQ);
  const targetQ = result.q;
  return [
    { duration: 1.5, targetQ, label: 'move-to-target', reachable: result.reachable },
  ];
}
