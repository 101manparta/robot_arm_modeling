/**
 * useRobotController
 *
 * Central controller for the robotic arm. Manages:
 *   - Joint targets and current angles (with smooth damping)
 *   - Forward kinematics (end effector position)
 *   - Inverse kinematics (move to XYZ target)
 *   - Pick & place demo sequencing
 *   - Gripper open/close state
 *
 * Designed to be used inside a React Three Fiber render loop via
 * a mutable ref that components read each frame.
 */

import { useRef, useCallback } from 'react';
import { HOME_POSE, JOINT_LIMITS, clampAllAngles } from '../robotics/robotConfig.js';
import { forwardKinematics } from '../robotics/kinematics.js';
import { inverseKinematics } from '../robotics/inverseKinematics.js';
import {
  buildPickAndPlaceTrajectory,
  buildMoveToTargetTrajectory,
  smoothstep,
} from '../robotics/trajectory.js';

// Damping factor (0..1). Lower = slower, smoother approach.
const DAMPING = 0.12;

export function useRobotController() {
  // ---- Mutable state (updated in the render loop, not React state) ----
  const state = useRef({
    // Current joint angles (radians) - what is rendered.
    currentQ: HOME_POSE.slice(),
    // Target joint angles (radians) - what sliders/IK set.
    targetQ: HOME_POSE.slice(),
    // End effector position [x, y, z].
    eePosition: [0, 0, 0],
    // Gripper open (1) / closed (0).
    gripper: 1,
    // Whether a demo is running.
    demoRunning: false,
    // Demo elapsed time.
    demoElapsed: 0,
    // Current demo trajectory steps.
    demoSteps: [],
    // IK status message.
    ikStatus: '',
    ikReachable: true,
    // Target position inputs.
    targetPos: { x: 0.5, y: 0.0, z: 0.4 },
  });

  // Refs for the Three.js objects (set by RobotArm component).
  const refs = useRef({
    jointMeshes: [], // array of { rotation: Euler, axis: 'x'|'y'|'z' }
    gripperMeshes: [], // left/right finger meshes
    eeMarker: null,
  });

  // ---- Forward kinematics update ----
  const updateFK = useCallback((q) => {
    const { p } = forwardKinematics(q);
    state.current.eePosition = p;
  }, []);

  // ---- Apply smooth damping to joint angles ----
  const step = useCallback(() => {
    const s = state.current;
    let changed = false;

    for (let i = 0; i < 6; i++) {
      const diff = s.targetQ[i] - s.currentQ[i];
      if (Math.abs(diff) > 1e-4) {
        s.currentQ[i] += diff * DAMPING;
        changed = true;
      } else {
        s.currentQ[i] = s.targetQ[i];
      }
    }

    if (changed) {
      updateFK(s.currentQ);
    }

    return changed;
  }, [updateFK]);

  // ---- Get current joint angles (read by RobotArm each frame) ----
  const getJoints = useCallback(() => state.current.currentQ, []);

  // ---- Set a single joint target (from slider) ----
  const setJointTarget = useCallback((index, angleDeg) => {
    const angleRad = (angleDeg * Math.PI) / 180;
    const clamped = clampAllAngles(
      state.current.targetQ.map((v, i) => (i === index ? angleRad : v))
    );
    state.current.targetQ = clamped;
  }, []);

  // ---- Set all joint targets ----
  const setAllTargets = useCallback((q) => {
    state.current.targetQ = clampAllAngles([...q]);
  }, []);

  // ---- Reset to home ----
  const resetToHome = useCallback(() => {
    state.current.demoRunning = false;
    state.current.demoElapsed = 0;
    state.current.demoSteps = [];
    state.current.targetQ = HOME_POSE.slice();
    state.current.currentQ = HOME_POSE.slice();
    state.current.gripper = 1;
    state.current.ikStatus = '';
    state.current.ikReachable = true;
    updateFK(HOME_POSE);
  }, [updateFK]);

  // ---- Move to XYZ target (IK) ----
  const moveToTarget = useCallback(({ x, y, z }) => {
    state.current.targetPos = { x, y, z };
    const result = inverseKinematics([x, y, z], state.current.currentQ);

    if (!result.reachable) {
      state.current.ikStatus = 'Target is outside the robot workspace.';
      state.current.ikReachable = false;
      // Still move toward best-effort pose so nothing breaks.
      state.current.targetQ = result.q;
    } else {
      state.current.ikStatus = 'Target reached.';
      state.current.ikReachable = true;
      state.current.targetQ = result.q;
    }
  }, []);

  // ---- Pick & place demo ----
  const runDemo = useCallback((objectPos, placePos) => {
    state.current.demoRunning = true;
    state.current.demoElapsed = 0;
    state.current.ikStatus = 'Running pick & place demo...';

    const steps = buildPickAndPlaceTrajectory(objectPos, placePos, HOME_POSE);
    state.current.demoSteps = steps;
  }, []);

  // ---- Advance the demo ----
  const advanceDemo = useCallback((dt) => {
    const s = state.current;
    if (!s.demoRunning || s.demoSteps.length === 0) return;

    s.demoElapsed += dt;

    const currentStepIndex = s.demoSteps.findIndex(
      (_, i) => s.demoElapsed < s.demoSteps.slice(0, i + 1).reduce((sum, st) => sum + st.duration, 0)
    );

    if (currentStepIndex < 0) {
      // Demo finished.
      s.demoRunning = false;
      s.demoElapsed = 0;
      s.demoSteps = [];
      s.targetQ = HOME_POSE.slice();
      s.gripper = 1;
      s.ikStatus = 'Demo complete — returned to home.';
      updateFK(HOME_POSE);
      return;
    }

    // Compute elapsed within current step.
    let stepElapsed = s.demoElapsed;
    for (let i = 0; i < currentStepIndex; i++) {
      stepElapsed -= s.demoSteps[i].duration;
    }
    const step = s.demoSteps[currentStepIndex];
    const t = Math.min(stepElapsed / step.duration, 1);

    // Interpolate joints from previous target to current target.
    const fromQ = currentStepIndex === 0 ? HOME_POSE : s.demoSteps[currentStepIndex - 1].targetQ;
    const eased = smoothstep(t);
    const interpolatedQ = fromQ.map((v, i) => v + (step.targetQ[i] - v) * eased);
    s.targetQ = clampAllAngles(interpolatedQ);

    // Smoothly interpolate gripper open/close (1=open, 0=closed).
    const fromGrip =
      currentStepIndex === 0 ? 1 : (s.demoSteps[currentStepIndex - 1].gripper ?? 1);
    const gripTarget = step.gripper ?? 1;
    s.gripper = fromGrip + (gripTarget - fromGrip) * eased;
  }, []);

  // ---- Expose refs setter (called by RobotArm) ----
  const setRefs = useCallback((newRefs) => {
    refs.current = newRefs;
  }, []);

  return {
    state,
    refs,
    step,
    getJoints,
    setJointTarget,
    setAllTargets,
    resetToHome,
    moveToTarget,
    runDemo,
    advanceDemo,
    setRefs,
    DAMPING,
  };
}
