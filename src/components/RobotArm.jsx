/**
 * RobotArm.jsx
 *
 * Industrial 6-axis articulated robotic arm built with React Three Fiber.
 *
 * Proper connected hierarchy with physically correct pivot points:
 *
 * RobotRoot (y=0.05, on floor)
 * └── J1_Base Rotation (about Y)
 *     ├── BasePlatform (wide mounting plate)
 *     ├── BaseHousing (heavy cylindrical body)
 *     └── J2_Shoulder (about X)
 *         ├── ShoulderHousing (large mechanical casing)
 *         └── UpperArm (thick structural link)
 *             └── J3_Elbow (about X)
 *                 ├── ElbowHousing (large circular joint)
 *                 └── Forearm (proportional mechanical link)
 *                     └── J4_Wrist Roll (about Y)
 *                         ├── WristHousing (compact assembly)
 *                         └── J5_Wrist Pitch (about X)
 *                             └── J6_End Effector (about Z)
 *                                 ├── GripperMount
 *                                 ├── GripperFingerLeft (opens/closes about Z)
 *                                 └── GripperFingerRight (opens/closes about Z)
 *
 * All rotations are applied at the actual physical joint center.
 * Smooth damping is handled by useRobotController via useFrame.
 */

import React, { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

export function RobotArm({ controller }) {
  // Joint refs — rotated each frame by the controller.
  const j1Ref = useRef(); // Base rotation (Y)
  const j2Ref = useRef(); // Shoulder pitch (X)
  const j3Ref = useRef(); // Elbow pitch (X)
  const j4Ref = useRef(); // Wrist roll (Y)
  const j5Ref = useRef(); // Wrist tilt (X)
  const j6Ref = useRef(); // End effector rotation (Z)

  const fingerLRef = useRef();
  const fingerRRef = useRef();

  // Expose refs to the controller (for potential raycasting / physics later).
  useEffect(() => {
    if (controller.setRefs) {
      controller.setRefs({
        jointMeshes: [j1Ref, j2Ref, j3Ref, j4Ref, j5Ref, j6Ref],
        gripperMeshes: [fingerLRef, fingerRRef],
        eeMarker: null,
      });
    }
  }, [controller]);

  const getJoints = () => {
    const q = controller.getJoints ? controller.getJoints() : null;
    return q;
  };

  useFrame((_, delta) => {
    // Advance smooth damping toward target joint angles.
    controller.step();
    // Advance the pick & place demo trajectory if running.
    controller.advanceDemo(delta);

    const q = getJoints();
    if (!q) return;

    const gripper = controller.state.current.gripper ?? 1;

    // Apply joint rotations at physical joint centers.
    j1Ref.current.rotation.y = q[0];
    j2Ref.current.rotation.x = q[1];
    j3Ref.current.rotation.x = q[2];
    j4Ref.current.rotation.y = q[3];
    j5Ref.current.rotation.x = q[4];
    j6Ref.current.rotation.z = q[5];

    // Gripper open/close: fingers swing inward about Z.
    // gripper = 1 (open) → fingers apart; gripper = 0 (closed) → fingers together.
    const closeAmount = 1 - (gripper ?? 1);
    const fingerAngle = closeAmount * 0.5; // radians (~28° when closed)
    if (fingerLRef.current) fingerLRef.current.rotation.z = fingerAngle;
    if (fingerRRef.current) fingerRRef.current.rotation.z = -fingerAngle;
  });

  // ---- Materials ----
  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0xe2e8f0, // light industrial gray/off-white
    roughness: 0.35,
    metalness: 0.55,
  });
  const jointMat = new THREE.MeshStandardMaterial({
    color: 0x475569, // darker metallic gray for joints
    roughness: 0.25,
    metalness: 0.85,
  });
  const accentMat = new THREE.MeshStandardMaterial({
    color: 0x38bdf8, // subtle cyan accent on links
    roughness: 0.3,
    metalness: 0.5,
  });
  const gripperMat = new THREE.MeshStandardMaterial({
    color: 0x64748b, // dark gray gripper
    roughness: 0.3,
    metalness: 0.7,
  });

  return (
    // RobotRoot sits on the floor (y=0.05 so base clears ground).
    <group position={[0, 0.05, 0]}>
      {/* ================= J1: Base Rotation (about Y) ================= */}
      <group ref={j1Ref}>
        {/* Wide circular mounting platform */}
        <mesh castShadow receiveShadow>
          <cylinderGeometry args={[0.5, 0.5, 0.06, 48]} />
          <meshStandardMaterial color={0x1e293b} roughness={0.3} metalness={0.7} />
        </mesh>

        {/* Heavy base housing */}
        <mesh castShadow receiveShadow>
          <cylinderGeometry args={[0.28, 0.28, 0.5, 48]} />
          <meshStandardMaterial
            color={0x1e293b}
            roughness={0.3}
            metalness={0.7}
          />
        </mesh>

        {/* Top flange / rotating platform */}
        <mesh castShadow>
          <cylinderGeometry args={[0.32, 0.32, 0.08, 48]} />
          <meshStandardMaterial color={0x334155} roughness={0.25} metalness={0.8} />
        </mesh>

        {/* ================= J2: Shoulder Pitch (about X) ================= */}
        <group position={[0, 0.54, 0]} ref={j2Ref}>
          {/* Shoulder joint housing */}
          <mesh castShadow>
            <cylinderGeometry args={[0.14, 0.14, 0.28, 48]} />
            <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
          </mesh>

          {/* ================= Upper Arm ================= */}
          <group>
            {/* Thick structural upper arm link, extends along +Y */}
            <mesh castShadow>
              <cylinderGeometry args={[0.11, 0.11, 0.55, 32]} />
              <meshStandardMaterial
                color={0xe2e8f0}
                roughness={0.35}
                metalness={0.55}
              />
            </mesh>

            {/* Shoulder-to-upper-arm connector plate */}
            <mesh castShadow>
              <cylinderGeometry args={[0.13, 0.13, 0.1, 48]} />
              <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
            </mesh>

            {/* ================= J3: Elbow Pitch (about X) ================= */}
            <group position={[0, 0.55, 0]} ref={j3Ref}>
              {/* Large circular elbow joint housing */}
              <mesh castShadow>
                <cylinderGeometry args={[0.16, 0.16, 0.32, 48]} />
                <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
              </mesh>

              {/* Elbow cap details */}
              <mesh castShadow>
                <cylinderGeometry args={[0.175, 0.175, 0.06, 48]} />
                <meshStandardMaterial color={0x1e293b} roughness={0.3} metalness={0.7} />
              </mesh>

              {/* ================= Forearm ================= */}
              <group>
                <mesh castShadow>
                  <cylinderGeometry args={[0.09, 0.09, 0.45, 32]} />
                  <meshStandardMaterial
                    color={0xe2e8f0}
                    roughness={0.35}
                    metalness={0.55}
                  />
                </mesh>

                {/* Forearm connector plate */}
                <mesh castShadow>
                  <cylinderGeometry args={[0.11, 0.11, 0.08, 48]} />
                  <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
                </mesh>

                {/* ================= J4: Wrist Roll (about Y) ================= */}
                <group position={[0, 0.45, 0]} ref={j4Ref}>
                  {/* Wrist roll housing */}
                  <mesh castShadow>
                    <cylinderGeometry args={[0.11, 0.11, 0.22, 48]} />
                    <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
                  </mesh>

                  {/* ================= J5: Wrist Pitch (about X) ================= */}
                  <group ref={j5Ref}>
                    {/* Wrist pitch housing */}
                    <mesh castShadow>
                      <cylinderGeometry args={[0.09, 0.09, 0.18, 48]} />
                      <meshStandardMaterial color={0x475569} roughness={0.25} metalness={0.85} />
                    </mesh>

                    {/* ================= J6: End Effector (about Z) ================= */}
                    <group ref={j6Ref}>
                      {/* End effector mounting flange */}
                      <mesh castShadow>
                        <cylinderGeometry args={[0.08, 0.08, 0.12, 32]} />
                        <meshStandardMaterial color={0x64748b} roughness={0.3} metalness={0.7} />
                      </mesh>

                      {/* Gripper mount (extends downward) */}
                      <group position={[0, -0.13, 0]}>
                        <mesh castShadow>
                          <cylinderGeometry args={[0.04, 0.04, 0.1, 32]} />
                          <meshStandardMaterial color={0x64748b} roughness={0.3} metalness={0.7} />
                        </mesh>

                        {/* Gripper fingers — swing inward about Z */}
                        <mesh
                          ref={fingerLRef}
                          position={[-0.035, -0.1, 0]}
                          castShadow
                        >
                          <boxGeometry args={[0.02, 0.14, 0.04]} />
                          <meshStandardMaterial color={0x64748b} roughness={0.3} metalness={0.7} />
                        </mesh>
                        <mesh
                          ref={fingerRRef}
                          position={[0.035, -0.1, 0]}
                          castShadow
                        >
                          <boxGeometry args={[0.02, 0.14, 0.04]} />
                          <meshStandardMaterial color={0x64748b} roughness={0.3} metalness={0.7} />
                        </mesh>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}
