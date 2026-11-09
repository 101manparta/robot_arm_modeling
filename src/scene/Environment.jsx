/**
 * Environment.jsx
 *
 * Minimal industrial simulation environment:
 *   - Dark neutral floor
 *   - Subtle grid
 *   - Compact work table
 *   - Clean robot mounting platform
 *
 * Supports the robot visually without competing with it.
 */

import * as THREE from 'three';

export function Environment() {
  return (
    <>
      {/* ---- Dark neutral floor ---- */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
        <planeGeometry args={[30, 30]} />
        <meshStandardMaterial color={0x0f172a} roughness={0.85} metalness={0.15} />
      </mesh>

      {/* ---- Subtle grid on the floor ---- */}
      <gridHelper
        args={[24, 48, 0x1e293b, 0x0f172a]}
      />

      {/* ---- Robot mounting platform (clean, no pillars) ---- */}
      <mesh position={[0, 0.05, 0]} receiveShadow>
        <cylinderGeometry args={[0.6, 0.6, 0.1, 48]} />
        <meshStandardMaterial color={0x1e293b} roughness={0.4} metalness={0.6} />
      </mesh>

      {/* ---- Compact work table ---- */}
      <group name="workTable">
        {/* Tabletop */}
        <mesh position={[1.4, 0.43, 0]} castShadow receiveShadow>
          <boxGeometry args={[1.4, 0.05, 0.9]} />
          <meshStandardMaterial color={0x1e293b} roughness={0.6} metalness={0.4} />
        </mesh>

        {/* Table legs */}
        {[
          [0.8, 0.2, -0.35],
          [2.0, 0.2, -0.35],
          [0.8, 0.2, 0.35],
          [2.0, 0.2, 0.35],
        ].map((pos, i) => (
          <mesh
            key={i}
            position={pos}
            castShadow
            receiveShadow
          >
            <boxGeometry args={[0.04, 0.38, 0.04]} />
            <meshStandardMaterial color={0x0f172a} roughness={0.8} />
          </mesh>
        ))}
      </group>
    </>
  );
}
