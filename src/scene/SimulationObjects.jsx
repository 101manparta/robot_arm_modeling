/**
 * SimulationObjects.jsx
 *
 * Clean pick object and target marker on the work table.
 * Neutral industrial colors — no random bright placeholder shapes.
 */

import * as THREE from 'three';

const boxGeo = new THREE.BoxGeometry(0.1, 0.1, 0.1);

const pickObjMat = new THREE.MeshStandardMaterial({
  color: 0x94a3b8,
  roughness: 0.4,
  metalness: 0.6,
});

const targetMat = new THREE.MeshStandardMaterial({
  color: 0xfbbf24,
  roughness: 0.3,
  metalness: 0.4,
  emissive: 0xfbbf24,
  emissiveIntensity: 0.1,
});

export function SimulationObjects() {
  return (
    <>
      {/* Pick object — neutral gray cube on the work table */}
      <mesh position={[1.4, 0.48, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.1, 0.1, 0.1]} />
        <meshStandardMaterial color={0x94a3b8} roughness={0.4} metalness={0.6} />
      </mesh>

      {/* Drop target — subtle amber marker */}
      <mesh position={[2.0, 0.48, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.08, 0.02, 0.08]} />
        <meshStandardMaterial color={0xfbbf24} roughness={0.3} metalness={0.4} />
      </mesh>
    </>
  );
}
