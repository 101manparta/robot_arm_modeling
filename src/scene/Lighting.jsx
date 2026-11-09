/**
 * Lighting.jsx
 *
 * Balanced three-point-style lighting for the robot simulation:
 *   1. Ambient light — base visibility
 *   2. Key directional light — main illumination with shadows
 *   3. Fill light — softens shadows
 *   4. Rim light — separates robot from background
 */

import * as THREE from 'three';

export function Lighting() {
  return (
    <>
      {/* Ambient: soft base fill so nothing is fully black. */}
      <ambientLight intensity={0.45} />

      {/* Key directional light — main illumination with soft shadows. */}
      <directionalLight
        position={[5, 8, 5]}
        intensity={1.3}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-near={0.5}
        shadow-camera-far={30}
        shadow-camera-left={-10}
        shadow-camera-right={10}
        shadow-camera-top={10}
        shadow-camera-bottom={-10}
        shadow-bias={-0.0005}
      />

      {/* Fill light — softens shadows from the opposite side. */}
      <directionalLight
        position={[-5, 3, -5]}
        intensity={0.4}
        color={0xbbd4ff}
      />

      {/* Rim/accent light — separates robot edges from background. */}
      <directionalLight
        position={[0, 5, -8]}
        intensity={0.6}
        color={0x88ccff}
      />
    </>
  );
}
