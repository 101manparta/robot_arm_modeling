/**
 * App.jsx
 *
 * Professional robotic arm simulator layout:
 *   - Top bar: title + status
 *   - Left: control panel (joint sliders, IK, demo)
 *   - Right: Three.js Canvas (simulation viewport)
 *
 * Canvas rendering pipeline is confirmed working.
 */

import React, { useRef, useCallback } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';

import { RobotArm } from './components/RobotArm';
import { ControlPanel } from './components/ControlPanel';
import { TelemetryPanel } from './components/TelemetryPanel';
import { SimulationControls } from './components/SimulationControls';
import { Environment } from './scene/Environment';
import { Lighting } from './scene/Lighting';
import { SimulationObjects } from './scene/SimulationObjects';
import { useRobotController } from './hooks/useRobotController';

export default function App() {
  const controller = useRobotController();

  const cameraRef = useRef();
  const orbitRef = useRef();

  const onResetCamera = useCallback(() => {
    if (cameraRef.current) {
      cameraRef.current.position.set(4, 3, 6);
      cameraRef.current.lookAt(0, 1, 0);
    }
    if (orbitRef.current) {
      orbitRef.current.target.set(0, 1, 0);
    }
  }, []);

  return (
    <div className="app">
      <header className="app-header">
        <div className="app-title">ROBOT ARM SIMULATOR</div>
        <div className="app-status">
          STATUS <span className="status-dot"></span> ONLINE
        </div>
      </header>

      <div className="main-body">
        {/* Left sidebar: controls + telemetry */}
        <aside className="sidebar">
          <ControlPanel controller={controller} />
          <TelemetryPanel controller={controller} />
          <SimulationControls onResetCamera={onResetCamera} />
        </aside>

        {/* Right: 3D simulation viewport */}
        <section className="viewport">
          <div className="canvas-wrapper">
            <Canvas
              shadows
              camera={{ position: [1.6, 1.3, 2.1], fov: 45 }}
              gl={{ antialias: true, alpha: true, preserveDrawingBuffer: true }}
            >
              <PerspectiveCamera ref={cameraRef} position={[1.6, 1.3, 2.1]} makeDefault />
              <OrbitControls
                ref={orbitRef}
                enableDamping
                dampingFactor={0.05}
                maxPolarAngle={Math.PI / 2 - 0.02}
                minDistance={0.8}
                maxDistance={10}
              />

              <Lighting />
              <Environment />
              <RobotArm controller={controller} />
              <SimulationObjects />
            </Canvas>
          </div>
        </section>
      </div>
    </div>
  );
}
