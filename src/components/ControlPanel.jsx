/**
 * ControlPanel.jsx
 *
 * Professional control panel outside the 3D canvas:
 *   - Joint sliders (J1..J6) with current angle display and limits
 *   - XYZ target position inputs + MOVE TO TARGET button (IK)
 *   - RUN PICK & PLACE DEMO button
 *   - Reset home button
 */

import React, { useState, useRef, useCallback } from 'react';
import {
  JOINT_LABELS,
  JOINT_LIMITS_DEG,
  DEFAULT_TARGET,
} from '../robotics/robotConfig.js';

export function ControlPanel({ controller }) {
  const [ikTarget, setIkTarget] = useState({ ...DEFAULT_TARGET });

  const inputRefs = useRef({ x: null, y: null, z: null });

  // Object and place positions for the demo (within workspace).
  const OBJECT_POS = [0.9, 0.49, 0.0]; // on the red cube
  const PLACE_POS = [-0.3, 0.49, 0.2]; // on the yellow placement marker

  const deg = (rad) => (rad * 180) / Math.PI;

  const handleJointChange = useCallback(
    (index, value) => {
      const v = parseFloat(value);
      if (Number.isNaN(v)) return;
      controller.setJointTarget(index, v);
    },
    [controller]
  );

  const handleReset = useCallback(() => {
    controller.resetToHome();
    setIkStatus('');
  }, [controller]);

  const handleMoveToTarget = useCallback(() => {
    const x = parseFloat(inputRefs.current.x.value);
    const y = parseFloat(inputRefs.current.y.value);
    const z = parseFloat(inputRefs.current.z.value);
    if (Number.isNaN(x) || Number.isNaN(y) || Number.isNaN(z)) {
      setIkStatus('Enter valid numeric values.');
      return;
    }
    setIkTarget({ x, y, z });
    controller.moveToTarget({ x, y, z });
  }, [controller]);

  const handleRunDemo = useCallback(() => {
    controller.runDemo(OBJECT_POS, PLACE_POS);
  }, [controller]);

  // Read controller status each render.
  const cIkStatus = controller.state.current.ikStatus;
  const cIkReachable = controller.state.current.ikReachable;
  const cDemoRunning = controller.state.current.demoRunning;

  return (
    <div className="control-panel">
      <div className="panel-section">
        <h2>Joint Control</h2>
        <div className="joint-controls">
          {['j1', 'j2', 'j3', 'j4', 'j5', 'j6'].map((key, i) => {
            const limits = JOINT_LIMITS_DEG[key];
            const q = controller.state.current.currentQ;
            const currentDeg = deg(q[i]);
            const targetDeg = deg(controller.state.current.targetQ[i]);
            return (
              <div key={key} className="joint-row">
                <div className="joint-label">
                  <span>{JOINT_LABELS[key]}</span>
                  <span className="joint-range">
                    [{limits.min}° … {limits.max}°]
                  </span>
                </div>
                <div className="slider-wrapper">
                  <input
                    type="range"
                    min={limits.min}
                    max={limits.max}
                    step="1"
                    value={targetDeg}
                    onChange={(e) => handleJointChange(i, e.target.value)}
                  />
                  <span className="current-angle">
                    Current: {currentDeg.toFixed(1)}°
                  </span>
                </div>
              </div>
            );
          })}
        </div>

        <div className="panel-actions">
          <button className="btn btn-secondary" onClick={handleReset}>
            ↺ Reset to Home
          </button>
        </div>
      </div>

      <div className="panel-section ik-section">
        <h2>XYZ Target Mode</h2>
        <p className="section-hint">
          Enter a target position for the end effector and press MOVE TO TARGET.
        </p>
        <div className="ik-inputs">
          <div className="ik-field">
            <label>X</label>
            <input
              ref={(el) => (inputRefs.current.x = el)}
              type="number"
              step="0.01"
              value={ikTarget.x}
              onChange={(e) => setIkTarget({ ...ikTarget, x: parseFloat(e.target.value) || 0 })}
            />
            <span className="unit">m</span>
          </div>
          <div className="ik-field">
            <label>Y</label>
            <input
              ref={(el) => (inputRefs.current.y = el)}
              type="number"
              step="0.01"
              value={ikTarget.y}
              onChange={(e) => setIkTarget({ ...ikTarget, y: parseFloat(e.target.value) || 0 })}
            />
            <span className="unit">m</span>
          </div>
          <div className="ik-field">
            <label>Z</label>
            <input
              ref={(el) => (inputRefs.current.z = el)}
              type="number"
              step="0.01"
              value={ikTarget.z}
              onChange={(e) => setIkTarget({ ...ikTarget, z: parseFloat(e.target.value) || 0 })}
            />
            <span className="unit">m</span>
          </div>
        </div>
        <div className="ik-actions">
          <button
            className={`btn btn-primary ${!cIkReachable ? 'btn-disabled' : ''}`}
            onClick={handleMoveToTarget}
            disabled={!cIkReachable}
          >
            MOVE TO TARGET
          </button>
        </div>
        <div className={`ik-status ${cIkReachable ? 'ok' : 'fail'}`}>
          {cIkStatus || 'Enter target and press MOVE TO TARGET'}
        </div>
      </div>

      <div className="panel-section demo-section">
        <h2>Automatic Demo</h2>
        <p className="section-hint">
          Smooth pick &amp; place sequence: home → approach → grasp → lift → place → release → home.
        </p>
        <div className="demo-actions">
          <button className="btn btn-demo" onClick={handleRunDemo}>
            ▶ RUN PICK & PLACE DEMO
          </button>
        </div>
        <div className={`demo-status ${cDemoRunning ? 'active' : 'idle'}`}>
          {cDemoRunning ? 'Demo in progress…' : 'Ready'}
        </div>
      </div>
    </div>
  );
}
