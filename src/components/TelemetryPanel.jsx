/**
 * TelemetryPanel.jsx
 *
 * Displays live forward-kinematics data:
 *   - Joint angles (radians and degrees)
 *   - End effector position (X, Y, Z in meters)
 *   - Reachability / workspace status
 */

import React from 'react';

const deg = (rad) => (rad * 180) / Math.PI;

export function TelemetryPanel({ controller }) {
  const q = controller.state.current.currentQ;
  const p = controller.state.current.eePosition;
  const reachable = controller.state.current.ikReachable;

  return (
    <div className="telemetry-panel">
      <div className="panel-header">
        <h2>Robot Telemetry</h2>
        <span className={`status-dot ${reachable ? 'ok' : 'warn'}`}></span>
        <span className="status-text">{reachable ? 'ONLINE' : 'TARGET UNREACHABLE'}</span>
      </div>

      <div className="telemetry-group">
        <h3>Joint Angles</h3>
        <div className="joint-readouts">
          {['j1', 'j2', 'j3', 'j4', 'j5', 'j6'].map((key, i) => (
            <div key={key} className="joint-readout">
              <span className="label">{key.toUpperCase()}</span>
              <span className="value">{deg(q[i]).toFixed(2)}°</span>
              <span className="value-rad">({q[i].toFixed(3)} rad)</span>
            </div>
          ))}
        </div>
      </div>

      <div className="telemetry-group">
        <h3>End Effector Position</h3>
        <div className="coord-readouts">
          <div className="coord-item">
            <span className="label">X</span>
            <span className="value">{p[0].toFixed(3)} m</span>
          </div>
          <div className="coord-item">
            <span className="label">Y</span>
            <span className="value">{p[1].toFixed(3)} m</span>
          </div>
          <div className="coord-item">
            <span className="label">Z</span>
            <span className="value">{p[2].toFixed(3)} m</span>
          </div>
        </div>
      </div>

      <div className="telemetry-group">
        <h3>Gripper</h3>
        <div className="gripper-readout">
          <span className="label">State</span>
          <span className={`value ${controller.state.current.gripper < 0.5 ? 'closed' : 'open'}`}>
            {controller.state.current.gripper < 0.5 ? 'CLOSED' : 'OPEN'}
          </span>
        </div>
      </div>

      <div className="telemetry-group">
        <h3>Workspace</h3>
        <div className="workspace-readout">
          <span className="label">Reachable</span>
          <span className={`value ${reachable ? 'yes' : 'no'}`}>
            {reachable ? 'Yes' : 'No'}
          </span>
        </div>
      </div>
    </div>
  );
}
