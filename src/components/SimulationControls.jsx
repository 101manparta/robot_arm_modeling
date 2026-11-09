/**
 * SimulationControls.jsx
 *
 * Camera and simulation-level controls:
 *   - Reset camera to professional default angle
 *   - Toggle grid / helpers
 */

import React from 'react';
import { useRef } from 'react';

export function SimulationControls({ onResetCamera }) {
  return (
    <div className="sim-controls">
      <h2>Camera</h2>
      <div className="camera-actions">
        <button className="btn btn-secondary" onClick={onResetCamera}>
          ↺ Reset Camera
        </button>
      </div>
      <div className="camera-hints">
        <span>• Left drag: orbit</span>
        <span>• Right drag: pan</span>
        <span>• Scroll: zoom</span>
      </div>
    </div>
  );
}
