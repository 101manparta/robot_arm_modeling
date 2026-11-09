# 🤖 Robot Arm Modeling & Simulation

> A modern, browser-based 6-axis industrial robotic arm simulator built with **React**, **Vite**, **Three.js**, **React Three Fiber**, **Drei**, and **React Three Rapier**. Features smooth joint interpolation, inverse kinematics (XYZ target mode), pick & place automation, forward kinematics telemetry, and rigid-body physics — all in a professional dark industrial UI.

[![React](https://img.shields.io/badge/React-19.3-61DAFB?style=for-the-badge&logo=react&logoColor=black)](https://react.dev/)
[![Vite](https://img.shields.io/badge/Vite-8.3-646CFF?style=for-the-badge&logo=vite&logoColor=white)](https://vite.dev/)
[![Three.js](https://img.shields.io/badge/Three.js-0.187-000000?style=for-the-badge&logo=three.js&logoColor=white)](https://threejs.org/)
[![React Three Fiber](https://img.shields.io/badge/R3F-9.8-000000?style=for-the-badge)](https://docs.pmnd.rs/react-three-fiber/)
[![React Three Drei](https://img.shields.io/badge/Drei-10.7-000000?style=for-the-badge)](https://docs.pmnd.rs/react-three-drei/)
[![React Three Rapier](https://img.shields.io/badge/Rapier-2.2-000000?style=for-the-badge)](https://github.com/pmndrs/react-three-rapier/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](LICENSE)

---

## 📋 Table of Contents

- [🎯 Overview](#-overview)
- [✨ Features](#-features)
- [🛠️ Tech Stack](#️-tech-stack)
- [📁 Project Structure](#-project-structure)
- [🚀 Getting Started](#-getting-started)
- [🎮 Controls & Usage](#-controls--usage)
- [🧠 How It Works](#-how-it-works)
- [🎨 UI Design](#-ui-design)
- [📊 Roadmap](#-roadmap)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

---

## 🎯 Overview

The **Robot Arm Simulator** is a realistic, smooth, interactive 3D industrial robotic arm simulation running entirely in the browser. It models a proper 6-axis articulated arm hierarchy with physically correct pivot points, smooth motor-like damping, inverse kinematics for XYZ targeting, and a pick & place automation demo — all presented through a premium dark engineering interface.

The robot is built purely from Three.js primitives (cylinders, spheres, boxes) grouped in a connected hierarchy, ensuring every part moves together naturally without floating or disconnected geometry.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🦾 **6-Axis Articulated Arm** | Base → Shoulder → Elbow → Forearm → Wrist → Gripper, all physically connected |
| 🎛️ **Joint Control Panel** | Individual sliders (J1–J6) with angle readouts and joint limits |
| 🎯 **XYZ Target Mode** | Inverse kinematics — enter X/Y/Z and move the end effector to a target |
| 📦 **Pick & Place Demo** | Automated sequence: home → approach → grasp → lift → place → release → home |
| 📡 **Forward Kinematics Telemetry** | Live joint angles (deg + rad) and end effector X/Y/Z position |
| 🎥 **Orbit Camera** | Orbit, zoom, pan, and reset camera controls |
| ⚡ **Smooth Damping** | Exponential interpolation — no instant jumps, realistic motor behavior |
| 🔒 **Joint Limits** | Each joint clamped to realistic physical limits |
| 🏭 **Industrial Environment** | Dark floor, work table, mounting platform, pick object, drop target |
| 💡 **Professional Lighting** | Ambient + key + fill + rim lighting with soft shadows |
| 🔧 **Rigid Body Physics** | Rapier physics for table objects (gravity, collision, resting contact) |
| 🌙 **Dark Industrial UI** | Glassmorphism sidebar, clean typography, premium engineering aesthetic |

---

## 🛠️ Tech Stack

<table>
  <tr>
    <td><b>Category</b></td>
    <td><b>Technology</b></td>
    <td><b>Purpose</b></td>
  </tr>
  <tr>
    <td>Framework</td>
    <td><a href="https://react.dev/"><img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/react/react-original-wordmark.svg" width="24" height="24" alt="React"/> React 19.3</a></td>
    <td>UI layer</td>
  </tr>
  <tr>
    <td>Build Tool</td>
    <td><a href="https://vite.dev/"><img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/vitejs/vitejs-original.svg" width="24" height="24" alt="Vite"/> Vite</a></td>
    <td>Dev server & bundling</td>
  </tr>
  <tr>
    <td>3D Engine</td>
    <td><a href="https://threejs.org/"><img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/threejs/threejs-original.svg" width="24" height="24" alt="Three.js"/> Three.js 0.187</a></td>
    <td>WebGL rendering</td>
  </tr>
  <tr>
    <td>3D React</td>
    <td><a href="https://docs.pmnd.rs/react-three-fiber/"><b>React Three Fiber</b></a></td>
    <td>Three.js + React bridge</td>
  </tr>
  <tr>
    <td>3D Helpers</td>
    <td><a href="https://docs.pmnd.rs/react-three-drei/"><b>React Three Drei</b></a></td>
    <td>OrbitControls, helpers, abstractions</td>
  </tr>
  <tr>
    <td>Physics</td>
    <td><a href="https://github.com/pmndrs/react-three-rapier"><b>React Three Rapier</b></a></td>
    <td>Rigid-body physics (Rapier)</td>
  </tr>
  <tr>
    <td>Styling</td>
    <td>CSS3 (Dark industrial theme)</td>
    <td>Premium engineering UI</td>
  </tr>
</table>

---

## 📁 Project Structure

```
robot_arm_modeling/
├── public/
│   ├── favicon.svg
│   └── icons.svg
├── src/
│   ├── assets/                 # Static assets (logos, images)
│   ├── components/             # React UI components
│   │   ├── ControlPanel.jsx    # Joint sliders, IK, demo controls
│   │   ├── RobotArm.jsx        # Hierarchical 3D robot arm
│   │   ├── SimulationControls.jsx # Camera controls
│   │   └── TelemetryPanel.jsx  # FK readouts & telemetry
│   ├── hooks/
│   │   └── useRobotController.js # Central robot state & damping
│   ├── robotics/               # Kinematics & trajectory math
│   │   ├── kinematics.js       # Forward kinematics (DH transforms)
│   │   ├── inverseKinematics.js # IK solver + reachability
│   │   ├── robotConfig.js      # DH params, joint limits, links
│   │   └── trajectory.js       # Smoothstep trajectories (pick & place)
│   ├── scene/                  # 3D scene elements
│   │   ├── Environment.jsx     # Floor, grid, table, platform
│   │   ├── Lighting.jsx        # Ambient, key, fill, rim lights
│   │   └── SimulationObjects.jsx # Pick object & drop target
│   ├── App.jsx                 # Main layout (sidebar + viewport)
│   ├── index.css               # Dark industrial styles
│   └── main.jsx                # Entry point
├── cpp/                        # C++ kinematics implementation
├── python/                     # Python kinematics implementation
├── simulation/                 # Legacy standalone HTML simulation
├── index.html                  # Vite entry HTML
├── vite.config.js              # Vite configuration
├── package.json                # Dependencies & scripts
└── README.md                   # This file
```

---

## 🚀 Getting Started

### Prerequisites

- [Node.js](https://nodejs.org/) (v18 or higher)
- [npm](https://www.npmjs.com/) (comes with Node.js)

### Installation

```bash
# Clone the repository
git clone https://github.com/101manparta/robot_arm_modeling.git

cd robot_arm_modeling

# Install dependencies
npm install

# Start the development server
npm run dev

# Open http://localhost:5173 in your browser
```

### Available Scripts

| Script | Description |
|--------|-------------|
| `npm run dev` | Start dev server with hot reload |
| `npm run build` | Production build (output to `dist/`) |
| `npm run preview` | Preview production build locally |
| `npm run lint` | Run ESLint/oxlint |

---

## 🎮 Controls & Usage

### Joint Control (Left Panel)

Each joint has a slider with:
- **Range limits** (e.g., J1: −180° to 180°)
- **Current angle** display (updates live)
- **Smooth interpolation** — drag the slider and watch the arm move naturally

```
J1 – Base Rotation     [−180° … 180°]
J2 – Shoulder          [−90° … 90°]
J3 – Elbow             [−140° … 10°]
J4 – Wrist Roll        [−180° … 180°]
J5 – Wrist Pitch       [−90° … 90°]
J6 – Gripper Rotation  [−90° … 90°]
```

### XYZ Target Mode (Inverse Kinematics)

1. Enter target **X**, **Y**, **Z** coordinates (in meters)
2. Click **MOVE TO TARGET**
3. The arm computes IK and moves its end effector toward the target
4. If unreachable: *"Target is outside the robot workspace."*

### Automatic Pick & Place Demo

Click **▶ RUN PICK & PLACE DEMO** for a smooth automated sequence:

1. 🏠 Move to home position
2. 📍 Approach pick object
3. ⬇️ Lower toward object
4. 🤏 Close gripper (grasp)
5. ⬆️ Lift object
6. 📦 Move toward drop target
7. ⬇️ Lower object
8. 👐 Open gripper (release)
9. ⬆️ Lift after placement
10. 🏠 Return to home

### Camera Controls

| Action | Control |
|--------|---------|
| Orbit | Left-click + drag |
| Pan | Right-click + drag |
| Zoom | Scroll wheel |
| Reset | Click **Reset Camera** button |

---

## 🧠 How It Works

### Robot Hierarchy

```
RobotRoot (y=0.05, on floor)
└── J1: Base Rotation (Y-axis)
    ├── Base Platform (wide mounting plate)
    ├── Base Housing (heavy cylindrical body)
    └── J2: Shoulder Pitch (X-axis)
        ├── Shoulder Housing
        └── Upper Arm (thick structural link)
            └── J3: Elbow Pitch (X-axis)
                ├── Elbow Housing
                └── Forearm (mechanical link)
                    └── J4: Wrist Roll (Y-axis)
                        ├── Wrist Housing
                        └── J5: Wrist Pitch (X-axis)
                            └── J6: End Effector (Z-axis)
                                ├── Gripper Mount
                                ├── Finger Left (swings about Z)
                                └── Finger Right (swings about Z)
```

### Smooth Movement (Damping)

Instead of snapping to target angles, the controller uses exponential interpolation:

```
currentAngle += (targetAngle - currentAngle) × dampingFactor
```

With `dampingFactor = 0.12` per frame, joints accelerate smoothly toward targets and decelerate as they approach — mimicking real servo motor behavior.

### Forward Kinematics

Uses Denavit-Hartenberg (DH) homogeneous transformation matrices to compute each frame's transform from the base to the end effector. The end effector position `[x, y, z]` is extracted from the final matrix and displayed in the telemetry panel.

### Inverse Kinematics

A geometric decomposed IK solver:
1. **J1** — Base yaw points toward the target's XY projection
2. **J2 & J3** — 2-link law-of-cosines solution in the radial-vertical plane
3. **J4 & J5** — Wrist orientation compensated to keep end effector level
4. **J6** — Gripper rotation set to neutral

Reachability is checked against the arm's max/min reach sphere before solving.

---

## 🎨 UI Design

The interface follows a **dark industrial engineering** aesthetic:

- **Dark background** (`#050a14`) with glassmorphism sidebar
- **Cyan accent** (`#38bdf8`) for primary actions and highlights
- **Typography**: Clean system-ui font stack, tabular numbers for readouts
- **Layout**: Top status bar | Left control panel (400px) | Right 3D viewport (flexible)
- **No fake futuristic decorations** — minimal, premium, professional

---

## 📊 Roadmap

- [x] 6-axis articulated robot arm with proper hierarchy
- [x] Smooth damped joint movement
- [x] Forward kinematics telemetry
- [x] Inverse kinematics (XYZ target mode)
- [x] Pick & place automation demo
- [x] Rigid-body physics for table objects
- [x] Professional dark industrial UI
- [ ] Real GLB/GLTF model import option
- [ ] Collision detection between arm and environment
- [ ] Joint trajectory recording & playback
- [ ] Export joint angles to CSV

---

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and submission process.

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- [React Three Fiber](https://docs.pmnd.rs/react-three-fiber/) — React renderer for Three.js
- [React Three Drei](https://docs.pmnd.rs/react-three-drei/) — Useful helpers for R3F
- [React Three Rapier](https://github.com/pmndrs/react-three-rapier) — Physics integration
- [Three.js](https://threejs.org/) — Core 3D engine
- [Vite](https://vite.dev/) — Fast build tool and dev server

---

<div align="center">
  <b>Built with ⚡ React · Three.js · Vite</b>
</div>
