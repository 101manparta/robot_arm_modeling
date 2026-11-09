"""
Robot Arm 3D Visualization
==========================
Visualisasi 3D robot arm menggunakan Matplotlib (mplot3d).

Tampilkan:
- Link sebagai garis tebal 3D
- Sendi sebagai titik biru
- Frame koordinat di setiap sendi (X=merah, Y=hijau, Z=biru)
- End effector sebagai bintang merah
- Animasi gerak dari konfigurasi awal ke target

Persyaratan:
    pip install numpy matplotlib
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (daftar proyeksi 3D)
from matplotlib.animation import FuncAnimation


# =====================================================================
# Bagian 1: Matematika inti (sama dengan robot_arm.py)
# =====================================================================

def dh_matrix(a, alpha, d, theta):
    """Matriks transformasi homogen 4x4 dari parameter Denavit-Hartenberg."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ],
    ], dtype=float)


class DHLink:
    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta


class RobotArm:
    def __init__(self, links):
        self.links = links
        self.n = len(links)

    def forward_kinematics(self, q):
        T = np.eye(4)
        transforms = [T.copy()]
        for i, link in enumerate(self.links):
            Ti = dh_matrix(link.a, link.alpha, link.d, q[i])
            T = T @ Ti
            transforms.append(T.copy())
        return T, transforms

    def end_effector_position(self, q):
        T, _ = self.forward_kinematics(q)
        return T[:3, 3].copy()


# =====================================================================
# Bagian 2: Visualisasi 3D
# =====================================================================

class RobotArm3D:
    def __init__(self, robot, frame_scale=0.1):
        self.robot = robot
        self.frame_scale = frame_scale
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self._setup_axis()

    def _setup_axis(self):
        self.ax.set_xlabel('X (m)', fontsize=11)
        self.ax.set_ylabel('Y (m)', fontsize=11)
        self.ax.set_zlabel('Z (m)', fontsize=11)
        self.ax.set_title('Robot Arm 3D Modeling — Forward Kinematics', fontsize=13, weight='bold')
        self.ax.grid(True, alpha=0.3)

    def _joint_positions(self, q):
        """Kembalikan daftar asal setiap frame [p0, p1, ..., pn]."""
        _, transforms = self.robot.forward_kinematics(q)
        return [T[:3, 3] for T in transforms]

    def _frame_rotations(self, q):
        """Kembalikan matriks rotasi 3x3 tiap frame (kolom = sumbu x,y,z di dunia)."""
        _, transforms = self.robot.forward_kinematics(q)
        return [T[:3, :3] for T in transforms]

    def _draw_frame(self, p, R, scale, label=""):
        """Gambar triad koordinat (X merah, Y hijau, Z biru) di titik p."""
        colors = ['r', 'g', 'b']
        axis_names = ['X', 'Y', 'Z']
        for j in range(3):
            axis = R[:, j] * scale
            tip = p + axis
            self.ax.quiver(
                p[0], p[1], p[2],
                axis[0], axis[1], axis[2],
                color=colors[j], length=scale, normalize=True,
                arrow_length_ratio=0.25, linewidth=2, alpha=0.9
            )
            self.ax.text(tip[0], tip[1], tip[2], axis_names[j],
                         color=colors[j], fontsize=9, weight='bold')
        if label:
            self.ax.text(p[0], p[1], p[2] + 0.02, label,
                         color='k', fontsize=10, weight='bold')

    def draw(self, q, show_frames=True, show_end_effector=True, save_path=None):
        """Gambar robot arm pada konfigurasi q."""
        positions = self._joint_positions(q)
        rotations = self._frame_rotations(q)

        self.ax.cla()
        self._setup_axis()

        # Gambar link (garis antara sendi)
        for i in range(1, len(positions)):
            p_prev = positions[i - 1]
            p_curr = positions[i]
            self.ax.plot(
                [p_prev[0], p_curr[0]],
                [p_prev[1], p_curr[1]],
                [p_prev[2], p_curr[2]],
                'k-', linewidth=5, solid_capstyle='round',
                label='Link' if i == 1 else ""
            )
            # Tambahkan efek "silinder" tipis dengan garis di bawah
            self.ax.plot(
                [p_prev[0], p_curr[0]],
                [p_prev[1], p_curr[1]],
                [p_prev[2], p_curr[2]],
                'gray', linewidth=1.5, alpha=0.4
            )

        # Gambar semua sendi
        pos_arr = np.array(positions)
        self.ax.scatter(
            pos_arr[:, 0], pos_arr[:, 1], pos_arr[:, 2],
            c='steelblue', s=100, edgecolor='k', linewidth=1.2,
            label='Joint', depthshade=True, zorder=5
        )

        # Gambar frame koordinat tiap sendi
        if show_frames:
            for i, (p, R) in enumerate(zip(positions, rotations)):
                self._draw_frame(p, R, scale=self.frame_scale,
                                 label=f'Frame {i}' if i == 0 else "")

        # Highlight end effector
        if show_end_effector:
            p_ee = positions[-1]
            self.ax.scatter(
                [p_ee[0]], [p_ee[1]], [p_ee[2]],
                c='red', s=220, marker='*', edgecolor='k', linewidth=1.2,
                label='End Effector', zorder=6
            )
            self.ax.text(p_ee[0], p_ee[1], p_ee[2] + 0.03, 'EE',
                         color='red', fontsize=10, weight='bold')

        # Batasi sumbu sesuai jangkauan robot
        reach = sum(link.a for link in self.robot.links) + \
                max(link.d for link in self.robot.links) + 0.4
        self.ax.set_xlim([-reach, reach])
        self.ax.set_ylim([-reach, reach])
        self.ax.set_zlim([-reach, reach])

        self.ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=9)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Gambar disimpan ke: {save_path}")

        plt.show()

    def animate(self, q_start, q_target, steps=80, interval=50, save_path=None):
        """
        Animasi gerak dari q_start ke q_target dengan interpolasi smoothstep.
        """
        def smoothstep(t):
            return t * t * (3 - 2 * t)

        def update(frame):
            t = frame / (steps - 1)
            ts = smoothstep(t)
            q = np.array(q_start) + ts * (np.array(q_target) - np.array(q_start))
            self.draw(q, show_frames=True, show_end_effector=True)
            self.ax.set_title(
                f'Robot Arm Animation — Step {frame + 1}/{steps} '
                f'(q = {[round(v, 2) for v in q]})',
                fontsize=12, weight='bold'
            )

        anim = FuncAnimation(self.fig, update, frames=steps,
                             interval=interval, repeat=True, blit=False)

        if save_path:
            print(f"Menyimpan animasi ke {save_path} ...")
            anim.save(save_path, writer='pillow', fps=1000 // interval)
            print("Selesai menyimpan animasi.")
        else:
            plt.show()

        return anim


# =====================================================================
# Contoh penggunaan
# =====================================================================

if __name__ == "__main__":
    # Robot 3-DOF: DHLink(a, alpha, d, theta)
    links = [
        DHLink(a=0.0, alpha=-math.pi / 2, d=0.30, theta=0.0),  # Sendi 1: Putar dasar
        DHLink(a=0.40, alpha=0.0,       d=0.00, theta=0.0),  # Sendi 2: Bahu
        DHLink(a=0.30, alpha=0.0,       d=0.00, theta=0.0),  # Sendi 3: Siku
    ]

    robot = RobotArm(links)
    viz = RobotArm3D(robot, frame_scale=0.12)

    # Konfigurasi awal dan target
    q_start = [0.0, 0.0, 0.0]
    q_target = [math.pi / 2, math.pi / 4, math.pi / 3]  # 90°, 45°, 60°

    print("=" * 60)
    print("ROBOT ARM 3D VISUALISASI")
    print("=" * 60)
    print(f"q_start (rad): {q_start}")
    print(f"q_target (rad): {[round(v, 4) for v in q_target]}")
    print(f"q_target (deg): {[round(math.degrees(v), 1) for v in q_target]}")

    # Posisi end effector di akhir gerak
    p_ee = robot.end_effector_position(q_target)
    print(f"End effector di q_target: [{p_ee[0]:.4f}, {p_ee[1]:.4f}, {p_ee[2]:.4f}]")

    # 1. Plot statis konfigurasi target
    print("\nMenampilkan plot statis ...")
    viz.draw(q_target)

    # 2. Animasi gerak dari awal ke target
    # Komentar baris berikut jika tidak ingin animasi
    print("Menjalankan animasi ...")
    viz.animate(q_start, q_target, steps=80, interval=40)
