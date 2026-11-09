"""
trajectory.py
-------------
Perencanaan lintasan gerak halus (trajectory planning) untuk robot arm.

Fungsi utama:
  - interpolasi linear antara waypoint
  - interpolasi smoothstep (cepat di awal/akhir, lambat di tengah)
  - generate sejumlah frame supaya animasi mulus
"""

import math
from typing import List, Tuple


def smoothstep(t: float) -> float:
    """
    Fungsi smoothing: t*t*(3-2t).
    Hasil 0 saat t=0, 1 saat t=1, turunan nol di kedua ujung.
    Cocok untuk gerak yang memulai dan berhenti secara halus.
    """
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def lerp(a: float, b: float, t: float) -> float:
    """Interpolasi linear antara a dan b dengan parameter t [0,1]."""
    return a + (b - a) * t


def interpolate_joint_space(
    waypoints: List[List[float]],
    frames_per_segment: int = 60,
    easing: str = "smoothstep",
) -> List[List[float]]:
    """
    Menghasilkan urutan konfigurasi sudut (q) yang halus melalui waypoint.

    Parameters
    ----------
    waypoints : list of list[float]
        Daftar konfigurasi sudut, misal [[0,0,0], [pi/2,pi/4,0], ...].
    frames_per_segment : int
        Berapa frame di antar setiap dua waypoint berturut-turut.
    easing : str
        "linear" atau "smoothstep".

    Returns
    -------
    list of list[float]
        Urutan q yang siap digunakan untuk animasi.
    """
    if len(waypoints) < 2:
        return [list(w) for w in waypoints]

    trajectory: List[List[float]] = []
    n_joints = len(waypoints[0])

    for seg in range(len(waypoints) - 1):
        q_start = waypoints[seg]
        q_end = waypoints[seg + 1]

        for f in range(frames_per_segment + 1):
            t = f / frames_per_segment
            if easing == "smoothstep":
                ts = smoothstep(t)
            else:
                ts = t

            q_frame = [
                lerp(q_start[j], q_end[j], ts) for j in range(n_joints)
            ]
            trajectory.append(q_frame)

    return trajectory


def interpolate_cartesian_space(
    fk_func,
    waypoints_cartesian: List[Tuple[float, float, float]],
    q_init: List[float],
    frames_per_segment: int = 60,
    easing: str = "smoothstep",
) -> List[List[float]]:
    """
    Menghasilkan urutan q berdasarkan waypoint posisi [x,y,z] menggunakan
    Inverse Kinematics di setiap waypoint, lalu diisi dengan interpolasi.

    Parameter fk_func adalah objek RobotArm yang memiliki metode
    inverse_kinematics(target) dan end_effector_position(q).
    """
    # Hitung q untuk setiap waypoint Cartesian
    q_waypoints = []
    for target in waypoints_cartesian:
        q_solved, ok = fk_func.inverse_kinematics(list(target), q_init=q_init)
        q_waypoints.append(q_solved)
        q_init = q_solved  # gunakan hasil sebelumnya sebagai tebakan awal berikutnya

    return interpolate_joint_space(q_waypoints, frames_per_segment, easing)


def save_trajectory_csv(trajectory: List[List[float]], filename: str) -> None:
    """Menyimpan lintasan ke file CSV untuk analisis atau video eksternal."""
    with open(filename, "w") as f:
        f.write("frame,q1,q2,q3\n")
        for i, q in enumerate(trajectory):
            f.write(f"{i}," + ",".join(f"{v:.6f}" for v in q) + "\n")


# =====================================================================
# Contoh penggunaan
# =====================================================================
if __name__ == "__main__":
    # Contoh waypoint sudut (radian)
    waypoints = [
        [0.0, 0.0, 0.0],           # Awal
        [math.pi / 2, 0.0, 0.0],   # Titik 1: putar dasar 90°
        [math.pi / 2, math.pi / 4, 0.0],  # Titik 2: bahu 45°
        [0.0, math.pi / 4, math.pi / 3],  # Titik 3: kembali dasar, siku 60°
        [0.0, 0.0, 0.0],           # Kembali awal
    ]

    traj = interpolate_joint_space(waypoints, frames_per_segment=30, easing="smoothstep")
    print(f"Total frame lintasan: {len(traj)}")
    print(f"Frame pertama: {[round(v,3) for v in traj[0]]}")
    print(f"Frame tengah   : {[round(v,3) for v in traj[len(traj)//2]]}")
    print(f"Frame terakhir : {[round(v,3) for v in traj[-1]]}")

    save_trajectory_csv(traj, "trajectory_example.csv")
    print("Disimpan ke trajectory_example.csv")
