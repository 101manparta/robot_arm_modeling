"""
Robot Arm Modeling - Forward Kinematics + Calculus (Jacobian)
==============================================================
Pendekatan: Matriks Transformasi Homogen + Parameter Denavit-Hartenberg (DH)
Bahasa: Python (NumPy)

Fungsi utama:
  - forward_kinematics(q)   -> matriks T_0n dan transformasi antar frame
  - end_effector_position(q)-> posisi [x, y, z] ujung efektif
  - jacobian(q)             -> matriks Jacobian analitis 6 x n
  - jacobian_numerical(q)   -> Jacobian numerik via finite difference (turunan)
"""

import numpy as np


class DHLink:
    """Satu link robot dengan parameter Denavit-Hartenberg."""
    def __init__(self, a: float, alpha: float, d: float, theta: float):
        self.a = a       # panjang link
        self.alpha = alpha  # geseran sudut sumbu (twist)
        self.d = d       # offset link
        self.theta = theta  # sudut sendi (diganti dengan q saat perhitungan)


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Matriks transformasi homogen 4x4 dari parameter DH.

    T = [ R  t ]
        [ 0  1 ]
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ]
    ], dtype=float)


class RobotArm:
    def __init__(self, links: list[DHLink]):
        self.links = links
        self.n = len(links)

    def forward_kinematics(self, q: np.ndarray):
        """
        Hitung transformasi dari frame dasar (0) ke ujung efektif (n).

        Returns
        -------
        T_final : np.ndarray (4,4)
            Matriks transformasi total T_0n.
        transforms : list[np.ndarray]
            Daftar matriks transformasi setiap frame [T_0, T_01, ..., T_0n].
        """
        T = np.eye(4)
        transforms = [T.copy()]

        for i, link in enumerate(self.links):
            Ti = dh_matrix(link.a, link.alpha, link.d, q[i])
            T = T @ Ti
            transforms.append(T.copy())

        return T, transforms

    def end_effector_position(self, q: np.ndarray) -> np.ndarray:
        """Posisi ujung efektif [x, y, z] dalam frame dasar."""
        T, _ = self.forward_kinematics(q)
        return T[:3, 3].copy()

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Matriks Jacobian analitis 6 x n.

        v = [ v_lin ; omega ] = J(q) * q_dot
        """
        _, transforms = self.forward_kinematics(q)
        p_n = transforms[-1][:3, 3]          # posisi ujung efektif

        J = np.zeros((6, self.n), dtype=float)
        for i in range(self.n):
            T_i = transforms[i]
            z_i = T_i[:3, 2]     # sumbu z frame i (sumbu rotasi sendi i+1)
            p_i = T_i[:3, 3]     # asal frame i

            # Bagian linear: z x (p_n - p_i)
            J[:3, i] = np.cross(z_i, p_n - p_i)
            # Bagian rotasi: z
            J[3:, i] = z_i

        return J

    def jacobian_numerical(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """
        Jacobian numerik menggunakan kalkulus (turunan parsial via finite difference).

        J[:, i] = (p(q + eps*e_i) - p(q)) / eps
        """
        J = np.zeros((3, self.n), dtype=float)
        p0 = self.end_effector_position(q)

        for i in range(self.n):
            dq = np.zeros_like(q)
            dq[i] = eps
            p1 = self.end_effector_position(q + dq)
            J[:, i] = (p1 - p0) / eps

        return J

    # -----------------------------------------------------------------
    # INVERSE KINEMATICS (Jacobian pseudoinverse + multi-start)
    # -----------------------------------------------------------------
    def inverse_kinematics(self, target, q_init=None, max_iter=100, tol=1e-4,
                           damping=1e-6):
        """
        Mencari sudut q sehingga end effector mendekati target [x, y, z].

        Metode: Jacobian pseudoinverse (Levenberg-Marquardt sederhana)
        q_{k+1} = q_k + (J^T J + lambda I)^{-1} J^T * error
        """
        q = np.array(q_init, dtype=float) if q_init is not None else \
            np.zeros(self.n, dtype=float)

        def solve_from(q0):
            q = q0.copy()
            for _ in range(max_iter):
                p = self.end_effector_position(q)
                error = np.array(target, dtype=float) - p
                err_norm = np.linalg.norm(error)
                if err_norm < tol:
                    return q, True, err_norm

                J = self.jacobian(q)[:3, :]  # bagian linear 3 x n
                JT = J.T
                JTJ = JT @ J
                JTJ[np.diag_indices_from(JTJ)] += damping
                JT_err = JT @ error

                try:
                    JTJ_inv = np.linalg.inv(JTJ)
                except np.linalg.LinAlgError:
                    return q, False, err_norm

                dq = JTJ_inv @ JT_err
                q = q + dq
                q = np.clip(q, -np.pi, np.pi)

            p = self.end_effector_position(q)
            error = np.array(target, dtype=float) - p
            return q, False, float(np.linalg.norm(error))

        if q_init is not None:
            q, ok, _ = solve_from(q)
            return q.tolist(), ok

        candidates = [
            np.zeros(self.n),
            np.array([0.0, np.pi / 4, 0.0]),
            np.array([0.0, -np.pi / 4, 0.0]),
            np.array([np.pi / 2, 0.0, 0.0]),
            np.array([-np.pi / 2, 0.0, 0.0]),
            np.array([0.0, np.pi / 2, 0.0]),
            np.array([0.0, 0.0, np.pi / 2]),
        ]
        best_q, best_ok, best_err = None, False, float('inf')
        for c in candidates:
            q, ok, err = solve_from(c)
            if err < best_err:
                best_err, best_q, best_ok = err, q, ok
            if ok:
                break
        return best_q.tolist(), best_ok


# =====================================================================
# Contoh penggunaan: Robot Arm 3-DOF
# =====================================================================
if __name__ == "__main__":
    # Definisi link: DHLink(a, alpha, d, theta)
    links = [
        DHLink(a=0.0, alpha=-np.pi / 2, d=0.30, theta=0.0),  # Sendi 1: Putar dasar
        DHLink(a=0.40, alpha=0.0,       d=0.00, theta=0.0),  # Sendi 2: Bahu
        DHLink(a=0.30, alpha=0.0,       d=0.00, theta=0.0),  # Sendi 3: Siku
    ]

    robot = RobotArm(links)

    # Sudut sendi dalam radian (contoh gerak awal)
    q = np.array([np.pi / 6, np.pi / 4, np.pi / 3])

    # Forward kinematics
    T_final, transforms = robot.forward_kinematics(q)
    p = robot.end_effector_position(q)

    # Jacobian (analitis & numerik)
    J_analytic = robot.jacobian(q)
    J_numeric = robot.jacobian_numerical(q)

    # Tampilkan hasil
    np.set_printoptions(precision=4, suppress=True)
    print("=" * 60)
    print("ROBOT ARM MODELING - FORWARD KINEMATICS + JACOBIAN")
    print("=" * 60)
    print(f"\nSudut sendi q (rad): {q}")
    print(f"Sudut sendi q (deg): {np.degrees(q)}")

    print("\nMatriks transformasi total T_0n:")
    print(T_final)

    print(f"\nPosisi ujung efektif p = [x, y, z]: {p}")

    print("\nJacobian analitis (6 x 3):")
    print(J_analytic)

    print("\nJacobian numerik via finite difference (3 x 3):")
    print(J_numeric)

    print("\nPerbandingan bagian linear Jacobian:")
    print("Analitis :", J_analytic[:3, :])
    print("Numerik  :", J_numeric)

    # =================================================================
    # Contoh INVERSE KINEMATICS
    # =================================================================
    print("\n" + "=" * 60)
    print("INVERSE KINEMATICS - Cari q dari posisi target [x, y, z]")
    print("=" * 60)

    target = [0.35, 0.15, 0.25]
    q_solved, ok = robot.inverse_kinematics(target)

    print(f"\nTarget posisi : [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print(f"Konvergen     : {ok}")
    print(f"q solved (rad): {q_solved}")
    print(f"q solved (deg): {np.degrees(q_solved)}")

    p_check = robot.end_effector_position(np.array(q_solved))
    err = np.linalg.norm(np.array(target) - p_check)
    print(f"Posisi hasil  : {p_check}")
    print(f"Error         : {err:.6f} m")
