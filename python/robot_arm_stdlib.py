"""
Robot Arm Modeling - Forward Kinematics + Calculus (Jacobian)
==============================================================
Versi murni Python (tanpa dependensi eksternal; hanya modul `math`).
Cocok untuk coba segera tanpa harus memasang NumPy.

Pendekatan: Matriks Transformasi Homogen + Parameter Denavit-Hartenberg (DH)
"""

import math


def _identity4():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mul4(A, B):
    C = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C


def _cross3(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _sub3(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def dh_matrix(a, alpha, d, theta):
    """Matriks transformasi homogen 4x4 dari parameter DH."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ],
    ]


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
        T = _identity4()
        transforms = [ [row[:] for row in T] ]
        for i, link in enumerate(self.links):
            Ti = dh_matrix(link.a, link.alpha, link.d, q[i])
            T = _mul4(T, Ti)
            transforms.append([row[:] for row in T])
        return T, transforms

    def end_effector_position(self, q):
        T, _ = self.forward_kinematics(q)
        return [T[0][3], T[1][3], T[2][3]]

    def jacobian(self, q):
        _, transforms = self.forward_kinematics(q)
        pn = transforms[-1][0:3]
        pn = [transforms[-1][0][3], transforms[-1][1][3], transforms[-1][2][3]]

        J = [[0.0] * self.n for _ in range(6)]
        for i in range(self.n):
            Ti = transforms[i]
            zi = [Ti[0][2], Ti[1][2], Ti[2][2]]   # sumbu z frame i
            pi = [Ti[0][3], Ti[1][3], Ti[2][3]]   # asal frame i
            lin = _cross3(zi, _sub3(pn, pi))
            J[0][i] = lin[0]
            J[1][i] = lin[1]
            J[2][i] = lin[2]
            J[3][i] = zi[0]
            J[4][i] = zi[1]
            J[5][i] = zi[2]
        return J

    def jacobian_numerical(self, q, eps=1e-6):
        """Jacobian numerik via kalkulus (finite difference / turunan parsial)."""
        J = [[0.0] * self.n for _ in range(3)]
        p0 = self.end_effector_position(q)
        for i in range(self.n):
            dq = list(q)
            dq[i] += eps
            p1 = self.end_effector_position(dq)
            J[0][i] = (p1[0] - p0[0]) / eps
            J[1][i] = (p1[1] - p0[1]) / eps
            J[2][i] = (p1[2] - p0[2]) / eps
        return J

    # -----------------------------------------------------------------
    # INVERSE KINEMATICS (numerik via Jacobian pseudoinverse)
    # -----------------------------------------------------------------
    def inverse_kinematics(self, target, q_init=None, max_iter=100, tol=1e-4,
                           damping=1e-6):
        """
        Mencari sudut sendi q sehingga end effector mendekati target [x, y, z].

        Metode: Jacobian pseudoinverse (iteratif Levenberg-Marquardt sederhana)
        q_{k+1} = q_k + (J^T J + lambda I)^{-1} J^T * error

        Jika q_init diberikan, hanya dicoba tebakan itu. Jika None, dicoba
        beberapa tebakan awal (multi-start) dan hasil terbaik dipilih.

        Parameters
        ----------
        target : list[float]  -> [x, y, z] posisi tujuan
        q_init : list[float]  -> tebakan awal sudut (default: multi-start)
        max_iter : int        -> batas iterasi per tebakan
        tol : float           -> toleransi error posisi (meter)
        damping : float       -> faktor damping (lambda) untuk stabilitas

        Returns
        -------
        q : list[float]
        converged : bool
        """
        import math

        # Helper: invers matriks 3x3 sederhana (untuk J^T J + lambda I)
        def inv3(M):
            a, b, c = M[0]
            d, e, f = M[1]
            g, h, i = M[2]
            det = (a * (e * i - f * h) -
                   b * (d * i - f * g) +
                   c * (d * h - e * g))
            if abs(det) < 1e-12:
                return None
            inv = [
                [(e * i - f * h), -(b * i - c * h), (b * f - c * e)],
                [-(d * i - f * g), (a * i - c * g), -(a * f - c * d)],
                [(d * h - e * g), -(a * h - b * g), (a * e - b * d)],
            ]
            return [[v / det for v in row] for row in inv]

        def mat_vec(M, v):
            return [sum(M[r][c] * v[c] for c in range(3)) for r in range(3)]

        def mat_transpose(M):
            return [[M[r][c] for r in range(3)] for c in range(3)]

        def solve_from(q0):
            q = list(q0)
            for _ in range(max_iter):
                p = self.end_effector_position(q)
                error = [target[0] - p[0], target[1] - p[1], target[2] - p[2]]
                err_norm = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
                if err_norm < tol:
                    return q, True, err_norm

                J = self.jacobian(q)  # 6 x n
                J3 = [row[:3] for row in J[:3]]  # 3 x n

                JT = mat_transpose(J3)
                JTJ = [[sum(JT[i][k] * J3[k][j] for k in range(3))
                        for j in range(3)] for i in range(3)]
                for i in range(3):
                    JTJ[i][i] += damping

                JT_err = [sum(JT[i][j] * error[j] for j in range(3))
                          for i in range(3)]

                JTJ_inv = inv3(JTJ)
                if JTJ_inv is None:
                    return q, False, err_norm
                dq = mat_vec(JTJ_inv, JT_err)

                for i in range(self.n):
                    q[i] += dq[i]
                    q[i] = max(-math.pi, min(math.pi, q[i]))
            p = self.end_effector_position(q)
            error = [target[0] - p[0], target[1] - p[1], target[2] - p[2]]
            err_norm = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
            return q, False, err_norm

        # Tebakan awal
        if q_init is not None:
            q, ok, err = solve_from(q_init)
            return q, ok

        # Multi-start: coba beberapa konfigurasi awal
        candidates = [
            [0.0, 0.0, 0.0],
            [0.0, math.pi / 4, 0.0],
            [0.0, -math.pi / 4, 0.0],
            [math.pi / 2, 0.0, 0.0],
            [-math.pi / 2, 0.0, 0.0],
            [0.0, math.pi / 2, 0.0],
            [0.0, 0.0, math.pi / 2],
        ]
        best_q = None
        best_ok = False
        best_err = float('inf')
        for c in candidates:
            q, ok, err = solve_from(c)
            if err < best_err:
                best_err = err
                best_q = q
                best_ok = ok
            if ok:
                break
        return best_q, best_ok


def _fmt(v, w=10, p=4):
    return f"{v:{w}.{p}f}"


if __name__ == "__main__":
    # Robot 3-DOF: DHLink(a, alpha, d, theta)
    links = [
        DHLink(0.0, -math.pi / 2, 0.30, 0.0),  # Sendi 1: Putar dasar
        DHLink(0.40, 0.0,      0.00, 0.0),  # Sendi 2: Bahu
        DHLink(0.30, 0.0,      0.00, 0.0),  # Sendi 3: Siku
    ]
    robot = RobotArm(links)

    q = [math.pi / 6, math.pi / 4, math.pi / 3]

    T_final, _ = robot.forward_kinematics(q)
    p = robot.end_effector_position(q)
    J_analytic = robot.jacobian(q)
    J_numeric = robot.jacobian_numerical(q)

    print("=" * 60)
    print("ROBOT ARM MODELING - FORWARD KINEMATICS + JACOBIAN (Python stdlib)")
    print("=" * 60)
    print(f"\nSudut sendi q (rad): {[round(v, 4) for v in q]}")
    print(f"Sudut sendi q (deg): {[round(math.degrees(v), 2) for v in q]}")

    print("\nMatriks transformasi total T_0n:")
    for row in T_final:
        print(" [" + ", ".join(_fmt(v) for v in row) + "]")

    print(f"\nPosisi ujung efektif p = [x, y, z]: "
          f"[{_fmt(p[0])}, {_fmt(p[1])}, {_fmt(p[2])}]")

    print("\nJacobian analitis (6 x 3):")
    for row in J_analytic:
        print(" [" + ", ".join(_fmt(v) for v in row) + "]")

    print("\nJacobian numerik via finite difference (3 x 3):")
    for row in J_numeric:
        print(" [" + ", ".join(_fmt(v) for v in row) + "]")

    print("\nPerbandingan bagian linear Jacobian:")
    print("Analitis:")
    for i in range(3):
        print(" [" + ", ".join(_fmt(J_analytic[i][j]) for j in range(3)) + "]")
    print("Numerik:")
    for i in range(3):
        print(" [" + ", ".join(_fmt(J_numeric[i][j]) for j in range(3)) + "]")

    print("\nSelesai.")

    # =================================================================
    # Contoh INVERSE KINEMATICS
    # =================================================================
    print("\n" + "=" * 60)
    print("INVERSE KINEMATICS - Cari q dari posisi target [x, y, z]")
    print("=" * 60)

    target = [0.35, 0.15, 0.25]  # target dalam jangkauan robot
    q_solved, ok = robot.inverse_kinematics(target)

    print(f"Target posisi : [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print(f"Konvergen     : {ok}")
    print(f"q solved (rad): {[round(v, 4) for v in q_solved]}")
    print(f"q solved (deg): {[round(math.degrees(v), 2) for v in q_solved]}")

    # Verifikasi: hitung posisi kembali dengan q solved
    p_check = robot.end_effector_position(q_solved)
    print(f"Posisi hasil  : [{p_check[0]:.4f}, {p_check[1]:.4f}, {p_check[2]:.4f}]")
    err = math.sqrt(sum((target[i] - p_check[i])**2 for i in range(3)))
    print(f"Error         : {err:.6f} m")
