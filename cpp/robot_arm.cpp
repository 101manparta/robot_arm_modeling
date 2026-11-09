/*
 * Robot Arm Modeling - Forward Kinematics + Calculus (Jacobian)
 * =============================================================
 * Pendekatan: Matriks Transformasi Homogen + Parameter Denavit-Hartenberg (DH)
 * Bahasa: C++17 (tanpa dependensi eksternal; hanya standar library)
 *
 * Fungsi utama:
 *   - forwardKinematics(q)   -> matriks T_0n dan transformasi antar frame
 *   - endEffectorPosition(q) -> posisi [x, y, z] ujung efektif
 *   - jacobian(q)            -> matriks Jacobian analitis 6 x n
 *   - jacobianNumerical(q)   -> Jacobian numerik via finite difference
 */

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Vec3 = std::array<double, 3>;
using Mat4 = std::array<std::array<double, 4>, 4>;

/* ------------------------------------------------------------------ */
/* Operasi matriks dasar                                              */
/* ------------------------------------------------------------------ */

static Mat4 identity4() {
    Mat4 I{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            I[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    return I;
}

static Mat4 multiply4(const Mat4 &A, const Mat4 &B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
    return C;
}

static Vec3 cross3(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

static Vec3 sub3(const Vec3 &a, const Vec3 &b) {
    return Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/* ------------------------------------------------------------------ */
/* Parameter DH                                                       */
/* ------------------------------------------------------------------ */

struct DHLink {
    double a;      // panjang link
    double alpha;  // geseran sudut sumbu (twist)
    double d;      // offset link
    double theta;  // sudut sendi (diganti dengan q saat perhitungan)
};

/* ------------------------------------------------------------------ */
/* Matriks transformasi homogen 4x4 dari parameter DH                 */
/* ------------------------------------------------------------------ */

static Mat4 dhMatrix(double a, double alpha, double d, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Mat4 T{};
    T[0][0] = ct;       T[0][1] = -st * ca;  T[0][2] = st * sa;  T[0][3] = a * ct;
    T[1][0] = st;       T[1][1] =  ct * ca;  T[1][2] = -ct * sa; T[1][3] = a * st;
    T[2][0] = 0.0;      T[2][1] = sa;        T[2][2] = ca;       T[2][3] = d;
    T[3][0] = 0.0;      T[3][1] = 0.0;       T[3][2] = 0.0;      T[3][3] = 1.0;
    return T;
}

/* ------------------------------------------------------------------ */
/* Kelas Robot Arm                                                    */
/* ------------------------------------------------------------------ */

class RobotArm {
public:
    explicit RobotArm(std::vector<DHLink> links) : links_(std::move(links)) {}

    int nJoints() const { return static_cast<int>(links_.size()); }

    /*
     * Forward Kinematics: T_0n = T_01 * T_12 * ... * T_(n-1)n
     * Mengembalikan matriks akhir dan daftar transformasi setiap frame.
     */
    std::pair<Mat4, std::vector<Mat4>> forwardKinematics(const std::vector<double> &q) const {
        Mat4 T = identity4();
        std::vector<Mat4> transforms;
        transforms.push_back(T);

        for (int i = 0; i < nJoints(); ++i) {
            Mat4 Ti = dhMatrix(links_[i].a, links_[i].alpha, links_[i].d, q[i]);
            T = multiply4(T, Ti);
            transforms.push_back(T);
        }
        return {T, transforms};
    }

    /*
     * Posisi ujung efektif [x, y, z] dalam frame dasar.
     * Diambil dari kolom ke-4 (baris 0-2) dari T_0n.
     */
    Vec3 endEffectorPosition(const std::vector<double> &q) const {
        auto [T, _] = forwardKinematics(q);
        return Vec3{T[0][3], T[1][3], T[2][3]};
    }

    /*
     * Matriks Jacobian analitis 6 x n.
     * v = [ v_lin ; omega ] = J(q) * q_dot
     */
    std::vector<std::vector<double>> jacobian(const std::vector<double> &q) const {
        auto [_, transforms] = forwardKinematics(q);
        Vec3 pn = Vec3{transforms.back()[0][3], transforms.back()[1][3], transforms.back()[2][3]};

        std::vector<std::vector<double>> J(6, std::vector<double>(nJoints(), 0.0));
        for (int i = 0; i < nJoints(); ++i) {
            const Mat4 &Ti = transforms[i];
            Vec3 zi{Ti[0][2], Ti[1][2], Ti[2][2]}; // sumbu z frame i
            Vec3 pi{Ti[0][3], Ti[1][3], Ti[2][3]}; // asal frame i

            Vec3 lin = cross3(zi, sub3(pn, pi));
            J[0][i] = lin[0];
            J[1][i] = lin[1];
            J[2][i] = lin[2];
            J[3][i] = zi[0];
            J[4][i] = zi[1];
            J[5][i] = zi[2];
        }
        return J;
    }

    /*
     * Jacobian numerik via kalkulus (finite difference / turunan parsial):
     * J[:, i] = (p(q + eps*e_i) - p(q)) / eps
     */
    std::vector<std::vector<double>> jacobianNumerical(
        const std::vector<double> &q, double eps = 1e-6) const {
        std::vector<std::vector<double>> J(3, std::vector<double>(nJoints(), 0.0));
        Vec3 p0 = endEffectorPosition(q);

        for (int i = 0; i < nJoints(); ++i) {
            std::vector<double> dq = q;
            dq[i] += eps;
            Vec3 p1 = endEffectorPosition(dq);
            J[0][i] = (p1[0] - p0[0]) / eps;
            J[1][i] = (p1[1] - p0[1]) / eps;
            J[2][i] = (p1[2] - p0[2]) / eps;
        }
        return J;
    }

    /*
     * INVERSE KINEMATICS (Jacobian pseudoinverse + multi-start)
     * q_{k+1} = q_k + (J^T J + lambda I)^{-1} J^T * error
     */
    std::pair<std::vector<double>, bool> inverseKinematics(
        const Vec3 &target,
        const std::vector<double> &q_init = {},
        int max_iter = 100,
        double tol = 1e-4,
        double damping = 1e-6) const {

        auto matVec3 = [](const std::vector<std::vector<double>> &M,
                          const Vec3 &v) -> Vec3 {
            return Vec3{
                M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
                M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
                M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
            };
        };

        auto matTrans3 = [](const std::vector<std::vector<double>> &M)
            -> std::vector<std::vector<double>> {
            return std::vector<std::vector<double>>{
                {M[0][0], M[1][0], M[2][0]},
                {M[0][1], M[1][1], M[2][1]},
                {M[0][2], M[1][2], M[2][2]}
            };
        };

        auto inv3x3 = [](const std::vector<std::vector<double>> &M)
            -> std::vector<std::vector<double>> {
            double a = M[0][0], b = M[0][1], c = M[0][2];
            double d = M[1][0], e = M[1][1], f = M[1][2];
            double g = M[2][0], h = M[2][1], i = M[2][2];
            double det = a * (e * i - f * h)
                       - b * (d * i - f * g)
                       + c * (d * h - e * g);
            if (std::abs(det) < 1e-12) return {};
            return std::vector<std::vector<double>>{
                {(e * i - f * h) / det, -(b * i - c * h) / det, (b * f - c * e) / det},
                {-(d * i - f * g) / det, (a * i - c * g) / det, -(a * f - c * d) / det},
                {(d * h - e * g) / det, -(a * h - b * g) / det, (a * e - b * d) / det}
            };
        };

        auto norm3 = [](const Vec3 &v) {
            return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        };

        auto solveFrom = [&](const std::vector<double> &q0)
            -> std::tuple<std::vector<double>, bool, double> {
            std::vector<double> q = q0;
            for (int it = 0; it < max_iter; ++it) {
                Vec3 p = endEffectorPosition(q);
                Vec3 error{target[0] - p[0], target[1] - p[1], target[2] - p[2]};
                double err = norm3(error);
                if (err < tol) return {q, true, err};

                auto J = jacobian(q);
                // J3 = 3 x n bagian linear
                std::vector<std::vector<double>> J3(3,
                    std::vector<double>(nJoints(), 0.0));
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < nJoints(); ++c)
                        J3[r][c] = J[r][c];

                auto JT = matTrans3(J3);

                // JTJ = JT * J3  (3x3)
                std::vector<std::vector<double>> JTJ(3,
                    std::vector<double>(3, 0.0));
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        for (int k = 0; k < nJoints(); ++k)
                            JTJ[r][c] += JT[r][k] * J3[k][c];

                // damping di diagonal
                for (int i = 0; i < 3; ++i) JTJ[i][i] += damping;

                // JT * error  (3x1)
                Vec3 JT_err = matVec3(JT, error);

                auto JTJ_inv = inv3x3(JTJ);
                if (JTJ_inv.empty()) return {q, false, err};

                Vec3 dq = matVec3(JTJ_inv, JT_err);

                for (int i = 0; i < nJoints(); ++i) {
                    q[i] += dq[i];
                    q[i] = std::max(-M_PI, std::min(M_PI, q[i]));
                }
            }
            Vec3 p = endEffectorPosition(q);
            Vec3 error{target[0] - p[0], target[1] - p[1], target[2] - p[2]};
            return {q, false, norm3(error)};
        };

        std::vector<double> start;
        if (!q_init.empty()) {
            start = q_init;
        } else {
            start = {0.0, 0.0, 0.0};
        }

        auto [q, ok, err] = solveFrom(start);

        // Multi-start jika belum konvergen dan q_init tidak disediakan
        if (!ok && q_init.empty()) {
            std::vector<std::vector<double>> candidates = {
                {0.0, 0.0, 0.0},
                {0.0, M_PI / 4, 0.0},
                {0.0, -M_PI / 4, 0.0},
                {M_PI / 2, 0.0, 0.0},
                {-M_PI / 2, 0.0, 0.0},
                {0.0, M_PI / 2, 0.0},
                {0.0, 0.0, M_PI / 2}
            };
            double bestErr = err;
            for (const auto &c : candidates) {
                auto [qc, okc, errc] = solveFrom(c);
                if (errc < bestErr) {
                    bestErr = errc;
                    q = qc;
                    ok = okc;
                }
                if (ok) break;
            }
        }

        return {q, ok};
    }

private:
    std::vector<DHLink> links_;
};

/* ------------------------------------------------------------------ */
/* Tampilan matriks                                                   */
/* ------------------------------------------------------------------ */

static void printMat4(const Mat4 &M) {
    for (int i = 0; i < 4; ++i) {
        std::cout << " [";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << M[i][j];
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]\n";
    }
}

static void printVec3(const Vec3 &v) {
    std::cout << "[" << std::setprecision(4) << v[0] << ", "
              << v[1] << ", " << v[2] << "]";
}

static void printMat(const std::vector<std::vector<double>> &M) {
    for (const auto &row : M) {
        std::cout << " [";
        for (size_t j = 0; j < row.size(); ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << row[j];
            if (j + 1 < row.size()) std::cout << ", ";
        }
        std::cout << "]\n";
    }
}

/* ------------------------------------------------------------------ */
/* Contoh penggunaan: Robot Arm 3-DOF                                 */
/* ------------------------------------------------------------------ */

int main() {
    // Definisi link: DHLink(a, alpha, d, theta)
    std::vector<DHLink> links = {
        DHLink{0.0, -M_PI / 2, 0.30, 0.0},  // Sendi 1: Putar dasar
        DHLink{0.40, 0.0,      0.00, 0.0},  // Sendi 2: Bahu
        DHLink{0.30, 0.0,      0.00, 0.0},  // Sendi 3: Siku
    };

    RobotArm robot(links);

    // Sudut sendi dalam radian (contoh gerak awal)
    std::vector<double> q = {M_PI / 6, M_PI / 4, M_PI / 3};

    // Forward kinematics
    auto [T_final, transforms] = robot.forwardKinematics(q);
    Vec3 p = robot.endEffectorPosition(q);

    // Jacobian (analitis & numerik)
    auto J_analytic = robot.jacobian(q);
    auto J_numeric = robot.jacobianNumerical(q);

    std::cout << std::fixed;
    std::cout << "============================================================\n";
    std::cout << "ROBOT ARM MODELING - FORWARD KINEMATICS + JACOBIAN (C++)\n";
    std::cout << "============================================================\n";

    std::cout << "\nSudut sendi q (rad): ";
    for (double v : q) std::cout << v << " ";
    std::cout << "\nSudut sendi q (deg): ";
    for (double v : q) std::cout << v * 180.0 / M_PI << " ";
    std::cout << "\n";

    std::cout << "\nMatriks transformasi total T_0n:\n";
    printMat4(T_final);

    std::cout << "\nPosisi ujung efektif p = [x, y, z]: ";
    printVec3(p);
    std::cout << "\n";

    std::cout << "\nJacobian analitis (6 x 3):\n";
    printMat(J_analytic);

    std::cout << "\nJacobian numerik via finite difference (3 x 3):\n";
    printMat(J_numeric);

    std::cout << "\nPerbandingan bagian linear Jacobian:\n";
    std::cout << "Analitis:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << " [";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << J_analytic[i][j];
            if (j < 2) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    std::cout << "Numerik:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << " [";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << J_numeric[i][j];
            if (j < 2) std::cout << ", ";
        }
        std::cout << "]\n";
    }

    // =================================================================
    // Contoh INVERSE KINEMATICS
    // =================================================================
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "INVERSE KINEMATICS - Cari q dari posisi target [x, y, z]\n";
    std::cout << std::string(60, '=') << "\n";

    Vec3 target{0.35, 0.15, 0.25};
    auto [q_solved, ok] = robot.inverseKinematics(target);

    std::cout << "\nTarget posisi : ["
              << std::fixed << std::setprecision(3)
              << target[0] << ", " << target[1] << ", " << target[2] << "]\n";
    std::cout << "Konvergen     : " << (ok ? "true" : "false") << "\n";
    std::cout << "q solved (rad): [";
    for (size_t i = 0; i < q_solved.size(); ++i) {
        std::cout << std::setprecision(4) << q_solved[i];
        if (i + 1 < q_solved.size()) std::cout << ", ";
    }
    std::cout << "]\n";
    std::cout << "q solved (deg): [";
    for (size_t i = 0; i < q_solved.size(); ++i) {
        std::cout << std::setprecision(2) << q_solved[i] * 180.0 / M_PI;
        if (i + 1 < q_solved.size()) std::cout << ", ";
    }
    std::cout << "]\n";

    Vec3 p_check = robot.endEffectorPosition(q_solved);
    double err = std::sqrt(std::pow(target[0]-p_check[0],2) +
                           std::pow(target[1]-p_check[1],2) +
                           std::pow(target[2]-p_check[2],2));
    std::cout << "Posisi hasil  : ["
              << std::setprecision(4) << p_check[0] << ", "
              << p_check[1] << ", " << p_check[2] << "]\n";
    std::cout << "Error         : " << std::setprecision(6) << err << " m\n";

    std::cout << "\nSelesai.\n";
    return 0;
}
