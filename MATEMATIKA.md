# Robot Arm Modeling — Rumus Kalkulus & Matriks (Langkah Pertama)

Dokumen ini menjelaskan dasar matematika untuk modeling gerak awal robot arm menggunakan **matriks transformasi homogen** dan **kalkulus (Jacobian)**.

---

## 1. Konsep Dasar

Robot arm terdiri dari rangkaian link yang terhubung oleh sendi (joint). Setiap sendi menghasilkan gerakan rotasi atau translasi. Untuk mengetahui posisi ujung efektif (end effector) berdasarkan sudut sendi, kita gunakan **Forward Kinematics**.

### Forward Kinematics
$$
T_{0n} = T_{01} \cdot T_{12} \cdot T_{23} \cdots T_{(n-1)n}
$$

di mana $T_{i-1,i}$ adalah matriks transformasi homogen 4×4 dari frame $i-1$ ke frame $i$.

Posisi ujung efektif diambil dari kolom ke-4 (baris 0–2) dari $T_{0n}$:
$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} = T_{0n}\begin{bmatrix}0\\0\\0\\1\end{bmatrix}_{1:3}
$$

---

## 2. Matriks Denavit-Hartenberg (DH)

Setiap link di-parameterkan oleh 4 variabel DH: $(a, \alpha, d, \theta)$.

Matriks transformasi DH:

$$
T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

- $a_i$ : panjang link (jarak sumbu $z_{i-1}$ ke $z_i$ sepanjang $x_i$)
- $\alpha_i$ : geseran sudut sumbu (twist)
- $d_i$ : offset link (jarak sepanjang $z_{i-1}$)
- $\theta_i$ : sudut sendi (variabel gerakan)

---

## 3. Kalkulus — Matriks Jacobian

Jacobian menghubungkan kecepatan sudut sendi $\dot{\mathbf{q}}$ dengan kecepatan ujung efektif $\mathbf{v}$:

$$
\mathbf{v} = \begin{bmatrix} \mathbf{v}_{lin} \\ \boldsymbol{\omega} \end{bmatrix} = \mathbf{J}(\mathbf{q}) \cdot \dot{\mathbf{q}}
$$

Untuk sendi rotasi (revolute), kolom ke-$i$ dari Jacobian:

$$
\mathbf{J}_{:,i} = \begin{bmatrix}
\mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\
\mathbf{z}_{i-1}
\end{bmatrix}
$$

di mana:
- $\mathbf{z}_{i-1}$ : sumbu rotasi sendi $i$ diekspresikan dalam frame dasar (kolom ke-3 dari matriks $T_{0,i-1}$)
- $\mathbf{p}_{i-1}$ : asal frame $i-1$ dalam frame dasar
- $\mathbf{p}_n$ : posisi ujung efektif

Jacobian juga bisa didefinisikan sebagai turunan (kalkulus) posisi terhadap sudut sendi:

$$
\mathbf{J} = \frac{\partial \mathbf{p}}{\partial \mathbf{q}}
$$

yang dapat dihitung numerik dengan finite difference (metode diferensi hingga).

---

## 4. Contoh Robot 3-DOF (Ide Pertama)

| Link | $a$ (m) | $\alpha$ | $d$ (m) | $\theta$ | Sendi |
|------|---------|----------|---------|----------|-------|
| 1 | 0 | $-\pi/2$ | 0.3 | $q_1$ | Base (putar z) |
| 2 | 0.4 | 0 | 0 | $q_2$ | Bahu (putar y) |
| 3 | 0.3 | 0 | 0 | $q_3$ | Siku (putar y) |

---

## 5. Cara Menjalankan

### Python
```bash
pip install -r requirements.txt
python python/robot_arm.py
```

### C++
```bash
cd cpp
mkdir build && cd build
cmake ..
cmake --build .
./robot_arm
```

## 6. Langkah Selanjutnya (Roadmap)

1. ✅ Forward Kinematics (posisi ujung efektif)
2. ✅ Jacobian (kecepatan & kalkulus)
3. Inverse Kinematics (menghitung $q$ dari posisi target)
4. Trajectory planning (gerak halus dari titik A ke B)
5. Simulasi visual (matplotlib / OpenGL / RViz)
