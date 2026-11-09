# Simulasi 3D Robot Arm

Simulasi visual robot arm 3D yang dapat dikontrol secara virtual melalui slider di browser.

## Cara Menjalankan

1. Masuk ke folder `simulation/`
2. **Doble-klik** file `simulasi.html`
3. Aplikasi akan terbuka di browser (Chrome, Edge, Firefox, dll.)

Tidak perlu install Node.js, npm, atau apa pun. Semua library (Three.js) dimuat langsung dari CDN.

## Fitur

- Visualisasi 3D interaktive (putar, zoom, geser kamera dengan mouse)
- 3 slider kontrol sendi (Dasar, Bahu, Siku)
- Animasi gerak smooth (lerp)
- Frame koordinat tiap sendi (X=merah, Y=hijau, Z=biru)
- Posisi end effector [x, y, z] ditampilkan real-time
- Matriks Jacobian bagian linear ditampilkan di panel
- Tombol preset: Reset dan Pose Contoh (90°, 45°, 60°)
- **Inverse Kinematics**: masukkan posisi target [X, Y, Z], robot otomatis menghitung sudut sendi dan bergerak ke sana
- **Trajectory Planning**: rekam beberapa waypoint (pose sudut), lalu putar ulang lintasan secara smooth

## Cara Menggunakan Inverse Kinematics

1. Masukkan nilai target X, Y, Z (dalam meter) di panel "Inverse Kinematics"
2. Klik tombol **Solve IK**
3. Robot akan secara otomatis menghitung sudut sendi yang diperlukan dan bergerak ke posisi target
4. Target ditampilkan sebagai bola merah berkerucut di ruang 3D
5. Klik **Hapus Target** untuk menyembunyikan target dan kembali ke kontrol manual

## Cara Menggunakan Trajectory Planning

1. Gerakkan slider ke posisi yang diinginkan (waypoint pertama)
2. Klik tombol **Rekam Waypoint** — posisi saat ini disimpan
3. Gerakkan slider ke posisi lain, klik **Rekam Waypoint** lagi (ulang sebanyak yang mau)
4. Klik **Putar Lintasan** — robot akan bergerak smooth melalui semua waypoint yang direkam
5. Klik **Berhenti** untuk menghentikan pemutaran, atau **Hapus Semua** untuk reset

Waypoint ditampilkan dalam daftar di panel dengan sudut masing-masing sendi (derajat).

## Model yang Digunakan

Parameter Denavit-Hartenberg yang sama dengan model Python/C++ di folder induk:

```js
const LINKS = [
  { a: 0.0, alpha: -Math.PI/2, d: 0.30 }, // Sendi 1
  { a: 0.40, alpha: 0.0,       d: 0.00 }, // Sendi 2
  { a: 0.30, alpha: 0.0,       d: 0.00 }, // Sendi 3
];
```

## Koneksi dengan Modeling Lainnya

Rumus matematika di `simulasi.html` adalah port JavaScript langsung dari:
- `python/robot_arm_stdlib.py`
- `cpp/robot_arm.cpp`

Jadi hasil perhitungan posisi, Jacobian, dan Inverse Kinematics akan konsisten di semua bahasa.

## Catatan IK

- IK menggunakan metode Jacobian pseudoinverse (iteratif) dengan multi-start (beberapa tebakan awal).
- Jika posisi target di luar jangkauan robot, IK akan mencoba mendekati sebanyak mungkin dan menampilkan error.
- Rentang sudut sendi dibatasi antara -180° dan 180°.
