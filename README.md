# ArUco ROS2 Package (Aruco_ros)

Ringkasan singkat
- Paket ini mendeteksi marker ArUco menggunakan OpenCV + aruco, mengestimasi pose marker dengan solvePnP, dan mem-broadcast transform (TF) ke ROS2.
- Lokasi penting:
  - Kode: src/aruco/src/aruco_detect.cpp
  - Launch: src/aruco/launch/aruco_detect_launch.py
  - Config: src/aruco/config/config.yaml
  - Ekspektasi file kalibrasi kamera: `calibration.yaml` (lihat bagian Kalibrasi)

Persyaratan
- ROS 2 (versi yang Anda gunakan; contoh: Humble/Foxy/Rolling) terinstall dan ter-source.
- colcon build untuk membangun workspace.
- OpenCV dengan modul aruco (biasanya bagian dari opencv_contrib).
  - Contoh paket di Ubuntu: libopencv-dev + opencv-contrib (nama paket bervariasi). Jika header aruco tidak ditemukan, install OpenCV/aruco yang sesuai atau gunakan build OpenCV dengan opencv_contrib.
- camera device yang dapat dibuka (USB V4L2 device path).
- File kalibrasi kamera `calibration.yaml` tersedia pada working directory saat menjalankan node (atau ubah kode untuk path lengkap).

Instalasi / Build
1. Pastikan dependensi ROS2 dan OpenCV terpasang.
2. Di root workspace:
   - colcon build --packages-select aruco
   - source install/setup.bash

Menjalankan
- Gunakan launch file:
  - ros2 launch aruco aruco_detect_launch.py
- Pastikan `config/config.yaml` dipanggil lewat launch (lihat bagian Parameter / YAML di bawah).
- Jika mengubah nama node di launch, pastikan YAML top-level key sesuai (lihat di bawah).

Parameter (config.yaml)
- Nama parameter di kode (C++):
  - marker_size_cm (float) — ukuran sisi marker dalam sentimeter.
  - camera_frame (string) — nama frame kamera (mis. "camera_link").
  - tf_to (string) — target frame untuk lookup transform (mis. "base_link").
  - path_cam (string) — path device kamera (mis. "/dev/…").
- File YAML harus memuat top-level key yang sama dengan nama node runtime. Contoh (opsi A: gunakan nama node di launch `aruco_detect`):

```yaml
aruco_detect:
  ros__parameters:
    marker_size_cm: 5.0
    camera_frame: "camera_link"
    tf_to: "base_link"
    path_cam: "/dev/v4l/by-id/..."
```

- Jika launch memberi node name `aruco_detect_node` (sesuaikan), gunakan:
```yaml
aruco_detect_node:
  ros__parameters:
    marker_size_cm: 5.0
    ...
```

Catatan penting tentang ukuran (PnP) dan unit
- solvePnP bekerja dengan satuan yang konsisten antara objectPoints (3D) dan hasil tvec (3D translation). Pada kode saat ini:
  - objectPoints dibuat dari `marker_size_cm` (dalam cm), sehingga tvec akan bernilai dalam cm.
  - Sebelum di-broadcast ke TF, kode membagi tvec dengan 100.0 untuk mengonversi cm -> meter (ROS TF umumnya memakai meter).
- Rekomendasi:
  - Gunakan satuan yang konsisten (mis. meter). Ubah parameter menjadi marker_size_m dan buat objectPoints dalam meter — ini mengurangi kebingungan konversi.
  - Pastikan order titik objectPoints sama dengan order corners yang diberikan OpenCV (umumnya top-left, top-right, bottom-right, bottom-left) agar orientasi benar.

Detail teknis penting
- Dictionary ArUco: saat ini menggunakan `aruco::DICT_5X5_100`. Jika marker Anda berbeda, ganti dictionary yang sesuai.
- solvePnP memerlukan correspondences 3D -> 2D:
  - 3D objectPoints: koordinat sudut marker relatif ke pusat marker (atau titik referensi lain).
  - 2D markerCorners: sudut yang terdeteksi pada citra (urutannya harus sesuai).
- Kalibrasi kamera (`calibration.yaml`) harus berisi:
  - cameraMatrix — 3x3
  - distCoeffs — distorsi
  - Format OpenCV FileStorage (yaml/xml) yang dapat dibaca dengan cv::FileStorage:
    example keys: `cameraMatrix`, `distCoeffs`

Tentang TF di ROS2
- Kode melakukan:
  - Broadcast transform dari `camera_frame` -> `aruco_marker_<id>` (pose marker relatif ke kamera).
  - Lalu memanggil tf_buffer->lookupTransform(tf_to, child_frame, tf2::TimePointZero) untuk mendapatkan pose marker relatif ke frame `tf_to` (contoh: base_link).
- Penting:
  - lookupTransform menerima argumen (target_frame, source_frame, time). Jadi hasilnya mengubah koordinat dari source_frame menjadi target_frame.
  - Jika transform belum tersedia, lookupTransform akan melempar exception. Gunakan try/catch atau canTransform/waitForTransform bila perlu.
  - Pastikan ada transform static/static publisher untuk base_link -> camera_link jika ingin konversi ke base_link (contoh launch file sudah menyertakan static_transform_publisher).

Troubleshooting singkat
- Header aruco tidak ditemukan saat compile:
  - Pastikan OpenCV + aruco terinstall dan CMakeLists.txt link ke include dirs / libraries OpenCV dengan benar.
- Parameter YAML tidak terbaca:
  - Pastikan launch menggunakan `parameters=[config_path]`.
  - Pastikan top-level YAML key sama dengan nama node di runtime (lihat contoh di atas).
  - Pastikan nama parameter di YAML cocok dengan yang dideklarasikan di kode (`marker_size_cm` vs `marker_size`).
- Hasil pose aneh:
  - Periksa ordering objectPoints vs markerCorners.
  - Periksa ukuran marker yang digunakan (harus sesuai ukuran fisik marker).
  - Verifikasi kamera kalibrasi (cameraMatrix & distCoeffs).

Contoh perbaikan cepat untuk konsistensi unit
- Ubah parameter jadi meter (mis. marker_size_m: 0.05) dan set objectPoints menggunakan meter supaya solvePnP langsung mengembalikan tvec dalam meter — tidak perlu pembagian 100.

Lisensi / Catatan
- README ini ringkas; sesuaikan dokumentasi internal sesuai kebutuhan eksperimen Anda.
