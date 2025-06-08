Berikut adalah versi yang telah diperbaiki dan dirapikan dalam format Markdown (`.md`) yang sesuai:

---

# Analisis Stabilitas Sistem Kontrol Massa-Pegas-Damper dengan Metode Runge-Kutta

## Informasi Mahasiswa

* **Nama Lengkap:** Daffa Hardhan
* **NPM:** 2306161763
* **Jurusan:** Teknik Komputer - Universitas Indonesia
* **Mata Kuliah:** Komputasi Numerik

## Deskripsi Program

Program ini mengimplementasikan simulasi dan analisis stabilitas sistem kontrol massa-pegas-damper menggunakan kontroler PID (Proportional-Integral-Derivative) dengan metode numerik Runge-Kutta orde keempat. Sistem massa-pegas-damper merupakan sistem fundamental dalam teknik kontrol yang banyak diaplikasikan pada:

* Sistem suspensi kendaraan
* Isolator getaran
* Sistem positioning

### Tujuan Program

1. Menganalisis stabilitas sistem massa-pegas-damper dengan kontroler PID
2. Mengimplementasikan metode Runge-Kutta orde 4 untuk solusi numerik
3. Melakukan analisis parametrik untuk optimasi parameter kontroler
4. Menghasilkan data dan visualisasi performa sistem kontrol

### Fitur Utama

* **Simulasi Default:** Menjalankan simulasi dengan parameter standar dari penelitian
* **Analisis Stabilitas:** Variasi parameter Kp untuk analisis karakteristik sistem
* **Parameter Custom:** Input parameter sistem dan kontroler sesuai kebutuhan
* **Export Data:** Menyimpan hasil simulasi dalam format CSV
* **Metrik Performa:** Perhitungan *settling time*, *overshoot*, dan *steady-state error*

## Struktur Program

```
ProyekUAS_2306161763_Daffa/
├── code/
│   ├── main/
│   │   ├── main.c                # Program utama dan menu interface
│   │   ├── system_dynamics.c    # Implementasi metode Runge-Kutta
│   │   └── utils.c              # Fungsi utilitas dan I/O
│   └── utils/
│       ├── system_dynamics.h    # Header definisi struktur data
│       └── utils.h              # Header fungsi utilitas
├── output/                      # Folder hasil simulasi (CSV files)
├── docs/                        # Dokumentasi dan laporan
└── README.md
```


### Komponen Utama

#### 1. System Dynamics (system_dynamics.c)
- Implementasi metode Runge-Kutta orde keempat
- Perhitungan dinamika sistem massa-pegas-damper
- Algoritma kontroler PID
- Fungsi simulasi dan analisis performa

#### 2. Utilities (utils.c)
- User interface dan menu interaktif
- Input/output parameter sistem
- Fungsi penyimpanan data ke CSV
- Perhitungan metrik performa (settling time, overshoot, steady-state error)

#### 3. Main Program (main.c)
- Control flow program utama
- Menu navigasi dan pilihan simulasi
- Koordinasi antara modul sistem dan utilities

## Metode Implementasi

### Sistem Massa-Pegas-Damper
Sistem dimodelkan dengan persamaan diferensial orde kedua:
```
m·ẍ + c·ẋ + k·x = F(t)
```

### Konversi ke State Space
```
dx/dt = v dv/dt = (F_control - c·v - k·x) / m
```

### Kontroler PID
```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

### Metode Runge-Kutta Orde 4
```
k₁ = h·f(tₙ, yₙ) k₂ = h·f(tₙ + h/2, yₙ + k₁/2) k₃ = h·f(tₙ + h/2, yₙ + k₂/2) k₄ = h·f(tₙ + h, yₙ + k₃) yₙ₊₁ = yₙ + (k₁ + 2k₂ + 2k₃ + k₄)/6
```


## Parameter Default

### Parameter Sistem
| Parameter | Nilai | Satuan |
|-----------|-------|---------|
| Massa (m) | 1.0 | kg |
| Koefisien Redaman (c) | 2.0 | Ns/m |
| Konstanta Pegas (k) | 10.0 | N/m |
| Time Step (dt) | 0.001 | s |
| Waktu Simulasi | 10.0 | s |

### Parameter Kontroler PID
| Parameter | Nilai |
|-----------|-------|
| Proportional Gain (Kp) | 15.0 |
| Integral Gain (Ki) | 5.0 |
| Derivative Gain (Kd) | 3.0 |
| Setpoint | 1.0 m |

## Cara Kompilasi dan Menjalankan

### Prasyarat
- Compiler C (GCC/MinGW)
- Sistem dengan support untuk standard C library

### Kompilasi
```bash
# Windows (MinGW)
gcc -o program main/main.c main/system_dynamics.c main/utils.c -lm

# Linux/Mac
gcc -o program main/main.c main/system_dynamics.c main/utils.c -lm
```

### Menjalankan Program
```
# Windows
./program.exe

# Linux/Mac
./program
```

## Output dan Hasil
### File Output
1. simulation_results.csv - Data simulasi lengkap (waktu, posisi, kecepatan, gaya kontrol)
2. stability_analysis.csv - Hasil analisis stabilitas untuk berbagai parameter Kp
3. custom_analysis.csv - Hasil analisis parameter custom (jika digunakan)

### Format CSV
```
Time_s,Position_m,Velocity_ms,Control_Force_N
0.000000,0.000000,0.000000,15.000000
0.001000,0.000007,0.014993,14.999925
```

### Metrik Performa
- Settling Time: Waktu untuk mencapai ±2% dari setpoint
- Overshoot: Persentase nilai maksimum terhadap setpoint
- Steady-State Error: Error permanen antara output dan setpoint

## Hasil Penelitian
Performa Optimal (Kp=15, Ki=5, Kd=3)
- Settling Time: 4.176 detik
- Overshoot: 19.78%
- Steady-State Error: 0.01%
- Status: STABIL

### Analisis Parameter Kp
| **Kp** | **Settling Time (s)** | **Overshoot (%)** | **Status** |
|-------:|-----------------------:|------------------:|:----------:|
| 1.0    | 4.999                 | 48.5              | **STABIL** |
| 5.0    | 4.855                 | 27.8              | **STABIL** |
| 10.0   | 4.729                 | 19.8              | **STABIL** |
| 15.0   | 4.176                 | 19.8              | **STABIL** |
| 20.0   | 3.844                 | 22.3              | **STABIL** |
| 50.0   | 4.999                 | 39.3              | **STABIL** |

## Validasi dan Verifikasi
### Validasi Teoretis
- Frekuensi natural: ωₙ = √(k/m) = 3.162 rad/s
- Damping ratio: ζ = c/(2√(mk)) = 0.316
- Sistem: Underdamped (ζ < 1)

### Verifikasi Numerik
- Time step dt = 0.001s untuk akurasi tinggi
- Konsistensi dengan teori sistem kontrol klasik
- Validasi dengan analytical solution untuk kasus sederhana

## Referensi
- Chapra, S.C. & Canale, R.P. (2010). Numerical Methods for Engineers, 6th Edition. McGraw-Hill.
- Ogata, K. (2010). Modern Control Engineering, 5th Edition. Prentice Hall.
- Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2014). Feedback Control of Dynamic Systems, 7th Edition. Pearson.
- Dorf, R.C. & Bishop, R.H. (2016). Modern Control Systems, 13th Edition. Pearson.
- Nise, N.S. (2019). Control Systems Engineering, 8th Edition. John Wiley & Sons.
