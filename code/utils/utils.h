/**
 * ================================================================================
 * UTILITY FUNCTIONS HEADER FILE
 * Header untuk Fungsi-Fungsi Pendukung Program Simulasi
 * ================================================================================
 *
 * File header ini berisi deklarasi fungsi-fungsi utility yang mendukung
 * operasi program simulasi sistem massa-pegas-damper, meliputi:
 *
 * 1. User Interface Functions - tampilan menu, header, dan interaksi user
 * 2. Input/Output Functions - input parameter dan penyimpanan hasil
 * 3. Analysis Functions - perhitungan metrik performa sistem kontrol
 * 4. Display Functions - tampilan parameter dan informasi sistem
 * 5. File Management Functions - operasi file CSV dan data management
 *
 * Fungsi-fungsi ini terpisah dari core simulation untuk:
 * - Modularitas kode yang lebih baik
 * - Kemudahan maintenance dan debugging
 * - Reusability untuk project lain
 * - Separation of concerns (UI vs Core Logic)
 *
 * Author: Daffa Hardhan
 * NPM: 2306161763
 * Jurusan: Teknik Komputer - Universitas Indonesia
 * ================================================================================
 */

// ===============================================================================
// HEADER GUARD - MENCEGAH MULTIPLE INCLUSION
// ===============================================================================

/**
 * Header guard menggunakan preprocessor directives untuk mencegah
 * file header di-include lebih dari sekali dalam satu compilation unit.
 * 
 * Mekanisme yang sama dengan system_dynamics.h:
 * - #ifndef UTILS_H: check apakah UTILS_H belum didefinisikan
 * - #define UTILS_H: definisikan UTILS_H jika belum ada
 * - #endif: akhiri blok conditional compilation
 * 
 * Nama guard harus unique untuk setiap header file.
 */
#ifndef UTILS_H
#define UTILS_H

// ===============================================================================
// INCLUDE DEPENDENCIES
// ===============================================================================

/**
 * Include system_dynamics.h untuk mengakses definisi struktur data:
 * - SystemParams: parameter sistem massa-pegas-damper
 * - ControllerParams: parameter kontroler PID
 * - AnalysisResult: hasil analisis performa sistem
 * 
 * Dependency ini penting karena fungsi utility membutuhkan akses
 * ke struktur data yang didefinisikan di system_dynamics.h.
 */
#include "system_dynamics.h"

// ===============================================================================
// FUNCTION PROTOTYPES - USER INTERFACE FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI TAMPILAN HEADER PROGRAM
 * ================================================================================
 *
 * Fungsi untuk menampilkan header program yang berisi informasi tentang:
 * - Judul program dan deskripsi singkat
 * - Metode numerik yang digunakan (Runge-Kutta Orde 4)
 * - Informasi mata kuliah dan project
 * - Data mahasiswa (nama dan NIM)
 * 
 * Parameter: void (tidak memerlukan input parameter)
 * Return: void (tidak mengembalikan nilai)
 * 
 * Fungsi ini dipanggil sekali di awal program untuk memberikan konteks
 * dan informasi identitas program kepada user.
 */
void print_header(void);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI TAMPILAN MENU UTAMA
 * ================================================================================
 *
 * Fungsi untuk menampilkan menu utama program dengan opsi-opsi yang tersedia:
 * 1. Simulasi dengan parameter default dari laporan penelitian
 * 2. Analisis stabilitas dengan variasi parameter Kp
 * 3. Simulasi dengan parameter yang dapat dikustomisasi user
 * 4. Analisis parameter custom dengan rentang yang ditentukan user
 * 5. Tampilkan parameter sistem yang sedang aktif
 * 0. Keluar dari program
 *
 * Parameter: void
 * Return: void
 *
 * Menu dirancang user-friendly dengan penjelasan singkat untuk setiap opsi
 * dan menggunakan numbering yang konsisten untuk kemudahan navigasi.
 */
void print_main_menu(void);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI INPUT VALIDASI PILIHAN USER
 * ================================================================================
 *
 * Fungsi untuk mengambil dan memvalidasi input pilihan menu dari user.
 * Fungsi ini menangani:
 * - Input validation untuk memastikan input berupa integer
 * - Error handling jika user memasukkan karakter non-numerik
 * - Input buffer clearing untuk mencegah cascading errors
 * - Loop retry hingga input valid
 *
 * Parameter: void
 * Return: int - pilihan menu yang valid dari user
 *
 * Robust input handling mencegah program crash atau infinite loop
 * akibat input yang tidak sesuai format.
 */
int get_user_choice(void);

// ===============================================================================
// FUNCTION PROTOTYPES - PARAMETER DISPLAY FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI TAMPILAN PARAMETER SISTEM
 * ================================================================================
 *
 * Fungsi untuk menampilkan parameter sistem massa-pegas-damper dalam format
 * yang mudah dibaca dan dipahami, meliputi:
 *
 * Parameter Fisik:
 * - Massa (m), Koefisien Redaman (c), Konstanta Pegas (k)
 * - Time step (dt) dan Waktu Simulasi Total
 *
 * Karakteristik Sistem Teoretis (dihitung otomatis):
 * - Frekuensi Natural (ωn = √(k/m))
 * - Damping Ratio (ζ = c/(2√(mk)))
 * - Klasifikasi sistem (Underdamped/Critical/Overdamped)
 *
 * Parameter:
 * - params: pointer ke struct SystemParams (read-only)
 * Return: void
 *
 * Fungsi ini membantu user memahami karakteristik sistem sebelum simulasi.
 */
void print_system_parameters(SystemParams *params);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI TAMPILAN PARAMETER KONTROLER PID
 * ================================================================================
 *
 * Fungsi untuk menampilkan parameter kontroler PID dalam format tabel
 * yang mudah dibaca:
 * - Kp (Proportional Gain): respons proporsional terhadap error
 * - Ki (Integral Gain): eliminasi steady-state error
 * - Kd (Derivative Gain): damping dan prediksi tren error
 * - Setpoint: target posisi yang ingin dicapai sistem
 *
 * Parameter:
 * - controller: pointer ke struct ControllerParams (read-only)
 * Return: void
 *
 * Informasi ini penting untuk verifikasi parameter sebelum simulasi
 * dan untuk dokumentasi hasil eksperimen.
 */
void print_controller_parameters(ControllerParams *controller);

// ===============================================================================
// FUNCTION PROTOTYPES - FILE I/O FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PENYIMPANAN HASIL SIMULASI KE CSV
 * ================================================================================
 *
 * Fungsi untuk menyimpan hasil simulasi dalam format CSV (Comma-Separated Values)
 * yang dapat dibaca oleh software analisis seperti Excel, MATLAB, atau Python.
 *
 * Format CSV Output:
 * - Header: Time_s, Position_m, Velocity_ms, Control_Force_N
 * - Data: nilai numerik dengan presisi tinggi (6 digit setelah koma)
 *
 * Parameter:
 * - time: array waktu simulasi (detik)
 * - position: array posisi massa (meter)
 * - velocity: array kecepatan massa (m/s)
 * - control_force: array gaya kontrol PID (Newton)
 * - n_points: jumlah titik data dalam array
 * - filename: nama file output (termasuk path)
 *
 * Return: void
 *
 * File CSV memungkinkan analisis lanjutan dan plotting grafik dengan
 * software eksternal untuk visualisasi dan validasi hasil.
 */
void save_results_to_csv(double *time, double *position, double *velocity, 
                        double *control_force, int n_points, const char* filename);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PENYIMPANAN HASIL ANALISIS STABILITAS KE CSV
 * ================================================================================
 *
 * Fungsi untuk menyimpan hasil analisis stabilitas (variasi parameter) dalam
 * format CSV untuk analisis tren dan plotting grafik performa vs parameter.
 *
 * Format CSV Output:
 * - Header: Kp, Settling_Time_s, Overshoot_percent, Steady_State_Error_percent, Is_Stable
 * - Data: hasil analisis untuk setiap nilai parameter yang dianalisis
 *
 * Parameter:
 * - kp_values: array nilai Kp yang dianalisis
 * - results: array struct AnalysisResult dengan metrik performa
 * - n_values: jumlah nilai parameter yang dianalisis
 * - filename: nama file output
 *
 * Return: void
 *
 * Data ini berguna untuk:
 * - Plotting grafik settling time vs Kp
 * - Analisis tren overshoot vs parameter
 * - Identifikasi parameter optimal
 * - Validasi hasil penelitian
 */
void save_stability_analysis_to_csv(double *kp_values, AnalysisResult *results, 
                                   int n_values, const char* filename);

// ===============================================================================
// FUNCTION PROTOTYPES - ANALYSIS FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PERHITUNGAN SETTLING TIME
 * ================================================================================
 *
 * Fungsi untuk menghitung settling time sistem kontrol berdasarkan kriteria
 * toleransi tertentu (biasanya ±2% atau ±5% dari nilai setpoint).
 *
 * Settling time adalah waktu yang diperlukan sistem untuk mencapai dan
 * tetap berada dalam band toleransi di sekitar setpoint.
 *
 * Algoritma:
 * 1. Definisikan band toleransi (upper dan lower bound)
 * 2. Scan dari akhir ke awal data untuk mencari titik terakhir
 *    dimana sistem keluar dari band toleransi
 * 3. Settling time = waktu setelah titik tersebut
 *
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - time: array waktu yang bersesuaian dengan posisi
 * - n_points: jumlah titik data
 * - tolerance: toleransi dalam bentuk desimal (0.02 = 2%)
 *
 * Return: double - settling time dalam detik
 *
 * Metrik ini penting untuk mengevaluasi kecepatan respons sistem kontrol.
 */
double calculate_settling_time(double *position, double *time, int n_points, double tolerance);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PERHITUNGAN OVERSHOOT
 * ================================================================================
 *
 * Fungsi untuk menghitung overshoot sistem kontrol sebagai persentase
 * nilai maksimum terhadap setpoint.
 *
 * Overshoot mengindikasikan seberapa jauh sistem "melampaui" target
 * sebelum mencapai kondisi stabil.
 *
 * Formula: Overshoot = ((nilai_max - setpoint) / setpoint) × 100%
 *
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - n_points: jumlah titik data dalam array
 * - setpoint: nilai target sistem (reference)
 *
 * Return: double - overshoot dalam persen
 *
 * Overshoot yang tinggi menunjukkan sistem terlalu agresif dan mungkin
 * memerlukan tuning parameter kontroler (khususnya Kp dan Kd).
 */
double calculate_overshoot(double *position, int n_points, double setpoint);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PERHITUNGAN STEADY-STATE ERROR
 * ================================================================================
 *
 * Fungsi untuk menghitung steady-state error sistem sebagai perbedaan
 * permanen antara output akhir sistem dengan setpoint.
 *
 * Menggunakan rata-rata beberapa titik terakhir (biasanya 100 titik)
 * untuk mengurangi efek noise dan mendapatkan estimasi steady-state
 * yang lebih akurat.
 *
 * Formula: SS_Error = |setpoint - nilai_final| / setpoint × 100%
 *
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - n_points: jumlah titik data dalam array
 * - setpoint: nilai target sistem
 *
 * Return: double - steady-state error dalam persen
 *
 * Steady-state error yang besar mengindikasikan gain integral (Ki)
 * kurang memadai untuk mengeliminasi error permanen.
 */
double calculate_steady_state_error(double *position, int n_points, double setpoint);

// ===============================================================================
// FUNCTION PROTOTYPES - INPUT FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI INPUT PARAMETER SISTEM CUSTOM
 * ================================================================================
 *
 * Fungsi untuk memungkinkan user memasukkan parameter sistem massa-pegas-damper
 * secara interaktif. Menampilkan nilai saat ini sebagai referensi sebelum
 * meminta input baru.
 *
 * Parameter yang dapat dimodifikasi:
 * - Massa (m): mempengaruhi inersia dan frekuensi natural
 * - Koefisien redaman (c): mempengaruhi damping ratio dan tipe respons
 * - Konstanta pegas (k): mempengaruhi frekuensi natural dan kekakuan
 * - Waktu simulasi: durasi total untuk analisis
 *
 * Parameter:
 * - params: pointer ke struct SystemParams yang akan dimodifikasi (in/out)
 * Return: void
 *
 * Time step (dt) tidak dimodifikasi untuk menjaga konsistensi akurasi numerik.
 * Fungsi ini memungkinkan eksperimen dengan konfigurasi sistem yang berbeda.
 */
void input_system_parameters(SystemParams *params);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI INPUT PARAMETER KONTROLER CUSTOM
 * ================================================================================
 *
 * Fungsi untuk memungkinkan user memasukkan parameter kontroler PID
 * secara interaktif. Setelah input, kontroler direset ke kondisi awal
 * untuk simulasi yang bersih.
 *
 * Parameter yang dapat dimodifikasi:
 * - Kp: proportional gain (respons proporsional terhadap error)
 * - Ki: integral gain (eliminasi steady-state error)
 * - Kd: derivative gain (damping dan prediksi tren)
 * - Setpoint: target posisi yang ingin dicapai
 *
 * Parameter:
 * - controller: pointer ke struct ControllerParams yang akan dimodifikasi (in/out)
 * Return: void
 *
 * Setelah input, fungsi otomatis mereset integral accumulator dan prev_error
 * untuk memastikan simulasi dimulai dari kondisi yang bersih.
 */
void input_controller_parameters(ControllerParams *controller);

// ===============================================================================
// FUNCTION PROTOTYPES - UTILITY FUNCTIONS
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PAUSE PROGRAM
 * ================================================================================
 *
 * Fungsi utility untuk menghentikan eksekusi program sementara dan menunggu
 * user menekan Enter untuk melanjutkan. Berguna dalam interface menu untuk:
 * - Memberikan waktu user membaca output yang ditampilkan
 * - Mengontrol flow program dalam menu interaktif
 * - Mencegah layar berubah terlalu cepat
 * - Improved user experience dalam console application
 *
 * Parameter: void
 * Return: void
 *
 * Implementasi menangani input buffer clearing untuk memastikan
 * fungsi bekerja dengan benar setelah operasi scanf sebelumnya.
 *
 * Fungsi ini penting untuk usability program berbasis console yang
 * menampilkan banyak informasi secara berurutan.
 */
void pause_program(void);

// ===============================================================================
// PENUTUP HEADER GUARD
// ===============================================================================

/**
 * Akhiri header guard yang dimulai dengan #ifndef UTILS_H
 * Semua deklarasi fungsi di atas akan di-include hanya sekali per compilation unit.
 */
#endif

/**
 * ================================================================================
 * CATATAN PENGGUNAAN DAN DEPENDENCY MANAGEMENT
 * ================================================================================
 *
 * Dependencies:
 * - File ini bergantung pada system_dynamics.h untuk definisi struct data
 * - Implementasi fungsi-fungsi ini berada di utils.c
 * - Standard C library functions digunakan dalam implementasi (stdio.h, stdlib.h, math.h)
 *
 * Include Order:
 * Dalam file .c, include order yang disarankan:
 * 1. system_dynamics.h
 * 2. utils.h
 * 3. Standard library headers
 *
 * Compilation:
 * - File ini harus di-compile bersama dengan utils.c dan system_dynamics.c
 * - Pastikan path include sudah benar sesuai struktur direktori project
 *
 * Error Handling:
 * - Sebagian besar fungsi menggunakan defensive programming
 * - Input validation dilakukan di level fungsi individual
 * - File I/O errors ditangani dengan graceful degradation
 *
 * Memory Management:
 * - Fungsi-fungsi ini tidak melakukan dynamic memory allocation
 * - Memory management dilakukan di calling functions
 * - Tidak ada memory leak concerns di level utility functions
 * ================================================================================
 */