/**
 * ================================================================================
 * SYSTEM DYNAMICS HEADER FILE
 * Header untuk Definisi Struktur Data dan Function Prototypes
 * ================================================================================
 * 
 * File header ini mendefinisikan semua struktur data dan function prototypes
 * yang digunakan dalam simulasi sistem massa-pegas-damper dengan kontroler PID
 * menggunakan metode Runge-Kutta orde keempat.
 * 
 * Komponen Utama:
 * 1. Struktur parameter sistem fisik (massa, redaman, pegas)
 * 2. Struktur state sistem (posisi, kecepatan)
 * 3. Struktur parameter kontroler PID
 * 4. Struktur hasil analisis performa
 * 5. Function prototypes untuk semua operasi sistem
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
 * Mekanisme:
 * - #ifndef: check apakah SYSTEM_DYNAMICS_H belum didefinisikan
 * - #define: definisikan SYSTEM_DYNAMICS_H jika belum ada
 * - #endif: akhiri blok conditional compilation
 * 
 * Ini penting untuk menghindari redefinition errors dan circular dependencies.
 */
#ifndef SYSTEM_DYNAMICS_H
#define SYSTEM_DYNAMICS_H

// ===============================================================================
// DEFINISI STRUKTUR DATA SISTEM
// ===============================================================================

/**
 * ================================================================================
 * STRUKTUR PARAMETER SISTEM MASSA-PEGAS-DAMPER
 * ================================================================================
 * 
 * Struktur ini menyimpan semua parameter fisik yang mendefinisikan karakteristik
 * sistem massa-pegas-damper dan parameter simulasi numerik.
 * 
 * Parameter Fisik:
 * - mass: massa objek yang bergerak dalam sistem (kg)
 *         Mempengaruhi inersia dan frekuensi natural sistem
 *         Frekuensi natural: ωn = √(k/m)
 * 
 * - damping: koefisien redaman linier (Ns/m atau kg/s)
 *           Menentukan tingkat peredaman dalam sistem
 *           Damping ratio: ζ = c/(2√(mk))
 * 
 * - stiffness: konstanta pegas (N/m atau kg/s²)
 *             Menentukan kekakuan sistem dan gaya restoring
 *             F_spring = -k * x
 * 
 * Parameter Numerik:
 * - dt: time step untuk integrasi numerik Runge-Kutta (s)
 *       Menentukan akurasi dan stabilitas numerik
 *       Nilai kecil = akurasi tinggi, computational cost tinggi
 * 
 * - t_final: waktu total simulasi (s)
 *           Harus cukup lama untuk analisis settling time dan steady-state
 */
typedef struct {
    double mass;        // massa (kg)
    double damping;     // koefisien redaman (Ns/m)
    double stiffness;   // konstanta pegas (N/m)
    double dt;          // time step (s)
    double t_final;     // waktu simulasi total (s)
} SystemParams;

/**
 * ================================================================================
 * STRUKTUR STATE SISTEM DINAMIS
 * ================================================================================
 * 
 * Struktur ini menyimpan state variables sistem massa-pegas-damper pada
 * suatu instant waktu. Sistem orde kedua dikonversi menjadi sistem orde
 * pertama dengan dua state variables.
 * 
 * State Variables:
 * - position: posisi massa relatif terhadap posisi kesetimbangan (m)
 *            Ini adalah displacement dari posisi natural pegas
 *            x(t) dalam persamaan m*x'' + c*x' + k*x = F
 * 
 * - velocity: kecepatan massa (m/s)
 *            Turunan pertama posisi: v = dx/dt
 *            Digunakan untuk mengkonversi sistem orde kedua ke orde pertama
 * 
 * Konversi Sistem:
 * Persamaan asli: m*x'' + c*x' + k*x = F
 * State space form:
 * dx/dt = v
 * dv/dt = (F - c*v - k*x) / m
 */
typedef struct {
    double position;    // posisi x (m)
    double velocity;    // kecepatan dx/dt (m/s)
} SystemState;

/**
 * ================================================================================
 * STRUKTUR PARAMETER KONTROLER PID
 * ================================================================================
 * 
 * Struktur ini menyimpan semua parameter dan state variables yang diperlukan
 * untuk implementasi kontroler PID (Proportional-Integral-Derivative).
 * 
 * Control Law: u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
 * 
 * Parameter Kontroler:
 * - Kp: proportional gain
 *       Mengontrol respons proporsional terhadap error saat ini
 *       ↑ Kp: respons lebih cepat, tetapi mungkin overshoot
 *       ↓ Kp: respons lebih lambat, tetapi lebih stabil
 * 
 * - Ki: integral gain
 *       Mengeliminasi steady-state error dengan mengintegralkan error
 *       ↑ Ki: eliminasi error lebih cepat, tetapi mungkin oscillatory
 *       ↓ Ki: eliminasi error lebih lambat
 * 
 * - Kd: derivative gain
 *       Memprediksi tren error dan memberikan damping
 *       ↑ Kd: damping lebih kuat, tetapi sensitif terhadap noise
 *       ↓ Kd: damping lebih lemah
 * 
 * Parameter Target:
 * - setpoint: nilai target yang ingin dicapai sistem (m)
 *            Reference signal untuk perhitungan error
 *            e(t) = setpoint - x(t)
 * 
 * State Variables Kontroler:
 * - integral: akumulasi error untuk komponen integral
 *            Diupdate setiap time step: integral += error * dt
 *            Perlu direset saat parameter berubah
 * 
 * - prev_error: error pada time step sebelumnya
 *              Diperlukan untuk perhitungan komponen derivative
 *              derivative = (error - prev_error) / dt
 */
typedef struct {
    double Kp;          // proportional gain
    double Ki;          // integral gain
    double Kd;          // derivative gain
    double setpoint;    // target posisi
    double integral;    // akumulasi error integral
    double prev_error;  // error sebelumnya untuk derivative
} ControllerParams;

/**
 * ================================================================================
 * STRUKTUR HASIL ANALISIS PERFORMA SISTEM
 * ================================================================================
 * 
 * Struktur ini menyimpan metrik-metrik performa sistem kontrol yang dihitung
 * setelah simulasi selesai. Metrik ini digunakan untuk mengevaluasi kualitas
 * respons sistem dan menentukan apakah parameter kontroler optimal.
 * 
 * Metrik Performa:
 * - settling_time: waktu yang diperlukan sistem untuk mencapai dan tetap
 *                 berada dalam ±2% dari nilai setpoint (detik)
 *                 Indikator kecepatan respons sistem
 *                 Settling time pendek = respons cepat
 * 
 * - overshoot: persentase nilai maksimum terhadap setpoint (%)
 *             Formula: ((max_value - setpoint) / setpoint) * 100
 *             Indikator stabilitas sistem
 *             Overshoot tinggi = sistem terlalu agresif
 * 
 * - steady_state_error: error permanen antara output dan setpoint (%)
 *                      Formula: |setpoint - final_value| / setpoint * 100
 *                      Indikator akurasi sistem
 *                      SS error tinggi = kontroler kurang efektif
 * 
 * - is_stable: flag boolean untuk status stabilitas sistem
 *             1 (true) = sistem stabil
 *             0 (false) = sistem tidak stabil
 *             Kriteria: settling_time < t_simulasi && overshoot < threshold
 */
typedef struct {
    double settling_time;       // waktu settling (s)
    double overshoot;          // overshoot (%)
    double steady_state_error; // steady-state error (%)
    int is_stable;            // flag stabilitas (1=stabil, 0=tidak stabil)
} AnalysisResult;

// ===============================================================================
// FUNCTION PROTOTYPES - DEKLARASI FUNGSI
// ===============================================================================

/**
 * ================================================================================
 * PROTOTYPE FUNGSI METODE RUNGE-KUTTA ORDE KEEMPAT
 * ================================================================================
 * 
 * Fungsi ini mengimplementasikan metode Runge-Kutta orde keempat untuk
 * integrasi numerik sistem persamaan diferensial massa-pegas-damper.
 * 
 * Parameter:
 * - state: pointer ke state sistem yang akan diupdate (in/out)
 * - params: pointer ke parameter sistem (read-only)
 * - controller: pointer ke parameter kontroler (in/out, untuk update integral)
 * - t: waktu saat ini (informasi tambahan, tidak digunakan dalam implementasi)
 * 
 * Fungsi ini memodifikasi state->position dan state->velocity berdasarkan
 * dinamika sistem dan gaya kontrol dari PID controller.
 */
void runge_kutta_4th_order(SystemState *state, SystemParams *params, 
                          ControllerParams *controller, double t);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI PERHITUNGAN GAYA KONTROL PID
 * ================================================================================
 * 
 * Fungsi ini menghitung gaya kontrol dari kontroler PID berdasarkan
 * error antara setpoint dan posisi actual sistem.
 * 
 * Parameter:
 * - state: pointer ke state sistem saat ini (read-only)
 * - controller: pointer ke parameter kontroler (in/out, untuk update integral dan prev_error)
 * - dt: time step untuk integrasi dan diferensiasi numerik
 * 
 * Return:
 * - Gaya kontrol dalam Newton (N)
 * 
 * Fungsi ini juga mengupdate controller->integral dan controller->prev_error
 * untuk iterasi berikutnya.
 */
double calculate_control_force(SystemState *state, ControllerParams *controller, double dt);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI DINAMIKA SISTEM MASSA-PEGAS-DAMPER
 * ================================================================================
 * 
 * Fungsi ini mengimplementasikan persamaan diferensial sistem massa-pegas-damper
 * dan menghitung turunan state variables (dx/dt dan dv/dt).
 * 
 * Parameter:
 * - state: pointer ke state sistem saat ini (read-only)
 * - params: pointer ke parameter sistem (read-only)
 * - control_force: gaya kontrol dari PID controller
 * - dx_dt: pointer untuk output turunan posisi (out)
 * - dv_dt: pointer untuk output turunan kecepatan (out)
 * 
 * Output:
 * - dx_dt: turunan posisi = kecepatan saat ini
 * - dv_dt: turunan kecepatan = percepatan berdasarkan F=ma
 * 
 * Persamaan yang diimplementasikan:
 * dx/dt = v
 * dv/dt = (F_control - c*v - k*x) / m
 */
void system_dynamics(SystemState *state, SystemParams *params, double control_force, 
                     double *dx_dt, double *dv_dt);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI SIMULASI SISTEM LENGKAP
 * ================================================================================
 * 
 * Fungsi utama yang menjalankan simulasi lengkap sistem massa-pegas-damper
 * dengan kontroler PID menggunakan metode Runge-Kutta orde keempat.
 * 
 * Parameter:
 * - params: pointer ke parameter sistem (read-only)
 * - controller: pointer ke parameter kontroler (in/out, untuk reset dan update)
 * - save_to_file: flag untuk menyimpan hasil ke file CSV (1=ya, 0=tidak)
 * 
 * Return:
 * - AnalysisResult: struct berisi metrik performa sistem
 * 
 * Fungsi ini melakukan:
 * 1. Inisialisasi state dan alokasi memori
 * 2. Loop simulasi dengan Runge-Kutta
 * 3. Perhitungan metrik performa
 * 4. Penyimpanan hasil (jika diminta)
 * 5. Cleanup memori
 */
AnalysisResult simulate_system(SystemParams *params, ControllerParams *controller, int save_to_file);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI ANALISIS STABILITAS RENTANG PARAMETER
 * ================================================================================
 * 
 * Fungsi ini melakukan analisis stabilitas dengan memvariasikan parameter
 * kontroler (khususnya Kp) dalam rentang yang telah ditentukan sesuai dengan
 * data yang ada di laporan penelitian.
 * 
 * Parameter:
 * - params: pointer ke parameter sistem (read-only)
 * - controller: pointer ke parameter kontroler baseline (read-only)
 * 
 * Fungsi ini melakukan:
 * 1. Loop untuk berbagai nilai Kp (1.0, 5.0, 10.0, 15.0, 20.0, 50.0)
 * 2. Simulasi sistem untuk setiap nilai Kp
 * 3. Perhitungan metrik performa untuk setiap konfigurasi
 * 4. Tampilan hasil dalam format tabel
 * 5. Penyimpanan hasil analisis ke file CSV
 * 
 * Output: file stability_analysis.csv dan tampilan tabel performa
 */
void analyze_stability_range(SystemParams *params, ControllerParams *controller);

/**
 * ================================================================================
 * PROTOTYPE FUNGSI ANALISIS PARAMETER CUSTOM
 * ================================================================================
 * 
 * Fungsi ini memungkinkan user untuk melakukan analisis stabilitas dengan
 * rentang parameter yang dapat dikustomisasi sesuai kebutuhan penelitian
 * atau eksperimen tertentu.
 * 
 * Parameter:
 * - params: pointer ke parameter sistem (read-only)
 * - controller: pointer ke parameter kontroler baseline (read-only)
 * 
 * Fungsi ini melakukan:
 * 1. Input rentang parameter dari user (start, end, step)
 * 2. Validasi input dan batasan jumlah analisis
 * 3. Loop analisis untuk rentang parameter yang ditentukan
 * 4. Simulasi dan perhitungan metrik untuk setiap nilai
 * 5. Tampilan hasil dan penyimpanan ke file CSV
 * 
 * Fleksibilitas:
 * - User dapat menentukan rentang Kp sesuai kebutuhan
 * - Dapat digunakan untuk fine-tuning parameter
 * - Hasil disimpan dengan nama file berbeda (custom_analysis.csv)
 */
void analyze_custom_parameters(SystemParams *params, ControllerParams *controller);

// ===============================================================================
// PENUTUP HEADER GUARD
// ===============================================================================

/**
 * Akhiri header guard yang dimulai dengan #ifndef SYSTEM_DYNAMICS_H
 * Semua definisi di atas akan di-include hanya sekali per compilation unit.
 */
#endif

/**
 * ================================================================================
 * CATATAN PENGGUNAAN HEADER FILE
 * ================================================================================
 * 
 * File header ini harus di-include di semua file .c yang menggunakan:
 * - Struktur data sistem (SystemParams, SystemState, ControllerParams, AnalysisResult)
 * - Function prototypes untuk simulasi dan analisis
 * 
 * Cara include:
 * #include "system_dynamics.h"  // untuk file dalam folder yang sama
 * #include "../utils/system_dynamics.h"  // untuk file di folder berbeda
 * 
 * Dependencies:
 * - File ini tidak memiliki dependency eksternal selain standard C library
 * - Semua tipe data menggunakan built-in types (double, int)
 * - Compatible dengan C99 standard dan newer
 * 
 * Memory Management:
 * - Semua struct menggunakan stack allocation (bukan pointer ke heap)
 * - Tidak ada dynamic memory allocation di level struct definition
 * - Dynamic allocation dilakukan di fungsi implementasi jika diperlukan
 * ================================================================================
 */