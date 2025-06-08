/**
 * ================================================================================
 * UTILITY FUNCTIONS - FUNGSI PENDUKUNG PROGRAM SIMULASI
 * ================================================================================
 * 
 * File ini berisi implementasi fungsi-fungsi utility yang mendukung program
 * simulasi sistem massa-pegas-damper, meliputi:
 * 
 * 1. User Interface Functions - tampilan menu dan header
 * 2. Input/Output Functions - input parameter dan save hasil
 * 3. Analysis Functions - perhitungan metrik performa sistem
 * 4. Parameter Display Functions - tampilan parameter sistem
 * 
 * Author: Daffa Hardhan
 * NPM: 2306161763
 * Jurusan: Teknik Komputer - Universitas Indonesia
 * ================================================================================
 */

// ===============================================================================
// INCLUDE LIBRARIES DAN HEADER FILES
// ===============================================================================
#include <stdio.h>          // Standard I/O untuk printf, scanf, file operations
#include <stdlib.h>         // Standard library untuk system functions
#include <math.h>           // Library matematika untuk sqrt, fabs, dll
#include <time.h>           // Library time untuk timestamp (jika diperlukan)
#include "../utils/utils.h" // Header file dengan definisi struct dan prototypes

/**
 * ================================================================================
 * FUNGSI TAMPILAN HEADER PROGRAM
 * ================================================================================
 * 
 * Menampilkan header program yang berisi informasi tentang:
 * - Judul program dan deskripsi
 * - Metode yang digunakan (Runge-Kutta Orde 4)
 * - Informasi mata kuliah dan proyek
 * - Data mahasiswa (nama dan NIM)
 * 
 * Fungsi ini dipanggil sekali di awal program untuk memberikan konteks
 * kepada user tentang program yang sedang dijalankan.
 */
void print_header(void) {
    printf("============================================================\n");
    printf("    ANALISIS STABILITAS SISTEM MASSA-PEGAS-DAMPER\n");
    printf("        Menggunakan Metode Runge-Kutta Orde 4\n");
    printf("============================================================\n");
    printf("Proyek UAS Komputasi Numerik\n");
    printf("Nama: Daffa Hardhan\n");
    printf("NIM:  2306161763\n");
    printf("============================================================\n");
}

/**
 * ================================================================================
 * FUNGSI TAMPILAN MENU UTAMA
 * ================================================================================
 * 
 * Menampilkan menu utama program dengan opsi-opsi yang tersedia:
 * 1. Simulasi default - menggunakan parameter dari laporan penelitian
 * 2. Analisis stabilitas - variasi parameter Kp untuk analisis
 * 3. Simulasi custom - user dapat input parameter sendiri
 * 4. Analisis custom - analisis dengan rentang parameter custom
 * 5. Display parameter - menampilkan parameter sistem saat ini
 * 0. Keluar - terminasi program
 * 
 * Menu dirancang user-friendly dengan penjelasan singkat setiap opsi.
 */
void print_main_menu(void) {
    printf("\n============================================================\n");
    printf("                       MENU UTAMA\n");
    printf("============================================================\n");
    printf("1. Simulasi dengan Parameter Default (Laporan)\n");
    printf("2. Analisis Stabilitas Rentang Parameter Kp\n");
    printf("3. Simulasi dengan Parameter Custom\n");
    printf("4. Analisis Parameter Custom\n");
    printf("5. Tampilkan Parameter Sistem Saat Ini\n");
    printf("0. Keluar Program\n");
    printf("============================================================\n");
    printf("Pilihan Anda: ");
}

/**
 * ================================================================================
 * FUNGSI INPUT VALIDASI PILIHAN USER
 * ================================================================================
 * 
 * Fungsi ini menangani input pilihan menu dari user dengan validasi:
 * - Memastikan input berupa angka integer
 * - Menangani error jika user memasukkan non-numerik
 * - Membersihkan input buffer untuk mencegah error cascading
 * 
 * Loop while digunakan untuk terus meminta input hingga valid.
 * 
 * Return: integer pilihan menu yang valid
 */
int get_user_choice(void) {
    int choice;
    /**
     * Loop validasi input:
     * - scanf mengembalikan jumlah item yang berhasil dibaca
     * - Jika tidak sama dengan 1, berarti input tidak valid
     * - Clear input buffer untuk menghindari infinite loop
     */
    while (scanf("%d", &choice) != 1) {
        printf("Input tidak valid! Masukkan angka: ");
        while (getchar() != '\n'); // Clear input buffer hingga newline
    }
    return choice;
}

/**
 * ================================================================================
 * FUNGSI TAMPILAN PARAMETER SISTEM MASSA-PEGAS-DAMPER
 * ================================================================================
 * 
 * Menampilkan parameter sistem dalam format yang mudah dibaca, meliputi:
 * 
 * 1. Parameter Fisik Sistem:
 *    - Massa (m): mempengaruhi inersia sistem
 *    - Koefisien Redaman (c): menentukan tingkat peredaman
 *    - Konstanta Pegas (k): menentukan kekakuan sistem
 *    - Time Step (dt): langkah waktu integrasi numerik
 *    - Waktu Simulasi: durasi total simulasi
 * 
 * 2. Karakteristik Sistem Teoretis:
 *    - Frekuensi Natural (ωn): frekuensi osilasi bebas sistem
 *    - Damping Ratio (ζ): rasio redaman terhadap critical damping
 *    - Jenis Sistem: klasifikasi berdasarkan damping ratio
 * 
 * Parameter:
 * - params: pointer ke struct SystemParams yang berisi parameter sistem
 */
void print_system_parameters(SystemParams *params) {
    printf("\n=== PARAMETER SISTEM ===\n");
    printf("Massa (m)           : %.3f kg\n", params->mass);
    printf("Koefisien Redaman (c): %.3f Ns/m\n", params->damping);
    printf("Konstanta Pegas (k) : %.3f N/m\n", params->stiffness);
    printf("Time Step (dt)      : %.6f s\n", params->dt);
    printf("Waktu Simulasi      : %.3f s\n", params->t_final);
    
    /**
     * Perhitungan karakteristik sistem berdasarkan teori kontrol:
     * 
     * 1. Frekuensi Natural (ωn):
     *    ωn = √(k/m)
     *    Menentukan frekuensi osilasi natural sistem tanpa redaman
     * 
     * 2. Damping Ratio (ζ):
     *    ζ = c / (2√(mk))
     *    Menentukan jenis respons sistem (underdamped, critical, overdamped)
     */
    double wn = sqrt(params->stiffness / params->mass);
    double zeta = params->damping / (2.0 * sqrt(params->mass * params->stiffness));
    
    printf("\nKarakteristik Sistem:\n");
    printf("Frekuensi Natural (ωn): %.3f rad/s\n", wn);
    printf("Damping Ratio (ζ)     : %.3f\n", zeta);
    
    /**
     * Klasifikasi sistem berdasarkan damping ratio:
     * - ζ < 1: Underdamped (akan berosilasi sebelum mencapai steady-state)
     * - ζ = 1: Critically Damped (respons tercepat tanpa overshoot)
     * - ζ > 1: Overdamped (respons lambat tanpa osilasi)
     */
    if (zeta < 1.0) {
        printf("Jenis Sistem: Underdamped (akan berosilasi)\n");
    } else if (zeta == 1.0) {
        printf("Jenis Sistem: Critically Damped\n");
    } else {
        printf("Jenis Sistem: Overdamped\n");
    }
}

/**
 * ================================================================================
 * FUNGSI TAMPILAN PARAMETER KONTROLER PID
 * ================================================================================
 * 
 * Menampilkan parameter kontroler PID dalam format yang mudah dibaca:
 * - Kp (Proportional Gain): mengontrol respons proporsional terhadap error
 * - Ki (Integral Gain): mengeliminasi steady-state error
 * - Kd (Derivative Gain): meredam overshoot dan meningkatkan stabilitas
 * - Setpoint: nilai target yang ingin dicapai sistem
 * 
 * Parameter:
 * - controller: pointer ke struct ControllerParams
 */
void print_controller_parameters(ControllerParams *controller) {
    printf("\n=== PARAMETER KONTROLER PID ===\n");
    printf("Proportional Gain (Kp): %.3f\n", controller->Kp);
    printf("Integral Gain (Ki)    : %.3f\n", controller->Ki);
    printf("Derivative Gain (Kd)  : %.3f\n", controller->Kd);
    printf("Setpoint              : %.3f m\n", controller->setpoint);
}

/**
 * ================================================================================
 * FUNGSI PENYIMPANAN HASIL SIMULASI KE FILE CSV
 * ================================================================================
 * 
 * Menyimpan hasil simulasi dalam format CSV untuk analisis lanjutan dan plotting.
 * File CSV berisi kolom:
 * - Time_s: waktu dalam detik
 * - Position_m: posisi massa dalam meter
 * - Velocity_ms: kecepatan massa dalam m/s
 * - Control_Force_N: gaya kontrol dalam Newton
 * 
 * Format CSV dipilih karena:
 * - Mudah dibaca oleh software analisis (Excel, MATLAB, Python)
 * - Human-readable untuk debugging
 * - Standar untuk data exchange
 * 
 * Parameter:
 * - time: array waktu simulasi
 * - position: array posisi massa
 * - velocity: array kecepatan massa
 * - control_force: array gaya kontrol
 * - n_points: jumlah titik data
 * - filename: nama file output
 */
void save_results_to_csv(double *time, double *position, double *velocity, 
                        double *control_force, int n_points, const char* filename) {
    
    /**
     * Buka file untuk writing, mode "w" akan membuat file baru
     * atau overwrite file yang sudah ada
     */
    FILE *file = fopen(filename, "w");
    if (!file) {
        printf("Warning: Tidak dapat membuat file %s!\n", filename);
        return;
    }
    
    /**
     * Tulis header CSV dengan nama kolom yang deskriptif
     * Header memudahkan interpretasi data saat dibuka di software lain
     */
    fprintf(file, "Time_s,Position_m,Velocity_ms,Control_Force_N\n");
    
    /**
     * Tulis data simulasi baris per baris
     * Format %.6f memberikan presisi 6 digit setelah koma
     * untuk akurasi data yang tinggi
     */
    for (int i = 0; i < n_points; i++) {
        fprintf(file, "%.6f,%.6f,%.6f,%.6f\n", 
                time[i], position[i], velocity[i], control_force[i]);
    }
    
    /**
     * Tutup file dan berikan konfirmasi ke user
     */
    fclose(file);
    printf("Hasil simulasi disimpan ke: %s\n", filename);
}

/**
 * ================================================================================
 * FUNGSI PENYIMPANAN HASIL ANALISIS STABILITAS KE FILE CSV
 * ================================================================================
 * 
 * Menyimpan hasil analisis stabilitas (variasi parameter) dalam format CSV.
 * File berisi hasil analisis untuk berbagai nilai parameter kontroler:
 * - Kp: nilai proportional gain yang dianalisis
 * - Settling_Time_s: waktu settling dalam detik
 * - Overshoot_percent: overshoot dalam persen
 * - Steady_State_Error_percent: steady-state error dalam persen
 * - Is_Stable: status stabilitas (STABLE/UNSTABLE)
 * 
 * Parameter:
 * - kp_values: array nilai Kp yang dianalisis
 * - results: array hasil analisis (AnalysisResult struct)
 * - n_values: jumlah nilai yang dianalisis
 * - filename: nama file output
 */
void save_stability_analysis_to_csv(double *kp_values, AnalysisResult *results, 
                                   int n_values, const char* filename) {
    
    /**
     * Buka file untuk writing
     */
    FILE *file = fopen(filename, "w");
    if (!file) {
        printf("Warning: Tidak dapat membuat file %s!\n", filename);
        return;
    }
    
    /**
     * Tulis header CSV untuk analisis stabilitas
     * Header menjelaskan setiap kolom dan satuannya
     */
    fprintf(file, "Kp,Settling_Time_s,Overshoot_percent,Steady_State_Error_percent,Is_Stable\n");
    
    /**
     * Tulis data analisis untuk setiap nilai Kp
     * String "STABLE"/"UNSTABLE" untuk kolom boolean
     */
    for (int i = 0; i < n_values; i++) {
        fprintf(file, "%.3f,%.6f,%.3f,%.3f,%s\n", 
                kp_values[i], 
                results[i].settling_time,
                results[i].overshoot,
                results[i].steady_state_error,
                results[i].is_stable ? "STABLE" : "UNSTABLE");
    }
    
    /**
     * Tutup file (tidak perlu konfirmasi karena dipanggil dari fungsi lain)
     */
    fclose(file);
}

/**
 * ================================================================================
 * FUNGSI PERHITUNGAN SETTLING TIME
 * ================================================================================
 * 
 * Menghitung settling time sistem kontrol berdasarkan kriteria ±2% tolerance.
 * Settling time adalah waktu yang diperlukan sistem untuk mencapai dan tetap
 * berada dalam band toleransi di sekitar setpoint.
 * 
 * Algoritma:
 * 1. Definisikan upper dan lower bound berdasarkan tolerance
 * 2. Scan dari akhir ke awal array posisi
 * 3. Cari titik terakhir dimana posisi keluar dari band toleransi
 * 4. Settling time = waktu setelah titik tersebut
 * 
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - time: array waktu yang bersesuaian
 * - n_points: jumlah titik data
 * - tolerance: toleransi dalam bentuk desimal (0.02 = 2%)
 * 
 * Return: settling time dalam detik
 */
double calculate_settling_time(double *position, double *time, int n_points, double tolerance) {
    
    /**
     * Setpoint diasumsikan 1.0 m (sesuai dengan konfigurasi default)
     * Dalam implementasi yang lebih robust, ini bisa diambil sebagai parameter
     */
    double setpoint = 1.0;
    
    /**
     * Hitung batas atas dan bawah band toleransi:
     * - Upper bound: setpoint × (1 + tolerance)
     * - Lower bound: setpoint × (1 - tolerance)
     * 
     * Contoh untuk tolerance 2%:
     * - Upper bound: 1.0 × 1.02 = 1.02 m
     * - Lower bound: 1.0 × 0.98 = 0.98 m
     */
    double upper_bound = setpoint * (1.0 + tolerance);
    double lower_bound = setpoint * (1.0 - tolerance);
    
    /**
     * Scan dari belakang ke depan untuk mencari titik terakhir
     * dimana sistem keluar dari band toleransi
     */
    for (int i = n_points - 1; i >= 0; i--) {
        if (position[i] > upper_bound || position[i] < lower_bound) {
            /**
             * Jika ditemukan titik keluar band, settling time adalah
             * waktu di titik berikutnya (atau waktu final jika di ujung)
             */
            return (i < n_points - 1) ? time[i + 1] : time[n_points - 1];
        }
    }
    
    /**
     * Jika sistem selalu dalam band toleransi, settling time = 0
     * (sistem langsung stabil dari awal)
     */
    return time[0];
}

/**
 * ================================================================================
 * FUNGSI PERHITUNGAN OVERSHOOT
 * ================================================================================
 * 
 * Menghitung overshoot sistem kontrol sebagai persentase nilai maksimum
 * terhadap setpoint. Overshoot mengindikasikan seberapa jauh sistem
 * "melampaui" target sebelum stabil.
 * 
 * Formula: Overshoot = ((nilai_max - setpoint) / setpoint) × 100%
 * 
 * Overshoot yang tinggi menunjukkan:
 * - Sistem terlalu agresif
 * - Gain proportional mungkin terlalu tinggi
 * - Perlu tuning parameter kontroler
 * 
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - n_points: jumlah titik data
 * - setpoint: nilai target sistem
 * 
 * Return: overshoot dalam persen
 */
double calculate_overshoot(double *position, int n_points, double setpoint) {
    
    /**
     * Inisialisasi nilai maksimum dengan posisi awal
     */
    double max_value = position[0];
    
    /**
     * Scan seluruh array untuk mencari nilai maksimum
     * Linear search O(n) - efisien untuk sekali perhitungan
     */
    for (int i = 1; i < n_points; i++) {
        if (position[i] > max_value) {
            max_value = position[i];
        }
    }
    
    /**
     * Hitung overshoot sebagai persentase
     * Cek setpoint != 0 untuk menghindari division by zero
     */
    if (setpoint != 0) {
        return ((max_value - setpoint) / setpoint) * 100.0;
    } else {
        return 0.0;  // Return 0 jika setpoint = 0
    }
}

/**
 * ================================================================================
 * FUNGSI PERHITUNGAN STEADY-STATE ERROR
 * ================================================================================
 * 
 * Menghitung steady-state error sistem sebagai perbedaan antara nilai final
 * sistem dengan setpoint. Menggunakan rata-rata 100 titik terakhir untuk
 * mengurangi noise dan mendapatkan estimasi steady-state yang lebih akurat.
 * 
 * Formula: SS_Error = |setpoint - nilai_final| / setpoint × 100%
 * 
 * Steady-state error yang besar mengindikasikan:
 * - Gain integral kurang memadai
 * - Ada bias sistemik dalam sistem
 * - Perlu tuning parameter Ki
 * 
 * Parameter:
 * - position: array posisi massa sepanjang simulasi
 * - n_points: jumlah titik data
 * - setpoint: nilai target sistem
 * 
 * Return: steady-state error dalam persen
 */
double calculate_steady_state_error(double *position, int n_points, double setpoint) {
    
    /**
     * Validasi jumlah data minimum untuk perhitungan rata-rata
     * Jika kurang dari 100 titik, tidak cukup data untuk analisis
     */
    if (n_points < 100) return 0.0;
    
    /**
     * Hitung rata-rata 100 titik terakhir untuk estimasi nilai steady-state
     * Averaging mengurangi efek noise dan fluktuasi kecil
     */
    double sum = 0.0;
    for (int i = n_points - 100; i < n_points; i++) {
        sum += position[i];
    }
    double final_value = sum / 100.0;
    
    /**
     * Hitung steady-state error sebagai persentase absolut
     * Menggunakan fabs() untuk mendapatkan nilai absolut
     */
    if (setpoint != 0) {
        return fabs((setpoint - final_value) / setpoint) * 100.0;
    } else {
        /**
         * Jika setpoint = 0, error = nilai absolut final value
         * (karena target adalah nol)
         */
        return fabs(final_value) * 100.0;
    }
}

/**
 * ================================================================================
 * FUNGSI INPUT PARAMETER SISTEM CUSTOM
 * ================================================================================
 * 
 * Memungkinkan user untuk memasukkan parameter sistem massa-pegas-damper
 * secara interaktif. Fungsi ini menampilkan nilai saat ini sebagai referensi
 * sebelum meminta input baru.
 * 
 * Parameter yang dapat diubah:
 * - Massa (m): mempengaruhi inersia dan frekuensi natural
 * - Koefisien redaman (c): mempengaruhi damping ratio
 * - Konstanta pegas (k): mempengaruhi frekuensi natural
 * - Waktu simulasi: durasi total simulasi
 * 
 * Time step (dt) tidak diubah untuk menjaga akurasi numerik konsisten.
 * 
 * Parameter:
 * - params: pointer ke struct SystemParams yang akan dimodifikasi
 */
void input_system_parameters(SystemParams *params) {
    
    printf("\n=== INPUT PARAMETER SISTEM ===\n");
    
    /**
     * Input massa dengan tampilan nilai saat ini sebagai referensi
     * Format %.3f memberikan 3 digit setelah koma untuk readability
     */
    printf("Massa saat ini: %.3f kg\n", params->mass);
    printf("Masukkan massa baru (kg): ");
    scanf("%lf", &params->mass);
    
    /**
     * Input koefisien redaman
     * Nilai ini mempengaruhi seberapa cepat osilasi sistem meredam
     */
    printf("Koefisien redaman saat ini: %.3f Ns/m\n", params->damping);
    printf("Masukkan koefisien redaman baru (Ns/m): ");
    scanf("%lf", &params->damping);
    
    /**
     * Input konstanta pegas
     * Nilai ini menentukan kekakuan sistem dan frekuensi natural
     */
    printf("Konstanta pegas saat ini: %.3f N/m\n", params->stiffness);
    printf("Masukkan konstanta pegas baru (N/m): ");
    scanf("%lf", &params->stiffness);
    
    /**
     * Input waktu simulasi
     * Waktu yang lebih lama memungkinkan analisis settling time yang lebih akurat
     */
    printf("Waktu simulasi saat ini: %.3f s\n", params->t_final);
    printf("Masukkan waktu simulasi baru (s): ");
    scanf("%lf", &params->t_final);
    
    /**
     * Konfirmasi bahwa parameter berhasil diupdate
     */
    printf("\nParameter sistem berhasil diupdate!\n");
}

/**
 * ================================================================================
 * FUNGSI INPUT PARAMETER KONTROLER CUSTOM
 * ================================================================================
 * 
 * Memungkinkan user untuk memasukkan parameter kontroler PID secara interaktif.
 * Setelah input, kontroler direset ke kondisi awal untuk simulasi yang bersih.
 * 
 * Parameter yang dapat diubah:
 * - Kp: proportional gain (respons proporsional terhadap error)
 * - Ki: integral gain (eliminasi steady-state error)
 * - Kd: derivative gain (damping dan prediksi tren)
 * - Setpoint: target posisi yang ingin dicapai
 * 
 * Parameter:
 * - controller: pointer ke struct ControllerParams yang akan dimodifikasi
 */
void input_controller_parameters(ControllerParams *controller) {
    
    printf("\n=== INPUT PARAMETER KONTROLER PID ===\n");
    
    /**
     * Input proportional gain
     * Nilai tinggi: respons cepat, tetapi mungkin overshoot
     * Nilai rendah: respons lambat, tetapi stabil
     */
    printf("Kp saat ini: %.3f\n", controller->Kp);
    printf("Masukkan Kp baru: ");
    scanf("%lf", &controller->Kp);
    
    /**
     * Input integral gain
     * Nilai tinggi: eliminasi error cepat, tetapi mungkin oscillatory
     * Nilai rendah: eliminasi error lambat
     */
    printf("Ki saat ini: %.3f\n", controller->Ki);
    printf("Masukkan Ki baru: ");
    scanf("%lf", &controller->Ki);
    
    /**
     * Input derivative gain
     * Nilai tinggi: damping kuat, tetapi sensitif noise
     * Nilai rendah: damping lemah
     */
    printf("Kd saat ini: %.3f\n", controller->Kd);
    printf("Masukkan Kd baru: ");
    scanf("%lf", &controller->Kd);
    
    /**
     * Input setpoint (target posisi)
     * Menentukan target yang ingin dicapai sistem
     */
    printf("Setpoint saat ini: %.3f m\n", controller->setpoint);
    printf("Masukkan setpoint baru (m): ");
    scanf("%lf", &controller->setpoint);
    
    /**
     * Reset kontroler ke kondisi awal setelah parameter berubah:
     * - integral: reset akumulator integral ke 0
     * - prev_error: set ke setpoint (asumsi posisi awal = 0)
     * 
     * Reset ini penting untuk memastikan simulasi dimulai dari kondisi bersih
     * tanpa "memori" dari simulasi sebelumnya.
     */
    controller->integral = 0.0;
    controller->prev_error = controller->setpoint;
    
    /**
     * Konfirmasi bahwa parameter berhasil diupdate
     */
    printf("\nParameter kontroler berhasil diupdate!\n");
}

/**
 * ================================================================================
 * FUNGSI PAUSE PROGRAM
 * ================================================================================
 * 
 * Fungsi utility untuk menghentikan eksekusi program sementara dan menunggu
 * input Enter dari user. Berguna untuk:
 * - Memberikan waktu user membaca output
 * - Mengontrol flow program dalam menu interaktif
 * - Mencegah tampilan yang terlalu cepat berganti
 * 
 * Implementasi menggunakan dua tahap:
 * 1. Clear input buffer dari karakter tersisa
 * 2. Wait untuk input Enter dari user
 */
void pause_program(void) {
    printf("\nTekan Enter untuk melanjutkan...");
    
    /**
     * Clear input buffer untuk menghilangkan karakter tersisa
     * dari input sebelumnya (seperti newline setelah scanf)
     */
    while (getchar() != '\n'); // Clear input buffer hingga newline
    
    /**
     * Wait untuk user menekan Enter
     * getchar() akan menunggu hingga ada input karakter
     */
    getchar(); // Wait for Enter
}