/**
 * ================================================================================
 * PROGRAM SIMULASI SISTEM KONTROL MASSA-PEGAS-DAMPER
 * Menggunakan Metode Runge-Kutta Orde 4 dan Kontroler PID
 * ================================================================================
 * 
 * Author: Daffa Hardhan
 * NPM: 2306161763
 * Jurusan: Teknik Komputer
 * Universitas Indonesia
 * 
 * Deskripsi Program:
 * Program ini mengimplementasikan simulasi numerik sistem massa-pegas-damper
 * dengan kontroler PID untuk analisis stabilitas sistem kontrol menggunakan
 * metode Runge-Kutta orde keempat.
 * 
 * Fitur Utama:
 * 1. Simulasi dengan parameter default (sesuai laporan penelitian)
 * 2. Analisis stabilitas dengan variasi parameter kontroler
 * 3. Input parameter custom untuk eksperimen
 * 4. Analisis performa sistem (settling time, overshoot, steady-state error)
 * 5. Penyimpanan hasil simulasi dalam format CSV
 * 
 * Persamaan Sistem:
 * m*x'' + c*x' + k*x = F_control
 * dimana F_control = Kp*e + Ki*∫e*dt + Kd*de/dt
 * ================================================================================
 */

// ===============================================================================
// INCLUDE LIBRARIES
// ===============================================================================
#include <stdio.h>          // Standard I/O untuk printf, scanf, file operations
#include <stdlib.h>         // Standard library untuk system(), malloc(), exit()
#include <math.h>           // Library matematika untuk sqrt(), sin(), cos(), fabs()
#include "../utils/system_dynamics.h"  // Header custom untuk fungsi dinamika sistem
#include "../utils/utils.h"            // Header custom untuk utility functions

/**
 * ================================================================================
 * FUNGSI MAIN - ENTRY POINT PROGRAM
 * ================================================================================
 * 
 * Fungsi utama yang mengatur alur program dan menyediakan interface menu
 * interaktif untuk berbagai mode simulasi dan analisis sistem kontrol.
 * 
 * Return: 0 jika program berhasil dijalankan
 */
int main() {
    
    // ===============================================================================
    // INISIALISASI DAN DISPLAY HEADER
    // ===============================================================================
    
    /**
     * Menampilkan header program yang berisi informasi tentang:
     * - Judul program
     * - Deskripsi singkat
     * - Informasi author
     * - Petunjuk penggunaan
     */
    print_header();
    
    // ===============================================================================
    // INISIALISASI PARAMETER SISTEM MASSA-PEGAS-DAMPER
    // ===============================================================================
    
    /**
     * Parameter sistem massa-pegas-damper yang digunakan dalam penelitian.
     * Parameter ini dipilih berdasarkan:
     * 
     * 1. MASSA (m = 1.0 kg):
     *    - Massa objek yang bergerak dalam sistem
     *    - Mempengaruhi inersia dan frekuensi natural sistem
     *    - Nilai 1 kg dipilih untuk kemudahan perhitungan
     * 
     * 2. DAMPING (c = 2.0 Ns/m):
     *    - Koefisien redaman yang meredam osilasi
     *    - Menentukan tingkat peredaman dalam sistem
     *    - Nilai ini menghasilkan sistem underdamped (ζ < 1)
     * 
     * 3. STIFFNESS (k = 10.0 N/m):
     *    - Konstanta pegas yang menentukan kekakuan sistem
     *    - Mempengaruhi frekuensi natural: ωn = √(k/m) = √10 ≈ 3.16 rad/s
     * 
     * 4. TIME STEP (dt = 0.001 s):
     *    - Langkah waktu untuk integrasi numerik Runge-Kutta
     *    - Nilai kecil untuk akurasi tinggi (1 ms)
     *    - Trade-off antara akurasi dan computational cost
     * 
     * 5. FINAL TIME (t_final = 10.0 s):
     *    - Durasi total simulasi
     *    - Cukup untuk menganalisis settling time dan steady-state
     */
    SystemParams params = {
        .mass = 1.0,        // massa 1 kg
        .damping = 2.0,     // koefisien redaman 2 Ns/m
        .stiffness = 10.0,  // konstanta pegas 10 N/m
        .dt = 0.001,        // time step 1 ms
        .t_final = 10.0     // simulasi 10 detik
    };
    
    // ===============================================================================
    // INISIALISASI PARAMETER KONTROLER PID
    // ===============================================================================
    
    /**
     * Parameter kontroler PID yang telah dioptimasi melalui tuning.
     * Setiap parameter memiliki fungsi spesifik:
     * 
     * 1. PROPORTIONAL GAIN (Kp = 15.0):
     *    - Mengontrol respons proporsional terhadap error saat ini
     *    - Nilai tinggi: respons cepat, tetapi dapat menyebabkan overshoot
     *    - Nilai rendah: respons lambat, tetapi lebih stabil
     * 
     * 2. INTEGRAL GAIN (Ki = 5.0):
     *    - Mengeliminasi steady-state error dengan mengintegralkan error
     *    - Nilai tinggi: eliminasi error cepat, tetapi dapat menyebabkan overshoot
     *    - Nilai rendah: eliminasi error lambat
     * 
     * 3. DERIVATIVE GAIN (Kd = 3.0):
     *    - Memprediksi tren error masa depan berdasarkan rate of change
     *    - Meredam overshoot dan meningkatkan stabilitas
     *    - Nilai tinggi: damping kuat, tetapi sensitif terhadap noise
     * 
     * 4. SETPOINT (target = 1.0 m):
     *    - Posisi target yang ingin dicapai sistem
     *    - Referensi untuk perhitungan error: e(t) = setpoint - x(t)
     * 
     * 5. INTEGRAL ACCUMULATOR (integral = 0.0):
     *    - Akumulator untuk komponen integral kontroler
     *    - Diinisialisasi nol untuk kondisi awal bersih
     * 
     * 6. PREVIOUS ERROR (prev_error = 1.0):
     *    - Error sebelumnya untuk perhitungan komponen derivative
     *    - Diinisialisasi dengan setpoint karena posisi awal x(0) = 0
     */
    ControllerParams controller = {
        .Kp = 15.0,         // proportional gain
        .Ki = 5.0,          // integral gain
        .Kd = 3.0,          // derivative gain
        .setpoint = 1.0,    // target posisi 1 meter
        .integral = 0.0,    // reset integral
        .prev_error = 1.0   // initial error = setpoint
    };
    
    // ===============================================================================
    // SETUP DIREKTORI OUTPUT
    // ===============================================================================
    
    /**
     * Membuat direktori 'output' untuk menyimpan hasil simulasi.
     * Command Windows dengan error suppression:
     * - mkdir output: membuat direktori bernama 'output'
     * - 2>nul: redirect error message ke null (suppress error)
     * - ||: logical OR operator
     * - echo Directory already exists: pesan jika direktori sudah ada
     */
    system("mkdir output 2>nul || echo Directory already exists");
    
    // ===============================================================================
    // VARIABEL UNTUK MENU SELECTION
    // ===============================================================================
    
    /**
     * Variable untuk menyimpan pilihan menu dari user.
     * Digunakan dalam loop do-while untuk kontrol alur program.
     */
    int choice;
    
    // ===============================================================================
    // MAIN PROGRAM LOOP - MENU INTERAKTIF
    // ===============================================================================
    
    /**
     * Loop utama program yang menyediakan menu interaktif.
     * Menggunakan do-while untuk memastikan menu ditampilkan minimal sekali.
     * Loop berlanjut hingga user memilih opsi keluar (choice = 0).
     */
    do {
        
        // ===========================================================================
        // DISPLAY MENU DAN INPUT USER
        // ===========================================================================
        
        /**
         * Menampilkan menu utama dengan opsi-opsi yang tersedia:
         * 1. Simulasi dengan parameter default
         * 2. Analisis stabilitas rentang parameter
         * 3. Simulasi dengan parameter custom
         * 4. Analisis parameter custom
         * 5. Tampilkan parameter saat ini
         * 0. Keluar
         */
        print_main_menu();
        
        /**
         * Mengambil input pilihan dari user dengan validasi.
         * Fungsi ini menangani:
         * - Input validation
         * - Error handling untuk input non-numerik
         * - Range checking untuk pilihan menu
         */
        choice = get_user_choice();
        
        // ===========================================================================
        // SWITCH CASE UNTUK MENANGANI PILIHAN MENU
        // ===========================================================================
        
        /**
         * Switch statement untuk menjalankan fungsi yang sesuai
         * berdasarkan pilihan user. Setiap case menangani skenario berbeda.
         */
        switch(choice) {
            
            // =======================================================================
            // CASE 1: SIMULASI DENGAN PARAMETER DEFAULT
            // =======================================================================
            
            /**
             * Menjalankan simulasi menggunakan parameter default yang telah
             * ditetapkan dalam penelitian. Mode ini ideal untuk:
             * - Reproduksi hasil penelitian
             * - Validasi implementasi algoritma
             * - Baseline comparison untuk parameter lain
             */
            case 1:
                printf("\n=== SIMULASI DENGAN PARAMETER DEFAULT ===\n");
                
                // Tampilkan parameter sistem yang akan digunakan
                print_system_parameters(&params);
                
                // Tampilkan parameter kontroler yang akan digunakan
                print_controller_parameters(&controller);
                
                /**
                 * Jalankan simulasi dengan:
                 * - params: parameter sistem massa-pegas-damper
                 * - controller: parameter kontroler PID
                 * - 1: flag untuk menyimpan data ke file CSV
                 */
                simulate_system(&params, &controller, 1);
                
                // Pause program untuk memberi waktu user membaca hasil
                pause_program();
                break;
                
            // =======================================================================
            // CASE 2: ANALISIS STABILITAS RENTANG PARAMETER
            // =======================================================================
            
            /**
             * Melakukan analisis stabilitas dengan memvariasikan parameter
             * kontroler dalam rentang tertentu. Analisis ini menghasilkan:
             * - Tabel performa untuk berbagai nilai Kp, Ki, Kd
             * - Grafik settling time vs parameter
             * - Rekomendasi parameter optimal
             * - Batas stabilitas sistem
             */
            case 2:
                printf("\n=== ANALISIS STABILITAS RENTANG PARAMETER ===\n");
                
                /**
                 * Fungsi yang melakukan sweep parameter dan analisis:
                 * - Variasi Kp: 1, 5, 10, 15, 20, 50
                 * - Variasi Ki: 0, 1, 5, 10
                 * - Variasi Kd: 0, 1, 3, 5
                 * - Perhitungan metrik performa untuk setiap kombinasi
                 */
                analyze_stability_range(&params, &controller);
                pause_program();
                break;
                
            // =======================================================================
            // CASE 3: SIMULASI DENGAN PARAMETER CUSTOM
            // =======================================================================
            
            /**
             * Memungkinkan user untuk memasukkan parameter sistem dan kontroler
             * secara manual. Mode ini berguna untuk:
             * - Eksperimen dengan konfigurasi berbeda
             * - Testing edge cases
             * - Educational purposes
             * - Research parameter baru
             */
            case 3:
                printf("\n=== SIMULASI DENGAN PARAMETER CUSTOM ===\n");
                
                /**
                 * Input parameter sistem dari user:
                 * - Massa (kg)
                 * - Koefisien redaman (Ns/m)
                 * - Konstanta pegas (N/m)
                 * - Time step (s)
                 * - Waktu simulasi (s)
                 */
                input_system_parameters(&params);
                
                /**
                 * Input parameter kontroler dari user:
                 * - Kp (proportional gain)
                 * - Ki (integral gain)
                 * - Kd (derivative gain)
                 * - Setpoint (target position)
                 */
                input_controller_parameters(&controller);
                
                // Jalankan simulasi dengan parameter custom
                simulate_system(&params, &controller, 1);
                pause_program();
                break;
                
            // =======================================================================
            // CASE 4: ANALISIS PARAMETER CUSTOM
            // =======================================================================
            
            /**
             * Melakukan analisis mendalam terhadap parameter custom yang
             * diinputkan user. Analisis meliputi:
             * - Karakteristik sistem (ωn, ζ, type)
             * - Prediksi performa berdasarkan teori kontrol
             * - Simulasi dan perbandingan dengan prediksi
             * - Rekomendasi perbaikan parameter
             */
            case 4:
                printf("\n=== ANALISIS PARAMETER CUSTOM ===\n");
                
                /**
                 * Fungsi analisis yang menggabungkan:
                 * - Input parameter custom
                 * - Perhitungan karakteristik teoretis
                 * - Simulasi numerik
                 * - Perbandingan hasil
                 * - Visualisasi data
                 */
                analyze_custom_parameters(&params, &controller);
                pause_program();
                break;
                
            // =======================================================================
            // CASE 5: TAMPILKAN PARAMETER SAAT INI
            // =======================================================================
            
            /**
             * Menampilkan parameter sistem dan kontroler yang sedang aktif.
             * Berguna untuk:
             * - Verifikasi parameter sebelum simulasi
             * - Debugging
             * - Dokumentasi konfigurasi
             */
            case 5:
                printf("\n=== PARAMETER SISTEM SAAT INI ===\n");
                
                // Tampilkan parameter sistem dalam format tabel
                print_system_parameters(&params);
                
                // Tampilkan parameter kontroler dalam format tabel
                print_controller_parameters(&controller);
                pause_program();
                break;
                
            // =======================================================================
            // CASE 0: KELUAR DARI PROGRAM
            // =======================================================================
            
            /**
             * Menangani opsi keluar dari program dengan:
             * - Pesan penutup yang informatif
             * - Informasi lokasi file hasil
             * - Cleanup (jika diperlukan)
             */
            case 0:
                printf("\n=== PROGRAM SELESAI ===\n");
                printf("Terima kasih telah menggunakan program simulasi!\n");
                printf("Hasil tersimpan di folder 'output/'\n");
                break;
                
            // =======================================================================
            // DEFAULT CASE: PENANGANAN INPUT TIDAK VALID
            // =======================================================================
            
            /**
             * Menangani input yang tidak sesuai dengan opsi menu.
             * Memberikan feedback kepada user dan kembali ke menu utama.
             */
            default:
                printf("\nPilihan tidak valid! Silakan coba lagi.\n");
                pause_program();
                break;
        }
        
    } while(choice != 0);  // Loop berlanjut selama user tidak memilih keluar
    
    // ===============================================================================
    // PROGRAM TERMINATION
    // ===============================================================================
    
    /**
     * Program berakhir dengan return code 0 yang menandakan
     * eksekusi sukses tanpa error. Return value ini dapat digunakan
     * oleh sistem operasi atau script lain untuk mengecek status program.
     */
    return 0;
}