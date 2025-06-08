/**
 * ================================================================================
 * SYSTEM DYNAMICS - IMPLEMENTASI METODE RUNGE-KUTTA DAN KONTROLER PID
 * ================================================================================
 * 
 * File ini mengimplementasikan:
 * 1. Metode Runge-Kutta orde keempat untuk integrasi numerik
 * 2. Kontroler PID untuk sistem massa-pegas-damper
 * 3. Fungsi simulasi dan analisis stabilitas sistem
 * 4. Utility untuk analisis performa sistem kontrol
 * 
 * Author: Daffa Hardhan
 * NPM: 2306161763
 * Jurusan: Teknik Komputer - Universitas Indonesia
 * ================================================================================
 */

// ===============================================================================
// INCLUDE LIBRARIES DAN HEADER FILES
// ===============================================================================
#include <stdio.h>          // Standard I/O untuk printf, scanf
#include <stdlib.h>         // Standard library untuk malloc, free
#include <math.h>           // Library matematika untuk operasi matematika
#include "../utils/system_dynamics.h"  // Header untuk definisi struct dan function prototypes
#include "../utils/utils.h"            // Header untuk utility functions

/**
 * ================================================================================
 * IMPLEMENTASI METODE RUNGE-KUTTA ORDE KEEMPAT
 * ================================================================================
 * 
 * Fungsi ini mengimplementasikan metode Runge-Kutta orde keempat untuk
 * menyelesaikan sistem persamaan diferensial massa-pegas-damper:
 * 
 * dx/dt = v (kecepatan)
 * dv/dt = (F_control - c*v - k*x) / m (percepatan)
 * 
 * Metode RK4 memberikan akurasi O(h^5) dengan error O(h^4)
 * 
 * Parameter:
 * - state: pointer ke state sistem (posisi dan kecepatan)
 * - params: pointer ke parameter sistem (massa, damping, stiffness, dt)
 * - controller: pointer ke parameter kontroler PID
 * - t: waktu saat ini (tidak digunakan dalam implementasi ini)
 */
void runge_kutta_4th_order(SystemState *state, SystemParams *params, 
                          ControllerParams *controller, double t __attribute__((unused))) {
    
    /**
     * Deklarasi variabel untuk koefisien Runge-Kutta k1, k2, k3, k4
     * - k1_x, k1_v: turunan di titik awal (t, x, v)
     * - k2_x, k2_v: turunan di titik tengah pertama (t+h/2, x+k1_x*h/2, v+k1_v*h/2)
     * - k3_x, k3_v: turunan di titik tengah kedua (t+h/2, x+k2_x*h/2, v+k2_v*h/2)
     * - k4_x, k4_v: turunan di titik akhir (t+h, x+k3_x*h, v+k3_v*h)
     */
    double k1_x, k1_v, k2_x, k2_v, k3_x, k3_v, k4_x, k4_v;
    
    /**
     * LANGKAH 1: Hitung k1 - turunan di titik awal
     * Menggunakan state saat ini untuk menghitung control force dan dynamics
     */
    double control_force = calculate_control_force(state, controller, params->dt);
    
    // Hitung turunan posisi dan kecepatan di titik awal
    system_dynamics(state, params, control_force, &k1_x, &k1_v);
    
    /**
     * LANGKAH 2: Hitung k2 - turunan di titik tengah menggunakan k1
     * Estimasi state di tengah interval menggunakan k1
     */
    SystemState temp_state = {
        .position = state->position + 0.5 * params->dt * k1_x,
        .velocity = state->velocity + 0.5 * params->dt * k1_v
    };
    control_force = calculate_control_force(&temp_state, controller, params->dt);
    system_dynamics(&temp_state, params, control_force, &k2_x, &k2_v);
    
    /**
     * LANGKAH 3: Hitung k3 - turunan di titik tengah menggunakan k2
     * Estimasi state di tengah interval menggunakan k2 (lebih akurat dari k1)
     */
    temp_state.position = state->position + 0.5 * params->dt * k2_x;
    temp_state.velocity = state->velocity + 0.5 * params->dt * k2_v;
    control_force = calculate_control_force(&temp_state, controller, params->dt);
    system_dynamics(&temp_state, params, control_force, &k3_x, &k3_v);
    
    /**
     * LANGKAH 4: Hitung k4 - turunan di titik akhir menggunakan k3
     * Estimasi state di akhir interval menggunakan k3
     */
    temp_state.position = state->position + params->dt * k3_x;
    temp_state.velocity = state->velocity + params->dt * k3_v;
    control_force = calculate_control_force(&temp_state, controller, params->dt);
    system_dynamics(&temp_state, params, control_force, &k4_x, &k4_v);
    
    /**
     * LANGKAH 5: Update state menggunakan kombinasi weighted dari k1, k2, k3, k4
     * Formula RK4: y_{n+1} = y_n + (h/6)(k1 + 2k2 + 2k3 + k4)
     * Bobot: k1=1/6, k2=2/6, k3=2/6, k4=1/6
     */
    state->position += (params->dt / 6.0) * (k1_x + 2*k2_x + 2*k3_x + k4_x);
    state->velocity += (params->dt / 6.0) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
}

/**
 * ================================================================================
 * IMPLEMENTASI KONTROLER PID
 * ================================================================================
 * 
 * Fungsi ini menghitung gaya kontrol menggunakan kontroler PID:
 * u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
 * 
 * Komponen PID:
 * - Proportional: respons proporsional terhadap error saat ini
 * - Integral: mengeliminasi steady-state error
 * - Derivative: meredam overshoot berdasarkan tren error
 * 
 * Parameter:
 * - state: state sistem saat ini
 * - controller: parameter kontroler (Kp, Ki, Kd, setpoint, integral, prev_error)
 * - dt: time step untuk integrasi dan diferensiasi numerik
 * 
 * Return: gaya kontrol yang dihitung
 */
double calculate_control_force(SystemState *state, ControllerParams *controller, double dt) {
    
    /**
     * Hitung error: selisih antara setpoint dan posisi aktual
     * Error positif: sistem berada di bawah target
     * Error negatif: sistem berada di atas target
     */
    double error = controller->setpoint - state->position;
    
    /**
     * Update komponen integral menggunakan metode Euler:
     * integral += error * dt
     * Ini adalah implementasi ∫e(τ)dτ secara numerik
     */
    controller->integral += error * dt;
    
    /**
     * Hitung komponen derivative menggunakan backward difference:
     * derivative = (error_sekarang - error_sebelumnya) / dt
     * Ini adalah implementasi de(t)/dt secara numerik
     */
    double derivative = (error - controller->prev_error) / dt;
    
    /**
     * Hitung total gaya kontrol PID:
     * F_control = Kp*e + Ki*∫e*dt + Kd*de/dt
     */
    double control_force = controller->Kp * error + 
                          controller->Ki * controller->integral + 
                          controller->Kd * derivative;
    
    /**
     * Update error sebelumnya untuk perhitungan derivative di iterasi berikutnya
     */
    controller->prev_error = error;
    
    return control_force;
}

/**
 * ================================================================================
 * FUNGSI DINAMIKA SISTEM MASSA-PEGAS-DAMPER
 * ================================================================================
 * 
 * Fungsi ini mengimplementasikan persamaan diferensial sistem massa-pegas-damper:
 * m*x'' + c*x' + k*x = F_control
 * 
 * Dikonversi ke sistem orde pertama:
 * dx/dt = v
 * dv/dt = (F_control - c*v - k*x) / m
 * 
 * Parameter:
 * - state: state sistem (posisi dan kecepatan)
 * - params: parameter sistem (m, c, k)
 * - control_force: gaya kontrol dari PID
 * - dx_dt: output turunan posisi
 * - dv_dt: output turunan kecepatan
 */
void system_dynamics(SystemState *state, SystemParams *params, double control_force, 
                     double *dx_dt, double *dv_dt) {
    
    /**
     * Turunan posisi = kecepatan
     * dx/dt = v
     */
    *dx_dt = state->velocity;
    
    /**
     * Turunan kecepatan = percepatan berdasarkan hukum Newton kedua
     * dv/dt = (ΣF) / m = (F_control - F_damping - F_spring) / m
     * 
     * Gaya-gaya yang bekerja:
     * - F_control: gaya dari kontroler PID
     * - F_damping: gaya redaman = -c * v (proporsional dengan kecepatan)
     * - F_spring: gaya pegas = -k * x (proporsional dengan displacement)
     */
    *dv_dt = (control_force - params->damping * state->velocity - 
              params->stiffness * state->position) / params->mass;
}

/**
 * ================================================================================
 * FUNGSI SIMULASI SISTEM LENGKAP
 * ================================================================================
 * 
 * Fungsi utama untuk menjalankan simulasi sistem massa-pegas-damper dengan
 * kontroler PID menggunakan metode Runge-Kutta orde keempat.
 * 
 * Parameter:
 * - params: parameter sistem (m, c, k, dt, t_final)
 * - controller: parameter kontroler PID
 * - save_to_file: flag untuk menyimpan hasil ke file CSV (1=ya, 0=tidak)
 * 
 * Return: AnalysisResult berisi metrik performa sistem
 */
AnalysisResult simulate_system(SystemParams *params, ControllerParams *controller, int save_to_file) {
    
    printf("\n=== SIMULASI SISTEM MASSA-PEGAS-DAMPER ===\n");
    
    /**
     * Inisialisasi state awal sistem:
     * - Posisi awal: 0.0 m (sistem dimulai dari origin)
     * - Kecepatan awal: 0.0 m/s (sistem dimulai dari diam)
     */
    SystemState state = {.position = 0.0, .velocity = 0.0};
    
    /**
     * Hitung jumlah titik data berdasarkan waktu simulasi dan time step:
     * n_points = (t_final / dt) + 1
     * +1 untuk memasukkan titik awal (t=0)
     */
    int n_points = (int)(params->t_final / params->dt) + 1;
    
    /**
     * Alokasi memori dinamis untuk menyimpan data simulasi:
     * - time: array waktu dari 0 hingga t_final
     * - position: array posisi massa pada setiap waktu
     * - velocity: array kecepatan massa pada setiap waktu
     * - control_force: array gaya kontrol pada setiap waktu
     */
    double *time = malloc(n_points * sizeof(double));
    double *position = malloc(n_points * sizeof(double));
    double *velocity = malloc(n_points * sizeof(double));
    double *control_force = malloc(n_points * sizeof(double));
    
    /**
     * Validasi alokasi memori - jika gagal, return hasil kosong
     */
    if (!time || !position || !velocity || !control_force) {
        printf("Error: Gagal mengalokasi memori!\n");
        AnalysisResult result = {0.0, 0.0, 0.0, 0};
        return result;
    }
    
    /**
     * Reset kontroler ke kondisi awal:
     * - integral: reset akumulator integral ke 0
     * - prev_error: set error awal = setpoint (karena posisi awal = 0)
     */
    controller->integral = 0.0;
    controller->prev_error = controller->setpoint;
    
    /**
     * Tampilkan informasi simulasi dan progress bar
     */
    printf("Memulai simulasi...\n");
    printf("Progress: ");
    
    /**
     * LOOP SIMULASI UTAMA
     * Iterasi dari t=0 hingga t=t_final dengan step dt
     */
    for (int i = 0; i < n_points; i++) {
        /**
         * Hitung waktu saat ini
         */
        double t = i * params->dt;
        
        /**
         * Simpan data state dan control force saat ini
         */
        time[i] = t;
        position[i] = state.position;
        velocity[i] = state.velocity;
        control_force[i] = calculate_control_force(&state, controller, params->dt);
        
        /**
         * Tampilkan progress bar (setiap 10% selesai)
         */
        if (i % (n_points/10) == 0) {
            printf("█");
            fflush(stdout);  // Force output untuk real-time progress
        }
        
        /**
         * Update state menggunakan Runge-Kutta (kecuali iterasi terakhir)
         * Iterasi terakhir hanya untuk menyimpan data final
         */
        if (i < n_points - 1) {
            runge_kutta_4th_order(&state, params, controller, t);
        }
    }
    printf(" Selesai!\n");
    
    /**
     * ANALISIS PERFORMA SISTEM
     * Hitung metrik-metrik performa sistem kontrol
     */
    
    /**
     * Settling Time: waktu yang diperlukan sistem untuk mencapai dan tetap
     * dalam band ±2% dari nilai setpoint
     */
    double settling_time = calculate_settling_time(position, time, n_points, 0.02);
    
    /**
     * Overshoot: persentase nilai maksimum terhadap setpoint
     * Overshoot = ((nilai_max - setpoint) / setpoint) * 100%
     */
    double overshoot = calculate_overshoot(position, n_points, controller->setpoint);
    
    /**
     * Steady-State Error: error antara nilai final dan setpoint
     * SS_Error = |setpoint - nilai_final| / setpoint * 100%
     */
    double steady_state_error = calculate_steady_state_error(position, n_points, controller->setpoint);
    
    /**
     * Kriteria Stabilitas:
     * Sistem dianggap stabil jika:
     * 1. Settling time < waktu simulasi (sistem mencapai steady-state)
     * 2. Overshoot < 50% (tidak berlebihan)
     */
    int is_stable = (settling_time < params->t_final && overshoot < 50.0) ? 1 : 0;
    
    /**
     * Tampilkan hasil analisis performa
     */
    printf("\n=== HASIL ANALISIS PERFORMA SISTEM ===\n");
    printf("Settling Time (2%% tolerance): %.3f s\n", settling_time);
    printf("Overshoot: %.2f%%\n", overshoot);
    printf("Steady-State Error: %.2f%%\n", steady_state_error);
    printf("Final Position: %.6f m\n", position[n_points-1]);
    printf("Final Velocity: %.6f m/s\n", velocity[n_points-1]);
    printf("Status Stabilitas: %s\n", is_stable ? "STABIL" : "TIDAK STABIL");
    
    /**
     * Simpan hasil ke file CSV jika diminta
     * File akan berisi: Time, Position, Velocity, Control_Force
     */
    if (save_to_file) {
        save_results_to_csv(time, position, velocity, control_force, n_points, 
                           "output/simulation_results.csv");
    }
    
    /**
     * Buat struct hasil analisis untuk dikembalikan
     */
    AnalysisResult result = {settling_time, overshoot, steady_state_error, is_stable};
    
    /**
     * Bebaskan memori yang dialokasikan untuk mencegah memory leak
     */
    free(time);
    free(position);
    free(velocity);
    free(control_force);
    
    return result;
}

/**
 * ================================================================================
 * FUNGSI ANALISIS STABILITAS RENTANG PARAMETER
 * ================================================================================
 * 
 * Fungsi ini melakukan analisis stabilitas dengan memvariasikan nilai
 * proportional gain (Kp) dalam rentang yang telah ditentukan sesuai laporan.
 * 
 * Tujuan:
 * - Menganalisis pengaruh Kp terhadap performa sistem
 * - Mencari nilai Kp optimal
 * - Memvalidasi data yang ada di laporan penelitian
 * 
 * Parameter:
 * - params: parameter sistem massa-pegas-damper
 * - controller: parameter kontroler PID (Ki dan Kd tetap)
 */
void analyze_stability_range(SystemParams *params, ControllerParams *controller) {
    
    printf("\n=== ANALISIS STABILITAS RENTANG PARAMETER ===\n");
    
    /**
     * Array nilai Kp yang akan dianalisis - sesuai dengan data di laporan:
     * Rentang dari 1.0 hingga 50.0 untuk melihat tren performa
     */
    double kp_values[] = {1.0, 5.0, 10.0, 15.0, 20.0, 50.0};
    int n_kp = sizeof(kp_values) / sizeof(kp_values[0]);
    
    /**
     * Alokasi memori untuk menyimpan hasil analisis setiap nilai Kp
     */
    AnalysisResult *results = malloc(n_kp * sizeof(AnalysisResult));
    if (!results) {
        printf("Error: Gagal mengalokasi memori untuk analisis!\n");
        return;
    }
    
    /**
     * Header tabel hasil analisis
     */
    printf("Menganalisis pengaruh Proportional Gain (Kp) terhadap stabilitas...\n");
    printf("\nKp\tSettling Time\tOvershoot\tSS Error\tStatus\n");
    printf("------\t------------\t---------\t--------\t--------\n");
    
    /**
     * Loop untuk setiap nilai Kp yang akan dianalisis
     */
    for (int i = 0; i < n_kp; i++) {
        
        /**
         * Buat copy kontroler dengan Kp yang baru
         * Ki dan Kd tetap sama, hanya Kp yang berubah
         */
        ControllerParams test_controller = *controller;
        test_controller.Kp = kp_values[i];
        test_controller.integral = 0.0;              // Reset integral
        test_controller.prev_error = test_controller.setpoint;  // Reset error
        
        /**
         * Gunakan waktu simulasi yang lebih singkat untuk analisis (5 detik)
         * Cukup untuk melihat karakteristik transient response
         */
        SystemParams test_params = *params;
        test_params.t_final = 5.0;
        
        /**
         * Jalankan simulasi dengan parameter test
         * save_to_file = 0 karena ini hanya untuk analisis
         */
        results[i] = simulate_system(&test_params, &test_controller, 0);
        
        /**
         * Tampilkan hasil dalam format tabel
         */
        printf("%.1f\t%.3f s\t\t%.1f%%\t\t%.2f%%\t\t%s\n", 
               kp_values[i], 
               results[i].settling_time,
               results[i].overshoot,
               results[i].steady_state_error,
               results[i].is_stable ? "STABIL" : "TIDAK STABIL");
    }
    
    /**
     * Simpan hasil analisis ke file CSV untuk plotting dan analisis lanjutan
     */
    save_stability_analysis_to_csv(kp_values, results, n_kp, 
                                  "output/stability_analysis.csv");
    
    printf("\nHasil analisis disimpan ke: output/stability_analysis.csv\n");
    
    /**
     * Bebaskan memori
     */
    free(results);
}

/**
 * ================================================================================
 * FUNGSI ANALISIS PARAMETER CUSTOM
 * ================================================================================
 * 
 * Fungsi ini memungkinkan user untuk melakukan analisis stabilitas dengan
 * rentang parameter yang dapat dikustomisasi. User dapat menentukan:
 * - Nilai Kp awal dan akhir
 * - Step size untuk analisis
 * 
 * Berguna untuk:
 * - Fine-tuning parameter di sekitar nilai optimal
 * - Eksplorasi parameter di luar rentang default
 * - Penelitian mendalam tentang pengaruh parameter
 * 
 * Parameter:
 * - params: parameter sistem massa-pegas-damper
 * - controller: parameter kontroler PID baseline
 */
void analyze_custom_parameters(SystemParams *params, ControllerParams *controller) {
    
    printf("\n=== ANALISIS PARAMETER CUSTOM ===\n");
    
    /**
     * Variabel untuk menyimpan input user
     */
    double start_kp, end_kp, step_kp;
    int n_values;
    
    /**
     * Input rentang analisis dari user
     */
    printf("Masukkan rentang analisis Kp:\n");
    printf("Kp awal: ");
    scanf("%lf", &start_kp);
    printf("Kp akhir: ");
    scanf("%lf", &end_kp);
    printf("Step Kp: ");
    scanf("%lf", &step_kp);
    
    /**
     * Hitung jumlah nilai yang akan dianalisis
     */
    n_values = (int)((end_kp - start_kp) / step_kp) + 1;
    
    /**
     * Validasi input - batasi maksimal 100 nilai untuk menghindari
     * simulasi yang terlalu lama
     */
    if (n_values <= 0 || n_values > 100) {
        printf("Error: Jumlah nilai tidak valid (max 100)!\n");
        return;
    }
    
    /**
     * Alokasi memori untuk array nilai Kp dan hasil analisis
     */
    double *kp_values = malloc(n_values * sizeof(double));
    AnalysisResult *results = malloc(n_values * sizeof(AnalysisResult));
    
    if (!kp_values || !results) {
        printf("Error: Gagal mengalokasi memori!\n");
        return;
    }
    
    /**
     * Header informasi dan tabel hasil
     */
    printf("\nMenganalisis %d nilai Kp...\n", n_values);
    printf("\nKp\tSettling Time\tOvershoot\tSS Error\tStatus\n");
    printf("------\t------------\t---------\t--------\t--------\n");
    
    /**
     * Loop analisis untuk setiap nilai Kp dalam rentang yang ditentukan
     */
    for (int i = 0; i < n_values; i++) {
        
        /**
         * Hitung nilai Kp untuk iterasi ini
         */
        kp_values[i] = start_kp + i * step_kp;
        
        /**
         * Setup kontroler test dengan Kp baru
         */
        ControllerParams test_controller = *controller;
        test_controller.Kp = kp_values[i];
        test_controller.integral = 0.0;
        test_controller.prev_error = test_controller.setpoint;
        
        /**
         * Setup parameter simulasi (5 detik untuk analisis cepat)
         */
        SystemParams test_params = *params;
        test_params.t_final = 5.0;
        
        /**
         * Jalankan simulasi dan simpan hasil
         */
        results[i] = simulate_system(&test_params, &test_controller, 0);
        
        /**
         * Tampilkan hasil dalam format tabel
         */
        printf("%.1f\t%.3f s\t\t%.1f%%\t\t%.2f%%\t\t%s\n", 
               kp_values[i], 
               results[i].settling_time,
               results[i].overshoot,
               results[i].steady_state_error,
               results[i].is_stable ? "STABIL" : "TIDAK STABIL");
    }
    
    /**
     * Simpan hasil ke file CSV dengan nama yang berbeda
     */
    save_stability_analysis_to_csv(kp_values, results, n_values, 
                                  "output/custom_analysis.csv");
    
    printf("\nHasil analisis disimpan ke: output/custom_analysis.csv\n");
    
    /**
     * Bebaskan memori yang dialokasikan
     */
    free(kp_values);
    free(results);
}