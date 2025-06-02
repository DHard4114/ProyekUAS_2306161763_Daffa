# Analisis Komprehensif Sistem Suspensi Kendaraan Menggunakan Integrasi Metode Numerik

## Informasi Program

**Nama Program:** Suspension Analysis - Comprehensive Vehicle Suspension System Analysis

**Nama Lengkap:** Daffa Hardhan  
**NPM:** 2306161763  
**Mata Kuliah:** Komputasi Numerik  
**Universitas:** Universitas Indonesia

## Deskripsi Program

Program ini merupakan implementasi analisis komprehensif sistem suspensi kendaraan yang mengintegrasikan tiga kategori metode numerik utama dari Chapman Numerical Methods:

### 🎯 **Tujuan Program**
Menganalisis sistem suspensi kendaraan secara holistik dengan mengintegrasikan metode numerik untuk:
- Mencari frekuensi resonansi sistem
- Mengoptimasi parameter suspensi
- Memvalidasi model dengan data eksperimen

### 📊 **Metode Numerik yang Diintegrasikan**

#### 1. **Root Finding Methods (Chapter 5)**
- **Newton-Raphson Method**: Pencarian frekuensi resonansi dengan konvergensi kuadratik
- **Bisection Method**: Verifikasi dan cross-validation hasil Newton-Raphson
- **Target**: Menemukan akar dari persamaan karakteristik sistem suspensi

#### 2. **Linear Systems (Chapter 6)**
- **Gaussian Elimination**: Optimasi parameter suspensi optimal
- **Partial Pivoting**: Meningkatkan stabilitas numerik
- **Target**: Menyelesaikan sistem persamaan linear untuk parameter k₁*, k₂*, c₁*, c₂*

#### 3. **Curve Fitting (Chapter 7)**
- **Least Squares Regression**: Fitting polynomial orde 2 untuk data eksperimen
- **Model Validation**: Evaluasi kualitas model dengan R², RMSE
- **Target**: Memvalidasi model teoritis dengan data eksperimen

### 🔧 **Spesifikasi Teknis**

#### Parameter Sistem Suspensi:
- **Massa kendaraan**: 1500 kg
- **Konstanta pegas depan (k₁)**: 25000 N/m
- **Konstanta pegas belakang (k₂)**: 22000 N/m
- **Konstanta damping depan (c₁)**: 1800 Ns/m
- **Konstanta damping belakang (c₂)**: 1600 Ns/m

#### Persamaan Karakteristik:
```
f(ω) = mω⁴ - (c₁+c₂)ω³ + (k₁+k₂+c₁c₂/m)ω² - (c₁k₂+c₂k₁)ω + k₁k₂ = 0
```

#### Model Polynomial Curve Fitting:
```
A(ω) = a₀ + a₁ω + a₂ω²
```

### 📈 **Fitur Utama Program**

1. **Multiple Initial Guess Newton-Raphson**
   - Menggunakan 4 initial guess berbeda: {1.0, 5.0, 10.0, 15.0} rad/s
   - Automatic duplicate root detection
   - Convergence tracking dengan error analysis

2. **Robust Gaussian Elimination**
   - Partial pivoting untuk stabilitas numerik
   - Matrix dan vector display untuk debugging
   - Constraint validation untuk parameter fisik

3. **Comprehensive Curve Fitting**
   - Normal equations setup untuk polynomial fitting
   - Statistical analysis: R², RMSE, SSE, SST
   - Data comparison table (experimental vs predicted)

4. **Cross-Validation System**
   - Perbandingan hasil antar metode
   - Peak frequency validation
   - Agreement assessment dengan threshold

5. **Results Documentation**
   - Detailed console output dengan formatting
   - Automatic file saving (hasil_analisis.txt)
   - Summary statistics dan performance metrics

### 🚀 **Cara Menjalankan Program**

#### Kompilasi:
```bash
gcc suspension_analysis.c -o suspension_analysis -lm
```

#### Eksekusi:
```bash
./suspension_analysis
```

#### Output Files:
- `hasil_analisis.txt`: Ringkasan hasil analisis lengkap

### 📊 **Expected Results**

#### Root Finding Results:
- Frekuensi resonansi utama: ~5.61 rad/s dan ~10.23 rad/s
- Newton-Raphson konvergensi: 3-6 iterasi
- Bisection verifikasi: 23-24 iterasi

#### Optimization Results:
- Parameter optimal yang memenuhi constraint:
  - Damping ratio: 0.3 ≤ ζ ≤ 0.7
  - Natural frequency: 4.0 ≤ ωₙ ≤ 8.0 rad/s
  - Settling time: tₛ ≤ 2.0 detik

#### Curve Fitting Results:
- R² > 0.98 (excellent fit quality)
- RMSE < 0.1 (low prediction error)
- Cross-validation error < 2%

### 🔬 **Validasi dan Verifikasi**

1. **Numerical Accuracy**
   - Tolerance: 1×10⁻⁶ untuk semua metode
   - Maximum iterations: 100
   - Error tracking untuk convergence analysis

2. **Physical Constraints**
   - Parameter bounds validation
   - Stability criteria checking
   - Engineering feasibility assessment

3. **Cross-Method Validation**
   - Newton-Raphson vs Bisection comparison
   - Root finding vs curve fitting peak analysis
   - Optimization results vs original parameters

### 🎓 **Significance Akademik**

Program ini mendemonstrasikan:
- **Integrasi metodologi**: Kombinasi 3 chapter Chapman untuk analisis holistik
- **Engineering application**: Aplikasi praktis untuk automotive engineering
- **Numerical robustness**: Implementasi multiple validation techniques
- **Professional coding**: Dokumentasi lengkap dengan C best practices

### 📝 **Struktur Output**

```
================================================================
     PROYEK UAS KOMPUTASI NUMERIK - 2306161763
================================================================
1. ROOT FINDING ANALYSIS (Chapter 5)
2. LINEAR SYSTEMS ANALYSIS (Chapter 6)  
3. CURVE FITTING ANALYSIS (Chapter 7)
4. CROSS-VALIDATION
================================================================
                    RINGKASAN HASIL
================================================================
```

### 🔗 **Dependencies**

- **Language**: C (C99 standard)
- **Libraries**: 
  - `stdio.h` - Input/output operations
  - `stdlib.h` - Standard library functions
  - `math.h` - Mathematical functions (requires -lm flag)
  - `string.h` - String manipulation

### ⚡ **Performance Characteristics**

- **Memory Usage**: Static allocation, no dynamic memory
- **Execution Time**: < 1 second untuk complete analysis
- **Numerical Stability**: Partial pivoting dan error checking
- **Scalability**: Configurable matrix size dan data points

---

**© 2024 Daffa Hardhan - Universitas Indonesia**  
*Proyek UAS Komputasi Numerik - Implementasi Integrasi Metode Numerik untuk Analisis Sistem Suspensi Kendaraan*