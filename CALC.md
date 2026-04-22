# Model Calculation - Algorithm & Methodology

>This document describes the complete mathematical pipeline used to build the motor model. All offline computation is done in [main.m](./octave/main.m). The C++ runtime in [src/core/motor.cpp](./src/core/motor.cpp) uses only precomputed tables — no polynomials at runtime.

## Table of Contents
- [Data Source](#data-source)
- [Pipeline Overview](#pipeline-overview)
- [Step 1: Load and Parse CSV](#step-1-load-and-parse-csv)
- [Step 2: Extract and Convert Columns](#step-2-extract-and-convert-columns)
- [Step 3: Filter Noisy Data](#step-3-filter-noisy-data)
- [Step 4: Normalize Throttle](#step-4-normalize-throttle)
- [Step 5/6: Iterative V_eff Normalization](#step-56-iterative-v_eff-normalization)
- [Step 7: Polynomial Fit Throttle -> RPM](#step-7-polynomial-fit-throttle---rpm)
- [Step 8: Semi-Empirical Thrust Model RPM -> Thrust](#step-8-semi-empirical-thrust-model-rpm---thrust)
- [Step 9: Polynomial Fit Throttle -> Current](#step-9-polynomial-fit-throttle---current)
- [Step 10: Generate 101-Point LUT](#step-10-generate-101-point-lut)
- [Step 11: Boundary Conditions](#step-11-boundary-conditions)
- [Step 12: Export to C++ Header](#step-12-export-to-c-header)
- [C++ Runtime Model](#c-runtime-model)
- [Voltage Scaling (V_eff Model)](#voltage-scaling-v_eff-model)
- [Visualization](#visualization)

## Data Source

Stand test data from [tytorobotics.com](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv):

- **Motor:** BrotherHobby 1404 KV4600
- **Propeller:** iFlight Nazgul T4030 (4-inch, 3-inch pitch, 2-blade)
- **Battery:** 2S LiPo (~7.4V nominal)
- **ESC:** VGood 60A SBEC 2-6S
- **Test points:** 11 measurements (1000–2000 µs throttle, 0–100%)
- **CSV file:** `references/Brother-Hobby-1404-4600KV_Propeller-T4030.csv`

Full component specifications: [SPEC.md](./SPEC.md)

## Pipeline Overview

```text
        CSV datasheet
            │
            ▼
┌───────────────────────────────┐
│ 1. Load & parse CSV           │
│ 2. Extract and convert columns│
│ 3. Filter noise (RPM>2000)    │
│ 4. Normalize throttle 0..1    │
│ 5. Normalize RPM to V_nom     │
│ 6. Normalize current to V_nom │
└─────────────┬─────────────────┘
              │
     ┌────────┴────────┐
     ▼                 ▼
┌──────────┐    ┌───────────────┐
│ polyfit  │    │ Semi-Empirical│
│ Gas->RPM │    │ model F=k·n²  │
│ (deg 2)  │    │ (LSM fit)     │
│ on norm  │    │ on raw data   │
└────┬─────┘    └──────┬────────┘
     │                 │
     ▼                 ▼
┌─────────────────────────────┐
│ 5. Evaluate on 101-pt grid  │
│    lut_rpm -> lut_thrust    │
│    Gas -> Current           │
│ 6. Clamp & boundary cond.   │
└─────────────┬───────────────┘
              │
              ▼
┌─────────────────────────────┐
│ 7. Export motor_lut.h       │
│    MOTOR_TAB_GAS[101]       │
│    MOTOR_TAB_SCHUB_N[101]   │
│    MOTOR_TAB_STROM[101]     │
│    MOTOR_TAB_DREHZAHL[101]  │
│    MOTOR_R_INTERNAL         │
└─────────────┬───────────────┘
              │
              ▼
┌─────────────────────────────┐
│ C++ Runtime:                │
│  O(1) table interpolation   │
│  + V_eff voltage scaling    │
└─────────────────────────────┘
```

## Step 1: Load and Parse CSV

The raw CSV contains quoted fields. The script strips quotes and parses 12 numeric columns:

```matlab
    raw_text = fileread(filename);
    clean_text = strrep(raw_text, '"', '');
    C = textscan(clean_text, '%f %f %f %f %f %f %f %f %f %f %f %f', ...
                 'Delimiter', ',', 'HeaderLines', 1, 'CollectOutput', 1);
    data = C{1};
```

Result: 11×12 matrix of measurement data.

## Step 2: Extract and Convert Columns

| Column | Variable | Unit | Description |
|--------|----------|------|-------------|
| 2 | `THROTTLE_US` | $\mu s$ | PWM signal (1000–2000) |
| 3 | `RPM` | RPM | Rotation speed |
| 4 | `THRUST_N` | N | Thrust (raw is kgf, converted ×9.81) |
| 5 | `TORQUE_NM` | N⋅m | Torque |
| 6 | `VOLTAGE_V` | V | Supply voltage |
| 7 | `CURRENT_A` | A | Current draw |

Thrust conversion from kgf to Newtons:

$$F[N] = F[kgf] \times 9.80665$$

## Step 3: Filter Noisy Data

The first two data points (0 RPM and 255 RPM) are excluded from polynomial regression. At very low RPM the motor is either idle or barely spinning — the measurements do not follow the physical model:

```matlab
    valid_idx = RPM > 2000;
```

This keeps 9 of 11 data points for fitting, while the last point (2000 $\mu s$) is also noisy (motor stall region — RPM drops from 21741 back to 21673).

## Step 4: Normalize Throttle

PWM signal 1000–2000 $\mu s$ mapped to 0.0–1.0:

$$t_{norm} = \frac{t_{\mu s} - 1000}{1000}$$

```matlab
    throttle_norm = (THROTTLE_US - 1000) / 1000;
```

## Step 5/6: Iterative V_eff Normalization

The stand test voltage drops from 7.41V (idle) to 6.83V (full throttle) as current increases. Without normalization, the polyfit would see values distorted by battery voltage sag.

Since $\omega \propto V_{eff}$ (back-EMF law), the physically correct normalization uses effective voltage:

$$RPM_{norm} = RPM_{actual} \cdot \frac{V_{eff,nom}}{V_{eff,actual}}, \quad I_{norm} = I_{idle} + (I_{actual} - I_{idle}) \cdot \left(\frac{V_{eff,nom}}{V_{eff,actual}}\right)^2$$

where $V_{eff,actual} = V_{actual} - I_{actual} \cdot R_m$ and $V_{eff,nom} = V_{nom} - I_{nom} \cdot R_m$.

- **The circular dependency.** $V_{eff,nom}$ requires $I_{nom}$ - the current the motor draws at $V_{nominal}$ for each throttle — which is exactly the quantity we are trying to fit. This creates a circular dependency: the normalization needs the polyfit result, but the polyfit needs the normalized data.

- **Solution: iterative bootstrap.** Starting from a $V_{terminal}$ approximation, the script iterates 3 times:

```matlab
    % Bootstrap: V_terminal normalization (first approximation)
    rpm_norm = rpm .* (MOTOR_V_NOMINAL ./ VOLTAGE_V);
    current_norm = SYSTEM_STANDBY_CURRENT + (CURRENT_A - SYSTEM_STANDBY_CURRENT) .* (MOTOR_V_NOMINAL ./ VOLTAGE_V).^2;

    V_eff_actual = VOLTAGE_V - CURRENT_A .* MOTOR_R_INTERNAL;

    for iter = 1:3
        p_cur_tmp = polyfit(throttle_norm_filtered, current_norm(valid_idx), 2);
        I_nom_est = max(polyval(p_cur_tmp, throttle_norm), SYSTEM_STANDBY_CURRENT);
        V_eff_nom = MOTOR_V_NOMINAL - I_nom_est .* MOTOR_R_INTERNAL;
        rpm_norm = rpm .* (V_eff_nom ./ V_eff_actual);
        current_norm = SYSTEM_STANDBY_CURRENT + (CURRENT_A - SYSTEM_STANDBY_CURRENT) .* (V_eff_nom ./ V_eff_actual).^2;
    end
```

## Step 7: Polynomial Fit Throttle -> RPM

2-degree polynomial fit on **normalized** RPM data:

$$RPM(t) = a_2 \cdot t^2 + a_1 \cdot t + a_0$$

```matlab
    p_rpm = polyfit(throttle_norm_filtered, rpm_norm_filtered, 2);
```

Because RPM is normalized to $V_{nom}$, the polynomial now describes the motor’s response at a consistent 7.4V reference, not at varying test voltages.

---

## Step 8: Semi-Empirical Thrust Model RPM -> Thrust

Instead of the classical electromechanical equations ($M = K_t \cdot I$, $\omega = (V - I \cdot R) / K_e$), the model uses the aerodynamic thrust law with an empirically fitted coefficient:

$$F = k \cdot n^2$$

where $F$ is thrust in Newtons and $n$ is RPM. This is the aerodynamic thrust equation for a fixed-pitch propeller at constant air density.

The coefficient $k$ is found via Least Squares Method (LSM) through the origin:

$$k = \frac{\sum (n_i^2 \cdot F_i)}{\sum (n_i^4)}$$

```matlab
    X_phys = rpm_filtered.^2;
    Y_phys = thrust_N_filtered;
    k_phys = sum(X_phys .* Y_phys) / sum(X_phys.^2);
```

## Step 9: Polynomial Fit Throttle -> Current

Current is modeled with a 2-degree polynomial on **normalized** current data:

$$I(t) = b_2 \cdot t^2 + b_1 \cdot t + b_0$$

```matlab
    p_current = polyfit(throttle_norm_filtered, current_norm_filtered, 2);
```

## Step 10: Generate 101-Point LUT

All models are evaluated on a uniform grid of 101 points (0.00, 0.01, ..., 1.00) using pchip (shape-preserving cubic hermite interpolation) on all normalized data points:

```matlab
    lut_throttle = linspace(0, 1, MOTOR_TAB_SIZE);  % MOTOR_TAB_SIZE = 101
    lut_rpm      = interp1(throttle_norm, rpm_norm, lut_throttle, 'pchip', 'extrap');     % pchip on all normalized points
    lut_thrust   = k_phys * (lut_rpm.^2);            % k fitted on raw data, applied to normalized RPM
    lut_current  = interp1(throttle_norm, current_norm, lut_throttle, 'pchip', 'extrap'); % pchip on all normalized points
```

101 points with 1% step give O(1) lookup at runtime — the index is simply `throttle × 100`.

## Step 11: Boundary Conditions

Physical constraints enforced after polynomial evaluation:

```matlab
    % Non-negative values
    lut_thrust(lut_thrust < 0)             = 0.0;
    lut_current(lut_current < SYSTEM_STANDBY_CURRENT) = SYSTEM_STANDBY_CURRENT;
    lut_rpm(lut_rpm < 0)                   = 0.0;

    % Zero throttle → exact zero
    lut_thrust(1)  = 0.0;
    lut_current(1) = SYSTEM_STANDBY_CURRENT;   % 0.11A system standby
    lut_rpm(1)     = 0.0;
```

Constants:
- `MOTOR_V_NOMINAL = 7.4V` — nominal 2S voltage
- `SYSTEM_STANDBY_CURRENT = 0.11A` — ESC/system standby current at 0 RPM (not the motor's mechanical idle at 0.45A)
- `MOTOR_R_INTERNAL = 0.20748ohm` — internal winding resistance from GROUP_SPEC

## Step 12: Export to C++ Header

The script writes `includes/motor_lut.h` with four arrays:

| Array | Size | Unit | Description |
|-------|------|------|-------------|
| `MOTOR_TAB_GAS[101]` | 101 | — | Throttle values 0.00–1.00 |
| `MOTOR_TAB_SCHUB_N[101]` | 101 | N | Thrust at V_nominal |
| `MOTOR_TAB_STROM[101]` | 101 | A | Current at V_nominal |
| `MOTOR_TAB_DREHZAHL[101]` | 101 | RPM | Rotation speed at V_nominal |

Plus scalar constants: `MOTOR_TAB_SIZE`, `MOTOR_V_NOMINAL`, `SYSTEM_STANDBY_CURRENT`, `MOTOR_R_INTERNAL`.

**This file is auto-generated and must not be edited manually.**

> **Note on `SYSTEM_STANDBY_CURRENT`:** The exported constant (0.11A) is the ESC/system standby current measured at 0 RPM on the test stand. The motor's mechanical idle current when spinning (0.45A from the datasheet) is a different quantity. In the C++ runtime, `SYSTEM_STANDBY_CURRENT` serves as the voltage-independent floor in the current scaling formula — the minimum current the system draws regardless of $V_{eff}$.

## C++ Runtime Model

Both `getMotorThrustNewtons()` and `getMotorCurrentAmps()` in `src/motor.cpp` use the same pattern:

1. **Clamp** throttle to [0.0, 1.0]
2. **O(1) table lookup** — compute array index directly from throttle
3. **Linear interpolation** between two adjacent table entries
4. **V_eff voltage scaling** to adjust for actual battery voltage

> No polynomials, no `pow()`, no `sqrt()` at runtime — only one multiply, one add.

## Voltage Scaling (V_eff Model)

### Effective voltage model

The model uses $V_{eff} = V - I_{nom}(throttle) \cdot R_m$ instead of $V$:

### Thrust

$$F(V) = F_{nom}(throttle) \cdot \left(\frac{V_{eff}}{V_{eff,nom}}\right)^2$$

where $V_{eff,nom} = V_{nom} - I_{nom}(throttle) \cdot R_m$.

### Current

Idle current (friction/windage) is voltage-independent. Only load current is scaled:

$$I(V) = I_{idle} + (I_{nom}(throttle) - I_{idle}) \cdot \left(\frac{V_{eff}}{V_{eff,nom}}\right)^2$$

### Algebraic loop resolution (Newton-Raphson)

> Since the aerodynamic load (and therefore the consumed current) is proportional to $V_{eff}^2$, while $V_{eff}$ itself depends on current through the motor's internal resistance ($V_{eff} = V_{bat} - I \cdot R_m$), an algebraic loop arises. The true $V_{eff}$ depends on $I_{actual}$, which itself depends on $V_{eff}$. Using $I_{nom}$ as a zeroth-order approximation, the runtime performs **two Newton-Raphson iterations** to converge:

```cpp
    float v_eff = voltage - i_nom * MOTOR_R_INTERNAL;  // zeroth-order
    for (int nr = 0; nr < 2; nr++) {
        float ratio = v_eff / v_eff_nom;
        float i_est = SYSTEM_STANDBY_CURRENT + (i_nom - SYSTEM_STANDBY_CURRENT) * (ratio * ratio);
        v_eff = voltage - i_est * MOTOR_R_INTERNAL;
    }
    float v_scale = v_eff / v_eff_nom;
```

> Each iteration refines the current estimate using the $V_{eff}^2$ scaling law, then recomputes $V_{eff}$. Two iterations converge to <1% of the exact solution across the operating range. The total cost is ~15 floating-point operations — still O(1) per call.

## Visualization

The Octave script generates three diagnostic plots in `plots/`:

| Plot | X-axis | Y-axis | Purpose |
|------|--------|--------|---------|
| `plot1_thrust_rpm.png` | RPM | Thrust [N] | Validates $F = k \cdot n^2$ physical model vs. raw data |
| `plot2_current_thrust.png` | Thrust [N] | Current [A] | Shows I(F) relationship — efficiency characteristic |
| `plot3_efficiency.png` | Thrust [g] | Efficiency [g/W] | Propeller efficiency in operational range |
| `plot4_errors.png` | Throttle | % Error | % error of polynomial fits vs. raw data |