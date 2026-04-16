# BrotherHobby 1404 KV4600 — Semi-Empirical Motor Model for SIL Simulation

A data-driven C++ motor model for drone flight simulation, built from publicly available stand test data using aerodynamic regression ($F = k \cdot n^2$), polynomial fits, and precomputed lookup tables. The model is semi-empirical: it combines the physical thrust law with black-box polynomials for RPM and current, bypassing idealized $K_v$/$K_t$ constants in favor of direct curve fitting that captures real ESC nonlinearities and propeller aerodynamics.

**Setup:** BrotherHobby 1404 KV4600 + iFlight Nazgul T4030 propeller on 2S LiPo (7.4V)

---

## 🚧 Work in Progress:
  - v1.0 (current): static LUT from datasheet stand test - no dynamics, no temperature effects, no motor-to-motor variation.
  - Newton-Raphson algebraic loop resolution and iterative $V_{eff}$ normalization documented in [GROUP_CALC.md § Model Classification](./GROUP_CALC.md#model-classification-and-known-approximations).
---

## Iteration Roadmap and publishing plan

| Version | What changes | Requires |
|---|---|---|
| v1.0 (current) STATIC | Static LUT from stand test datasheet | Octave + datasheet |
| v2.0 DYNAMIC | Rotor inertia + back-EMF dynamics | Oscilloscope + test bench |
| v3.0 HIL | Eddy current + temperature + commutation noise | Full drone on tether |

> v1.0 is a static model based on the stand test from tytorobotics.com.
> v2.0 measurements planned on physical test bench.
> v3.0 measurements planned on a real drone in a controlled environment.
> Results will be published on [meugenom.com](https://meugenom.com) as open-source data and code.


## Table of Contents
- [Iteration Roadmap](#iteration-roadmap-and-publishing-plan)
- [What This Is](#what-this-is)
- [Project Documentation](#project-documentation)
- [How the Model Works](#how-the-model-works)
- [Code Pipeline](#code-pipeline)
- [Build & Test](#build--test)
- [Regenerating the Model](#regenerating-the-model-gnu-octave)
- [Usage in a Simulator](#usage-in-a-simulator)
- [Known Limitations](#known-limitations)
- [Model Specifications](#model-specifications)
- [Model Accuracy](#model-accuracy)
- [What This Model Does Not Yet Capture](#what-this-model-does-not-yet-capture)
- [References](#references)
- [License](#license)


## What This Is

This project models the thrust and current output of the **BrotherHobby 1404 KV4600** brushless motor with an **iFlight Nazgul T4030** propeller as a function of throttle position and battery voltage.

The model is intended for use in drone simulators where realistic motor behavior is required — including voltage sag effects and propeller load characteristics.

**Data source:** [BrotherHobby 1404 KV4600 stand test](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv) from tytorobotics.com  
11 measurement points (0–100% throttle, 1000–2000 µs) collected on 2S LiPo (~7.4V), with an iFlight Nazgul T4030 propeller and VGood 60A ESC.

---

## Project Documentation

| File | Description |
|------|-------------|
| [GROUP_SPEC.md](./GROUP_SPEC.md) | Component specifications, raw stand test data |
| [GROUP_CALC.md](./GROUP_CALC.md) | Model derivation: math, pipeline, voltage scaling |

---

## How the Model Works

All computation in `src/motor.cpp` is **table lookup + linear interpolation**, no math at runtime.

### Why V_nominal = 7.4V

The stand test was conducted on a 2S LiPo: voltage varied from 7.41V (idle) to 6.83V (100% throttle) as current increased. At full throttle with $I = 7.5A$ and $R_m = 207\text{m}\Omega$, the $I \cdot R$ drop is $1.55V$ — over 20% of the supply voltage. All RPM and current values are normalized to $V_{nominal}$ before fitting. 7.4V is the nominal 2S voltage (2×3.7V).

### Octave (`octave/motor_model.m`) — offline preprocessing

The Octave script does all the physics once and bakes the results into arrays:

1. **Load CSV** from `datasheets/Brother-Hobby-1404_4600KV_Blane_Townsend.csv`
2. **Filter noisy data:** exclude points with RPM < 2000 (idle and near-stall)
3. **Normalize throttle:** PWM 1000–2000 µs → 0.0–1.0
4. **Normalize RPM** to $V_{nominal}$: $RPM_{norm} = RPM_{actual} \cdot V_{nom}/V_{actual}$
5. **Normalize current** to $V_{nominal}$: load component scaled by $(V_{nom}/V_{actual})^2$, idle current kept constant
6. **Polynomial fit (Gas → RPM):** `polyfit(throttle, rpm_norm, 2)` — 2nd-degree polynomial on normalized data
7. **Physical thrust model:** $F = k \cdot n^2$ with coefficient $k$ from Least Squares on raw data, applied to normalized RPM
6. **Polynomial fit (Gas → Current):** `polyfit(throttle, current_norm, 2)` — separate 2nd-degree polynomial on normalized data
7. **Evaluate all models** on a uniform 101-point grid (0.00–1.00, step 0.01)
8. **Enforce boundary conditions:** zero thrust/RPM at 0% throttle, idle current floor
9. **Export** everything to `includes/motor_lut.h` (including `MOTOR_R_INTERNAL`)

The polynomials and intermediate RPM values are **not used at runtime** — they only exist inside the Octave script.

Full algorithm description: [GROUP_CALC.md](./GROUP_CALC.md)

### C++ (`src/motor.cpp`) — runtime

Both public functions do the same thing: O(1) table lookup + linear interpolation + voltage scaling.

**Thrust** scales with $V_{eff}^2$, where $V_{eff} = V - I_{nom} \cdot R_m$ accounts for the internal resistance drop:

$$F[N] = F_{nom}(throttle) \cdot \left(\frac{V_{eff}}{V_{eff,nom}}\right)^2$$

**Current** uses split scaling — idle current stays constant, load current scales with $V_{eff}^2$:

$$I[A] = I_{idle} + (I_{nom}(throttle) - I_{idle}) \cdot \left(\frac{V_{eff}}{V_{eff,nom}}\right)^2$$

---

## Code Pipeline

```text
  octave/motor_model.m          includes/motor_lut.h          src/motor.cpp
  ──────────────────            ────────────────────          ──────────────────
  Octave script         ──►     Auto-generated         ──►    C++ runtime
  - filter noise                C++ header                    - tabInterp O(1)
  - normalize RPM to V_nom      (DO NOT EDIT)                 - V_eff thrust scaling
  - normalize current to V_nom                                - V_eff current scaling
  - polyfit Gas→RPM                                           - no polynomials at runtime
  - F=k·n² thrust model
  - polyfit Gas→Current
  - bake 101-pt tables
  - kgf→N conversion
```

`includes/motor_lut.h` is fully auto-generated by `octave/motor_model.m` and must never be edited manually.

### Workflow (end-to-end)

1. `octave/motor_model.m` loads CSV, fits models, generates lookup tables.
2. `includes/motor_lut.h` is auto-generated with four 101-point arrays.
3. `src/motor.cpp` compiles the runtime model used by simulators.
4. `src/test_motor.cpp` validates thrust/current against datasheet values.

---

## Build & Test

The model uses a semi-empirical $F = k \cdot n^2$ fit for thrust and 2nd-degree polynomial for current. This provides smooth transitions between points and prevents step-response artifacts in PID controllers during simulation.

```sh
cd build && rm -rf * && cmake ..
make -j$(sysctl -n hw.ncpu)
./test_motor
```

Expected output (all green):

```text
=== Motor Model Tests: BrotherHobby 1404 KV4600 + T4030 (2S) ===
V_nominal = 7.4V

--- Thrust Tests ---
✓ zero throttle → zero thrust
✓ thrust at 30%: 0.250N (expected 0.244N)
✓ thrust at 50%: 0.446N (expected 0.453N)
✓ thrust at 70%: 0.615N (expected 0.615N)
✓ thrust is monotonically increasing
✓ higher voltage → higher thrust

--- Current Tests ---
✓ current at 30%: 1.69A (expected 1.59A)
✓ current at 50%: 3.51A (expected 3.60A)
✓ current at 70%: 5.21A (expected 5.32A)
✓ current is monotonically increasing
✓ current V_eff-scaling: I(8.4V)/I(7.0V) = 1.485

✓ All tests passed.
Model accuracy (30/50/70%): 2.2% / 1.4% / 0.0% | avg=1.2%
```
---

## Regenerating the Model (GNU Octave)

If the propeller or motor changes, update the data in `motor_model.m` and re-run:

```sh
cd octave && octave ./motor_model.m
```
This overwrites `includes/motor_lut.h` with new precomputed tables. Then rebuild the C++ project.
> **Note:** Run from `octave/` directory — the script uses relative paths to `../includes/`, `../datasheets/`, and `../plots/`.

---

## Usage in a Simulator

Copy `src/motor.cpp`, `includes/motor.h`, `includes/motor_lut.h` into your project, or use CMake:

```cmake
add_subdirectory(path/to/tmotor-f1404-model)
target_link_libraries(your_target PRIVATE motor_model)
```

```cpp
#include "motor.h"

float thrustN  = getMotorThrustNewtons(0.5f, 7.4f);  // throttle, voltage
float currentA = getMotorCurrentAmps(0.5f, 7.4f);
```

---

## Known Limitations

### Low-throttle region (0–9%)

The first two datasheet points (0% and 10% throttle) are filtered out due to noise (RPM < 2000). The polynomial extrapolates into this region from the 20%+ data — treat results below 20% as estimates.

### High-throttle saturation (90–100%)

The last two datasheet points (90% and 100%) show nearly identical RPM and thrust (21741→21673, 79.5g→79.5g), indicating motor/propeller saturation or stall. The model polynomial does not capture this plateau — it continues to increase slightly.

### Voltage scaling

Thrust and current use $V_{eff}^2$ scaling, where $V_{eff} = V - I_{nom} \cdot R_m$ accounts for the internal resistance drop under load. Valid for the practical 2S operating range: **6.6V–8.4V**.

---

## Model Specifications

| Parameter | Value |
|---|---|
| Motor | BrotherHobby 1404 KV4600 |
| Propeller | iFlight Nazgul T4030 |
| KV | 4600 RPM/V |
| Internal resistance | 207.48 mΩ |
| Max current | 13.6 A |
| Idle current (measured) | 0.11 A |
| Weight (incl. cable) | 8.7 g |
| Rated voltage | 2S LiPo |
| V_nominal (model) | 7.4V (2S) |

Full specifications: [GROUP_SPEC.md](./GROUP_SPEC.md)

---

## Model Accuracy

| Throttle | Thrust Error | Current Error |
|----------|-------------|---------------|
| 30%      | 2.2%        | +6.3%         |
| 50%      | 1.4%        | 2.5%          |
| 70%      | 0.0%        | 2.1%          |
| **avg**  | **1.2%**    | **3.6%**      |

Tolerance: ±5% thrust, ±10% current.

| Region | Status |
|---|---|
| 20–90% throttle, ~7.4V | Measured data, avg thrust error 1.2% |
| 0–19% throttle | Polynomial extrapolation — filtered from regression |
| 90–100% throttle | Motor saturation zone — model overestimates slightly |
| Voltage scaling | V_eff model, valid for 6.6V–8.4V (2S operating range) |
| Temperature effects | Not modelled |
| Motor-to-motor variation | Not modelled (~2–3% in practice) |

---

## What This Model Does Not Yet Capture

Physical phenomena intentionally excluded from v1.0.
Planned for future iterations with real hardware measurements.

### Rotor Inertia

The model assumes instantaneous throttle response.
In reality the rotor has angular momentum — it cannot change speed instantly.

Time constant $\tau$ from motor parameters:

$$\tau = \frac{J \cdot R}{k_e \cdot k_T}$$

For 1404 geometry (rotor ~6 g, r ≈ 9 mm): $\tau \approx 5–15 \text{ ms}$.

**Impact on simulation:** PID tuned on this model will be optimistic.
Real step response is slower than predicted.

### Back-EMF Dynamics

Static model uses $\omega \propto V$ (KV law).
Transient back-EMF interaction is not captured:

$$V_{eff}(t) = V_{supply} - k_e \cdot \omega(t)$$

where $k_e = \frac{60}{2\pi \cdot KV} = 0.00208 \; \text{V·s/rad}$

Affects current prediction accuracy during rapid throttle changes.

### Eddy Current Losses

At high electrical frequencies, eddy currents in the stator core cause losses beyond DC winding resistance.

Electrical frequency at maximum RPM (9N12P motor, 6 pole pairs):

$$f_{elec} = \frac{21741 \cdot 6}{60} \approx 2174 \; \text{Hz}$$

At ~2 kHz, core losses become significant and explain part of the gap between theoretical and measured efficiency.

### Commutation Noise

PWM switching (30–60 kHz) generates harmonic current spikes not visible in averaged datasheet values.

**System-level effects:**
- Magnetic field interference with IMU magnetometer
- Power supply ripple on flight controller
- EMC behavior of the complete drone system

### Temperature Effects

Copper winding resistance increases with temperature:

$$R(T) = R_{20°C} \cdot [1 + \alpha \cdot (T - 20°C)]$$

where $ \alpha = 0.00393 /°C$ for copper.

At 60°C operating temperature: R increases ~16%, reducing current and thrust at constant throttle.

---

## References

1. [BrotherHobby 1404 KV4600 — Stand Test Data](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv) tytorobotics.com. Primary data source for this model

2. [BrotherHobby 1404 KV4600 — Motor Specifications](https://www.brotherhobbystore.com/products/tc-1404-ultralight-motor-139) brotherhobbystore.com

3. [iFlight Nazgul T4030 — Propeller Specifications](https://shop.iflight.com/nazgul-t4030-propellers-cw-ccw-3sets-6pairs-pro1258) shop.iflight.com

4. "A Comparative Study on Thrust Map Estimation for Multirotor Aerial Vehicles", Francisco J. Anguita, Rafael Perez-Segui, Carmen DR.Pita-Romero, Miguel Fernandez-Cortizas, Javier Melero-Deza, http://www.imavs.org

5. "Modelling and Control of a Large Quadrotor Robot", P.Pounds, R.Mahony, P.Corke, 2010

6. Propeller Performance Data at Low Reynolds Numbers, John B. Brandt and Michael S. Selig 2011, pages 1-18.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE)