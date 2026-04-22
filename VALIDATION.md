# Validation Protocol & Test Results

**Generated on:** 2026-04-22 15:40:09

## 1. HIL Test Summary (POSIX/STM32)

 ### Motor-Model Test Bench: BrotherHobby 1404 KV4600+T4030 (2S)

| ID | Parameter | Value | Unit |
|----|-----------|-------|------|
| REQ-PHY-001 | GRAMS_TO_NEWTONS | 9.80665 | N/A |
| REQ-PHY-002 | MOTOR_V_NOMINAL | 7.4 | V |
| REQ-PHY-003 | SYSTEM_STANDBY_CURRENT | 0.11 | A |
| REQ-PHY-004 | MOTOR_R_INTERNAL | 0.20748 | Ohm |
| REQ-ALG-002 | MOTOR_TAB_SIZE | 101 | N/A |


### Thrust Tests
- PASS zero throttle -> zero thrust
- PASS thrust is monotonically increasing (10%–90%)
- WARN thrust drops at 100% throttle: 0.9148N -> 0.8389N (propeller saturation / Hall sensor RPM under read at > 20k RPM)
- PASS higher voltage -> higher thrust
### Current Tests
- PASS current is monotonically increasing (10%–90%)
- WARN current drops at 100% throttle: 8.7945A -> 8.1069A (consistent with propeller saturation at >20k RPM)
- PASS current V_eff-scaling: I(8.4V)/I(7.0V) = 1.385


## 2. Accuracy Analysis (Octave vs POSIX/STM32)

**Max Thrust Error:** 17.33%
**Max Current Error:** 17.78%

| Throttle | Thrust Error % | Current Error % |
|----------|----------------|----------------|
| 0.00 | -100.00 | -0.79|
| 0.10 | -99.83 | 1.16|
| 0.20 | 4.56 | 2.41|
| 0.30 | 3.16 | 2.03|
| 0.40 | 7.30 | 4.02|
| 0.50 | 7.11 | 6.80|
| 0.60 | 5.57 | 3.77|
| 0.70 | 11.35 | 9.48|
| 0.80 | 13.93 | 14.60|
| 0.90 | 17.33 | 17.78|
| 1.00 | 7.58 | 8.81|

![Discrepancy between real stand data and Motor-Model](./octave/plots/plot4_errors.png)

*Document Version: v.0.3.0 | Part of LIGHT-MBSE-PIPELINE-SKELETON*
