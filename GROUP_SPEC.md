# Component Spec

Based on [Brother Hobby 1404 - 4600KV test data](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv) from tytorobotics.com.

Motor Specifications are sourced from [Brother Hobby Store](https://www.brotherhobbystore.com/products/tc-1404-ultralight-motor-139).

Source for propeller specifications is the [iFlight Nazgul T4030 product page](https://shop.iflight.com/nazgul-t4030-propellers-cw-ccw-3sets-6pairs-pro1258).

## Components
1. **Motor:** Brother Hobby 1404 KV4600
2. **Propeller:** iFlight Nazgul T4030
3. **ESC:** VGood 60A SBEC 2-6S LIPO

## 1. Motor Specifications

These values represent the physical and electrical properties of the two motor variants.

|Specification|KV4600|
|---|---|
|**Motor Dimensions**|$\Phi 18.2 \times 16$ mm|
|**Stator Dimensions**|14 mm|
|**Stator/Magnet Config**|9N12P/NSK5x2x2.5mm|
|**Idle Current@7V (A)**|0.45 A|
|**Shaft Diameter**|1.5 mm|
|**Lead Wire**|30#AWG 100 mm|
|**Weight (Incl. Cable)**|8.7 g|
|**Internal Resistance**|$207.48 m\Omega$|
|**Rated Voltage (LiPo)**|3-4S|
|**Max Current(A)**|13.6 A|
|**Max Power (W)**|217.6 W|

## 2. Propeller Specifications

|Specification|iFlight Nazgul T4030|
|---|---|
|**Diameter**|4 inch|
|**Pitch**|3 inch|
|**Number of Blades**|2|
|**Material**|Plastic|
|**Weight**|1.2 g|

## 3. Test Report from tytorobotics.com

| Time (s) | Throttle (µs) | Rotation speed (rpm) | Thrust (kgf) | Torque (N⋅m) | Voltage (V) | Current (A) | Electrical power (W) | Mechanical power (W) | Motor & ESC efficiency (%) | Propeller efficiency (gf/W) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 1000 | 0 | 0.0005 | ~0 | 7.414 | 0.1109 | 0.8221 | 0 | 0 |
| 4.651 | 1100 | 255 | 0.0059 | 0.001 | 7.395 | 0.3707 | 2.741 | 0.0267 | 0.974 | 220.5 |
| 9.276 | 1200 | 9673 | 0.0154 | 0.0028 | 7.35 | 0.9244 | 6.794 | 2.815 | 41.43 | 5.461 |
| 13.93 | 1300 | 12263 | 0.0249 | 0.0044 | 7.296 | 1.589 | 11.59 | 5.699 | 49.15 | 4.369 |
| 18.55 | 1400 | 15017 | 0.0366 | 0.0067 | 7.22 | 2.56 | 18.48 | 10.6 | 57.36 | 3.453 |
| 23.18 | 1500 | 16627 | 0.0462 | 0.0084 | 7.138 | 3.597 | 25.67 | 14.63 | 56.97 | 3.157 |
| 27.83 | 1600 | 17717 | 0.0516 | 0.0096 | 7.106 | 4.01 | 28.49 | 17.85 | 62.66 | 2.892 |
| 32.48 | 1700 | 19505 | 0.0626 | 0.0117 | 7.002 | 5.322 | 37.26 | 23.81 | 63.9 | 2.631 |
| 37.13 | 1800 | 20811 | 0.073 | 0.0135 | 6.909 | 6.508 | 44.96 | 29.39 | 65.36 | 2.484 |
| 41.75 | 1900 | 21741 | 0.0795 | 0.0147 | 6.828 | 7.467 | 50.99 | 33.51 | 65.73 | 2.372 |
| 46.38 | 2000 | 21673 | 0.0795 | 0.0147 | 6.828 | 7.451 | 50.88 | 33.38 | 65.61 | 2.382 |