# Specifications for Brother Hobby 1404 KV4600 Motor with iFlight Nazgul T4030 Propeller:
> AUTO-GENERATED from `/octave/main.m` on 2026-04-22 12:16:01

Based on [Brother Hobby 1404 - 4600KV test data](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv) from tytorobotics.com.

Motor Specifications are sourced from [Brother Hobby Store](https://www.brotherhobbystore.com/products/tc-1404-ultralight-motor-139).

Source for propeller specifications is the [iFlight Nazgul T4030 product page](https://shop.iflight.com/nazgul-t4030-propellers-cw-ccw-3sets-6pairs-pro1258).

## Components:
1. **Motor:** Brother Hobby 1404 KV4600
2. **Propeller:** iFlight Nazgul T4030
3. **ESC:** VGood 60A SBEC 2-6S LIPO
## 1. Motor Specifications

These values represent the physical and electrical properties of the two motor variants.

|Specification|KV4600|
|---|---|
|**Motor Dimensions**|$\Phi 18.2 	imes 16$ mm|
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
## 3. Constraints and Parameters

|Constraint Name| Parameter Name | Parameter Value| Parameter Unit|
|---|---|---|---|
|REQ-ALG-002|MOTOR_TAB_SIZE|101|N/A|
|REQ-PHY-002|MOTOR_V_NOMINAL|7.4000|V|
|REQ-PHY-003|SYSTEM_STANDBY_CURRENT|0.1100|A|
|REQ-PHY-004|MOTOR_R_INTERNAL|0.2075|Ohm|
## 4. Test Report from tytorobotics.com

| Throttle (µs) | Rotation speed (rpm) | Thrust (kgf) | Voltage (V) | Current (A) | Electrical power (W) |
|---|---|---|---|---|---|
| **THROTTLE_US** | **RPM** | **THRUST_KGF** | **VOLTAGE_V** | **CURRENT_A** | **POWER_ELECTRICAL_W** |
| _REQ-PHY-005_ | _REQ-PHY-006_ | _REQ-PHY-007_ | _REQ-PHY-008_ | _REQ-PHY-009_ | _REQ-PHY-010_ |
|1000|0|0.0046|7.4144|0.1109|0.8221|
|1100|255|0.0577|7.3948|0.3707|2.7413|
|1200|9673|0.1507|7.3496|0.9244|6.7937|
|1300|12263|0.2442|7.2964|1.5891|11.5948|
|1400|15017|0.3590|7.2197|2.5602|18.4842|
|1500|16627|0.4528|7.1375|3.5968|25.6726|
|1600|17717|0.5063|7.1063|4.0096|28.4936|
|1700|19505|0.6144|7.0017|5.3220|37.2631|
|1800|20811|0.7158|6.9085|6.5084|44.9634|
|1900|21741|0.7797|6.8283|7.4667|50.9851|
|2000|21673|0.7798|6.8285|7.4508|50.8775|
