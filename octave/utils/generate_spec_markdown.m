function [] = generate_spec_markdown(filename, MOTOR_TAB_SIZE, MOTOR_V_NOMINAL, SYSTEM_STANDBY_CURRENT, MOTOR_R_INTERNAL, ...
                                        THROTTLE_US, RPM, THRUST_N, VOLTAGE_V, CURRENT_A, POWER_ELECTRICAL_W)


    % Putting SPEC to the file ./SPEC.md
    fid = fopen('../SPEC.md', 'w');
        if fid == -1
            error('Datei konnte nicht geöffnet werden: ../SPEC.md');
    end

    gen_time = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    % Write the specifications to the file
    fprintf(fid, '# Specifications for Brother Hobby 1404 KV4600 Motor with iFlight Nazgul T4030 Propeller:\n');
    fprintf(fid, '> AUTO-GENERATED from `/octave/main.m` on %s\n\n', gen_time);
    fprintf(fid, 'Based on [Brother Hobby 1404 - 4600KV test data](https://database.tytorobotics.com/tests/7xzn/brother-hobby-1404-4600kv) from tytorobotics.com.\n\n');
    fprintf(fid, 'Motor Specifications are sourced from [Brother Hobby Store](https://www.brotherhobbystore.com/products/tc-1404-ultralight-motor-139).\n\n');
    fprintf(fid, 'Source for propeller specifications is the [iFlight Nazgul T4030 product page](https://shop.iflight.com/nazgul-t4030-propellers-cw-ccw-3sets-6pairs-pro1258).\n\n');

    fprintf(fid, '## Components:\n');
    fprintf(fid, '1. **Motor:** Brother Hobby 1404 KV4600\n');
    fprintf(fid, '2. **Propeller:** iFlight Nazgul T4030\n');
    fprintf(fid, '3. **ESC:** VGood 60A SBEC 2-6S LIPO\n');

    fprintf(fid, '## 1. Motor Specifications\n\n');
    fprintf(fid, 'These values represent the physical and electrical properties of the two motor variants.\n\n');
    fprintf(fid, '|Specification|KV4600|\n');
    fprintf(fid, '|---|---|\n');
    fprintf(fid, '|**Motor Dimensions**|$\\Phi 18.2 \times 16$ mm|\n');
    fprintf(fid, '|**Stator Dimensions**|14 mm|\n');
    fprintf(fid, '|**Stator/Magnet Config**|9N12P/NSK5x2x2.5mm|\n');
    fprintf(fid, '|**Idle Current@7V (A)**|0.45 A|\n');
    fprintf(fid, '|**Shaft Diameter**|1.5 mm|\n');
    fprintf(fid, '|**Lead Wire**|30#AWG 100 mm|\n');
    fprintf(fid, '|**Weight (Incl. Cable)**|8.7 g|\n');
    fprintf(fid, '|**Internal Resistance**|$207.48 m\\Omega$|\n');
    fprintf(fid, '|**Rated Voltage (LiPo)**|3-4S|\n');
    fprintf(fid, '|**Max Current(A)**|13.6 A|\n');
    fprintf(fid, '|**Max Power (W)**|217.6 W|\n');

    fprintf(fid, '## 2. Propeller Specifications\n\n');
    fprintf(fid, '|Specification|iFlight Nazgul T4030|\n');
    fprintf(fid, '|---|---|\n');
    fprintf(fid, '|**Diameter**|4 inch|\n');
    fprintf(fid, '|**Pitch**|3 inch|\n');
    fprintf(fid, '|**Number of Blades**|2|\n');
    fprintf(fid, '|**Material**|Plastic|\n');
    fprintf(fid, '|**Weight**|1.2 g|\n');


    fprintf(fid, '## 3. Constraints and Parameters\n\n');
    fprintf(fid, '|Constraint Name| Parameter Name | Parameter Value| Parameter Unit|\n');
    fprintf(fid, '|---|---|---|---|\n');
    fprintf(fid, '|%s|%s|%g|%s|\n', ...
    MOTOR_TAB_SIZE.constraint_name, ...
    MOTOR_TAB_SIZE.parameter_name, ...
    MOTOR_TAB_SIZE.parameter_value, ...
    MOTOR_TAB_SIZE.parameter_unit);

    fprintf(fid, '|%s|%s|%.4f|%s|\n', MOTOR_V_NOMINAL.constraint_name, MOTOR_V_NOMINAL.parameter_name, MOTOR_V_NOMINAL.parameter_value, MOTOR_V_NOMINAL.parameter_unit);    
    fprintf(fid, '|%s|%s|%.4f|%s|\n', SYSTEM_STANDBY_CURRENT.constraint_name, SYSTEM_STANDBY_CURRENT.parameter_name, SYSTEM_STANDBY_CURRENT.parameter_value, SYSTEM_STANDBY_CURRENT.parameter_unit);
    fprintf(fid, '|%s|%s|%.4f|%s|\n', MOTOR_R_INTERNAL.constraint_name, MOTOR_R_INTERNAL.parameter_name, MOTOR_R_INTERNAL.parameter_value, MOTOR_R_INTERNAL.parameter_unit);


    fprintf(fid, '## 4. Test Report from tytorobotics.com\n\n');
    fprintf(fid, '| Throttle (µs) | Rotation speed (rpm) | Thrust (kgf) | Voltage (V) | Current (A) | Electrical power (W) |\n');
    fprintf(fid, '|---|---|---|---|---|---|\n');
    fprintf(fid, '| **THROTTLE_US** | **RPM** | **THRUST_KGF** | **VOLTAGE_V** | **CURRENT_A** | **POWER_ELECTRICAL_W** |\n');
    fprintf(fid, '| _REQ-PHY-005_ | _REQ-PHY-006_ | _REQ-PHY-007_ | _REQ-PHY-008_ | _REQ-PHY-009_ | _REQ-PHY-010_ |\n');
    for i = 1:length(THROTTLE_US)
        fprintf(fid, '|%g|%g|%.4f|%.4f|%.4f|%.4f|\n', THROTTLE_US(i), RPM(i), THRUST_N(i), VOLTAGE_V(i), CURRENT_A(i), POWER_ELECTRICAL_W(i));
    end

    fclose(fid);
end
