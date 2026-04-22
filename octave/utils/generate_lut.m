function [] = generate_lut(filename, ...
                        MOTOR_TAB_SIZE, ...
                        MOTOR_V_NOMINAL, ...
                        SYSTEM_STANDBY_CURRENT, ...
                        MOTOR_R_INTERNAL, ...
                        lut_throttle, ...
                        lut_thrust, ...
                        lut_current, ...
                        lut_rpm ...
                        )

    fid = fopen(filename, 'w');
    if fid == -1
        error('Datei konnte nicht geöffnet werden: %s', filename);
    end

    gen_time = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    fprintf(fid, '// ==========================================\n');
    fprintf(fid, '// AUTO-GENERATED LUT FOR C++ (2S - 7.4V)\n');
    fprintf(fid, '// Generated on: %s\n', gen_time);
    fprintf(fid, '// Motor: BrotherHobby 1404 4600KV\n');
    fprintf(fid, '// Prop: T4030\n');
    fprintf(fid, "// SYSTEM_STANDBY_CURRENT is the system standby current at 0 RPM (ESC + system),\n");
    fprintf(fid, "// It's not the motor's mechanical idle current (0.45A at spinning idle).\n");
    fprintf(fid, '// ==========================================\n');

    fprintf(fid, '#ifndef MOTOR_LUT_H\n');
    fprintf(fid, '#define MOTOR_LUT_H\n\n');

    fprintf(fid, '// %s %s %g %s \n', MOTOR_TAB_SIZE.constraint_name, MOTOR_TAB_SIZE.parameter_name, MOTOR_TAB_SIZE.parameter_value, MOTOR_TAB_SIZE.parameter_unit);
    fprintf(fid, 'static constexpr int MOTOR_TAB_SIZE = %d;\n\n', MOTOR_TAB_SIZE.parameter_value);

    fprintf(fid, '// %s %s %g %s \n', MOTOR_V_NOMINAL.constraint_name, MOTOR_V_NOMINAL.parameter_name, MOTOR_V_NOMINAL.parameter_value, MOTOR_V_NOMINAL.parameter_unit);
    fprintf(fid, 'static constexpr float MOTOR_V_NOMINAL = %.2ff;\n\n', MOTOR_V_NOMINAL.parameter_value);

    fprintf(fid, '// %s %s %g %s \n', SYSTEM_STANDBY_CURRENT.constraint_name, SYSTEM_STANDBY_CURRENT.parameter_name, SYSTEM_STANDBY_CURRENT.parameter_value, SYSTEM_STANDBY_CURRENT.parameter_unit);
    fprintf(fid, 'static constexpr float SYSTEM_STANDBY_CURRENT = %.2ff;\n\n', SYSTEM_STANDBY_CURRENT.parameter_value);

    fprintf(fid, '// %s %s %g %s \n', MOTOR_R_INTERNAL.constraint_name, MOTOR_R_INTERNAL.parameter_name, MOTOR_R_INTERNAL.parameter_value, MOTOR_R_INTERNAL.parameter_unit);
    fprintf(fid, 'static constexpr float MOTOR_R_INTERNAL = %.5ff;\n\n', MOTOR_R_INTERNAL.parameter_value);


    % --- Generation Tabelle MOTOR_TAB_GAS ---
    fprintf(fid, '// REQ-PHY-005\n');
    fprintf(fid, 'static constexpr float MOTOR_TAB_GAS[] = {\n    ');
    for i = 1:length(lut_throttle)
        fprintf(fid, '%.2ff', lut_throttle(i));
        if i < length(lut_throttle), fprintf(fid, ', '); end
        if mod(i, 10) == 0 && i < length(lut_throttle), fprintf(fid, '\n    '); end
    end
    fprintf(fid, '\n};\n\n');

    % 9. Print array of thrust's values
    % Writing Thrust
    fprintf(fid, '// REQ-PHY-007\n');
    fprintf(fid, 'static constexpr float MOTOR_TAB_SCHUB_N[] = {\n    ');
    for i = 1:length(lut_thrust)
        fprintf(fid, '%.4ff', lut_thrust(i));
        if i < length(lut_thrust)
            fprintf(fid, ', ');
        end
        if mod(i, 10) == 0 && i < length(lut_thrust)
            fprintf(fid, '\n    ');
        end
    end
    fprintf(fid, '\n};\n\n');

    % Writing Strom (Current)
    fprintf(fid, '// REQ-PHY-009\n');
    fprintf(fid, 'static constexpr float MOTOR_TAB_STROM[] = {\n    ');
    for i = 1:length(lut_current)
        fprintf(fid, '%.4ff', lut_current(i));
        if i < length(lut_current)
            fprintf(fid, ', ');
        end
        if mod(i, 10) == 0 && i < length(lut_current)
            fprintf(fid, '\n    ');
        end
    end
    fprintf(fid, '\n};\n\n');

    % Writing RPM (Current)
    fprintf(fid, '// REQ-PHY-006\n');
    fprintf(fid, 'static constexpr float MOTOR_TAB_DREHZAHL[] = {\n    ');
    for i = 1:length(lut_rpm)
        fprintf(fid, '%.4ff', lut_rpm(i));
        if i < length(lut_rpm)
            fprintf(fid, ', ');
        end
        if mod(i, 10) == 0 && i < length(lut_rpm)
            fprintf(fid, '\n    ');
        end
    end
    fprintf(fid, '\n};\n\n');

    fprintf(fid, '\n#endif // MOTOR_LUT_H\n');
    fclose(fid);

end