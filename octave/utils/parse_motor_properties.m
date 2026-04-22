function [GRAMS_TO_NEWTONS, MOTOR_TAB_SIZE, MOTOR_V_NOMINAL, SYSTEM_STANDBY_CURRENT, MOTOR_R_INTERNAL] = parse_motor_properties(filename)

    printf('Parsing motor properties from file: %s\n', filename);

    % read file
    raw_text = fileread(filename);
    clean_text = strrep(raw_text, '"', '');
    lines = strsplit(clean_text, '\n');

    % Initialize variables
    GRAMS_TO_NEWTONS = {};
    MOTOR_TAB_SIZE = {};
    MOTOR_V_NOMINAL = {};
    SYSTEM_STANDBY_CURRENT = {};
    MOTOR_R_INTERNAL = {};

    % Parse lines
    for i = 1:length(lines)

        line = strtrim(lines{i});

        if isempty(line)
            continue; % skip empty lines
        end

        parts = strsplit(line, ';');

        if length(parts) ~= 4
            continue; % skip malformed lines
        end

        constraint_name = strtrim(parts{1});
        parameter_name = strtrim(parts{2});
        parameter_value = strtrim(parts{3});
        parameter_unit = strtrim(parts{4}); % not used in this function

        % Assign values based on parameter name
        switch parameter_name
            case 'GRAMS_TO_NEWTONS'
                GRAMS_TO_NEWTONS.constraint_name = constraint_name;
                GRAMS_TO_NEWTONS.parameter_name = parameter_name;
                GRAMS_TO_NEWTONS.parameter_value = str2double(parameter_value);
                GRAMS_TO_NEWTONS.parameter_unit = parameter_unit;
            case 'MOTOR_TAB_SIZE'
                MOTOR_TAB_SIZE.constraint_name = constraint_name;
                MOTOR_TAB_SIZE.parameter_name = parameter_name;
                MOTOR_TAB_SIZE.parameter_value = str2double(parameter_value);
                MOTOR_TAB_SIZE.parameter_unit = parameter_unit;
            case 'MOTOR_V_NOMINAL'
                MOTOR_V_NOMINAL.constraint_name = constraint_name;
                MOTOR_V_NOMINAL.parameter_name = parameter_name;
                MOTOR_V_NOMINAL.parameter_value = str2double(parameter_value);
                MOTOR_V_NOMINAL.parameter_unit = parameter_unit;
            case 'SYSTEM_STANDBY_CURRENT'
                SYSTEM_STANDBY_CURRENT.constraint_name = constraint_name;
                SYSTEM_STANDBY_CURRENT.parameter_name = parameter_name;
                SYSTEM_STANDBY_CURRENT.parameter_value = str2double(parameter_value);
                SYSTEM_STANDBY_CURRENT.parameter_unit = parameter_unit;
            case 'MOTOR_R_INTERNAL'
                MOTOR_R_INTERNAL.constraint_name = constraint_name;
                MOTOR_R_INTERNAL.parameter_name = parameter_name;
                MOTOR_R_INTERNAL.parameter_value = str2double(parameter_value);
                MOTOR_R_INTERNAL.parameter_unit = parameter_unit;
        end
    end
end
