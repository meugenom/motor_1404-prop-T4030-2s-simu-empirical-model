function [THROTTLE_US, RPM, THRUST_N, VOLTAGE_V, CURRENT_A, POWER_ELECTRICAL_W] = parse_motor_propeller_properties(filename, GRAMS_TO_NEWTONS)
    
    printf('Parsing motor properties from file: %s\n', filename);

    % read file
    raw_text = fileread(filename);
    clean_text = strrep(raw_text, '"', '');
    C = textscan(clean_text, '%f %f %f %f %f %f %f %f %f %f %f %f', ...
                'Delimiter', ',', 'HeaderLines', 1, 'CollectOutput', 1);
    data = C{1};
    printf("Strings loaded: %d\n", size(data, 1));

    % 2. Extracting columns
    % Col 2: Throttle (us), Col 3: RPM, Col 4: Thrust (N), Col 6: Voltage, Col 7: Current (A)

    THROTTLE_US = data(:, 2);
    RPM = data(:, 3);
    THRUST_N = data(:, 4) * GRAMS_TO_NEWTONS; % from kgf to Newtons
    VOLTAGE_V = data(:, 6);
    CURRENT_A = data(:, 7);
    POWER_ELECTRICAL_W = data(:, 8);
    % Columns 5 (torque), 9 (mech power), 10-12 (efficiencies) are available in
    % the CSV but not used in this semi-empirical model. 
    % Torque would enable a
    % canonical M = K_t·I electromechanical path, but the polynomial approach
    % captures ESC + propeller nonlinearities more robustly for SIL use.

end