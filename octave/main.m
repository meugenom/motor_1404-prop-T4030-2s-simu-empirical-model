% main.m
addpath("plots", "utils", "core", "validation");

% Define the filename for the motor properties CSV file
filename_motor_properties = '../references/Brother-Hobby-1404-4600KV-Properties.csv';
% Define the filename for the motor model CSV file
filename_motor_propeller_properties = '../references/Brother-Hobby-1404-4600KV_Propeller-T4030.csv';
% Define the filename for the output SPEC.md file
filename_spec_md = './SPEC.md';
% Define the filename for the output LUT header file
filename_lut = '../src/includes/motor_lut.h';
% Define the directory for saving plots
plot_dir = '../octave/plots';



% 1. Run parsing from datasheet
[GRAMS_TO_NEWTONS, MOTOR_TAB_SIZE, MOTOR_V_NOMINAL, SYSTEM_STANDBY_CURRENT, MOTOR_R_INTERNAL] = parse_motor_properties(filename_motor_properties);

% 2. Run parsing from motor model CSV file
[THROTTLE_US, RPM, THRUST_N, VOLTAGE_V, CURRENT_A, POWER_ELECTRICAL_W] = parse_motor_propeller_properties(filename_motor_propeller_properties, GRAMS_TO_NEWTONS.parameter_value);

printf("Ok! Data parsed from CSV files.\n");

% 3. Generate SPEC.md file
generate_spec_markdown(filename_spec_md, ...
    MOTOR_TAB_SIZE, MOTOR_V_NOMINAL, SYSTEM_STANDBY_CURRENT, MOTOR_R_INTERNAL, ...
    THROTTLE_US, RPM, THRUST_N, VOLTAGE_V, CURRENT_A, POWER_ELECTRICAL_W);

printf("Ok! SPEC.md generated in %s\n", filename_spec_md);    

% 4-5. Iterative V_eff normalization of RPM and current
valid_idx = RPM > 2000;

THROTTLE_NORM_FILTERED = (THROTTLE_US(valid_idx) - 1000) / 1000;
THRUST_N_FILTERED = THRUST_N(valid_idx);
CURRENT_A_FILTERED = CURRENT_A(valid_idx);
RPM_FILTERED = RPM(valid_idx);

THROTTLE_NORM = (THROTTLE_US - 1000) / 1000;

% The LUT should represent motor behavior at V_eff_nom(t) = V_nom - I_nom(t)·R_m,
% where I_nom(t) is the current the motor draws at V_nominal for each throttle.
% But I_nom(t) depends on the polyfit, which depends on the normalization —
% a circular dependency. Solved by iterating from a V_terminal bootstrap.
%
% V_eff_actual uses measured current (known from the stand test).
% V_eff_nom uses estimated I_nom from the previous polyfit iteration.
% After 3 iterations the I_nom estimate converges to <0.1% change.

% Bootstrap: V_terminal normalization (first approximation)
RPM_NORM = RPM .* (MOTOR_V_NOMINAL.parameter_value ./ VOLTAGE_V);
CURRENT_NORM = SYSTEM_STANDBY_CURRENT.parameter_value + (CURRENT_A - SYSTEM_STANDBY_CURRENT.parameter_value) .* (MOTOR_V_NOMINAL.parameter_value ./ VOLTAGE_V).^2;

% V_eff at actual conditions (fixed — uses measured current from stand test)
V_eff_actual = VOLTAGE_V - CURRENT_A .* MOTOR_R_INTERNAL.parameter_value;

% Iterative refinement to V_eff basis
for iter = 1:3
  % Fit current polynomial on current normalization estimate
  p_cur_tmp = polyfit(THROTTLE_NORM_FILTERED, CURRENT_NORM(valid_idx), 2);

  % Estimate I_nom(t) at V_nominal for each measurement throttle
  I_nom_est = max(polyval(p_cur_tmp, THROTTLE_NORM), SYSTEM_STANDBY_CURRENT.parameter_value);

  % V_eff at V_nominal with estimated nominal current
  V_eff_nom = MOTOR_V_NOMINAL.parameter_value - I_nom_est .* MOTOR_R_INTERNAL.parameter_value;

  % Re-normalize RPM and current to V_eff_nom
  RPM_NORM = RPM .* (V_eff_nom ./ V_eff_actual);
  CURRENT_NORM = SYSTEM_STANDBY_CURRENT.parameter_value + (CURRENT_A - SYSTEM_STANDBY_CURRENT.parameter_value) .* (V_eff_nom ./ V_eff_actual).^2;
end

RPM_NORM_FILTERED = RPM_NORM(valid_idx);
CURRENT_NORM_FILTERED = CURRENT_NORM(valid_idx);


% 6. Generating LUT in 100 steps
lut_throttle = linspace(0, 1, MOTOR_TAB_SIZE.parameter_value);

% Model RPM — fitted on NORMALIZED rpm (at V_nominal)
p_rpm = polyfit(THROTTLE_NORM_FILTERED, RPM_NORM_FILTERED, 2);
%lut_rpm = polyval(p_rpm, lut_throttle);
lut_rpm = interp1(THROTTLE_NORM, RPM_NORM, lut_throttle, 'pchip', 'extrap');


% --- Model Thrust (Physical model F = k * n^2) ---
% k is a propeller aerodynamic constant — fitted on RAW (unnormalized) data.
% Both rpm and thrust were measured simultaneously at the same conditions.
X_phys = RPM_FILTERED.^2;
Y_phys = THRUST_N_FILTERED;
k_phys = sum(X_phys .* Y_phys) / sum(X_phys.^2);
% Thrust in LUT: k applied to NORMALIZED rpm → gives thrust at V_nominal
lut_thrust = k_phys * (lut_rpm.^2);


% --- Model Current (Polynomial based on NORMALIZED current) ---
p_current = polyfit(THROTTLE_NORM_FILTERED, CURRENT_NORM_FILTERED, 2);
%lut_current = polyval(p_current, lut_throttle);
lut_current = interp1(THROTTLE_NORM, CURRENT_NORM, lut_throttle, 'pchip', 'extrap');



% Secure LUT values to be non-negative
lut_thrust(lut_thrust < 0) = 0.0;
lut_current(lut_current < SYSTEM_STANDBY_CURRENT.parameter_value) = SYSTEM_STANDBY_CURRENT.parameter_value;
lut_rpm(lut_rpm < 0) = 0.0;

% Boundary conditions: zero throttle → zero thrust/RPM, standby current floor
lut_thrust(1)  = 0.0;
lut_current(1) = SYSTEM_STANDBY_CURRENT.parameter_value;
lut_rpm(1) = 0.0;



% 7. Putting LUT to the file /includes/motor_lut.h
generate_lut(filename_lut, ...
                    MOTOR_TAB_SIZE, ...
                    MOTOR_V_NOMINAL, ...
                    SYSTEM_STANDBY_CURRENT, ...
                    MOTOR_R_INTERNAL, ...
                    lut_throttle, ...
                    lut_thrust, ...
                    lut_current, ...
                    lut_rpm ...
                    )
printf("Ok! LUT saved in %s\n", filename_lut);


% 8. Plot graphics
generate_plot(plot_dir, ...                        
                        RPM, ...
                        THRUST_N, ...
                        CURRENT_A, ...
                        POWER_ELECTRICAL_W, ...
                        GRAMS_TO_NEWTONS ...
                        )
printf("Ok! Plots generated in %s\n", plot_dir);


