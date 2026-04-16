% =========================================================================
% Generation Script LUT для BrotherHobby 1404 4600KV + T4030 on 2S (7.4V)
% =========================================================================

% CONSTANTS:
GRAMS_TO_NEWTONS = 9.80665; % from kgf to Newtons
MOTOR_TAB_SIZE = 101; % 100 steps + null point
MOTOR_V_NOMINAL = 7.4;
SYSTEM_STANDBY_CURRENT = 0.11; % ESC and system standby current at 0 RPM (from test stand data)
% Note: Actual motor idle current is 0.45A, but for V_eff calculations at throttle=0 we use system standby.

MOTOR_R_INTERNAL = 0.20748; % internal resistance in Ohm (207.48 mOhm from GROUP_SPEC)


% 1. Loading Data and remove the first line from csv file
filename = '../datasheets/Brother-Hobby-1404_4600KV_Blane_Townsend.csv';
% read file
raw_text = fileread(filename);
clean_text = strrep(raw_text, '"', '');
C = textscan(clean_text, '%f %f %f %f %f %f %f %f %f %f %f %f', ...
             'Delimiter', ',', 'HeaderLines', 1, 'CollectOutput', 1);
data = C{1};
printf("Strings loaded: %d\n", size(data, 1));

% 2. Extracting columns
% Col 2: Throttle (us), Col 3: RPM, Col 4: Thrust (N), Col 6: Voltage, Col 7: Current (A)
throttle_us = data(:, 2);
rpm = data(:, 3);
thrust_N = data(:, 4) * GRAMS_TO_NEWTONS; % from kgf to Newtons
voltage_V = data(:, 6);
current_A = data(:, 7);
power_electrical_W = data(:, 8);
% Columns 5 (torque), 9 (mech power), 10-12 (efficiencies) are available in
% the CSV but not used in this semi-empirical model. Torque would enable a
% canonical M = K_t·I electromechanical path, but the polynomial approach
% captures ESC + propeller nonlinearities more robustly for SIL use.


% Last point is very noisy (probably due to motor stall), so we will exclude it from the regression
valid_idx = rpm > 2000;

throttle_norm_filtered = (throttle_us(valid_idx) - 1000) / 1000;
thrust_N_filtered = thrust_N(valid_idx);
current_A_filtered = current_A(valid_idx);
rpm_filtered = rpm(valid_idx);


% 3. Normalization
throttle_norm = (throttle_us - 1000) / 1000;

% 4-5. Iterative V_eff normalization of RPM and current
%
% The LUT should represent motor behavior at V_eff_nom(t) = V_nom - I_nom(t)·R_m,
% where I_nom(t) is the current the motor draws at V_nominal for each throttle.
% But I_nom(t) depends on the polyfit, which depends on the normalization —
% a circular dependency. Solved by iterating from a V_terminal bootstrap.
%
% V_eff_actual uses measured current (known from the stand test).
% V_eff_nom uses estimated I_nom from the previous polyfit iteration.
% After 3 iterations the I_nom estimate converges to <0.1% change.

% Bootstrap: V_terminal normalization (first approximation)
rpm_norm = rpm .* (MOTOR_V_NOMINAL ./ voltage_V);
current_norm = SYSTEM_STANDBY_CURRENT + (current_A - SYSTEM_STANDBY_CURRENT) .* (MOTOR_V_NOMINAL ./ voltage_V).^2;

% V_eff at actual conditions (fixed — uses measured current from stand test)
V_eff_actual = voltage_V - current_A .* MOTOR_R_INTERNAL;

% Iterative refinement to V_eff basis
for iter = 1:3
  % Fit current polynomial on current normalization estimate
  p_cur_tmp = polyfit(throttle_norm_filtered, current_norm(valid_idx), 2);

  % Estimate I_nom(t) at V_nominal for each measurement throttle
  I_nom_est = max(polyval(p_cur_tmp, throttle_norm), SYSTEM_STANDBY_CURRENT);

  % V_eff at V_nominal with estimated nominal current
  V_eff_nom = MOTOR_V_NOMINAL - I_nom_est .* MOTOR_R_INTERNAL;

  % Re-normalize RPM and current to V_eff_nom
  rpm_norm = rpm .* (V_eff_nom ./ V_eff_actual);
  current_norm = SYSTEM_STANDBY_CURRENT + (current_A - SYSTEM_STANDBY_CURRENT) .* (V_eff_nom ./ V_eff_actual).^2;
end

rpm_norm_filtered = rpm_norm(valid_idx);
current_norm_filtered = current_norm(valid_idx);


% 6. Generating LUT in 100 steps
lut_throttle = linspace(0, 1, MOTOR_TAB_SIZE);

% Model RPM — fitted on NORMALIZED rpm (at V_nominal)
p_rpm = polyfit(throttle_norm_filtered, rpm_norm_filtered, 2);
lut_rpm = polyval(p_rpm, lut_throttle);

% --- Model Thrust (Physical model F = k * n^2) ---
% k is a propeller aerodynamic constant — fitted on RAW (unnormalized) data.
% Both rpm and thrust were measured simultaneously at the same conditions.
X_phys = rpm_filtered.^2;
Y_phys = thrust_N_filtered;
k_phys = sum(X_phys .* Y_phys) / sum(X_phys.^2);
% Thrust in LUT: k applied to NORMALIZED rpm → gives thrust at V_nominal
lut_thrust = k_phys * (lut_rpm.^2);


% --- Model Current (Polynomial based on NORMALIZED current) ---
p_current = polyfit(throttle_norm_filtered, current_norm_filtered, 2);
lut_current = polyval(p_current, lut_throttle);





% Secure LUT values to be non-negative
lut_thrust(lut_thrust < 0) = 0.0;
lut_current(lut_current < SYSTEM_STANDBY_CURRENT) = SYSTEM_STANDBY_CURRENT;
lut_rpm(lut_rpm < 0) = 0.0;

% Boundary conditions: zero throttle → zero thrust/RPM, standby current floor
lut_thrust(1)  = 0.0;
lut_current(1) = SYSTEM_STANDBY_CURRENT;
lut_rpm(1) = 0.0;


% 7. Putting LUT to the file /includes/motor_lut.h
fid = fopen('../includes/motor_lut.h', 'w');
if fid == -1
    error('Datei konnte nicht geöffnet werden: ../includes/motor_lut.h');
end

fprintf(fid, '// ==========================================');
fprintf(fid, '// AUTO-GENERATED LUT FOR C++ (2S - 7.4V)');
fprintf(fid, '// Motor: BrotherHobby 1404 4600KV');
fprintf(fid, '// Prop: T4030');
fprintf(fid, '// ==========================================');
fprintf(fid, '#ifndef MOTOR_LUT_H\n#define MOTOR_LUT_H\n\n');
fprintf(fid, 'static constexpr int MOTOR_TAB_SIZE = %d;\n\n', MOTOR_TAB_SIZE);
fprintf(fid, 'static constexpr float MOTOR_V_NOMINAL = %.2ff;\n\n', MOTOR_V_NOMINAL);
fprintf(fid, "// SYSTEM_STANDBY_CURRENT is the system standby current at 0 RPM (ESC + system),\n");
fprintf(fid, "// not the motor's mechanical idle current (0.45A at spinning idle).\n");
fprintf(fid, 'static constexpr float SYSTEM_STANDBY_CURRENT = %.2ff;\n\n', SYSTEM_STANDBY_CURRENT);
fprintf(fid, 'static constexpr float MOTOR_R_INTERNAL = %.5ff;\n\n', MOTOR_R_INTERNAL);

% --- Generation Tabelle MOTOR_TAB_GAS ---
fprintf(fid, 'static constexpr float MOTOR_TAB_GAS[] = {\n    ');
for i = 1:length(lut_throttle)
    fprintf(fid, '%.2ff', lut_throttle(i));
    if i < length(lut_throttle), fprintf(fid, ', '); end
    if mod(i, 10) == 0 && i < length(lut_throttle), fprintf(fid, '\n    '); end
end
fprintf(fid, '\n};\n\n');

% 9. Print array of thrust's values
% Writing Thrust
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

fclose(fid);
printf("Ok! LUT saved in %s\n");


% =========================================================================
% 5. VISUALIZATION
% =========================================================================
plot_dir = '../plots';
if ~exist(plot_dir, 'dir'), mkdir(plot_dir); end

% --- Graphik 1: Thrust vs RPM (F(n)) ---
fig1 = figure('Name', 'Thrust_RPM', 'Color', 'w');

% 1. Filtering: Exclude low RPM points (below 2000) where the data is very noisy and does not follow the physical model well
% It removes the noises
valid_idx = rpm > 2000;
n_filtered = rpm(valid_idx);
f_filtered = thrust_N(valid_idx);

% 2. Linearization: F = k * n^2  =>  F = k * X, where X = n^2
X = n_filtered.^2;
Y = f_filtered;

% 3. Calculation of the coefficient k using the least squares method (LSM) for a line through (0,0)
% Formula: k = sum(x*y) / sum(x^2)
k_phys = sum(X .* Y) / sum(X.^2);

% 4. Plotting the ideal curve based on the found k
rpm_line = linspace(0, max(rpm), 100);
thrust_model = k_phys * (rpm_line.^2);

% Plotting
plot(rpm, thrust_N, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Real Data');
hold on;
plot(rpm_line, thrust_model, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('Semi-Empirical Model'));

title('Thrust vs RPM');
xlabel('RPM [U/min]'); ylabel('Thrust [N]');
grid on; legend('Location', 'northwest');
saveas(fig1, fullfile(plot_dir, 'plot1_thrust_rpm.png'));

% --- Graphik 2: Current vs Thrust (I(F)) ---
% This graph is already excellent, we leave it as is
fig2 = figure('Name', 'Current_Thrust', 'Color', 'w');
plot(thrust_N, current_A, 'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Real Data');
hold on;
p_curr_thrust = polyfit(thrust_N, current_A, 2);
f_line = linspace(0, max(thrust_N), 100);
plot(f_line, polyval(p_curr_thrust, f_line), 'b-', 'LineWidth', 2, 'DisplayName', 'Poly-Fit I(F)');
title('Current vs Thrust');
xlabel('Thrust [N]'); ylabel('Current [A]');
grid on; legend('Location', 'northwest');
saveas(fig2, fullfile(plot_dir, 'plot2_current_thrust.png'));

% --- Graphik 3: Efficiency (eta = F/P) ---
fig3 = figure('Name', 'Efficiency', 'Color', 'w');
thrust_g = thrust_N / (GRAMS_TO_NEWTONS/1000);
eff_real = thrust_g ./ power_electrical_W;

plot(thrust_g, eff_real, 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Real Efficiency');
hold on;

% FIX: Exclude the first point (where thrust < 5g) from the trend calculation
active_idx = thrust_g > 5;
p_eff = polyfit(thrust_g(active_idx), eff_real(active_idx), 1);

eff_line = linspace(min(thrust_g(active_idx)), max(thrust_g), 100);
plot(eff_line, polyval(p_eff, eff_line), 'b-', 'LineWidth', 2, 'DisplayName', 'Operational Trend');

title('Propeller Efficiency (Operational Range)');
xlabel('Thrust [g]'); ylabel('Efficiency [g/W]');
grid on; legend('Location', 'northeast');
saveas(fig3, fullfile(plot_dir, 'plot3_efficiency.png'));

printf("Ok! LUT saved and plots generated in %s\n", plot_dir);

