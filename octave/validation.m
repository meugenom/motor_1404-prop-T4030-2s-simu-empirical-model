% Validation Test Results
addpath("plots", "utils", "core", "validation");

KGF_TO_NEWTONS.parameter_value = 9.80665;

% Define the directory for saving plots
plot_dir = '../octave/plots';

% 1. Define filenames and get tested result data
filename_tested_table = '../logs/test_posix_parsed.csv';
[THROTTLE_TEST, THRUST_TEST_N, CURRENT_TEST_A] = parse_test_table(filename_tested_table);

% 2. Define the filename for the motor pure row data
filename_motor_propeller_properties = '../references/Brother-Hobby-1404-4600KV_Propeller-T4030.csv';
[THROTTLE_US, RPM, THRUST_N, VOLTAGE_V, CURRENT_A, POWER_ELECTRICAL_W] = parse_motor_propeller_properties(filename_motor_propeller_properties, KGF_TO_NEWTONS.parameter_value);

% 3. Calculate relative errors between tested results and motor pure row data
err_thrust_pct = (THRUST_TEST_N - THRUST_N) ./ max(THRUST_N, 0.001)*100;
err_current_pct = (CURRENT_TEST_A - CURRENT_A) ./ max(CURRENT_A, 0.001)*100;

% 4. Draw the error analysis plot
figure_1 = figure('Name', 'Analysis Errors', 'Position', [100, 100, 800, 500]);
hold on; grid on;

% Thrust error line (blue, with markers)
plot(THROTTLE_TEST * 100, err_thrust_pct, '-ob', 'LineWidth', 2, 'DisplayName', 'Thrust Error (%)');

% Current error line (red, with markers)
plot(THROTTLE_TEST * 100, err_current_pct, '-sr', 'LineWidth', 2, 'DisplayName', 'Current Error (%)');

title('Discrepancy between C++ model (Posix/STM32) and real test stand (Datasheet)');
xlabel('Throttle Level (%)');
ylabel('Relative Error (%)');
legend('Location', 'southeast');

% Add a "tolerance limit" line (e.g., 5%)
yline(5, '--k', 'Tolerance 5%', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');

hold off;

% Optionally: Save the plot as an image for SPEC.md
saveas(figure_1, fullfile(plot_dir, 'plot4_errors.png'));
