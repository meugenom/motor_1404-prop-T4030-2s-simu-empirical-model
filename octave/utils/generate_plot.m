function [] = generate_plot(plot_dir, ...                        
                        RPM, ...
                        THRUST_N, ...
                        CURRENT_A, ...
                        POWER_ELECTRICAL_W, ...
                        GRAMS_TO_NEWTONS ...                        
                        )

    if ~exist(plot_dir, 'dir'), mkdir(plot_dir); end

    % --- Graphik 1: Thrust vs RPM (F(n)) ---
    fig1 = figure('Name', 'Thrust_RPM', 'Color', 'w');

    % 1. Filtering: Exclude low RPM points (below 2000) where the data is very noisy and does not follow the physical model well
    % It removes the noises
    valid_idx = RPM > 2000;
    n_filtered = RPM(valid_idx);
    f_filtered = THRUST_N(valid_idx);

    % 2. Linearization: F = k * n^2  =>  F = k * X, where X = n^2
    X = n_filtered.^2;
    Y = f_filtered;

    % 3. Calculation of the coefficient k using the least squares method (LSM) for a line through (0,0)
    % Formula: k = sum(x*y) / sum(x^2)
    k_phys = sum(X .* Y) / sum(X.^2);

    % 4. Plotting the ideal curve based on the found k
    rpm_line = linspace(0, max(RPM), 100);
    thrust_model = k_phys * (rpm_line.^2);

    % Plotting
    plot(RPM, THRUST_N, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Real Data');
    hold on;
    plot(rpm_line, thrust_model, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('Semi-Empirical Model'));

    title('Thrust vs RPM');
    xlabel('RPM [U/min]'); ylabel('Thrust [N]');
    grid on; legend('Location', 'northwest');
    saveas(fig1, fullfile(plot_dir, 'plot1_thrust_rpm.png'));

    % --- Graphik 2: Current vs Thrust (I(F)) ---
    % This graph is already excellent, we leave it as is
    fig2 = figure('Name', 'Current_Thrust', 'Color', 'w');
    plot(THRUST_N, CURRENT_A, 'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Real Data');
    hold on;
    p_curr_thrust = polyfit(THRUST_N, CURRENT_A, 2);
    f_line = linspace(0, max(THRUST_N), 100);
    plot(f_line, polyval(p_curr_thrust, f_line), 'b-', 'LineWidth', 2, 'DisplayName', 'Poly-Fit I(F)');
    title('Current vs Thrust');
    xlabel('Thrust [N]'); ylabel('Current [A]');
    grid on; legend('Location', 'northwest');
    saveas(fig2, fullfile(plot_dir, 'plot2_current_thrust.png'));

    % --- Graphik 3: Efficiency (eta = F/P) ---
    fig3 = figure('Name', 'Efficiency', 'Color', 'w');
    thrust_g = THRUST_N / (GRAMS_TO_NEWTONS.parameter_value/1000);
    eff_real = thrust_g ./ POWER_ELECTRICAL_W;

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


end