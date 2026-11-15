%RUN_ALL_SIMS  Master script for PS11 Quadcopter simulation & comparative analysis.
%   Executes all required studies from PS11:
%     1. Linearization, transfer functions, Bode margins, pole-zero map
%     2. Nominal step tracking (Roll, Pitch, Yaw, Altitude)
%     3. Sine / multi-step reference tracking
%     4. External disturbance rejection (torque and wind pulses)
%     5. Parameter variation sensitivity (+/-20% mass & inertia, +/-15% thrust constant)
%     6. Sensor noise and actuator saturation effects
%     7. Comparative analysis: Cascaded PID vs LQR+I controller
%   Saves figures (.png) and a summary table (metrics_summary.csv) in matlab/results/.

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
res_dir = fullfile(fileparts(mfilename('fullpath')), 'results', 'hw');
if ~exist(res_dir, 'dir'), mkdir(res_dir); end

p        = params();
use_hw   = true;   % true: hardware-tuned firmware gains (config.h); false: model-based design
ctrl_pid = design_gains(p, 'PID', struct('hw', use_hw));
ctrl_lqr = design_gains(p, 'LQR');

fprintf('=== FIRMWARE EQUIVALENT GAINS (for firmware/quad_fc/config.h) ===\n');
disp(ctrl_pid.fw);

%% 1. Linear stability analysis
lin = linearize_quad(p, ctrl_pid);

%% 2. Scenario definitions
% S1: Nominal step response (Roll 10 deg @ 1s, Pitch -8 deg @ 3s, Yaw 30 deg @ 5s, Alt 0.5 m @ 1s)
sc_nom = struct('t_end', 10, ...
    'roll_ref',  @(t) deg2rad(10)*(t >= 1), ...
    'pitch_ref', @(t) deg2rad(-8)*(t >= 3), ...
    'yaw_ref',   @(t) deg2rad(30)*(t >= 5), ...
    'alt_ref',   @(t) 0.5*(t >= 1));

% S2: External disturbance (0.10 N m roll torque pulse @ 4..4.15s, 1.5 N downward gust @ 6..6.5s)
sc_dist = sc_nom;
sc_dist.dist = @(t) [0; 0; 1.5*(t>=6 && t<6.5); 0.10*(t>=4 && t<4.15); -0.08*(t>=4 && t<4.15); 0];

% S3: Parameter variation (+20% mass, +20% inertia, -15% motor thrust constant, +25% motor lag)
sc_pert = sc_nom;
sc_pert.mScale = 1.20; sc_pert.JScale = 1.20; sc_pert.kTScale = 0.85; sc_pert.tauMScale = 1.25;

% S4: Realistic hardware conditions (IMU/altitude sensor noise + 1 sample delay + saturation)
sc_real = sc_dist;
sc_real.noise = true; sc_real.delay = 1; sc_real.sat = true;

% S5: Sinusoidal tracking (Roll & Pitch 10 deg @ 0.4 Hz)
sc_sine = struct('t_end', 10, ...
    'roll_ref',  @(t) deg2rad(10)*sin(2*pi*0.4*t).*(t >= 0.5), ...
    'pitch_ref', @(t) deg2rad(10)*cos(2*pi*0.4*t).*(t >= 0.5), ...
    'yaw_ref',   @(t) 0, 'alt_ref', @(t) 0.5*(t >= 0.2), 'noise', true);

%% 3. Run simulations
fprintf('\nRunning nonlinear 6-DOF simulations...\n');
out_nom_pid  = sim_quad(sc_nom,  p, ctrl_pid);
out_nom_lqr  = sim_quad(sc_nom,  p, ctrl_lqr);
out_dist_pid = sim_quad(sc_dist, p, ctrl_pid);
out_dist_lqr = sim_quad(sc_dist, p, ctrl_lqr);
out_pert_pid = sim_quad(sc_pert, p, ctrl_pid);
out_pert_lqr = sim_quad(sc_pert, p, ctrl_lqr);
out_real_pid = sim_quad(sc_real, p, ctrl_pid);
out_sine_pid = sim_quad(sc_sine, p, ctrl_pid);

%% 4. Compute metrics table
runs  = {out_nom_pid, out_nom_lqr, out_dist_pid, out_dist_lqr, out_pert_pid, out_pert_lqr, out_real_pid};
names = {'Nominal_PID','Nominal_LQR','Disturbance_PID','Disturbance_LQR','ParamVar_PID','ParamVar_LQR','Realistic_Noisy_PID'};

fprintf('\n%-20s | %-24s | %-24s | %-24s\n', 'Scenario', 'Roll (10 deg @ 1s)', 'Yaw (30 deg @ 5s)', 'Alt (0.5 m @ 1s)');
fprintf('%-20s | %-24s | %-24s | %-24s\n', '', 'OS%   Ts(2%)  SSE(deg)', 'OS%   Ts(2%)  SSE(deg)', 'OS%   Ts(5%)  SSE(m)');
fprintf('%s\n', repmat('-',1,102));

rows = {};
for i = 1:numel(runs)
    o = runs{i};
    mr = compute_metrics(o.t, rad2deg(o.eul(:,1)), 10, 1, 0);
    mp = compute_metrics(o.t, rad2deg(o.eul(:,2)), -8, 3, 0);
    my = compute_metrics(o.t, rad2deg(o.eul(:,3)), 30, 5, 0);
    ma = compute_metrics(o.t, o.alt,              0.5, 1, 0);
    fprintf('%-20s | %5.1f %6.2fs %7.3f    | %5.1f %6.2fs %7.3f    | %5.1f %6.2fs %7.3f\n', ...
        names{i}, mr.overshoot_pct, mr.settling_2pct_s, mr.sse, ...
        my.overshoot_pct, my.settling_2pct_s, my.sse, ...
        ma.overshoot_pct, ma.settling_5pct_s, ma.sse);
    rows(end+1,:) = {names{i}, mr.overshoot_pct, mr.rise_time_s, mr.settling_2pct_s, mr.sse, mr.rmse, ...
                               mp.overshoot_pct, mp.settling_2pct_s, mp.sse, ...
                               my.overshoot_pct, my.settling_2pct_s, my.sse, ...
                               ma.overshoot_pct, ma.settling_5pct_s, ma.sse}; %#ok<SAGROW>
end

T = cell2table(rows, 'VariableNames', ...
    {'Scenario','Roll_OS_pct','Roll_Tr_s','Roll_Ts2_s','Roll_SSE_deg','Roll_RMSE_deg', ...
     'Pitch_OS_pct','Pitch_Ts2_s','Pitch_SSE_deg', ...
     'Yaw_OS_pct','Yaw_Ts2_s','Yaw_SSE_deg', ...
     'Alt_OS_pct','Alt_Ts5_s','Alt_SSE_m'});
writetable(T, fullfile(res_dir, 'metrics_summary.csv'));

%% 5. Generate publication figures
% Fig 2: Nominal PID vs LQR step tracking
fig = figure('Visible','off','Position',[80 80 1100 700]);
labels = {'Roll \phi (deg)','Pitch \theta (deg)','Yaw \psi (deg)','Altitude h (m)'};
for k = 1:4
    subplot(2,2,k); hold on; grid on;
    if k <= 3
        plot(out_nom_pid.t, rad2deg(out_nom_pid.ref(:,k)), 'k--', 'LineWidth', 1.2);
        plot(out_nom_pid.t, rad2deg(out_nom_pid.eul(:,k)), 'b-',  'LineWidth', 1.6);
        plot(out_nom_lqr.t, rad2deg(out_nom_lqr.eul(:,k)), 'r-.', 'LineWidth', 1.4);
    else
        plot(out_nom_pid.t, out_nom_pid.ref(:,4), 'k--', 'LineWidth', 1.2);
        plot(out_nom_pid.t, out_nom_pid.alt,      'b-',  'LineWidth', 1.6);
        plot(out_nom_lqr.t, out_nom_lqr.alt,      'r-.', 'LineWidth', 1.4);
    end
    xlabel('Time (s)'); ylabel(labels{k});
    if k == 1, legend('Reference','Cascaded PID','LQR + I','Location','best'); end
    title(['Nominal Step Response: ', labels{k}]);
end
saveas(fig, fullfile(res_dir, 'fig2_nominal_pid_vs_lqr.png')); close(fig);

% Fig 3: Disturbance rejection & Parameter uncertainty & Realistic noise
fig = figure('Visible','off','Position',[80 80 1100 700]);
subplot(2,2,1); hold on; grid on;
plot(out_nom_pid.t,  rad2deg(out_nom_pid.ref(:,1)),  'k--', 'LineWidth', 1.1);
plot(out_nom_pid.t,  rad2deg(out_nom_pid.eul(:,1)),  'b-',  'LineWidth', 1.4);
plot(out_dist_pid.t, rad2deg(out_dist_pid.eul(:,1)), 'r-',  'LineWidth', 1.4);
plot(out_pert_pid.t, rad2deg(out_pert_pid.eul(:,1)), 'g-.', 'LineWidth', 1.4);
plot(out_real_pid.t, rad2deg(out_real_pid.eul(:,1)), 'm:',  'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Roll (deg)'); title('Roll Tracking Under Robustness Scenarios');
legend('Ref','Nominal','Disturbance (4s)','Param Var (+20%m/J, -15%k_T)','Noisy + Delay','Location','best');

subplot(2,2,2); hold on; grid on;
plot(out_nom_pid.t,  out_nom_pid.ref(:,4),  'k--', 'LineWidth', 1.1);
plot(out_nom_pid.t,  out_nom_pid.alt,       'b-',  'LineWidth', 1.4);
plot(out_dist_pid.t, out_dist_pid.alt,      'r-',  'LineWidth', 1.4);
plot(out_pert_pid.t, out_pert_pid.alt,      'g-.', 'LineWidth', 1.4);
plot(out_real_pid.t, out_real_pid.alt,      'm:',  'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Altitude (m)'); title('Altitude Hold & Wind Gust Rejection (6s)');

subplot(2,2,3); hold on; grid on;
plot(out_real_pid.t, out_real_pid.T, 'LineWidth', 1.2);
yline(p.Tmax, 'r--', 'Max Thrust'); yline(p.Tidle, 'k:', 'Idle');
xlabel('Time (s)'); ylabel('Motor Thrust (N)'); title('Motor Thrusts (Realistic Scenario)');
legend('M1 (FR)','M2 (FL)','M3 (RL)','M4 (RR)','Location','best');

subplot(2,2,4); hold on; grid on;
plot(out_sine_pid.t, rad2deg(out_sine_pid.ref(:,1)), 'k--', 'LineWidth', 1.2);
plot(out_sine_pid.t, rad2deg(out_sine_pid.eul(:,1)), 'b-',  'LineWidth', 1.4);
plot(out_sine_pid.t, rad2deg(out_sine_pid.eul(:,2)), 'r-',  'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('Angle (deg)'); title('Sinusoidal Reference Tracking (0.4 Hz, Noisy)');
legend('Roll Ref','Roll','Pitch','Location','best');
saveas(fig, fullfile(res_dir, 'fig3_robustness_and_actuators.png')); close(fig);

%% 6. Hardware Flight Log Validation (Fig 4)
hw_csv = fullfile(fileparts(mfilename('fullpath')), 'final_flight_log.csv');
if ~exist(hw_csv, 'file')
    hw_csv = fullfile(fileparts(mfilename('fullpath')), '..', 'logs', 'final_flight_log.csv');
end
if exist(hw_csv, 'file')
    fprintf('\nPlotting experimental hardware flight log: %s\n', hw_csv);
    raw_lines = readlines(hw_csv);
    t_mask = startsWith(raw_lines, "T,");
    t_lines = raw_lines(t_mask);
    data = zeros(numel(t_lines), 16);
    n_valid = 0;
    for k = 1:numel(t_lines)
        parts = str2double(split(extractAfter(t_lines(k), 2), ","));
        if numel(parts) == 16 && all(~isnan(parts))
            n_valid = n_valid + 1;
            data(n_valid, :) = parts.';
        end
    end
    data = data(1:n_valid, :);
    % Find longest contiguous armed block
    best_seg = []; cur_start = 0;
    for k = 1:size(data, 1)
        if data(k, 2) == 1
            if cur_start == 0
                cur_start = k;
            elseif data(k, 1) < data(k-1, 1)
                if (k - cur_start) > size(best_seg, 1), best_seg = data(cur_start:k-1, :); end
                cur_start = k;
            end
        elseif cur_start > 0
            if (k - cur_start) > size(best_seg, 1), best_seg = data(cur_start:k-1, :); end
            cur_start = 0;
        end
    end
    if cur_start > 0 && (size(data, 1) - cur_start + 1) > size(best_seg, 1)
        best_seg = data(cur_start:end, :);
    end
    if size(best_seg, 1) > 750, best_seg = best_seg(251:end, :); end
    if ~isempty(best_seg)
        thw = (best_seg(:,1) - best_seg(1,1)) / 1000.0;
        fig4 = figure('Visible','off','Position',[60 60 1200 750]);
        subplot(2,2,1); hold on; grid on;
        plot(thw, best_seg(:,9),  'b--', 'LineWidth', 1.2);
        plot(thw, best_seg(:,3),  'b-',  'LineWidth', 1.5);
        plot(thw, best_seg(:,10), 'r--', 'LineWidth', 1.2);
        plot(thw, best_seg(:,4),  'r-',  'LineWidth', 1.5);
        xlabel('Time (s)'); ylabel('Angle (deg)');
        title('A. Roll & Pitch Setpoint Tracking (+-20 deg Steps & 0 deg Hold)');
        legend('Roll SP','Measured Roll','Pitch SP','Measured Pitch','Location','best');

        subplot(2,2,2); hold on; grid on;
        plot(thw, best_seg(:,11), 'g--', 'LineWidth', 1.2);
        plot(thw, best_seg(:,5),  'g-',  'LineWidth', 1.5);
        plot(thw, best_seg(:,6),  'm:',  'LineWidth', 1.0);
        plot(thw, best_seg(:,8),  'k:',  'LineWidth', 1.0);
        xlabel('Time (s)'); ylabel('Yaw (deg) / Rate (deg/s)');
        title('B. Yaw Heading Tracking (10 deg Steps) & Gyro Rates');
        legend('Yaw SP','Measured Yaw','Roll Rate gx','Yaw Rate gz','Location','best');

        subplot(2,2,3); hold on; grid on;
        plot(thw, best_seg(:,13:16), 'LineWidth', 1.2);
        xlabel('Time (s)'); ylabel('Motor PWM (us)');
        title('C. Hardware Closed-Loop Motor PWM Outputs (M0..M3)');
        legend('M0 (FR)','M1 (FL)','M2 (RL)','M3 (RR)','Location','best');

        subplot(2,2,4); hold on; grid on;
        ur_s = min(max(2.60*(min(max(2.80*(best_seg(:,9)-best_seg(:,3)),-150),150)-best_seg(:,6)),-250),250);
        up_s = min(max(2.60*(min(max(2.80*(best_seg(:,10)-best_seg(:,4)),-150),150)-best_seg(:,7)),-250),250);
        m0_mod = min(max(best_seg(:,12) - ur_s + up_s + 35, 1050), 1750);
        plot(thw, best_seg(:,13), 'r-',  'LineWidth', 1.3);
        plot(thw, m0_mod,         'b--', 'LineWidth', 1.2);
        xlabel('Time (s)'); ylabel('PWM Command (us)');
        title('D. Hardware vs. Analytical Cascaded PID Model (M0 Channel)');
        legend('Measured Hardware M0','Analytical PID Model','Location','best');
        saveas(fig4, fullfile(res_dir, 'fig4_hardware_validation.png')); close(fig4);
    end
end

fprintf('\nAll figures and metrics_summary.csv saved in: %s\n', res_dir);
