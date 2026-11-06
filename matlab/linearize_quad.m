function lin = linearize_quad(p, ctrl)
%LINEARIZE_QUAD  Linearize the quadcopter about hover and evaluate loop stability.
%   lin = linearize_quad(p, ctrl)
%   Produces:
%     - State-space (A,B,C,D) of the 12-state rigid-body hover model
%     - Decoupled transfer functions including motor first-order lag:
%         G_rate(s)  = 1 / (J s (tau_m s + 1))           [rad/s per N m]
%         G_ang(s)   = 1 / (J s^2 (tau_m s + 1))         [rad   per N m]
%         G_alt(s)   = 1 / (m s^2 (tau_m s + 1))         [m     per N]
%     - Open-loop and closed-loop transfer functions for Roll/Pitch, Yaw, and Altitude
%     - Gain/phase margins and closed-loop poles
%     - Saves Bode / root-locus / pole-map plots into matlab/results/

if nargin < 1, p = params(); end
if nargin < 2, ctrl = design_gains(p, 'PID'); end

res_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(res_dir, 'dir'), mkdir(res_dir); end

% 12-state hover linearization: x = [pn pe pd u v w phi theta psi p q r]'
% Inputs: u = [dT_tot, tau_phi, tau_theta, tau_psi]'
A = zeros(12,12);
A(1:3, 4:6) = eye(3);
A(4, 8) = -p.g;                  % u_dot = -g * theta
A(5, 7) =  p.g;                  % v_dot =  g * phi
A(4:6, 4:6) = -(p.cd/p.m)*eye(3);
A(7:9, 10:12) = eye(3);

B = zeros(12,4);
B(6, 1)  = -1/p.m;               % pd_dot = -dT_tot / m  (altitude h = -pd)
B(10, 2) =  1/p.Ixx;
B(11, 3) =  1/p.Iyy;
B(12, 4) =  1/p.Izz;

lin.A = A; lin.B = B; lin.C = eye(12); lin.D = zeros(12,4);

s = tf('s');
Tf = p.Ts / ctrl.d_alpha;        % continuous equivalent of derivative LPF time constant
J  = [p.Ixx p.Iyy p.Izz];
names = {'Roll','Pitch','Yaw'};

for k = 1:3
    G_rate = 1 / (J(k) * s * (p.tau_m * s + 1));
    C_pi   = ctrl.Kp(k) + ctrl.Ki(k)/s;
    C_d    = ctrl.Kd(k) * s / (Tf * s + 1);

    % Inner rate loop (with derivative on measurement)
    L_rate = (C_pi + C_d) * G_rate;
    T_rate = minreal((C_pi * G_rate) / (1 + L_rate));

    % Outer angle loop: angle = (1/s) * rate
    L_ang  = ctrl.Kang(k) * T_rate / s;
    T_ang  = minreal(L_ang / (1 + L_ang));

    [Gm_r, Pm_r, Wcg_r, Wcp_r] = margin(L_rate);
    [Gm_a, Pm_a, Wcg_a, Wcp_a] = margin(L_ang);

    lin.axis(k).name   = names{k};
    lin.axis(k).G_rate = G_rate;
    lin.axis(k).L_rate = L_rate;
    lin.axis(k).T_rate = T_rate;
    lin.axis(k).L_ang  = L_ang;
    lin.axis(k).T_ang  = T_ang;
    lin.axis(k).Gm_rate_dB = 20*log10(Gm_r);
    lin.axis(k).Pm_rate_deg = Pm_r;
    lin.axis(k).Wcp_rate = Wcp_r;
    lin.axis(k).Gm_ang_dB  = 20*log10(Gm_a);
    lin.axis(k).Pm_ang_deg  = Pm_a;
    lin.axis(k).Wcp_ang  = Wcp_a;
    lin.axis(k).poles    = pole(T_ang);
end

% Altitude loop
G_alt = 1 / (p.m * s^2 * (p.tau_m * s + 1));
Tf_a  = p.Ts / 0.05;
C_alt = ctrl.alt.kp + ctrl.alt.ki/s + ctrl.alt.kd * s / (Tf_a * s + 1);
L_alt = C_alt * G_alt;
T_alt = minreal((ctrl.alt.kp + ctrl.alt.ki/s) * G_alt / (1 + L_alt));
[Gm_z, Pm_z, ~, Wcp_z] = margin(L_alt);
lin.alt.L = L_alt; lin.alt.T = T_alt;
lin.alt.Gm_dB = 20*log10(Gm_z); lin.alt.Pm_deg = Pm_z; lin.alt.Wcp = Wcp_z;
lin.alt.poles = pole(T_alt);

% Print summary table
fprintf('\n=== LINEAR STABILITY & FREQUENCY-DOMAIN ANALYSIS ===\n');
for k = [1 3]
    a = lin.axis(k);
    fprintf('%-5s Inner Rate Loop : Crossover = %5.2f rad/s, PM = %5.1f deg, GM = %5.1f dB\n', ...
            a.name, a.Wcp_rate, a.Pm_rate_deg, a.Gm_rate_dB);
    fprintf('%-5s Outer Angle Loop: Crossover = %5.2f rad/s, PM = %5.1f deg, GM = %5.1f dB | max Re(pole) = %.3f\n', ...
            a.name, a.Wcp_ang, a.Pm_ang_deg, a.Gm_ang_dB, max(real(a.poles)));
end
fprintf('Alt   Loop            : Crossover = %5.2f rad/s, PM = %5.1f deg, GM = %5.1f dB | max Re(pole) = %.3f\n', ...
        lin.alt.Wcp, lin.alt.Pm_deg, lin.alt.Gm_dB, max(real(lin.alt.poles)));

% Save Bode & Pole-Zero figure
fig = figure('Visible','off','Position',[100 100 1100 450]);
subplot(1,2,1);
margin(lin.axis(1).L_rate); hold on; margin(lin.axis(1).L_ang);
legend('Inner Rate Loop L_{rate}(s)','Outer Angle Loop L_{ang}(s)','Location','southwest');
title('Roll/Pitch Open-Loop Bode & Stability Margins'); grid on;

subplot(1,2,2);
pzmap(lin.axis(1).T_ang, lin.axis(3).T_ang, lin.alt.T);
legend('Roll/Pitch T_{ang}','Yaw T_{ang}','Altitude T_{alt}','Location','best');
title('Closed-Loop Pole-Zero Map'); grid on;
saveas(fig, fullfile(res_dir, 'fig1_linear_stability.png'));
close(fig);
end
