function ctrl = design_gains(p, type, opts)
%DESIGN_GAINS  Controller design from the physical model.
%   ctrl = design_gains(p)               cascaded PID (default)
%   ctrl = design_gains(p,'LQR')         LQR with integral action (comparison controller)
%   opts.wr   rate-loop bandwidth [rad/s] (default 15)   opts.wy yaw rate bandwidth (default 8)
%
% Design idea (cascaded PID):
%   Inner rate loop. Plant: tau -> body rate  =  1/(J s)  (ignoring the motor lag).
%     Kp = J*wr,   Ki = Kp*wr/5 (integral time Ti = 5/wr),   Kd = Kp*Td (Td = 0.02 s)
%   This gives a rate-loop crossover near wr. The motor lag (tau_m) and sample delay must
%   keep the phase margin >= 45 deg, which is verified in design_analysis.m.
%   Outer angle loop: P gain  Kang = wr/4  (crossover a factor 4 below the rate loop).
%
%   The result also contains the equivalent gains in firmware units (microseconds of
%   motor command per deg/s) so the same design can be copied into firmware/quad_fc/config.h.

if nargin < 2 || isempty(type), type = 'PID'; end
if nargin < 3, opts = struct(); end
if ~isfield(opts,'wr'), opts.wr = 15; end
if ~isfield(opts,'wy'), opts.wy = 8;  end
if ~isfield(opts,'Td'), opts.Td = 0.02; end

ctrl.type = upper(type);
J = [p.Ixx p.Iyy p.Izz];           % roll, pitch, yaw
w = [opts.wr opts.wr opts.wy];

% default limits
ctrl.rate_max   = deg2rad(150);    % [rad/s]
ctrl.yawrate_max= deg2rad(90);
ctrl.tau_max    = [p.tau_max p.tau_max p.kq*2*p.Tmax];   % [N m] per axis
ctrl.Ilim       = 0.5*ctrl.tau_max;                       % anti-windup clamp
ctrl.d_alpha    = 0.2;                                    % derivative low-pass

% Altitude loop (PID on altitude with hover feed-forward; needs an altitude sensor)
ctrl.alt.kp = 12.0; ctrl.alt.ki = 6.0;  ctrl.alt.kd = 11.0;    % [N/m], [N/(m s)], [N s/m]
ctrl.alt.ilim = 0.60*p.hover_T;

switch ctrl.type
    case 'PID'
        ctrl.Kang = w/4;                  % [1/s]
        ctrl.Kp   = J.*w;                 % [N m /(rad/s)]
        ctrl.Ki   = ctrl.Kp.*w/5;
        ctrl.Kd   = ctrl.Kp*opts.Td;
    case 'LQR'
        % Per axis: x = [integral of angle error; angle error; rate], u = torque
        ctrl.Klqr = zeros(3,3);
        Q = diag([60 400 1]);   Rw = 1;      % weights (tuned for ~1 s settling)
        for k = 1:3
            A = [0 1 0; 0 0 1; 0 0 0];
            B = [0; 0; 1/J(k)];
            % R is scaled by 1/J^2 so the weights act on angular acceleration (u/J)
            ctrl.Klqr(k,:) = lqr(A, B, Q, Rw/J(k)^2);
        end
    otherwise
        error('Unknown controller type');
end

% ---- equivalent gains for the firmware (per axis roll, pitch, yaw) -------------------
% roll/pitch: tau = 4*d*dT  with  dT = Tmax*u_us/1000  ->  u_us = tau*1000/(4 d Tmax)
% yaw:        tau = 4*kq*dT                           ->  u_us = tau*1000/(4 kq Tmax)
if strcmp(ctrl.type,'PID')
    k_us = [1000/(4*p.d*p.Tmax), 1000/(4*p.d*p.Tmax), 1000/(4*p.kq*p.Tmax)];
    ctrl.fw.ang_kp    = ctrl.Kang(1);                       % deg/s per deg (same numerically)
    ctrl.fw.yaw_ang_kp= ctrl.Kang(3);
    ctrl.fw.rate_kp   = ctrl.Kp(1)*pi/180*k_us(1);          % us per (deg/s)
    ctrl.fw.rate_ki   = ctrl.Ki(1)*pi/180*k_us(1);
    ctrl.fw.rate_kd   = ctrl.Kd(1)*pi/180*k_us(1);
    ctrl.fw.yawr_kp   = ctrl.Kp(3)*pi/180*k_us(3);
    ctrl.fw.yawr_ki   = ctrl.Ki(3)*pi/180*k_us(3);
    ctrl.fw.yawr_kd   = ctrl.Kd(3)*pi/180*k_us(3);
end
end
