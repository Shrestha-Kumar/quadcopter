function p = params()
%PARAMS  Physical parameters of the quadcopter (F450 frame, A2212/13T 1000KV, 10x4.5 props).
%   ALL VALUES ARE ESTIMATES (typical F450 numbers). They MUST be replaced by measured
%   values from the hardware (mass on a scale, arm length with a ruler, inertia by
%   bifilar pendulum, thrust curve on a scale/thrust stand). See docs in PROJECT_STATUS.md.
%
%   Frame convention used everywhere in the simulation (aerospace NED body frame):
%     x forward, y right, z down. Euler angles ZYX: phi=roll (+ right side down),
%     theta=pitch (+ nose up), psi=yaw (+ clockwise seen from above).
%   Motor numbering (seen from above, nose at top):
%     1 = front-right (CCW prop), 2 = front-left (CW), 3 = rear-left (CCW), 4 = rear-right (CW)
%   (the firmware uses the same motors, numbered 0..3).

p.g      = 9.81;        % [m/s^2]
p.m      = 1.10;        % [kg]   total mass incl. battery/supply cables  (MEASURE)
p.L      = 0.225;       % [m]    motor-to-centre distance of an F450 (diagonal 450 mm)  (MEASURE)
p.d      = p.L/sqrt(2); % [m]    lever arm of each motor about the roll/pitch axes (X frame)
p.Ixx    = 0.0110;      % [kg m^2] (MEASURE: bifilar pendulum)
p.Iyy    = 0.0110;
p.Izz    = 0.0210;
p.J      = diag([p.Ixx p.Iyy p.Izz]);

% Motor/propeller (linear thrust map vs normalised command u = (us-1000)/1000)
p.Tmax   = 8.0;         % [N]  max thrust of one motor at full command (MEASURE; ~800 gf)
p.Tidle  = 0.05*p.Tmax; % [N]  thrust at the firmware idle command
p.kq     = 0.016;       % [m]  yaw drag torque / thrust ratio (kQ/kT) of the propeller
p.tau_m  = 0.05;        % [s]  motor + ESC first-order time constant (MEASURE from step response)
p.cd     = 0.10;        % [N s/m] linear translational drag coefficient

% Sensors / control
p.Ts     = 1/200;       % [s]  controller sample time (matches firmware LOOP_US)
p.sig_ang  = deg2rad(0.15);  % [rad]   angle noise std (BNO055 fused output)
p.sig_gyro = deg2rad(0.40);  % [rad/s] gyro noise std
p.sig_alt  = 0.02;           % [m]     altitude sensor noise std (sensor not installed yet)
p.tau_max  = 4*p.d*p.Tmax/2; % [N m] maximum attainable roll/pitch torque (two motors saturated)
p.hover_T  = p.m*p.g;        % [N] total hover thrust
end
