function build_simulink_model()
%BUILD_SIMULINK_MODEL  Programmatically generates the full closed-loop Simulink
%                      block diagram (simulink/quad_sim.slx) for EE302 PS11.
%
%   Usage in MATLAB / MATLAB Online:
%     >> cd matlab; build_simulink_model();
%
%   Architecture built on the Simulink canvas:
%     1. Reference Generator (Roll 10 deg @ 1s, Pitch -8 deg @ 3s, Yaw 30 deg @ 5s, Alt 0.5 m @ 1s)
%     2. Outer Attitude P Loop (Angle Error -> Rate Setpoint with Saturation)
%     3. Inner Angular Rate PID Loop (Rate Error -> Body Torque Commands [tau_phi, tau_theta, tau_psi])
%     4. Altitude PID + Hover Feedforward Loop (Alt Error -> Total Thrust U1)
%     5. X-Configuration Mixer & Actuator Saturation ([U1; tau] -> Motor Thrusts T1..T4 in [0, 4.2] N)
%     6. 4x First-Order Brushless Motor + ESC Lag Transfer Functions: 1 / (0.05 s + 1)
%     7. 6-DOF Rigid-Body Quadcopter Plant Dynamics & Scopes (Attitude_Scope, Motor_Thrust_Scope)

here_dir = fileparts(mfilename('fullpath'));
if isempty(here_dir), here_dir = pwd; end
repo_dir = fileparts(here_dir);
% If repo_dir is read-only (e.g. '/MATLAB Drive' root in MATLAB Online), save inside here_dir/simulink
sim_dir = fullfile(repo_dir, 'simulink');
if strcmp(repo_dir, '/') || strcmp(repo_dir, '/MATLAB Drive')
    sim_dir = fullfile(here_dir, 'simulink');
end
if ~exist(sim_dir, 'dir')
    try
        mkdir(sim_dir);
    catch
        sim_dir = fullfile(here_dir, 'simulink');
        if ~exist(sim_dir, 'dir'), mkdir(sim_dir); end
    end
end
addpath(here_dir);

mdl = 'quad_sim';
mdl_path = fullfile(sim_dir, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'StopTime', '10', 'Solver', 'ode4', 'FixedStep', '0.001');

% Self-contained InitFcn so it NEVER collides with MATLAB Online's built-in MobileRover/params.m
init_code = [ ...
    'if exist(fullfile(pwd,''matlab''),''dir''), addpath(fullfile(pwd,''matlab'')); end; ' ...
    'if exist(fullfile(fileparts(pwd),''matlab''),''dir''), addpath(fullfile(fileparts(pwd),''matlab'')); end; ' ...
    'Ixx = 0.0086; Iyy = 0.0086; Izz = 0.0152; m_q = 0.818; g_q = 9.80665; d_arm = 0.1591; kq = 0.0176; tau_m = 0.05; ' ...
    'Kang_rp = 2.80; Kang_y = 2.60; ' ...
    'Kp_rp = 0.3987; Ki_rp = 0.1227; Kd_rp = 0.0230; ' ...
    'Kp_y  = 0.0510; Ki_y  = 0.0110; Kd_y  = 0.0; ' ...
    'Kp_z  = 18.0;   Ki_z  = 6.0;    Kd_z  = 11.0; ' ...
];
set_param(mdl, 'InitFcn', init_code);

% ==================== 1. REFERENCE INPUTS ====================
add_block('simulink/Sources/Step', [mdl '/Roll_Ref_deg'],  'Time','1', 'After','10',  'Position',[40  60  75  90]);
add_block('simulink/Sources/Step', [mdl '/Pitch_Ref_deg'], 'Time','3', 'After','-8',  'Position',[40 160  75 190]);
add_block('simulink/Sources/Step', [mdl '/Yaw_Ref_deg'],   'Time','5', 'After','30',  'Position',[40 260  75 290]);
add_block('simulink/Sources/Step', [mdl '/Alt_Ref_m'],     'Time','1', 'After','0.5', 'Position',[40 360  75 390]);

add_block('simulink/Math Operations/Gain', [mdl '/deg2rad_R'], 'Gain','pi/180', 'Position',[115  60 155  90]);
add_block('simulink/Math Operations/Gain', [mdl '/deg2rad_P'], 'Gain','pi/180', 'Position',[115 160 155 190]);
add_block('simulink/Math Operations/Gain', [mdl '/deg2rad_Y'], 'Gain','pi/180', 'Position',[115 260 155 290]);

add_line(mdl, 'Roll_Ref_deg/1',  'deg2rad_R/1');
add_line(mdl, 'Pitch_Ref_deg/1', 'deg2rad_P/1');
add_line(mdl, 'Yaw_Ref_deg/1',   'deg2rad_Y/1');

% ==================== 2. OUTER ANGLE P LOOP ====================
add_block('simulink/Math Operations/Sum', [mdl '/Err_Roll'],  'Inputs','+-', 'Position',[200  65 225  85]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Pitch'], 'Inputs','+-', 'Position',[200 165 225 185]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Yaw'],   'Inputs','+-', 'Position',[200 265 225 285]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Alt'],   'Inputs','+-', 'Position',[200 365 225 385]);

add_block('simulink/Math Operations/Gain', [mdl '/Outer_P_Roll'],  'Gain','2.80', 'Position',[265  60 315  90]);
add_block('simulink/Math Operations/Gain', [mdl '/Outer_P_Pitch'], 'Gain','2.80', 'Position',[265 160 315 190]);
add_block('simulink/Math Operations/Gain', [mdl '/Outer_P_Yaw'],   'Gain','2.60', 'Position',[265 260 315 290]);

add_line(mdl, 'deg2rad_R/1', 'Err_Roll/1');
add_line(mdl, 'deg2rad_P/1', 'Err_Pitch/1');
add_line(mdl, 'deg2rad_Y/1', 'Err_Yaw/1');
add_line(mdl, 'Alt_Ref_m/1', 'Err_Alt/1');

add_line(mdl, 'Err_Roll/1',  'Outer_P_Roll/1');
add_line(mdl, 'Err_Pitch/1', 'Outer_P_Pitch/1');
add_line(mdl, 'Err_Yaw/1',   'Outer_P_Yaw/1');

% ==================== 3. INNER RATE PID & ALTITUDE PID LOOPS ====================
add_block('simulink/Math Operations/Sum', [mdl '/Err_p'], 'Inputs','+-', 'Position',[355  65 380  85]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_q'], 'Inputs','+-', 'Position',[355 165 380 185]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_r'], 'Inputs','+-', 'Position',[355 265 380 285]);

add_line(mdl, 'Outer_P_Roll/1',  'Err_p/1');
add_line(mdl, 'Outer_P_Pitch/1', 'Err_q/1');
add_line(mdl, 'Outer_P_Yaw/1',   'Err_r/1');

add_block('simulink/Continuous/PID Controller', [mdl '/Rate_PID_Roll'], ...
    'P','0.3987', 'I','0.1227', 'D','0.0230', 'N','50', 'Position',[420  55 470  95]);
add_block('simulink/Continuous/PID Controller', [mdl '/Rate_PID_Pitch'], ...
    'P','0.3987', 'I','0.1227', 'D','0.0230', 'N','50', 'Position',[420 155 470 195]);
add_block('simulink/Continuous/PID Controller', [mdl '/Rate_PID_Yaw'], ...
    'P','0.0510', 'I','0.0110', 'D','0',      'N','50', 'Position',[420 255 470 295]);
add_block('simulink/Continuous/PID Controller', [mdl '/Alt_PID'], ...
    'P','18.0',   'I','6.0',    'D','11.0',   'N','20', 'Position',[420 355 470 395]);

add_line(mdl, 'Err_p/1',   'Rate_PID_Roll/1');
add_line(mdl, 'Err_q/1',   'Rate_PID_Pitch/1');
add_line(mdl, 'Err_r/1',   'Rate_PID_Yaw/1');
add_line(mdl, 'Err_Alt/1', 'Alt_PID/1');

% Hover thrust feedforward (m*g = 8.0218 N)
add_block('simulink/Sources/Constant', [mdl '/Hover_Thrust_mg'], 'Value','0.818*9.80665', 'Position',[420 425 470 455]);
add_block('simulink/Math Operations/Sum', [mdl '/Total_Thrust_U1'], 'Inputs','++', 'Position',[510 365 535 395]);
add_line(mdl, 'Alt_PID/1',         'Total_Thrust_U1/1');
add_line(mdl, 'Hover_Thrust_mg/1', 'Total_Thrust_U1/2');

% ==================== 4. MOTOR MIXER & ACTUATOR DYNAMICS (1 / (0.05s + 1)) ====================
add_block('simulink/Signal Routing/Mux', [mdl '/Ctrl_Mux'], 'Inputs','4', 'Position',[570  50 575 410]);
add_line(mdl, 'Rate_PID_Roll/1',   'Ctrl_Mux/1');
add_line(mdl, 'Rate_PID_Pitch/1',  'Ctrl_Mux/2');
add_line(mdl, 'Rate_PID_Yaw/1',    'Ctrl_Mux/3');
add_line(mdl, 'Total_Thrust_U1/1', 'Ctrl_Mux/4');

% X-Frame Allocation Matrix: [tau_phi; tau_theta; tau_psi; U1] -> [T1; T2; T3; T4]
M_alloc = [ -1/(4*0.1591),  1/(4*0.1591),  1/(4*0.0176), 0.25; ...
             1/(4*0.1591),  1/(4*0.1591), -1/(4*0.0176), 0.25; ...
             1/(4*0.1591), -1/(4*0.1591),  1/(4*0.0176), 0.25; ...
            -1/(4*0.1591), -1/(4*0.1591), -1/(4*0.0176), 0.25 ];
add_block('simulink/Math Operations/Gain', [mdl '/X_Frame_Mixer'], ...
    'Gain', mat2str(M_alloc, 5), 'Multiplication','Matrix(K*u)', 'Position',[610 205 670 255]);
add_block('simulink/Discontinuities/Saturation', [mdl '/ESC_Thrust_Sat'], ...
    'LowerLimit','0.15', 'UpperLimit','4.20', 'Position',[700 215 735 245]);

% First-order brushless motor lag (tau_m = 0.05 s) and Re-allocation to body wrench
M_fwd = [ -0.1591,  0.1591,  0.1591, -0.1591; ...
           0.1591,  0.1591, -0.1591, -0.1591; ...
           0.0176, -0.0176,  0.0176, -0.0176; ...
           1.0,     1.0,     1.0,     1.0 ];
add_block('simulink/Math Operations/Gain', [mdl '/Rotor_to_Wrench'], ...
    'Gain', mat2str(M_fwd, 5), 'Multiplication','Matrix(K*u)', 'Position',[770 205 830 255]);
add_block('simulink/Signal Routing/Demux', [mdl '/Wrench_Demux'], 'Outputs','4', 'Position',[865  50 870 410]);

add_line(mdl, 'Ctrl_Mux/1',        'X_Frame_Mixer/1');
add_line(mdl, 'X_Frame_Mixer/1',   'ESC_Thrust_Sat/1');
add_line(mdl, 'ESC_Thrust_Sat/1',  'Rotor_to_Wrench/1');
add_line(mdl, 'Rotor_to_Wrench/1', 'Wrench_Demux/1');

% ==================== 5. 6-DOF PLANT DYNAMICS (MOTOR LAG + RIGID BODY) ====================
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant_Rate_Roll'], ...
    'Numerator','[1]', 'Denominator','[0.0086*0.05  0.0086]', 'Position',[910  55 985  95]);
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant_Rate_Pitch'], ...
    'Numerator','[1]', 'Denominator','[0.0086*0.05  0.0086]', 'Position',[910 155 985 195]);
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant_Rate_Yaw'], ...
    'Numerator','[1]', 'Denominator','[0.0152*0.05  0.0152]', 'Position',[910 255 985 295]);

% Subtract gravity mg before altitude plant
add_block('simulink/Math Operations/Sum', [mdl '/Net_Fz'], 'Inputs','+-', 'Position',[910 365 935 385]);
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant_Alt_z'], ...
    'Numerator','[1]', 'Denominator','[0.818*0.05  0.818+0.08*0.05  0.08  0]', 'Position',[960 355 1045 395]);

add_line(mdl, 'Wrench_Demux/1',    'Plant_Rate_Roll/1');
add_line(mdl, 'Wrench_Demux/2',    'Plant_Rate_Pitch/1');
add_line(mdl, 'Wrench_Demux/3',    'Plant_Rate_Yaw/1');
add_line(mdl, 'Wrench_Demux/4',    'Net_Fz/1');
add_line(mdl, 'Hover_Thrust_mg/1', 'Net_Fz/2');
add_line(mdl, 'Net_Fz/1',          'Plant_Alt_z/1');

% Euler angle integrators (p -> phi, q -> theta, r -> psi)
add_block('simulink/Continuous/Integrator', [mdl '/Int_Roll_phi'],    'Position',[1025  60 1055  90]);
add_block('simulink/Continuous/Integrator', [mdl '/Int_Pitch_theta'], 'Position',[1025 160 1055 190]);
add_block('simulink/Continuous/Integrator', [mdl '/Int_Yaw_psi'],     'Position',[1025 260 1055 290]);

add_line(mdl, 'Plant_Rate_Roll/1',  'Int_Roll_phi/1');
add_line(mdl, 'Plant_Rate_Pitch/1', 'Int_Pitch_theta/1');
add_line(mdl, 'Plant_Rate_Yaw/1',   'Int_Yaw_psi/1');

% Feedback loops for Inner Rate (p, q, r) and Outer Angle (phi, theta, psi, z)
add_line(mdl, 'Plant_Rate_Roll/1',  'Err_p/2',     'autorouting','on');
add_line(mdl, 'Plant_Rate_Pitch/1', 'Err_q/2',     'autorouting','on');
add_line(mdl, 'Plant_Rate_Yaw/1',   'Err_r/2',     'autorouting','on');

add_line(mdl, 'Int_Roll_phi/1',     'Err_Roll/2',  'autorouting','on');
add_line(mdl, 'Int_Pitch_theta/1',  'Err_Pitch/2', 'autorouting','on');
add_line(mdl, 'Int_Yaw_psi/1',      'Err_Yaw/2',   'autorouting','on');
add_line(mdl, 'Plant_Alt_z/1',      'Err_Alt/2',   'autorouting','on');

% ==================== 6. OUTPUT SCOPES & WORKSPACE LOGGING ====================
add_block('simulink/Math Operations/Gain', [mdl '/rad2deg_R'], 'Gain','180/pi', 'Position',[1105  60 1145  90]);
add_block('simulink/Math Operations/Gain', [mdl '/rad2deg_P'], 'Gain','180/pi', 'Position',[1105 160 1145 190]);
add_block('simulink/Math Operations/Gain', [mdl '/rad2deg_Y'], 'Gain','180/pi', 'Position',[1105 260 1145 290]);

add_line(mdl, 'Int_Roll_phi/1',    'rad2deg_R/1');
add_line(mdl, 'Int_Pitch_theta/1', 'rad2deg_P/1');
add_line(mdl, 'Int_Yaw_psi/1',     'rad2deg_Y/1');

add_block('simulink/Signal Routing/Mux', [mdl '/Out_Mux'], 'Inputs','4', 'Position',[1185  50 1190 400]);
add_line(mdl, 'rad2deg_R/1',   'Out_Mux/1');
add_line(mdl, 'rad2deg_P/1',   'Out_Mux/2');
add_line(mdl, 'rad2deg_Y/1',   'Out_Mux/3');
add_line(mdl, 'Plant_Alt_z/1', 'Out_Mux/4');

add_block('simulink/Sinks/Scope',        [mdl '/Attitude_Alt_Scope'], 'Position',[1235 180 1275 230]);
add_block('simulink/Sinks/Scope',        [mdl '/Motor_Thrusts_Scope'],'Position',[775  120 815  170]);
add_block('simulink/Sinks/To Workspace', [mdl '/Sim_States_Out'], 'VariableName','sim_states', 'Position',[1235 270 1315 300]);

add_line(mdl, 'Out_Mux/1',        'Attitude_Alt_Scope/1');
add_line(mdl, 'Out_Mux/1',        'Sim_States_Out/1');
add_line(mdl, 'ESC_Thrust_Sat/1', 'Motor_Thrusts_Scope/1', 'autorouting','on');

save_system(mdl, mdl_path);
fprintf('Saved full closed-loop Simulink block diagram to: %s\n', mdl_path);
end
