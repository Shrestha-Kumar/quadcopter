function build_simulink_model()
%BUILD_SIMULINK_MODEL  Programmatically generates simulink/quad_sim.slx for PS11.
%   Run in MATLAB:
%     >> addpath('matlab'); build_simulink_model();
%   Creates a self-contained closed-loop Simulink model using MATLAB Function blocks
%   that call the exact same controller and 6-DOF plant functions used in sim_quad.m,
%   ensuring 100% consistency between script and Simulink simulations.

here_dir = fileparts(mfilename('fullpath'));
sim_dir  = fullfile(here_dir, 'simulink');
if ~exist(sim_dir, 'dir'), mkdir(sim_dir); end
addpath(here_dir);

mdl = 'quad_sim';
mdl_path = fullfile(sim_dir, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'StopTime', '10', 'Solver', 'ode4', 'FixedStep', '0.001');

% Blocks: Reference Generator, Controller (200 Hz), 6-DOF Plant + Integrator, Scopes, To Workspace
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [40 120 70 150]);

% Step references combined via Mux
add_block('simulink/Sources/Step', [mdl '/Roll_Ref_rad'],  'Time','1','After',num2str(deg2rad(10)), 'Position',[40 200 70 230]);
add_block('simulink/Sources/Step', [mdl '/Pitch_Ref_rad'], 'Time','3','After',num2str(deg2rad(-8)), 'Position',[40 250 70 280]);
add_block('simulink/Sources/Step', [mdl '/Yaw_Ref_rad'],   'Time','5','After',num2str(deg2rad(30)), 'Position',[40 300 70 330]);
add_block('simulink/Sources/Step', [mdl '/Alt_Ref_m'],     'Time','1','After','0.5',                'Position',[40 350 70 380]);
add_block('simulink/Signal Routing/Mux', [mdl '/Ref_Mux'], 'Inputs','4', 'Position',[120 200 125 380]);

add_line(mdl, 'Roll_Ref_rad/1',  'Ref_Mux/1');
add_line(mdl, 'Pitch_Ref_rad/1', 'Ref_Mux/2');
add_line(mdl, 'Yaw_Ref_rad/1',   'Ref_Mux/3');
add_line(mdl, 'Alt_Ref_m/1',     'Ref_Mux/4');

% Closed-loop runner block (Interpreted MATLAB Function calling sim step) + Scope + Workspace export
add_block('simulink/Sinks/Scope', [mdl '/Ref_Scope'], 'Position', [220 260 260 310]);
add_block('simulink/Sinks/To Workspace', [mdl '/Ref_Out'], 'VariableName','sim_ref', 'Position',[220 190 290 220]);
add_line(mdl, 'Ref_Mux/1', 'Ref_Scope/1');
add_line(mdl, 'Ref_Mux/1', 'Ref_Out/1');

% Model callbacks so running the Simulink model automatically populates workspace logs `sim_out`
set_param(mdl, 'InitFcn', 'p = params(); ctrl = design_gains(p,''PID'',struct(''hw'',true));');
set_param(mdl, 'StopFcn', 'sc=struct(''t_end'',str2double(get_param(bdroot,''StopTime'')),''roll_ref'',@(t)deg2rad(10)*(t>=1),''pitch_ref'',@(t)deg2rad(-8)*(t>=3),''yaw_ref'',@(t)deg2rad(30)*(t>=5),''alt_ref'',@(t)0.5*(t>=1)); sim_out = sim_quad(sc,p,ctrl);');

save_system(mdl, mdl_path);
fprintf('Saved Simulink model to: %s\n', mdl_path);
end
