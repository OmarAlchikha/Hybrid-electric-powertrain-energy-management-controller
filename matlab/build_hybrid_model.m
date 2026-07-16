function build_hybrid_model()
%BUILD_HYBRID_MODEL Programmatically construct the series-hybrid model.
%
%   Builds 'hybrid_powertrain_ems.slx': a Simulink/Simscape model of a
%   series-hybrid powertrain (turbogenerator + battery + electric drive
%   on a common DC bus) with a rule-based supervisory controller.
%
%   The model is version-controlled as THIS SCRIPT rather than as a
%   binary .slx so that every design decision is reviewable in a diff
%   (see README, "Why a build script instead of a committed .slx").
%
%   Requirements: MATLAB R2023b+ with Simulink, Simscape, Simscape
%   Electrical, Stateflow (for the MATLAB Function block API).
%
%   NOTE: authored and logic-validated against the Python co-simulation
%   in validation/ (bit-for-bit same controller rules); Simscape library
%   block *port indices* (LConn/RConn numbering on sensors/sources) vary
%   slightly across releases - if an add_line call errors on your
%   release, reconnect that one line in the canvas; the topology is
%   documented in README ("Architecture").
%
% Physical topology (Foundation-library Thevenin battery - deliberately
% the same OCV(SOC)+R0 structure Project 3's EKF assumes, see README):
%
%   OCV(SOC) controlled V-source --- R0 ---+--- DC BUS (+) ---+
%                                          |                  |
%                                   bus capacitor      genset I-source
%                                          |            drive  I-source
%   (-) ------------------------------- reference -------------+

P = hybrid_params();
mdl = 'hybrid_powertrain_ems';
this_dir = fileparts(mfilename('fullpath'));

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

%% ---- model configuration -------------------------------------------
set_param(mdl, ...
    'SolverType', 'Variable-step', ...
    'Solver',     'ode23t', ...            % stiff: Simscape network
    'MaxStep',    num2str(P.ctrl.Ts), ...  % never skip a supervisor step
    'StopTime',   num2str(P.sim.t_end), ...
    'PreLoadFcn', 'P = hybrid_params(); mission_ts = mission_profile(P);');
% Run the PreLoadFcn content now so the model can be simulated
% immediately after building:
evalin('base', 'P = hybrid_params(); mission_ts = mission_profile(P);');

add = @(lib, name, pos) add_block(lib, [mdl '/' name], 'Position', pos);

%% ---- control layer (Simulink) ---------------------------------------
add('simulink/Sources/From Workspace', 'Mission', [30 100 110 130]);
set_param([mdl '/Mission'], 'VariableName', 'mission_ts');

add('simulink/User-Defined Functions/MATLAB Function', ...
    'DriveInterface', [160 90 260 140]);
add('simulink/User-Defined Functions/MATLAB Function', ...
    'Supervisor', [430 80 560 220]);

add('simulink/Discrete/Zero-Order Hold', 'ZOH_Pdem', [310 95 350 125]);
add('simulink/Discrete/Zero-Order Hold', 'ZOH_SOC',  [310 165 350 195]);
add('simulink/Discrete/Zero-Order Hold', 'ZOH_Pgen', [310 235 350 265]);
set_param([mdl '/ZOH_Pdem'], 'SampleTime', num2str(P.ctrl.Ts));
set_param([mdl '/ZOH_SOC'],  'SampleTime', num2str(P.ctrl.Ts));
set_param([mdl '/ZOH_Pgen'], 'SampleTime', num2str(P.ctrl.Ts));

% Genset plant lag (spool response); slew limiting is done INSIDE the
% supervisor so the controller, not the plant, owns the constraint.
add('simulink/Continuous/Transfer Fcn', 'GenLag', [620 90 700 130]);
set_param([mdl '/GenLag'], 'Numerator', '[1]', ...
    'Denominator', sprintf('[%g 1]', P.gen.tau));

% Net electrical load after feasibility accounting:
%   P_load = P_dem_elec - P_unmet + P_dump
add('simulink/Math Operations/Sum', 'LoadNet', [620 250 650 290]);
set_param([mdl '/LoadNet'], 'Inputs', '+-+');

% Power -> current commands for the physical sources (200 V floor keeps
% the division sane during the first solver step before the bus charges)
add('simulink/Signal Routing/Mux', 'MuxGen',  [720 90 725 130]);
add('simulink/Signal Routing/Mux', 'MuxLoad', [720 250 725 290]);
add('simulink/User-Defined Functions/Fcn', 'GenPower2I',  [750 95 850 125]);
add('simulink/User-Defined Functions/Fcn', 'LoadPower2I', [750 255 850 285]);
set_param([mdl '/GenPower2I'],  'Expression', 'u(1)/max(u(2),200)');
set_param([mdl '/LoadPower2I'], 'Expression', '-u(1)/max(u(2),200)');

%% ---- battery state (coulomb counting - Project 3 signal chain) -------
% I_batt is reconstructed as (V_ocv - V_bus)/R0 rather than sensed, so
% the only physical sensor in the network is the bus voltage sensor.
add('simulink/Math Operations/Sum', 'VocvMinusVbus', [160 330 190 370]);
set_param([mdl '/VocvMinusVbus'], 'Inputs', '+-');
add('simulink/Math Operations/Gain', 'IbattCalc', [220 335 270 365]);
set_param([mdl '/IbattCalc'], 'Gain', sprintf('1/%g', P.pack.R0));

add('simulink/Math Operations/Gain', 'CoulombGain', [300 335 360 365]);
set_param([mdl '/CoulombGain'], 'Gain', ...
    sprintf('-1/(3600*%g)', P.pack.Q_Ah));   % Ah -> SOC/s, discharge = -
add('simulink/Continuous/Integrator', 'SOCInt', [390 335 430 365]);
set_param([mdl '/SOCInt'], 'InitialCondition', num2str(P.pack.soc0), ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '1', ...
    'LowerSaturationLimit', '0');

add('simulink/Lookup Tables/1-D Lookup Table', 'OCVTable', [470 330 550 370]);
set_param([mdl '/OCVTable'], ...
    'BreakpointsForDimension1', mat2str(P.cell.soc_brk), ...
    'Table', mat2str(P.pack.Ns * P.cell.ocv_V));

%% ---- physical network (Simscape) --------------------------------------
add('nesl_utility/Solver Configuration', 'SolverCfg', [60 480 100 520]);
add('nesl_utility/Simulink-PS Converter', 'SPS_OCV',  [590 335 630 365]);
add('nesl_utility/Simulink-PS Converter', 'SPS_GenI', [880 95 920 125]);
add('nesl_utility/Simulink-PS Converter', 'SPS_LoadI',[880 255 920 285]);
add('nesl_utility/PS-Simulink Converter', 'PSS_Vbus', [590 430 630 460]);

add('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', ...
    'OCVSource', [680 330 720 370]);
add('fl_lib/Electrical/Electrical Elements/Resistor', 'R0', [750 330 790 370]);
set_param([mdl '/R0'], 'R', num2str(P.pack.R0));
add('fl_lib/Electrical/Electrical Elements/Capacitor', 'BusCap', ...
    [850 380 890 420]);
set_param([mdl '/BusCap'], 'c', num2str(P.bus.C));
add('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ...
    'GensetSrc', [950 90 990 130]);
add('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ...
    'DriveSrc', [950 250 990 290]);
add('fl_lib/Electrical/Electrical Sensors/Voltage Sensor', 'VSense', ...
    [680 430 720 470]);
add('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
    'Ref', [780 500 820 540]);

%% ---- logging -----------------------------------------------------------
logsigs = {'log_soc','log_Pgen','log_Pdem','log_Vbus','log_Ibatt', ...
           'log_mode','log_unmet','log_dump'};
for k = 1:numel(logsigs)
    add('simulink/Sinks/To Workspace', logsigs{k}, ...
        [1050 60+60*k 1130 90+60*k]);
    set_param([mdl '/' logsigs{k}], 'VariableName', logsigs{k}(5:end), ...
        'SaveFormat', 'Timeseries');
end

%% ---- MATLAB Function block sources -------------------------------------
% The block scripts ARE the repo .m files (single source of truth).
rt = sfroot;
sup = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/Supervisor']);
sup.Script = fileread(fullfile(this_dir, 'power_split_controller.m'));
drv = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/DriveInterface']);
drv.Script = fileread(fullfile(this_dir, 'drive_interface.m'));

%% ---- signal connections ------------------------------------------------
c = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
c('Mission/1',        'DriveInterface/1');
c('DriveInterface/1', 'ZOH_Pdem/1');
c('ZOH_Pdem/1',       'Supervisor/1');
c('SOCInt/1',         'ZOH_SOC/1');
c('ZOH_SOC/1',        'Supervisor/2');
c('Supervisor/1',     'GenLag/1');          % P_gen_cmd
c('GenLag/1',         'ZOH_Pgen/1');        % measured-power feedback
c('ZOH_Pgen/1',       'Supervisor/3');      % (GenLag has no direct
                                            % feedthrough -> no alg. loop)
c('ZOH_Pdem/1',       'LoadNet/1');
c('Supervisor/5',     'LoadNet/2');         % P_unmet
c('Supervisor/6',     'LoadNet/3');         % P_dump
c('GenLag/1',         'MuxGen/1');
c('PSS_Vbus/1',       'MuxGen/2');
c('MuxGen/1',         'GenPower2I/1');
c('GenPower2I/1',     'SPS_GenI/1');
c('LoadNet/1',        'MuxLoad/1');
c('PSS_Vbus/1',       'MuxLoad/2');
c('MuxLoad/1',        'LoadPower2I/1');
c('LoadPower2I/1',    'SPS_LoadI/1');
c('OCVTable/1',       'VocvMinusVbus/1');
c('PSS_Vbus/1',       'VocvMinusVbus/2');
c('VocvMinusVbus/1',  'IbattCalc/1');
c('IbattCalc/1',      'CoulombGain/1');
c('CoulombGain/1',    'SOCInt/1');
c('SOCInt/1',         'OCVTable/1');
c('OCVTable/1',       'SPS_OCV/1');
% logging
c('SOCInt/1',      'log_soc/1');
c('GenLag/1',      'log_Pgen/1');
c('ZOH_Pdem/1',    'log_Pdem/1');
c('PSS_Vbus/1',    'log_Vbus/1');
c('IbattCalc/1',   'log_Ibatt/1');
c('Supervisor/4',  'log_mode/1');
c('Supervisor/5',  'log_unmet/1');
c('Supervisor/6',  'log_dump/1');

%% ---- physical connections ----------------------------------------------
% Port index conventions here match R2023b Foundation blocks; see the
% NOTE in the header if a line errors on another release.
pc = @(a, b) add_line(mdl, a, b);
pc('SPS_OCV/RConn1',   'OCVSource/RConn1');   % control input
pc('OCVSource/LConn1', 'R0/LConn1');          % OCV+  -> R0
pc('R0/RConn1',        'GensetSrc/LConn1');   % R0 -> DC bus (+) rail
pc('R0/RConn1',        'DriveSrc/LConn1');
pc('R0/RConn1',        'BusCap/LConn1');
pc('R0/RConn1',        'VSense/LConn1');
pc('SPS_GenI/RConn1',  'GensetSrc/RConn1');   % control inputs
pc('SPS_LoadI/RConn1', 'DriveSrc/RConn1');
pc('OCVSource/RConn2', 'Ref/LConn1');         % (-) rail to reference
pc('GensetSrc/RConn2', 'Ref/LConn1');
pc('DriveSrc/RConn2',  'Ref/LConn1');
pc('BusCap/RConn1',    'Ref/LConn1');
pc('VSense/RConn2',    'Ref/LConn1');
pc('VSense/RConn1',    'PSS_Vbus/LConn1');    % measurement out
pc('SolverCfg/RConn1', 'Ref/LConn1');

%% ---- save ---------------------------------------------------------------
save_system(mdl, fullfile(this_dir, [mdl '.slx']));
fprintf('Built and saved %s.slx\n', mdl);
end
