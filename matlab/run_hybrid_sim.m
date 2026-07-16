%RUN_HYBRID_SIM Build (if needed), simulate, check, and plot the mission.
%
%   Top-level entry point. From this folder:
%       >> run_hybrid_sim
%
%   Produces the same plots and pass/fail checks as the Python
%   co-simulation (validation/simulate_hybrid.py), so the two
%   implementations can be compared directly.

clear; clc;
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);

P = hybrid_params();
mdl = 'hybrid_powertrain_ems';

if ~isfile(fullfile(this_dir, [mdl '.slx']))
    build_hybrid_model();
end
load_system(mdl);

mission_ts = mission_profile(P); %#ok<NASGU> % used by From Workspace
sim(mdl, 'ReturnWorkspaceOutputs', 'off');   % logs land in base workspace

%% ---- mission feasibility checks (same asserts as the Python sim) ------
soc  = evalin('base', 'soc');   Vbus = evalin('base', 'Vbus');
unmet = evalin('base', 'unmet');

assert(all(unmet.Data < 1), ...
    'FAIL: %d supervisor steps with unmet demand', nnz(unmet.Data >= 1));
assert(min(soc.Data) > 0.20, ...
    'FAIL: SOC floor violated (min %.2f)', min(soc.Data));
assert(min(Vbus.Data(Vbus.Time > 1)) > P.bus.V_min_warn, ...
    'FAIL: bus undervoltage (min %.0f V)', min(Vbus.Data(Vbus.Time > 1)));
fprintf('PASS: unmet=0, SOC in [%.2f %.2f], Vbus min %.0f V\n', ...
    min(soc.Data), max(soc.Data), min(Vbus.Data(Vbus.Time > 1)));

plot_results();
