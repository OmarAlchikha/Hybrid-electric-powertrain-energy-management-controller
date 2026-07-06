function plot_results()
%PLOT_RESULTS Four-panel mission summary from base-workspace log signals.
%   Mirrors the panel layout of validation/simulate_hybrid.py so the
%   MATLAB and Python runs can be compared side by side.

soc   = evalin('base', 'soc');
Pgen  = evalin('base', 'Pgen');
Pdem  = evalin('base', 'Pdem');
Vbus  = evalin('base', 'Vbus');
Ibatt = evalin('base', 'Ibatt');
mode  = evalin('base', 'mode');

f = figure('Name', 'Series-hybrid mission', 'Position', [50 50 1100 800]);
tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact');

nexttile(tl);
plot(Pdem.Time/60, Pdem.Data/1e3, 'LineWidth', 1.2); hold on;
plot(Pgen.Time/60, Pgen.Data/1e3, 'LineWidth', 1.2);
Pbatt = interp1(Pdem.Time, Pdem.Data, Pgen.Time) - Pgen.Data;
plot(Pgen.Time/60, Pbatt/1e3, 'LineWidth', 1.2);
ylabel('kW'); legend('bus demand', 'genset', 'battery (+=dis)');
title('Power split'); grid on;

nexttile(tl);
plot(soc.Time/60, 100*soc.Data, 'LineWidth', 1.2);
yline(45, '--'); yline(75, '--');
ylabel('SOC %'); title('Battery SOC (dashed: hysteresis band)'); grid on;

nexttile(tl);
plot(Vbus.Time/60, Vbus.Data, 'LineWidth', 1.2); hold on;
yline(96*2.8, 'r--');
ylabel('V'); title('DC bus voltage'); grid on;

nexttile(tl);
stairs(mode.Time/60, mode.Data, 'LineWidth', 1.2);
yticks(0:3); yticklabels({'batt-only', 'assist', 'recharge', 'regen'});
xlabel('mission time, min'); title('Supervisor mode'); grid on;

exportgraphics(f, fullfile(fileparts(mfilename('fullpath')), ...
    '..', 'results', 'mission_matlab.png'), 'Resolution', 140);
end
