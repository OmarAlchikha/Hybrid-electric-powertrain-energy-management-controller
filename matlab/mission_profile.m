function mission = mission_profile(P)
%MISSION_PROFILE Shaft-power demand vs. time for a 30-minute mission.
%
%   mission = mission_profile(P) returns a timeseries of MECHANICAL power
%   demanded at the propulsor shaft [W]. Electrical demand on the DC bus
%   is derived inside the model via the drive efficiency.
%
% The profile is shaped like a short flight (taxi / takeoff / climb /
% cruise / descent / taxi) because a flight profile is the canonical
% stress case for a series hybrid: one large short peak (takeoff) that
% exceeds genset rating, a sustained medium load (climb) that also
% exceeds it, then a long segment (cruise) below rating where the pack
% must be recharged. All numbers are generic public-knowledge values -
% this is NOT any specific aircraft.
%
% Segments (t in s, shaft P in W; bus demand = P/0.92 for motoring):
%   0- 120  taxi      6 kW
% 120- 180  takeoff  65 kW   (bus ~71 kW > genset 40 kW -> battery ~31 kW)
% 180- 480  climb    48 kW   (bus ~52 kW > genset       -> battery ~12 kW)
% 480-1500  cruise   30 kW   (bus ~33 kW < genset       -> recharge window)
% 1500-1560 descent  -5 kW   (windmilling regen, tests charge-limit path)
% 1560-1680 descent   4 kW
% 1680-1800 taxi      6 kW
%
% Ramps of 5 s between segments avoid step discontinuities that would
% only exercise the solver, not the energy management.

seg_t = [   0  120  180  480  1500 1560 1680 1800];  % segment start times
seg_P = [ 6e3 65e3 48e3 30e3  -5e3  4e3  6e3  6e3];  % power after that time

t = (0:P.ctrl.Ts:P.sim.t_end)';
p = zeros(size(t));
ramp = 5;  % s
for k = 1:numel(t)
    idx = find(seg_t <= t(k), 1, 'last');
    p(k) = seg_P(idx);
    % linear ramp into each new segment
    if idx > 1 && (t(k) - seg_t(idx)) < ramp
        a = (t(k) - seg_t(idx)) / ramp;
        p(k) = (1-a)*seg_P(idx-1) + a*seg_P(idx);
    end
end

mission = timeseries(p, t, 'Name', 'P_shaft_demand_W');
end
