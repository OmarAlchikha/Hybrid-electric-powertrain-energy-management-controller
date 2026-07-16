function [P_gen_cmd, P_batt_exp, gen_on_out, mode, P_unmet, P_dump] = ...
    power_split_controller(P_dem_elec, soc, P_gen_meas)
%POWER_SPLIT_CONTROLLER Rule-based supervisory energy management.
%
% This is the source of the "Supervisor" MATLAB Function block in the
% Simulink model (build_hybrid_model.m loads this file into the block).
% It runs at Ts = params.ctrl.Ts (0.1 s), enforced by zero-order holds
% on its inputs.
%
% Inputs
%   P_dem_elec : electrical power demanded from the DC bus, W
%                (shaft demand / drive efficiency, regen already scaled)
%   soc        : battery state of charge, 0..1  (in the real system this
%                is Project 3's EKF estimate, NOT true SOC - see README)
%   P_gen_meas : MEASURED genset electrical power, W. The feasibility
%                accounting (R6) runs on telemetry, not on the command:
%                the turbine lag makes actual power trail the command by
%                up to rate*tau (~15 kW) during slews, and accounting on
%                the command let the battery charge limit be breached by
%                ~50% during the cruise->descent transition (bug found
%                by the co-simulation checks - see README).
% Outputs
%   P_gen_cmd  : genset electrical power setpoint, W (rate-limited here,
%                so the plant lag is the only remaining dynamics)
%   P_batt_exp : battery power the supervisor EXPECTS (+ = discharge), W.
%                The battery is the slack node on the bus, so this is a
%                prediction/limit-check, not an actuation.
%   gen_on_out : genset run state (0/1)
%   mode       : 0 batt-only | 1 assist/load-follow | 2 recharge | 3 regen
%   P_unmet    : demand the hybrid cannot meet this step (both sources
%                saturated), W - must be 0 for a feasible mission
%   P_dump     : regen power exceeding the charge limit, W (would go to a
%                brake resistor; propulsor would feather instead)
%
% Rule set (rationale in README, "Controller design"):
%   R1  Battery power limits are SOC-tapered near the rails.
%   R2  Genset ON if soc < soc_lo OR demand > P_dem_on;
%       OFF only if soc > soc_hi AND demand < genset minimum stable load
%       (below P_min the genset cannot load-follow anyway, so battery-only
%       is strictly better; above P_min, staying ON with charge bias beats
%       thermostat-cycling the pack - see README).
%       Both transitions respect minimum on/off dwell times.
%   R3  When ON, command = demand + charge bias k_chg*(soc_target - soc),
%       clamped to [P_min, P_rated].
%   R4  Best-efficiency shaping: if R3 lands below the best-BSFC band and
%       the battery can absorb the surplus, raise the command to the
%       bottom of the band (load-levelling).
%   R5  Command is slew-limited (turbine spool) before it leaves.
%   R6  Whatever the genset does NOT actually deliver (measured), the
%       battery does - subject to R1; any residual is reported as
%       P_unmet / P_dump, never hidden.

params = hybrid_params();
Ts = params.ctrl.Ts;

persistent gen_on t_switch P_gen_prev
if isempty(gen_on)
    gen_on = 0; t_switch = -params.ctrl.t_min_off; P_gen_prev = 0;
end
persistent t_now
if isempty(t_now), t_now = 0; else, t_now = t_now + Ts; end

%% R1: SOC-tapered battery limits
% Linear taper over the last 5% at each rail: prevents the supervisor
% from commanding full power into a nearly-full/empty pack, which is
% where a flat-limit controller overshoots V_max/V_min.
taper_hi = min(1, max(0, (0.95 - soc) / 0.05));   % ->0 as soc->0.95+
taper_lo = min(1, max(0, (soc - 0.05) / 0.05));   % ->0 as soc->0.05-
P_dis_lim = params.pack.P_dis_max * taper_lo;     % max discharge, W
P_chg_lim = params.pack.P_chg_max * taper_hi;     % max charge, W (>=0)

%% R2: genset on/off with hysteresis + dwell times
dwell_ok_on  = (t_now - t_switch) >= params.ctrl.t_min_off; % may turn ON
dwell_ok_off = (t_now - t_switch) >= params.ctrl.t_min_on;  % may turn OFF

want_on  = (soc < params.ctrl.soc_lo) || (P_dem_elec > params.ctrl.P_dem_on);
want_off = (soc > params.ctrl.soc_hi) && (P_dem_elec < params.gen.P_min);

if ~gen_on && want_on && dwell_ok_on
    gen_on = 1; t_switch = t_now;
elseif gen_on && want_off && dwell_ok_off
    gen_on = 0; t_switch = t_now;
end

%% R3 + R4: genset power target
if gen_on
    P_chg_bias = params.ctrl.k_chg * (params.ctrl.soc_target - soc);
    P_chg_bias = min(P_chg_bias, P_chg_lim);      % can't charge past limit
    P_chg_bias = max(P_chg_bias, 0);              % never discharge via bias
    P_tgt = P_dem_elec + P_chg_bias;
    P_tgt = min(max(P_tgt, params.gen.P_min), params.gen.P_rated);

    % R4: load-level up into the best-BSFC band if the battery can absorb
    P_band_lo = params.gen.eff_lo * params.gen.P_rated;
    if P_tgt < P_band_lo
        surplus_cap = P_chg_lim - max(P_tgt - P_dem_elec, 0);
        P_tgt = min(P_band_lo, P_tgt + max(surplus_cap, 0));
    end

    % Regen while running: drop to minimum stable load (turbine cannot
    % absorb power; shutting down would violate dwell logic for a
    % 60-second descent segment).
    if P_dem_elec < 0
        P_tgt = params.gen.P_min;
    end
else
    P_tgt = 0;
end

%% R5: slew limit (turbine spool)
dP_max = params.gen.rate_W_s * Ts;
P_gen_cmd = min(max(P_tgt, P_gen_prev - dP_max), P_gen_prev + dP_max);
% A stopped genset starts from zero; a stopping one ramps down through
% P_min to 0 (modeled as part of the same slew).
P_gen_cmd = max(P_gen_cmd, 0);
P_gen_prev = P_gen_cmd;

%% R6: battery expectation + feasibility accounting (on MEASURED power)
P_batt_exp = P_dem_elec - P_gen_meas;  % + = discharge
P_unmet = 0; P_dump = 0;
if P_batt_exp > P_dis_lim
    P_unmet   = P_batt_exp - P_dis_lim;
    P_batt_exp = P_dis_lim;
elseif P_batt_exp < -P_chg_lim
    P_dump    = -(P_batt_exp + P_chg_lim);
    P_batt_exp = -P_chg_lim;
end

%% mode annunciation
if P_dem_elec < 0
    mode = 3;
elseif ~gen_on
    mode = 0;
elseif P_batt_exp > 1e2
    mode = 1;
else
    mode = 2;
end
gen_on_out = gen_on;
end
