function P = hybrid_params()
%HYBRID_PARAMS Single source of truth for all plant & controller parameters.
%
%   P = hybrid_params() returns a struct used by the model build script,
%   the supervisory controller, and the mission profile generator.
%
% ==================== PROVENANCE / STAND-IN NOTICE ======================
% This project is designed to integrate with two other portfolio projects:
%
%   Project 3 - Battery SOC estimator (coulomb counting vs. EKF, 18650):
%       would supply the cell OCV(SOC) table, ohmic resistance R0, and
%       usable capacity from HPPC / capacity-test characterization.
%   Project 4 - Three-phase inverter + PMSM FOC (Simscape):
%       would supply the machine ratings and a measured drive
%       efficiency map (inverter + motor).
%
% Neither project's data files were reachable from this environment, so
% every parameter tagged [STAND-IN] below is a REPRESENTATIVE value taken
% from public datasheets / textbook-typical numbers, NOT measured data.
% Replace the tagged fields with characterized values and nothing else
% needs to change: the controller and model read only this struct.
% ========================================================================

%% ---------------- Battery: cell level (Project 3 interface) -----------
% [STAND-IN] Representative of a Samsung INR18650-25R class power cell
% (public datasheet values). Project 3's HPPC fit replaces these.
P.cell.Q_Ah      = 2.5;      % [STAND-IN] rated capacity, Ah
P.cell.R0        = 0.025;    % [STAND-IN] ohmic resistance, ohm (DC-IR class)
P.cell.V_max     = 4.20;     % charge voltage limit, V
P.cell.V_min     = 2.80;     % discharge cutoff used here (datasheet 2.5 V,
                             % derated for cycle life), V
P.cell.I_dis_max = 20;       % [STAND-IN] continuous discharge, A (8C)
P.cell.I_chg_max = 4;        % [STAND-IN] standard charge, A (1.6C)

% [STAND-IN] Representative NMC OCV-SOC curve (monotonic, 0..1).
% Project 3 delivers this table from low-current OCV testing.
P.cell.soc_brk = 0:0.1:1;
P.cell.ocv_V   = [3.00 3.30 3.43 3.49 3.53 3.58 3.64 3.71 3.80 3.94 4.15];

%% ---------------- Battery: pack level ----------------------------------
% Sizing rationale (see README "Pack sizing"): pack must cover the worst
% transient deficit (takeoff, ~30 kW for 60 s) and the climb deficit
% (~13 kW for 300 s) with SOC staying inside the operating band.
P.pack.Ns = 96;              % series cells -> 345.6 V nominal
P.pack.Np = 15;              % parallel strings
P.pack.V_nom  = P.pack.Ns * 3.6;                       % V
P.pack.Q_Ah   = P.pack.Np * P.cell.Q_Ah;               % Ah
P.pack.E_kWh  = P.pack.V_nom * P.pack.Q_Ah / 1e3;      % ~13 kWh
P.pack.R0     = P.cell.R0 * P.pack.Ns / P.pack.Np;     % ohm (~0.16)
P.pack.soc0   = 0.90;        % mission start SOC

% Power limits the SUPERVISOR enforces (plant can physically exceed them
% briefly; the controller must not command it).
P.pack.P_dis_max = 90e3;     % W  (< Np*I_dis_max*V_min_pack, with margin)
P.pack.P_chg_max = 18e3;     % W  (~1.4C charge, conservative for life)

%% ---------------- Electric drive (Project 4 interface) ------------------
% [STAND-IN] Representative 45 kW-continuous traction PMSM + inverter.
% Project 4's FOC model would supply the ratings and efficiency map;
% here the drive is modeled at SYSTEM level (averaged power port, fixed
% efficiency) - see README "Model fidelity ladder".
P.drive.P_cont  = 45e3;      % W continuous
P.drive.P_peak  = 70e3;      % W, 60 s peak
P.drive.eta     = 0.92;      % [STAND-IN] combined inverter+motor efficiency
P.drive.eta_regen = 0.85;    % [STAND-IN] shaft->DC recovery efficiency

%% ---------------- Turbogenerator (genset) -------------------------------
% [STAND-IN] Representative small turbogenerator (turbine + PM generator
% + rectifier), modeled as a rate-limited controlled power source.
% Rated power is sized so CRUISE electrical demand (~33 kW) sits inside
% the best-BSFC band (60-90% of rated) rather than saturating the unit -
% a 35 kW genset was tried first and spent the whole cruise pinned at
% 93-100% of rating (see README, "Genset sizing").
P.gen.P_rated  = 40e3;       % W electrical, continuous
P.gen.P_min    = 8e3;        % W, minimum stable load when running (20%)
P.gen.eff_lo   = 0.60;       % best-BSFC band, fraction of rated (lower)
P.gen.eff_hi   = 0.90;       % best-BSFC band, fraction of rated (upper)
P.gen.rate_W_s = 5e3;        % W/s slew (turbine spool dynamics)
P.gen.tau      = 3.0;        % s, first-order lag of power response

%% ---------------- DC bus -------------------------------------------------
P.bus.C        = 10e-3;      % F, bus capacitance (holds bus between
                             % controller samples; battery is the stiff node)
P.bus.V_min_warn = 96*P.cell.V_min;  % undervoltage warning level

%% ---------------- Supervisory controller --------------------------------
P.ctrl.Ts        = 0.1;      % s, supervisor sample time
P.ctrl.soc_lo    = 0.45;     % genset forced ON below this
P.ctrl.soc_hi    = 0.75;     % genset allowed OFF above this
P.ctrl.soc_target= 0.65;     % charge-sustaining target
P.ctrl.t_min_on  = 60;       % s, genset minimum ON time  (anti short-cycle)
P.ctrl.t_min_off = 60;       % s, genset minimum OFF time
P.ctrl.k_chg     = 40e3;     % W per unit SOC error, charge-bias gain
P.ctrl.P_dem_on  = 40e3;     % W, demand above which genset turns ON
                             % regardless of SOC (peak assist headroom)

%% ---------------- Simulation ---------------------------------------------
P.sim.Ts_plant = 0.01;       % s, plant step (fixed-step solver)
P.sim.t_end    = 1800;       % s, mission length
end
