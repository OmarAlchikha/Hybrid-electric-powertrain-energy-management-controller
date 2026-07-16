function P_dem_elec = drive_interface(P_shaft)
%DRIVE_INTERFACE Map shaft power demand to DC-bus electrical demand.
%
% Source of the "DriveInterface" MATLAB Function block. Models the
% inverter + PMSM (Project 4) at SYSTEM level: an averaged power port
% with fixed motoring / regen efficiencies and a peak-power clamp.
% Project 4's FOC model is switching-level; re-using it here would make
% a 30-minute energy study intractable (see README, "Model fidelity
% ladder"). An efficiency MAP (torque x speed) from Project 4 would slot
% in here as a 2-D lookup replacing the constants.

params = hybrid_params();

% Peak-power clamp: the drive cannot deliver more than P_peak no matter
% what the mission asks; sizing must catch this, not hide it (a warning
% is asserted in post-processing if the clamp ever engages).
P_shaft = min(P_shaft, params.drive.P_peak);

if P_shaft >= 0
    P_dem_elec = P_shaft / params.drive.eta;        % motoring: bus supplies
else
    P_dem_elec = P_shaft * params.drive.eta_regen;  % regen: bus receives
end
end
