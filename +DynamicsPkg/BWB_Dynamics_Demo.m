function [Sizing, TrimCase, CgSweep] = BWB_Dynamics_Demo()
%
% [Sizing, TrimCase, CgSweep] = BWB_Dynamics_Demo()
%
% Demonstrate the new DynamicsPkg trim and elevon-sizing workflow using a
% FAST aircraft structure. Replace the aircraft spec with a BWB-specific
% file when one is available.
%
% Placeholder aero numbers are kept here for the demo. A real BWB concept
% should define these in its AircraftSpecsPkg file.
%

clc, close all

Aircraft = AircraftSpecsPkg.Example();

%% LONGITUDINAL AERO INPUTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lift-curve slope, dCL/dalpha [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.CLalpha = 4.8;

% Zero-alpha lift coefficient [-]. Used in the Cm0 reference shift.
Aircraft.Specs.Dynamics.Longitudinal.CL0 = 0.15;

% Elevon lift effectiveness, dCL/ddelta_e [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.CLdelta = 0.30;

% Zero-alpha pitching moment about the aero reference point [-].
Aircraft.Specs.Dynamics.Longitudinal.Cm0 = 0.015;

% Pitching-moment slope about the aero reference point, dCm/dalpha [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.Cmalpha = -0.35;

% Elevon moment effectiveness, dCm/ddelta_e [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.Cmdelta = -0.85;

% Simple trim drag model: CD = CD0 + K * CL^2.
Aircraft.Specs.Dynamics.Longitudinal.CD0 = 0.019;
Aircraft.Specs.Dynamics.Longitudinal.K = 0.050;

% CLmax assumptions used to place low-speed trim cases.
Aircraft.Specs.Dynamics.Longitudinal.CLmaxTko = 1.8;
Aircraft.Specs.Dynamics.Longitudinal.CLmaxLnd = 1.9;

% Aero moment reference location, x_ref / MAC [-].
Aircraft.Specs.Dynamics.Longitudinal.XrefMAC = 0.25;

%% FAST PRE-PROCESSING %%
%%%%%%%%%%%%%%%%%%%%%%%%%

% Fill FAST defaults/regressions before using sizing outputs.
Aircraft = DataStructPkg.PreSpecProcessing(Aircraft);
Aircraft = DataStructPkg.SpecProcessing(Aircraft);

%% TRIM ENVELOPE AND CONTROL SURFACE SIZING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Build low-speed and cruise trim cases from FAST outputs.
TrimCase = DynamicsPkg.SweepTrimEnvelope(Aircraft);

% CG location, x_cg / MAC [-]. Moments are shifted here before trim.
TrimCase.XcgMAC = 0.32;

% Elevon geometry model: Selevon/Sref ~= span_fraction * chord_fraction.
TrimCase.Elevon.EtaControl = 0.85;
TrimCase.Elevon.ChordFraction = 0.25;
TrimCase.Elevon.SpanFractions = linspace(0.05, 1.00, 192)';

% Sweep elevon span until all cases trim within limits.
Sizing = DynamicsPkg.SizeElevon(Aircraft, TrimCase);

% Sweep CG to show the wing/control-surface sizing coupling.
XcgMAC = linspace(0.22, 0.42, 25)';
AreaFraction = zeros(size(XcgMAC));
Converged = zeros(size(XcgMAC));

for icg = 1:length(XcgMAC)
    SweepCase = TrimCase;
    SweepCase.XcgMAC = XcgMAC(icg);
    SweepSizing = DynamicsPkg.SizeElevon(Aircraft, SweepCase);
    AreaFraction(icg) = SweepSizing.AreaFraction;
    Converged(icg) = SweepSizing.Converged;
end

CgSweep.XcgMAC = XcgMAC;
CgSweep.AreaFraction = AreaFraction;
CgSweep.Converged = Converged;

fprintf(1, "Required elevon span fraction: %.3f\n", Sizing.SpanFraction);
fprintf(1, "Required elevon area fraction: %.3f\n", Sizing.AreaFraction);
fprintf(1, "Elevon eta-control: %.3f\n", Sizing.EtaControl);
fprintf(1, "Maximum trim deflection: %.2f deg\n", max(abs(Sizing.Trim.DeltaTrim)) * 180 / pi);
fprintf(1, "All trim cases feasible: %d\n", Sizing.Converged);
disp(table(TrimCase.CaseName, TrimCase.CLtarget, TrimCase.Nz, TrimCase.Vel, ...
    'VariableNames', ["Case", "CL", "Nz", "TAS_mps"]));

%% PLOT THE SIZING TRADE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Deflection demand versus elevon area fraction.
figure;
plot(Sizing.AreaFractions, Sizing.MaxAbsDeflection * 180 / pi, "LineWidth", 1.5);
hold on
yline(TrimCase.MaxDeflection * 180 / pi, "--");
grid on
xlabel("Elevon area fraction, S_e / S");
ylabel("Maximum trim deflection [deg]");
title("Conceptual BWB Trim Authority Sweep");

figure;
plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "LineWidth", 1.5);
grid on
xlabel("CG location, x_{cg} / MAC");
ylabel("Required elevon area fraction, S_e / S");
title("Control Surface Sizing vs CG");

end
