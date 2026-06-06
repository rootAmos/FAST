function [Sizing, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo()
%
% [Sizing, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo()
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

% Pitch-rate derivatives for the pull-up check.
Aircraft.Specs.Dynamics.Longitudinal.CLq = 3.0;
Aircraft.Specs.Dynamics.Longitudinal.Cmq = -8.0;

% Simple drag model: CD = CD0 + K * CL^2 + CDdelta * Se/S * delta_e^2.
Aircraft.Specs.Dynamics.Longitudinal.CD0 = 0.019;
Aircraft.Specs.Dynamics.Longitudinal.K = 0.050;
Aircraft.Specs.Dynamics.Longitudinal.CDdelta = 0.08;

% CLmax assumptions used to place low-speed trim cases.
Aircraft.Specs.Dynamics.Longitudinal.CLmaxTko = 1.8;
Aircraft.Specs.Dynamics.Longitudinal.CLmaxLnd = 1.9;

% Control/alpha limits used by the paper-style checks.
Aircraft.Specs.Dynamics.Longitudinal.DeltaMax = deg2rad(25);
Aircraft.Specs.Dynamics.Longitudinal.AlphaMax = deg2rad(28);

% Aero moment reference location, x_ref / MAC [-].
Aircraft.Specs.Dynamics.Longitudinal.XrefMAC = 0.25;

%% LATERAL AND GEOMETRY INPUTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Placeholder roll derivatives for the time-to-bank check.
Aircraft.Specs.Dynamics.Lateral.Clda = 0.08;
Aircraft.Specs.Dynamics.Lateral.Clp = -0.45;

% Placeholder geometry/inertia values used by pull-up, bank, and rotation.
Aircraft.Specs.Dynamics.Geometry.b = 35;
Aircraft.Specs.Dynamics.Geometry.cbar = 4.0;
Aircraft.Specs.Dynamics.Inertia.Ixx = 1.2e6;

% CG and main-gear locations are nondimensionalized by MAC.
Aircraft.Specs.Dynamics.CG.ForwardMAC = 0.28;
Aircraft.Specs.Dynamics.CG.AftMAC = 0.38;
Aircraft.Specs.Dynamics.Gear.XmlgMAC = 0.36;

%% FAST PRE-PROCESSING %%
%%%%%%%%%%%%%%%%%%%%%%%%%

% Fill FAST defaults/regressions before using sizing outputs.
Aircraft = DataStructPkg.PreSpecProcessing(Aircraft);
Aircraft = DataStructPkg.SpecProcessing(Aircraft);

OutputDir = fullfile("+DynamicsPkg", "outputs");
if ~exist(OutputDir, "dir")
    mkdir(OutputDir);
end

%% PAPER-STYLE CONTROL SURFACE SIZING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Build named control cases from the Flying-V control sizing method.
Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);

% Elevon geometry model: Selevon/Sref ~= span_fraction * chord_fraction.
Elevon.EtaControl = 0.85;
Elevon.ChordFractions = linspace(0.10, 0.35, 16)';
Elevon.SpanFractions = linspace(0.05, 1.00, 96)';

% Sweep elevon chord and span until all paper-style checks pass.
Sizing = DynamicsPkg.SizeElevons(Aircraft, Cases, Elevon);

% Continuous version of the same problem using CasADi/IPOPT.
SizingOpt = DynamicsPkg.OptimizeElevonsCasadi(Aircraft, Cases, Elevon);

% Sweep CG to show the wing/control-surface sizing coupling.
XcgMAC = linspace(0.22, 0.42, 15)';
AreaFraction = zeros(size(XcgMAC));
Converged = zeros(size(XcgMAC));

for icg = 1:length(XcgMAC)
    SweepCases = Cases;
    SweepCases.LongitudinalTrim.XcgMAC = XcgMAC(icg);
    SweepCases.Pullup.XcgMAC = XcgMAC(icg);
    SweepCases.TakeoffRotation.XcgMAC = XcgMAC(icg);
    SweepSizing = DynamicsPkg.SizeElevons(Aircraft, SweepCases, Elevon);
    AreaFraction(icg) = SweepSizing.AreaFraction;
    Converged(icg) = SweepSizing.Converged;
end

CgSweep.XcgMAC = XcgMAC;
CgSweep.AreaFraction = AreaFraction;
CgSweep.Converged = Converged;

fprintf(1, "Required elevon span fraction: %.3f\n", Sizing.SpanFraction);
fprintf(1, "Required elevon chord fraction: %.3f\n", Sizing.ChordFraction);
fprintf(1, "Required elevon area fraction: %.3f\n", Sizing.AreaFraction);
fprintf(1, "CasADi elevon area fraction: %.3f\n", SizingOpt.AreaFraction);
fprintf(1, "Elevon eta-control: %.3f\n", Sizing.EtaControl);
fprintf(1, "Maximum selected trim deflection: %.2f deg\n", max(abs([Sizing.Checks.Trim.Delta; Sizing.Checks.Pullup.DeltaFinal; Sizing.Checks.Cruise.Delta])) * 180 / pi);
fprintf(1, "All trim cases feasible: %d\n", Sizing.Converged);

%% PLOT THE SIZING TRADE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Required deflection over the chord/span grid.
figure;
MaxDeflectionDeg = Sizing.MaxDeflection * 180 / pi;
DeflectionLimitDeg = Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi;
MaxDeflectionPlot = min(MaxDeflectionDeg, 1.5 * DeflectionLimitDeg);
contourf(Sizing.ChordFractions, Sizing.SpanFractions, MaxDeflectionPlot, 20, "LineColor", "none");
colorbar
hold on
contour(Sizing.ChordFractions, Sizing.SpanFractions, MaxDeflectionDeg, ...
    [DeflectionLimitDeg, DeflectionLimitDeg], "k", "LineWidth", 1.5);
plot(Sizing.ChordFraction, Sizing.SpanFraction, "rx", "MarkerSize", 10, "LineWidth", 2);
plot(SizingOpt.ChordFraction, SizingOpt.SpanFraction, "wo", "MarkerSize", 7, "LineWidth", 1.5);
grid on
xlabel("Elevon chord fraction");
ylabel("Elevon span fraction");
title("Required Elevon Deflection [deg]");
saveas(gcf, fullfile(OutputDir, "elevon_feasibility.png"));

figure;
plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "LineWidth", 1.5);
hold on
xline(Cases.LongitudinalTrim.XcgMAC, "--", "Forward CG");
xline(Cases.TimeToBank.XcgMAC, "--", "Aft CG");
grid on
xlabel("CG location, x_{cg} / MAC");
ylabel("Required elevon area fraction, S_e / S");
title("Control Surface Sizing vs CG");
saveas(gcf, fullfile(OutputDir, "elevon_area_vs_cg.png"));

% Final selected design margins.
CaseLabels = categorical(["Trim"; "Pull-up"; "Cruise"]);
CaseLabels = reordercats(CaseLabels, ["Trim"; "Pull-up"; "Cruise"]);

DeltaDeg = abs([Sizing.Checks.Trim.Delta; ...
                Sizing.Checks.Pullup.DeltaFinal; ...
                Sizing.Checks.Cruise.Delta]) * 180 / pi;

AlphaDeg = abs([Sizing.Checks.Trim.Alpha; ...
                Sizing.Checks.Pullup.AlphaFinal; ...
                Sizing.Checks.Cruise.Alpha]) * 180 / pi;

figure;
subplot(2, 2, 1)
bar(CaseLabels, DeltaDeg)
hold on
yline(Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi, "--");
grid on
ylabel("|delta_e| [deg]");
title("Elevon Deflection")

subplot(2, 2, 2)
bar(CaseLabels, AlphaDeg)
hold on
yline(Aircraft.Specs.Dynamics.Longitudinal.AlphaMax * 180 / pi, "--");
grid on
ylabel("|alpha| [deg]");
title("Angle of Attack")

subplot(2, 2, 3)
bar(categorical("Time to bank"), abs(Sizing.Checks.Bank.Phi) * 180 / pi)
hold on
yline(Cases.TimeToBank.BankTarget * 180 / pi, "--");
grid on
ylabel("Bank angle in 7 s [deg]");
title("Roll Authority")

subplot(2, 2, 4)
bar(categorical("Rotation"), Sizing.Checks.Rotation.VR)
hold on
yline(Cases.TakeoffRotation.V2min - Cases.TakeoffRotation.Margin, "--");
grid on
ylabel("Speed [m/s]");
title("Takeoff Rotation")
saveas(gcf, fullfile(OutputDir, "selected_design_margins.png"));

end
