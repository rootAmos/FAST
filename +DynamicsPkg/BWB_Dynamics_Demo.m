function [SizingSweep, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo(RunFullReport, RunCgSweep)
%
% [SizingSweep, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo(RunFullReport, RunCgSweep)
%
% Demonstrate the DynamicsPkg trim and control-surface sizing workflow using a
% FAST aircraft structure. Replace the aircraft spec with a BWB-specific
% file when one is available.
%
% Placeholder aero numbers are kept here for the demo. A real BWB concept
% should define these in its AircraftSpecsPkg file.
%

if nargin < 1
    RunFullReport = false;
end
if nargin < 2
    RunCgSweep = false;
end

clc, close all

Aircraft = AircraftSpecsPkg.Example();
BaselineWingLoading = Aircraft.Specs.Aero.W_S.SLS;

%% LONGITUDINAL AERO INPUTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lift-curve slope, dCL/dalpha [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.CLalpha = 4.8;

% Zero-alpha lift coefficient [-]. Used in the Cm0 reference shift.
Aircraft.Specs.Dynamics.Longitudinal.CL0 = 0.15;

% Elevator lift effectiveness, dCL/ddelta_e [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.CLdelta = 0.30;

% Zero-alpha pitching moment about the aero reference point [-].
Aircraft.Specs.Dynamics.Longitudinal.Cm0 = 0.015;

% Pitching-moment slope about the aero reference point, dCm/dalpha [1/rad].
Aircraft.Specs.Dynamics.Longitudinal.Cmalpha = -0.35;

% Elevator moment effectiveness, dCm/ddelta_e [1/rad].
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
Aircraft.Specs.Dynamics.Lateral.Cndr = -0.25;

% Geometry uses the NASA/Boeing X-48B aspect ratio scaled to 100 ft span.
BaselineSpan = 35;
BaselineIxx = 1.2e6;
X48Span = 20.4 * 0.3048;
X48Area = 100.5 * 0.092903;
X48AspectRatio = X48Span ^ 2 / X48Area;
Aircraft.Specs.Dynamics.Geometry.b = 100 * 0.3048;
Aircraft.Specs.Aero.S = Aircraft.Specs.Dynamics.Geometry.b ^ 2 / X48AspectRatio;
Aircraft.Specs.Weight.MTOW = BaselineWingLoading * Aircraft.Specs.Aero.S;
Aircraft.Specs.Aero.W_S.SLS = BaselineWingLoading;
% Demo-only placeholder; a real BWB spec should provide MLW directly.
Aircraft.Specs.Weight.MLW = 0.86 * Aircraft.Specs.Weight.MTOW;
Aircraft.Specs.Dynamics.Geometry.cbar = Aircraft.Specs.Aero.S / Aircraft.Specs.Dynamics.Geometry.b;
Aircraft.Specs.Dynamics.Inertia.Ixx = BaselineIxx * (Aircraft.Specs.Dynamics.Geometry.b / BaselineSpan) ^ 2;

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

ChordFile = fullfile(OutputDir, "bwb_chord_vs_span.csv");
ChordTable = readtable(ChordFile);
ChordArea = abs(trapz(ChordTable.span_station_x, ChordTable.chord_length));
ChordScale = (Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS) / ChordArea;
RightChord = ChordTable(ChordTable.normalized_span_eta >= 0, :);
ChordEta = RightChord.normalized_span_eta;
ChordLength = RightChord.chord_length * ChordScale;
ChordLeadingEdgeX = -RightChord.leading_edge_y * ChordScale;
ChordTrailingEdgeX = -RightChord.trailing_edge_y * ChordScale;
MaxModelHalfSpanStation = max(abs(ChordTable.span_station_x));

%% PAPER-STYLE CONTROL SURFACE SIZING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Build named control cases from the Flying-V control sizing method.
Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);

% Pitch, dual-use, and roll elevons use station 0-5 or 10-outboard only.
StationToEta = @(Station) Station / MaxModelHalfSpanStation;
Surfaces.ForbiddenEta = StationToEta([5, 10]);
Surfaces.MaxModelHalfSpanStation = MaxModelHalfSpanStation;
Surfaces.PanelStationWidth = 0.2;
Surfaces.OutboardEtaStations = StationToEta([10, 11, 12, 13, 14, 15, 16, 16.45])';
Surfaces.PitchOutEtaStations = StationToEta([1, 2, 3, 4, 5])';

Surfaces.SharedTrailingEdge = 1;
Surfaces.Elevator.Name = "Pitch Elevon";
Surfaces.Elevator.EtaControl = 0.85;
Surfaces.Elevator.ChordFractions = [0.60; 0.75; 0.90];
Surfaces.Elevator.EtaStations = Surfaces.PitchOutEtaStations;
Surfaces.Elevator.ChordEta = ChordEta;
Surfaces.Elevator.ChordLength = ChordLength;
Surfaces.Elevator.ChordLeadingEdgeX = ChordLeadingEdgeX;
Surfaces.Elevator.ChordTrailingEdgeX = ChordTrailingEdgeX;
Surfaces.Elevator.SectionClDelta = 3.0;
Surfaces.Elevator.UsePhysicalArea = 1;

Surfaces.DualElevon.Name = "Dual-Use Elevon";
Surfaces.DualElevon.EtaControl = 0.85;
Surfaces.DualElevon.ChordFractions = [0.60; 0.75; 0.90];
Surfaces.DualElevon.EtaStations = Surfaces.OutboardEtaStations;
Surfaces.DualElevon.ChordEta = ChordEta;
Surfaces.DualElevon.ChordLength = ChordLength;
Surfaces.DualElevon.ChordLeadingEdgeX = ChordLeadingEdgeX;
Surfaces.DualElevon.ChordTrailingEdgeX = ChordTrailingEdgeX;
Surfaces.DualElevon.SectionClDelta = 3.0;
Surfaces.DualElevon.UsePhysicalArea = 1;

Surfaces.Aileron.Name = "Roll Elevon";
Surfaces.Aileron.EtaControl = 0.85;
Surfaces.Aileron.ChordFractions = [0.25; 0.35; 0.50];
Surfaces.Aileron.EtaStations = Surfaces.OutboardEtaStations;
Surfaces.Aileron.ReferenceChord = (Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS) / Aircraft.Specs.Dynamics.Geometry.b;
Surfaces.Aileron.ChordEta = ChordEta;
Surfaces.Aileron.ChordLength = ChordLength;
Surfaces.Aileron.ChordLeadingEdgeX = ChordLeadingEdgeX;
Surfaces.Aileron.ChordTrailingEdgeX = ChordTrailingEdgeX;
Surfaces.Aileron.SectionClDelta = 2.5;
Surfaces.Aileron.UsePhysicalArea = 1;

Surfaces.Rudder.EtaControl = 0.85;
Surfaces.Rudder.ChordFractions = linspace(0.10, 0.35, 6)';
Surfaces.Rudder.SpanFractions = linspace(0.10, 0.80, 15)';

% Optimize the shared pitch/dual-use/roll elevon layout against the checks.
SizingSweep = DynamicsPkg.OptimizeSharedElevons(Aircraft, Cases, Surfaces);
SizingOpt = SizingSweep;

if RunCgSweep
    XcgMAC = linspace(Cases.LongitudinalTrim.XcgMAC, Cases.CruiseTrim.XcgMAC, 3)';
    AreaFraction = zeros(size(XcgMAC));
    Converged = zeros(size(XcgMAC));

    for icg = 1:length(XcgMAC)
        SweepCases = Cases;
        SweepCases.LongitudinalTrim.XcgMAC = XcgMAC(icg);
        SweepCases.Pullup.XcgMAC = XcgMAC(icg);
        SweepCases.TakeoffRotation.XcgMAC = XcgMAC(icg);
        SweepSizing = DynamicsPkg.OptimizeSharedElevons(Aircraft, SweepCases, Surfaces);
        AreaFraction(icg) = SweepSizing.Elevator.AreaFraction;
        Converged(icg) = SweepSizing.Converged;
    end
else
    XcgMAC = Cases.LongitudinalTrim.XcgMAC;
    AreaFraction = SizingSweep.Elevator.AreaFraction;
    Converged = SizingSweep.Converged;
end

CgSweep.XcgMAC = XcgMAC;
CgSweep.AreaFraction = AreaFraction;
CgSweep.Converged = Converged;

fprintf(1, "Required pitch authority area fraction: %.3f\n", SizingSweep.Elevator.AreaFraction);
fprintf(1, "Selected pitch elevon option: %s\n", SizingSweep.Elevator.Name);
fprintf(1, "Required roll authority area fraction: %.3f\n", SizingSweep.Aileron.AreaFraction);
fprintf(1, "Physical pitch-only elevon area fraction: %.3f\n", SizingSweep.Elevator.PhysicalAreaFraction);
fprintf(1, "Physical dual-use elevon area fraction: %.3f\n", SizingSweep.DualElevon.AreaFraction);
fprintf(1, "Physical roll-only elevon area fraction: %.3f\n", SizingSweep.Aileron.PhysicalAreaFraction);
fprintf(1, "Required rudder area fraction: %.3f\n", SizingSweep.Rudder.AreaFraction);
fprintf(1, "Total required control area fraction: %.3f\n", SizingSweep.AreaFraction);
fprintf(1, "Maximum selected elevator deflection: %.2f deg\n", max(abs([SizingSweep.Elevator.Checks.Trim.Delta; SizingSweep.Elevator.Checks.Pullup.DeltaFinal; SizingSweep.Elevator.Checks.Cruise.Delta])) * 180 / pi);
fprintf(1, "Selected roll-elevon deflection: %.2f deg\n", abs(SizingSweep.Aileron.Checks.Bank.Delta) * 180 / pi);
fprintf(1, "Selected rudder deflection: %.2f deg\n", abs(SizingSweep.Rudder.Checks.Direction.Delta) * 180 / pi);
fprintf(1, "All control-surface cases feasible: %d\n", SizingSweep.Converged);
fprintf(1, "Pitch authority station range: %.2f to %.2f, max chord %.2f\n", ...
    SizingSweep.Elevator.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.Elevator.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.Elevator.ChordFraction);
fprintf(1, "Dual-use station range: %.2f to %.2f, chord %.2f\n", ...
    SizingSweep.DualElevon.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.DualElevon.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.DualElevon.ChordFraction);
fprintf(1, "Roll authority station range: %.2f to %.2f, max chord %.2f\n", ...
    SizingSweep.Aileron.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.Aileron.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingSweep.Aileron.ChordFraction);

%% PLOT THE SIZING TRADE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

DynamicsPkg.PlotSharedElevonAreas(SizingSweep, ChordEta, ChordLength, MaxModelHalfSpanStation, ...
    fullfile(OutputDir, "control_surface_areas.png"));

if ~RunFullReport
    return
end

% Required pitch deflection for the selected combined pitch-elevon system.
figure;
MaxDeflectionDeg = max(abs([SizingSweep.Elevator.Checks.Trim.Delta; ...
                            SizingSweep.Elevator.Checks.Pullup.DeltaFinal; ...
                            SizingSweep.Elevator.Checks.Cruise.Delta])) * 180 / pi;
DeflectionLimitDeg = Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi;
bar(categorical("Pitch elevons"), MaxDeflectionDeg)
hold on
yline(DeflectionLimitDeg, "--");
grid on
ylabel("Required pitch deflection [deg]");
title("Combined Pitch Elevon Requirement");
saveas(gcf, fullfile(OutputDir, "elevator_feasibility.png"));

figure;
MaxDeflectionDeg = abs(SizingSweep.Aileron.Checks.Bank.Delta) * 180 / pi;
DeflectionLimitDeg = Cases.TimeToBank.MaxDeflection * 180 / pi;
bar(categorical("Roll elevon"), MaxDeflectionDeg)
hold on
yline(DeflectionLimitDeg, "--");
grid on
ylabel("Required roll deflection [deg]");
title("Selected Roll Elevon Requirement");
saveas(gcf, fullfile(OutputDir, "aileron_feasibility.png"));

figure;
MaxDeflectionDeg = SizingSweep.Rudder.MaxDeflection * 180 / pi;
DeflectionLimitDeg = Cases.DirectionalTrim.MaxDeflection * 180 / pi;
MaxDeflectionPlot = min(MaxDeflectionDeg, 1.5 * DeflectionLimitDeg);
contourf(SizingSweep.Rudder.ChordFractions, SizingSweep.Rudder.SpanFractions, MaxDeflectionPlot, 20, "LineColor", "none");
colorbar
hold on
contour(SizingSweep.Rudder.ChordFractions, SizingSweep.Rudder.SpanFractions, MaxDeflectionDeg, ...
    [DeflectionLimitDeg, DeflectionLimitDeg], "k", "LineWidth", 1.5);
grid on
xlabel("Rudder chord fraction");
ylabel("Rudder span fraction");
title("Required Rudder Deflection [deg]");
saveas(gcf, fullfile(OutputDir, "rudder_feasibility.png"));

figure;
if RunCgSweep
    plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "LineWidth", 1.5);
else
    plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "o", "MarkerSize", 8, "LineWidth", 1.5);
end
hold on
xline(Cases.LongitudinalTrim.XcgMAC, "--", "Forward CG");
xline(Cases.TimeToBank.XcgMAC, "--", "Aft CG");
grid on
xlabel("CG location, x_{cg} / MAC");
ylabel("Required elevator area fraction, S_e / S");
if RunCgSweep
    title("Elevator Sizing vs CG");
else
    title("Selected Elevator Sizing Point");
end
saveas(gcf, fullfile(OutputDir, "elevator_area_vs_cg.png"));

% Final selected design margins.
CaseLabels = categorical(["Trim"; "Pull-up"; "Cruise"]);
CaseLabels = reordercats(CaseLabels, ["Trim"; "Pull-up"; "Cruise"]);

DeltaDeg = abs([SizingSweep.Elevator.Checks.Trim.Delta; ...
                SizingSweep.Elevator.Checks.Pullup.DeltaFinal; ...
                SizingSweep.Elevator.Checks.Cruise.Delta]) * 180 / pi;

AlphaDeg = abs([SizingSweep.Elevator.Checks.Trim.Alpha; ...
                SizingSweep.Elevator.Checks.Pullup.AlphaFinal; ...
                SizingSweep.Elevator.Checks.Cruise.Alpha]) * 180 / pi;

figure;
subplot(2, 2, 1)
bar(CaseLabels, DeltaDeg)
hold on
yline(Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi, "--");
grid on
ylabel("|delta_e| [deg]");
title("Elevator Deflection")

subplot(2, 2, 2)
bar(CaseLabels, AlphaDeg)
hold on
yline(Aircraft.Specs.Dynamics.Longitudinal.AlphaMax * 180 / pi, "--");
grid on
ylabel("|alpha| [deg]");
title("Angle of Attack")

subplot(2, 2, 3)
bar(categorical("Time to bank"), abs(SizingSweep.Aileron.Checks.Bank.Phi) * 180 / pi)
hold on
yline(Cases.TimeToBank.BankTarget * 180 / pi, "--");
grid on
ylabel("Bank angle in 7 s [deg]");
title("Roll Authority")

subplot(2, 2, 4)
bar(categorical("Rotation"), SizingSweep.Elevator.Checks.Rotation.VR)
hold on
yline(Cases.TakeoffRotation.V2min - Cases.TakeoffRotation.Margin, "--");
grid on
ylabel("Speed [m/s]");
title("Takeoff Rotation")
saveas(gcf, fullfile(OutputDir, "selected_design_margins.png"));

end
