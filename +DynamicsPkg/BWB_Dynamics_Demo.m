function [SizingSweep, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo(RunFullReport, RunCgSweep, RunCgSensitivity)
%
% [SizingSweep, Cases, CgSweep, SizingOpt] = BWB_Dynamics_Demo(RunFullReport, RunCgSweep, RunCgSensitivity)
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
if nargin < 3
    RunCgSensitivity = false;
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
MaxModelHalfSpanStation = max(abs(ChordTable.span_station_x));
ChordScale = (Aircraft.Specs.Dynamics.Geometry.b / 2) / MaxModelHalfSpanStation;
RightChord = ChordTable(ChordTable.normalized_span_eta >= 0, :);
ChordEta = RightChord.normalized_span_eta;
ChordLength = RightChord.chord_length * ChordScale;
ChordLeadingEdgeX = -RightChord.leading_edge_y * ChordScale;
ChordTrailingEdgeX = -RightChord.trailing_edge_y * ChordScale;

%% PAPER-STYLE CONTROL SURFACE SIZING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Build named control cases from the Flying-V control sizing method.
Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);

% Pitch elevons use 0-10 ft; dual-use and roll elevons use 20-45 ft.
StationToEta = @(Station) Station / MaxModelHalfSpanStation;
FtToStation = @(DistanceFt) (DistanceFt / 3.280839895) / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation;
FtPerM = 3.280839895;
MaxControlSurfaceSpan = 5 / FtPerM;
Surfaces.ForbiddenEta = StationToEta([FtToStation(10), FtToStation(20)]);
Surfaces.MaxModelHalfSpanStation = MaxModelHalfSpanStation;
Surfaces.MaxControlSurfaceSpan = MaxControlSurfaceSpan;
Surfaces.PanelStationWidth = 0.2;
Surfaces.PitchStationRange = FtToStation([0, 10]);
Surfaces.OutboardStationRange = FtToStation([20, 45]);
Surfaces.OutboardEtaStations = StationToEta(linspace(Surfaces.OutboardStationRange(1), Surfaces.OutboardStationRange(2), 8))';
Surfaces.PitchOutEtaStations = StationToEta(linspace(Surfaces.PitchStationRange(1), Surfaces.PitchStationRange(2), 6))';

Surfaces.SharedTrailingEdge = 1;
Surfaces.Elevator.Name = "Pitch Elevon";
Surfaces.Elevator.EtaControl = 0.85;
Surfaces.Elevator.ChordFractions = 0.25;
Surfaces.Elevator.EtaStations = Surfaces.PitchOutEtaStations;
Surfaces.Elevator.ChordEta = ChordEta;
Surfaces.Elevator.ChordLength = ChordLength;
Surfaces.Elevator.ChordLeadingEdgeX = ChordLeadingEdgeX;
Surfaces.Elevator.ChordTrailingEdgeX = ChordTrailingEdgeX;
Surfaces.Elevator.SectionClDelta = 3.0;
Surfaces.Elevator.UsePhysicalArea = 1;

Surfaces.DualElevon.Name = "Dual-Use Elevon";
Surfaces.DualElevon.EtaControl = 0.85;
Surfaces.DualElevon.ChordFractions = 0.40;
Surfaces.DualElevon.EtaStations = Surfaces.OutboardEtaStations;
Surfaces.DualElevon.ChordEta = ChordEta;
Surfaces.DualElevon.ChordLength = ChordLength;
Surfaces.DualElevon.ChordLeadingEdgeX = ChordLeadingEdgeX;
Surfaces.DualElevon.ChordTrailingEdgeX = ChordTrailingEdgeX;
Surfaces.DualElevon.SectionClDelta = 3.0;
Surfaces.DualElevon.UsePhysicalArea = 1;

Surfaces.Aileron.Name = "Roll Elevon";
Surfaces.Aileron.EtaControl = 0.85;
Surfaces.Aileron.ChordFractions = 0.40;
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
SizingOpt = DynamicsPkg.OptimizeSharedElevons(Aircraft, Cases, Surfaces);
SizingSweep = SizingOpt;

if RunCgSweep
    XcgMAC = linspace(Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC, 3)';
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
    AreaFraction = SizingOpt.Elevator.AreaFraction;
    Converged = SizingOpt.Converged;
end

CgSweep.XcgMAC = XcgMAC;
CgSweep.AreaFraction = AreaFraction;
CgSweep.Converged = Converged;

fprintf(1, "Required pitch authority area fraction: %.3f\n", SizingOpt.Elevator.AreaFraction);
fprintf(1, "Selected pitch elevon option: %s\n", SizingOpt.Elevator.Name);
fprintf(1, "Required roll authority area fraction: %.3f\n", SizingOpt.Aileron.AreaFraction);
fprintf(1, "Physical pitch-only elevon area fraction: %.3f\n", SizingOpt.Elevator.PhysicalAreaFraction);
fprintf(1, "Physical dual-use elevon area fraction: %.3f\n", SizingOpt.DualElevon.AreaFraction);
fprintf(1, "Physical roll-only elevon area fraction: %.3f\n", SizingOpt.Aileron.PhysicalAreaFraction);
fprintf(1, "Required rudder area fraction: %.3f\n", SizingOpt.Rudder.AreaFraction);
fprintf(1, "Total required control area fraction: %.3f\n", SizingOpt.AreaFraction);
fprintf(1, "Maximum selected elevator deflection: %.2f deg\n", max(abs([SizingOpt.Elevator.Checks.Trim.Delta; SizingOpt.Elevator.Checks.Pullup.DeltaFinal; SizingOpt.Elevator.Checks.Cruise.Delta])) * 180 / pi);
fprintf(1, "Selected roll-elevon deflection: %.2f deg\n", abs(SizingOpt.Aileron.Checks.Bank.Delta) * 180 / pi);
fprintf(1, "Selected rudder deflection: %.2f deg\n", abs(SizingOpt.Rudder.Checks.Direction.Delta) * 180 / pi);
fprintf(1, "All control-surface cases feasible: %d\n", SizingOpt.Converged);
fprintf(1, "Pitch authority station range: %.2f to %.2f, max chord %.2f\n", ...
    SizingOpt.Elevator.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.Elevator.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.Elevator.ChordFraction);
fprintf(1, "Dual-use station range: %.2f to %.2f, chord %.2f\n", ...
    SizingOpt.DualElevon.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.DualElevon.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.DualElevon.ChordFraction);
fprintf(1, "Roll authority station range: %.2f to %.2f, max chord %.2f\n", ...
    SizingOpt.Aileron.YInboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.Aileron.YOutboard / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation, ...
    SizingOpt.Aileron.ChordFraction);

%% PLOT THE SIZING TRADE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

DynamicsPkg.PlotSharedElevonAreas(SizingOpt, ChordEta, ChordLength, MaxModelHalfSpanStation, ...
    Aircraft.Specs.Dynamics.Geometry.b / 2, ...
    fullfile(OutputDir, "control_surface_areas.png"));

if ~RunFullReport
    return
end

% Required pitch deflection for the selected combined pitch-elevon system.
figure;
MaxDeflectionDeg = max(abs([SizingOpt.Elevator.Checks.Trim.Delta; ...
                            SizingOpt.Elevator.Checks.Pullup.DeltaFinal; ...
                            SizingOpt.Elevator.Checks.Cruise.Delta])) * 180 / pi;
DeflectionLimitDeg = Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi;
bar(categorical("Pitch elevons"), MaxDeflectionDeg)
hold on
yline(DeflectionLimitDeg, "--");
grid on
ylabel("Required pitch deflection [deg]");
title("Combined Pitch Elevon Requirement");
saveas(gcf, fullfile(OutputDir, "elevator_feasibility.png"));

figure;
MaxDeflectionDeg = abs(SizingOpt.Aileron.Checks.Bank.Delta) * 180 / pi;
DeflectionLimitDeg = Cases.TimeToBank.MaxDeflection * 180 / pi;
bar(categorical("Roll elevon"), MaxDeflectionDeg)
hold on
yline(DeflectionLimitDeg, "--");
grid on
ylabel("Required roll deflection [deg]");
title("Selected Roll Elevon Requirement");
saveas(gcf, fullfile(OutputDir, "aileron_feasibility.png"));

figure;
MaxDeflectionDeg = SizingOpt.Rudder.MaxDeflection * 180 / pi;
DeflectionLimitDeg = Cases.DirectionalTrim.MaxDeflection * 180 / pi;
bar(categorical("Winglet rudder"), MaxDeflectionDeg)
hold on
yline(DeflectionLimitDeg, "--");
grid on
ylabel("Required rudder deflection [deg]");
title("Selected Rudder Requirement");
saveas(gcf, fullfile(OutputDir, "rudder_feasibility.png"));

figure;
if RunCgSweep
    plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "LineWidth", 1.5);
else
    plot(CgSweep.XcgMAC, CgSweep.AreaFraction, "o", "MarkerSize", 8, "LineWidth", 1.5);
end
hold on
xlim([Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC]);
AreaLimit = max(0.06, 1.2 * max(CgSweep.AreaFraction));
ylim([0, AreaLimit]);
ForwardCG = Aircraft.Specs.Dynamics.CG.ForwardMAC;
AftCG = Aircraft.Specs.Dynamics.CG.AftMAC;
xline(ForwardCG, "--");
xline(AftCG, "--");
LabelInset = 0.005 * (AftCG - ForwardCG);
text(ForwardCG + LabelInset, 0.08 * AreaLimit, "Forward CG limit", ...
    "HorizontalAlignment", "left", "VerticalAlignment", "bottom");
text(AftCG - LabelInset, 0.08 * AreaLimit, "Aft CG limit", ...
    "HorizontalAlignment", "right", "VerticalAlignment", "bottom");
grid on
xlabel("CG location, x_{cg} / MAC");
ylabel("Required elevon area fraction, S_e / S");
if RunCgSweep
    title("Elevon Sizing vs CG");
else
    title("Selected Elevon Sizing Point");
end
saveas(gcf, fullfile(OutputDir, "elevator_area_vs_cg.png"));

if RunCgSensitivity
    XcgSensitivity = linspace(Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC, 25)';
    CgSensitivity = DynamicsPkg.SweepElevonCgSensitivity(Aircraft, Cases, SizingOpt.Elevator, XcgSensitivity);

    figure;
    subplot(2, 1, 1)
    plot(CgSensitivity.XcgMAC, [CgSensitivity.DeltaTrim, CgSensitivity.DeltaPullup, CgSensitivity.DeltaCruise] * 180 / pi, "LineWidth", 1.5)
    hold on
    yline(Aircraft.Specs.Dynamics.Longitudinal.DeltaMax * 180 / pi, "--");
    xline(Aircraft.Specs.Dynamics.CG.ForwardMAC, "--", "Forward CG");
    xline(Aircraft.Specs.Dynamics.CG.AftMAC, "--", "Aft CG");
    xlim([Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC]);
    grid on
    ylabel("|delta_e| [deg]");
    legend(["Trim", "Pull-up", "Cruise"], "Location", "best");
    title("Selected Elevon CG Sensitivity")

    subplot(2, 1, 2)
    plot(CgSensitivity.XcgMAC, [CgSensitivity.AlphaTrim, CgSensitivity.AlphaPullup, CgSensitivity.AlphaCruise] * 180 / pi, "LineWidth", 1.5)
    hold on
    yline(Aircraft.Specs.Dynamics.Longitudinal.AlphaMax * 180 / pi, "--");
    xline(Aircraft.Specs.Dynamics.CG.ForwardMAC, "--", "Forward CG");
    xline(Aircraft.Specs.Dynamics.CG.AftMAC, "--", "Aft CG");
    xlim([Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC]);
    grid on
    xlabel("CG location, x_{cg} / MAC");
    ylabel("|alpha| [deg]");
    legend(["Trim", "Pull-up", "Cruise"], "Location", "best");
    saveas(gcf, fullfile(OutputDir, "selected_elevon_cg_sensitivity.png"));
end

% Final selected design margins.
CaseLabels = categorical(["Trim"; "Pull-up"; "Cruise"]);
CaseLabels = reordercats(CaseLabels, ["Trim"; "Pull-up"; "Cruise"]);

DeltaDeg = abs([SizingOpt.Elevator.Checks.Trim.Delta; ...
                SizingOpt.Elevator.Checks.Pullup.DeltaFinal; ...
                SizingOpt.Elevator.Checks.Cruise.Delta]) * 180 / pi;

AlphaDeg = abs([SizingOpt.Elevator.Checks.Trim.Alpha; ...
                SizingOpt.Elevator.Checks.Pullup.AlphaFinal; ...
                SizingOpt.Elevator.Checks.Cruise.Alpha]) * 180 / pi;

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
bar(categorical("Time to bank"), abs(SizingOpt.Aileron.Checks.Bank.Phi) * 180 / pi)
hold on
yline(Cases.TimeToBank.BankTarget * 180 / pi, "--");
grid on
ylabel("Bank angle in 7 s [deg]");
title("Roll Authority")

subplot(2, 2, 4)
bar(categorical("Rotation"), SizingOpt.Elevator.Checks.Rotation.VR)
hold on
yline(Cases.TakeoffRotation.V2min - Cases.TakeoffRotation.Margin, "--");
grid on
ylabel("Speed [m/s]");
title("Takeoff Rotation")
saveas(gcf, fullfile(OutputDir, "selected_design_margins.png"));

end
