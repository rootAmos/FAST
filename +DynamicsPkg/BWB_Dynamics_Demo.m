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
FtPerM = 3.280839895;      % [ft/m] exact unit conversion used for reporting and station limits.
MPerFt = 1 / FtPerM;       % [m/ft] exact inverse conversion used for X-48 dimensions.
M2PerFt2 = MPerFt ^ 2;     % [m^2/ft^2] exact area conversion used for X-48 reference area.

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
% Chosen so the assumed CG envelope stays forward of the zero-trim-deflection
% point; forward CG should remain the limiting pitch-control case.
Aircraft.Specs.Dynamics.Longitudinal.Cmalpha = -0.80;

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

% Control/alpha limits used by the sizing-method feasibility checks.
Aircraft.Specs.Dynamics.Longitudinal.DeltaMax = deg2rad(25);
Aircraft.Specs.Dynamics.Longitudinal.AlphaMax = deg2rad(28);

% Aero moment reference location, x_ref / MAC [-].
Aircraft.Specs.Dynamics.Longitudinal.XrefMAC = 0.25;

%% LATERAL AND GEOMETRY INPUTS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Placeholder aircraft roll derivatives for the time-to-bank check. CrlDeltaA
% is aircraft-level dCrl/d(delta_a), not the local section lift derivative.
Aircraft.Specs.Dynamics.Lateral.CrlDeltaA = 0.08; % [1/rad] aircraft rolling-moment coefficient derivative.
Aircraft.Specs.Dynamics.Lateral.Crlp = -0.45; % [1/rad] aircraft roll-damping derivative dCrl/d(p*b/2V).
Aircraft.Specs.Dynamics.Lateral.Cndr = -0.25;

% Geometry uses the NASA/Boeing X-48B aspect ratio scaled to 100 ft span.
BaselineSpan = 35 * MPerFt;         % [m] reference span for the inertia scaling placeholder.
BaselineIxx = 1.2e6;                % [kg-m^2] placeholder roll inertia at BaselineSpan.
X48Span = 20.4 * MPerFt;            % [m] published X-48B span.
X48Area = 100.5 * M2PerFt2;         % [m^2] published X-48B planform area.
X48AspectRatio = X48Span ^ 2 / X48Area;
Aircraft.Specs.Dynamics.Geometry.b = 100 * MPerFt; % [m] target full span for this BWB demo.
Aircraft.Specs.Aero.S = Aircraft.Specs.Dynamics.Geometry.b ^ 2 / X48AspectRatio;
Aircraft.Specs.Weight.MTOW = BaselineWingLoading * Aircraft.Specs.Aero.S;
Aircraft.Specs.Aero.W_S.SLS = BaselineWingLoading;
LandingWeightFraction = 0.86; % [-] demo-only placeholder; a real BWB spec should provide MLW directly.
Aircraft.Specs.Weight.MLW = LandingWeightFraction * Aircraft.Specs.Weight.MTOW;
Aircraft.Specs.Dynamics.Geometry.cbar = Aircraft.Specs.Aero.S / Aircraft.Specs.Dynamics.Geometry.b;
Aircraft.Specs.Dynamics.Inertia.Ixx = BaselineIxx * (Aircraft.Specs.Dynamics.Geometry.b / BaselineSpan) ^ 2;

% Demo CG and main-gear locations are nondimensionalized by MAC. These CG
% limits are assumed for the placeholder BWB aero model, not derived from a
% mass-properties/loading envelope.
Aircraft.Specs.Dynamics.CG.ForwardMAC = 0.28;
Aircraft.Specs.Dynamics.CG.AftMAC = 0.32;
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

% The chord table comes from an X-48 planform outline. The table's station
% coordinates are model-space half-span stations, so this scale maps every
% extracted length to the 100 ft aircraft half-span used in the demo.
MaxModelHalfSpanStation = max(abs(ChordTable.span_station_x));
ChordScale = (Aircraft.Specs.Dynamics.Geometry.b / 2) / MaxModelHalfSpanStation;
RightChord = ChordTable(ChordTable.normalized_span_eta >= 0, :);

% ChordEta is the nondimensional half-span location. ChordLength and the
% leading/trailing-edge x locations are physical lengths after scaling.
ChordEta = RightChord.normalized_span_eta;
ChordLength = RightChord.chord_length * ChordScale;
ChordLeadingEdgeX = -RightChord.leading_edge_y * ChordScale;
ChordTrailingEdgeX = -RightChord.trailing_edge_y * ChordScale;

%% BWB CONTROL SURFACE SIZING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Build named control cases from the Flying-V control sizing method.
Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);

% Convert physical half-span distances into the X-48 station coordinate used
% by the chord table. Pitch elevons use 0-10 ft; dual-use and roll elevons
% use 20-45 ft, leaving 10-20 ft clear for the body-to-wing transition.
FtToStation = @(DistanceFt) (DistanceFt / FtPerM) / (Aircraft.Specs.Dynamics.Geometry.b / 2) * MaxModelHalfSpanStation;

Surfaces.MaxModelHalfSpanStation = MaxModelHalfSpanStation;
Surfaces.MaxControlSurfaceSpan = 5 / FtPerM; % [m] reporting split limit; optimized panels are no longer than 5 ft.
Surfaces.PanelStationWidth = FtToStation(5); % [station] each optimizer panel spans at most 5 physical ft.
Surfaces.PitchStationRange = FtToStation([0, 10]); % [station] pitch elevon allowed from centerline to 10 ft.
Surfaces.OutboardStationRange = FtToStation([20, 45]); % [station] dual-use pitch/roll elevons allowed from 20 to 45 ft.

% Planform fields let the optimizer interpolate local chord and x-location
% from span station. TauControlEff is the assumed hinge/control efficiency.
Planform.TauControlEff = 0.85;              % [-] control effectiveness factor multiplying section derivatives.
Planform.ChordEta = ChordEta;               % [-] nondimensional half-span lookup coordinate.
Planform.ChordLength = ChordLength;         % [m] local full chord from the scaled X-48 outline.
Planform.ChordLeadingEdgeX = ChordLeadingEdgeX;   % [m] local leading-edge x-location.
Planform.ChordTrailingEdgeX = ChordTrailingEdgeX; % [m] local trailing-edge x-location.

Surfaces.SharedTrailingEdge = 1; % outboard panels are one physical dual-use pitch/roll elevon surface.

Surfaces.Elevator = Planform;
Surfaces.Elevator.Name = "Pitch Elevon";
Surfaces.Elevator.ChordFractions = 0.20; % [-] max local chord fraction inside 10 ft; CasADi chooses actual panel fractions.
Surfaces.Elevator.SectionClDelta = 3.0;  % [1/rad] local 2D dcl/d(delta_elevon), where delta_elevon is panel deflection.

Surfaces.DualElevon = Planform;
Surfaces.DualElevon.Name = "Dual-Use Elevon";
Surfaces.DualElevon.ChordFractions = 0.40; % [-] max local chord fraction outside 20 ft for shared pitch+roll use.
Surfaces.DualElevon.SectionClDelta = 3.0;  % [1/rad] local 2D dcl/d(delta_elevon) for pitch/roll dual use.

Surfaces.Aileron = Surfaces.DualElevon;
Surfaces.Aileron.Name = "Roll Role of Dual-Use Elevon"; % same physical panels, evaluated for differential roll motion.

Surfaces.Rudder.TauControlEff = 0.85; % [-] rudder hinge/control effectiveness factor.
% Rudder still uses a small candidate grid because it is independent of the
% shared elevon CasADi allocation.
Surfaces.Rudder.ChordFractions = linspace(0.10, 0.35, 6)'; % [-] candidate rudder chord fractions.
Surfaces.Rudder.SpanFractions = linspace(0.10, 0.80, 15)'; % [-] candidate rudder span fractions.

% This is the main sizing call. OptimizeSharedElevons builds the panel
% geometry and calls the CasADi/Ipopt solve; it returns the selected panels
% plus post-solve numeric feasibility checks for reporting.
SizingOpt = DynamicsPkg.OptimizeSharedElevons(Aircraft, Cases, Surfaces);
SizingSweep = SizingOpt;

if RunCgSweep
    XcgMAC = linspace(Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC, 3)';
    PitchCaseSweep = DynamicsPkg.SweepPitchCaseAreas(Aircraft, Cases, Surfaces, XcgMAC);
    PitchAreaFraction = PitchCaseSweep.MaxAreaFraction;
    Converged = zeros(size(XcgMAC));

    for icg = 1:length(XcgMAC)
        SweepCases = Cases;
        SweepCases.LongitudinalTrim.XcgMAC = XcgMAC(icg);
        SweepCases.Pullup.XcgMAC = XcgMAC(icg);
        SweepCases.TakeoffRotation.XcgMAC = XcgMAC(icg);
        SweepCases.CruiseTrim.XcgMAC = XcgMAC(icg);

        % Re-run the full CasADi sizing at this CG location. This is an
        % optimized sizing sweep, not just a post-processing sensitivity.
        SweepSizing = DynamicsPkg.OptimizeSharedElevons(Aircraft, SweepCases, Surfaces);
        PitchAreaFraction(icg) = SweepSizing.Casadi.PitchAreaFraction;
        Converged(icg) = SweepSizing.Converged;
    end
else
    XcgMAC = Cases.LongitudinalTrim.XcgMAC;
    PitchCaseSweep = DynamicsPkg.SweepPitchCaseAreas(Aircraft, Cases, Surfaces, XcgMAC);
    PitchAreaFraction = SizingOpt.Casadi.PitchAreaFraction;
    Converged = SizingOpt.Converged;
end

CgSweep.XcgMAC = XcgMAC;
CgSweep.PitchAreaFraction = PitchAreaFraction;
CgSweep.PitchCaseSweep = PitchCaseSweep;
CgSweep.Converged = Converged;

fprintf(1, "Required pitch authority area fraction: %.3f\n", SizingOpt.Elevator.AreaFraction);
fprintf(1, "Selected pitch elevon option: %s\n", SizingOpt.Elevator.Name);
fprintf(1, "Roll authority uses dual-use elevon area fraction: %.3f\n", SizingOpt.Aileron.AreaFraction);
fprintf(1, "Physical pitch-only elevon area fraction: %.3f\n", SizingOpt.Elevator.PhysicalAreaFraction);
fprintf(1, "Physical dual-use elevon area fraction: %.3f\n", SizingOpt.DualElevon.AreaFraction);
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

figure;
if RunCgSweep
    plot(CgSweep.XcgMAC, [CgSweep.PitchCaseSweep.Trim, CgSweep.PitchCaseSweep.Pullup, ...
        CgSweep.PitchCaseSweep.Cruise, CgSweep.PitchCaseSweep.Rotation, ...
        CgSweep.PitchCaseSweep.MaxAreaFraction], "LineWidth", 1.5);
    legend(["Trim", "Pull-up", "Cruise", "Rotation", "Envelope"], "Location", "best");
else
    plot(CgSweep.XcgMAC, CgSweep.PitchAreaFraction, "o", "MarkerSize", 8, "LineWidth", 1.5);
end
hold on
xlim([Aircraft.Specs.Dynamics.CG.ForwardMAC, Aircraft.Specs.Dynamics.CG.AftMAC]);
AreaLimit = max(0.06, 1.2 * max(CgSweep.PitchAreaFraction));
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
ylabel("Pitch-required elevon area fraction, S_{e,pitch} / S");
if RunCgSweep
    title("Pitch Elevon Requirement vs CG");
else
    title("Selected Pitch Elevon Sizing Point");
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
