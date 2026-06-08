function [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% Optimize 0.2-station shared elevon panels with CasADi.
%

if ~isfield(Surfaces, "SharedTrailingEdge") || ~Surfaces.SharedTrailingEdge
    error("ERROR - OptimizeSharedElevons: Surfaces.SharedTrailingEdge must be true.");
end

try
    import casadi.*
catch
    error("ERROR - OptimizeSharedElevons: CasADi is not on the MATLAB path.");
end

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
MaxStation = Surfaces.MaxModelHalfSpanStation;
PanelWidth = Surfaces.PanelStationWidth;

PitchPanels = BuildStationPanels(0, 5, PanelWidth);
OutboardPanels = BuildStationPanels(10, max(Surfaces.OutboardEtaStations) * MaxStation, PanelWidth);

PitchCoeff = PanelAreaCoefficients(Aircraft, Surfaces.Elevator, PitchPanels, MaxStation);
OutboardCoeff = PanelAreaCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);
RollCoeff = PanelRollCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);

Opti = casadi.Opti();
PitchChord = Opti.variable(length(PitchPanels.Inboard), 1);
DualChord = Opti.variable(length(OutboardPanels.Inboard), 1);
RollChord = Opti.variable(length(OutboardPanels.Inboard), 1);

PitchMax = max(Surfaces.Elevator.ChordFractions);
DualMax = max(Surfaces.DualElevon.ChordFractions);
RollMax = max(Surfaces.Aileron.ChordFractions);
OutboardMax = max(DualMax, RollMax);

Opti.subject_to(PitchChord >= 0);
Opti.subject_to(PitchChord <= PitchMax);
Opti.subject_to(DualChord >= 0);
Opti.subject_to(DualChord <= DualMax);
Opti.subject_to(RollChord >= 0);
Opti.subject_to(RollChord <= RollMax);
Opti.subject_to(DualChord + RollChord <= OutboardMax);

PitchArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord;
RollArea = OutboardCoeff' * (DualChord + RollChord);
RollIntegral = RollCoeff' * (DualChord + RollChord);

ApplyPitchConstraints(Opti, Aircraft, Cases.LongitudinalTrim, PitchArea);
ApplyPullupConstraints(Opti, Aircraft, Cases.Pullup, PitchArea);
ApplyPitchConstraints(Opti, Aircraft, Cases.CruiseTrim, PitchArea);
ApplyRollConstraint(Opti, Aircraft, Cases.TimeToBank, RollIntegral);

PhysicalArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord + OutboardCoeff' * RollChord;
CenterBias = 1.0e-5 * (PitchPanels.Center' * PitchChord + OutboardPanels.Center' * DualChord) / MaxStation;
Opti.minimize(PhysicalArea + CenterBias);
Opti.solver('ipopt', struct('print_time', false), struct('print_level', 0));
Opti.set_initial(PitchChord, 0.05);
Opti.set_initial(DualChord, 0.05);
Opti.set_initial(RollChord, 0.05);

Sol = Opti.solve();
PitchChordValue = full(Sol.value(PitchChord));
DualChordValue = full(Sol.value(DualChord));
RollChordValue = full(Sol.value(RollChord));

PitchOnlySegments = BuildSegmentsFromPanels(Aircraft, Surfaces.Elevator, PitchPanels, PitchChordValue, MaxStation, "Pitch-only elevon");
DualSegments = BuildSegmentsFromPanels(Aircraft, Surfaces.DualElevon, OutboardPanels, DualChordValue, MaxStation, "Dual-use elevon");
RollOnlySegments = BuildSegmentsFromPanels(Aircraft, Surfaces.Aileron, OutboardPanels, RollChordValue, MaxStation, "Roll-only elevon");

Elevator = CombineSegments(Surfaces.Elevator, [PitchOnlySegments; DualSegments], "Pitch Elevon");
Aileron = CombineSegments(Surfaces.Aileron, [DualSegments; RollOnlySegments], "Roll Elevon");
DualElevon = CombineSegments(Surfaces.DualElevon, DualSegments, "Dual-Use Elevon");
Rudder = SizeRudder(Aircraft, Cases.DirectionalTrim, Surfaces.Rudder);

Elevator.Checks = CheckElevator(Aircraft, Cases, Elevator);
Aileron.Checks = CheckAileron(Aircraft, Cases, Aileron);
Rudder.Checks = DynamicsPkg.CheckDirectionalTrim(Aircraft, Cases.DirectionalTrim, Rudder);

Elevator.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, PitchOnlySegments));
Aileron.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, RollOnlySegments));
Elevator.MaxDeflection = Elevator.Checks.MaxDeflection;
Aileron.MaxDeflection = Aileron.Checks.MaxDeflection;
Rudder.MaxDeflection = abs(Rudder.Checks.Delta);

Elevator.Converged = Elevator.Checks.Feasible;
Aileron.Converged = Aileron.Checks.Feasible;
Rudder.Converged = Rudder.Checks.Feasible;
DualElevon.Converged = true;

Sizing.Elevator = Elevator;
Sizing.Aileron = Aileron;
Sizing.DualElevon = DualElevon;
Sizing.Rudder = Rudder;
Sizing.Converged = Elevator.Converged && Aileron.Converged && Rudder.Converged;
Sizing.AreaFraction = Elevator.PhysicalAreaFraction + DualElevon.AreaFraction + ...
    Aileron.PhysicalAreaFraction + Rudder.AreaFraction;
Sizing.Casadi = struct( ...
    "PitchAreaFraction", full(Sol.value(PitchArea)), ...
    "RollAreaFraction", full(Sol.value(RollArea)), ...
    "PhysicalElevonAreaFraction", full(Sol.value(PhysicalArea)));

end

function [Panels] = BuildStationPanels(StationIn, StationOut, PanelWidth)
% Build short station panels and preserve the exact requested outer edge.

Edges = StationIn:PanelWidth:StationOut;
if Edges(end) < StationOut
    Edges = [Edges, StationOut];
end

Panels.Inboard = Edges(1:end - 1)';
Panels.Outboard = Edges(2:end)';
Panels.Center = 0.5 * (Panels.Inboard + Panels.Outboard);

end

function [Coeff] = PanelAreaCoefficients(Aircraft, Surface, Panels, MaxStation)
% Area per unit chord fraction for each panel.

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Coeff = zeros(length(Panels.Inboard), 1);

for ipanel = 1:length(Coeff)
    y = linspace(Panels.Inboard(ipanel), Panels.Outboard(ipanel), 5) / MaxStation * HalfSpan;
    LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, y / HalfSpan, "linear", "extrap");
    Coeff(ipanel) = 2 * trapz(y, LocalChord) / Sref;
end

end

function [Coeff] = PanelRollCoefficients(Aircraft, Surface, Panels, MaxStation)
% Roll integral per unit chord fraction for each panel.

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Coeff = zeros(length(Panels.Inboard), 1);

for ipanel = 1:length(Coeff)
    y = linspace(Panels.Inboard(ipanel), Panels.Outboard(ipanel), 5) / MaxStation * HalfSpan;
    LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, y / HalfSpan, "linear", "extrap");
    Coeff(ipanel) = Surface.SectionClDelta * trapz(y, LocalChord .* y);
end

end

function ApplyPitchConstraints(Opti, Aircraft, Case, AreaFraction)
% Apply trim deflection and alpha constraints for one longitudinal case.

[Delta, Alpha] = PitchTrimExpressions(Aircraft, Case, AreaFraction);
Opti.subject_to(Delta <= Case.MaxDeflection);
Opti.subject_to(-Delta <= Case.MaxDeflection);
Opti.subject_to(Alpha <= Case.AlphaMax);
Opti.subject_to(-Alpha <= Case.AlphaMax);

end

function ApplyPullupConstraints(Opti, Aircraft, Case, AreaFraction)
% Apply pull-up final deflection and alpha constraints.

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[DeltaTrim, AlphaTrim, AeroCg, TAS, qbar] = PitchTrimExpressions(Aircraft, Case, AreaFraction);

EtaArea = CaseElevatorEta(Aero, AreaFraction);
CLdelta = Aero.CLdelta * EtaArea;
Cmdelta = AeroCg.Cmdelta * EtaArea;
DeltaCL = (Case.NzFinal - 1) * Case.Mass * g / (qbar * Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g / (2 * TAS ^ 2);

b1 = DeltaCL - Aero.CLq * qhat;
b2 = -Aero.Cmq * qhat;
DetA = Aero.CLalpha * Cmdelta - AeroCg.Cmalpha * CLdelta;
DeltaAlpha = (b1 * Cmdelta - CLdelta * b2) / DetA;
DeltaElevator = (Aero.CLalpha * b2 - AeroCg.Cmalpha * b1) / DetA;

AlphaFinal = AlphaTrim + DeltaAlpha;
DeltaFinal = DeltaTrim + DeltaElevator;
Opti.subject_to(DeltaFinal <= Case.MaxDeflection);
Opti.subject_to(-DeltaFinal <= Case.MaxDeflection);
Opti.subject_to(AlphaFinal <= Case.AlphaMax);
Opti.subject_to(-AlphaFinal <= Case.AlphaMax);

end

function ApplyRollConstraint(Opti, Aircraft, Case, RollIntegral)
% Require the 60-degree bank target to be reachable within the time limit.

Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Case.Alt, 0, Case.VelType, Case.Vel);
qbar = 0.5 * Rho * V ^ 2;
Lp = qbar * Sref * Geom.b ^ 2 * Lat.Clp / (2 * V * Inertia.Ixx);
BankGainPerRollIntegral = abs((2 * V / Geom.b) * ...
    ((2 * 0.85 / (Sref * Geom.b)) / Lat.Clp) * ...
    (Case.TimeLimit + (1 / Lp) * (1 - exp(Lp * Case.TimeLimit))));
RequiredRollIntegral = Case.BankTarget / (Case.MaxDeflection * BankGainPerRollIntegral);

Opti.subject_to(RollIntegral >= RequiredRollIntegral);

end

function [DeltaTrim, AlphaTrim, AeroCg, TAS, qbar] = PitchTrimExpressions(Aircraft, Case, AreaFraction)
% Express linear trim equations as CasADi-compatible scalar algebra.

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
[~, TAS, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Case.Alt, 0, Case.VelType, Case.Vel);

qbar = 0.5 * Rho * TAS ^ 2;
CLtrim = Case.Mass * g * Case.Nz / (qbar * Sref);
DxOverC = Case.XcgMAC - Aero.XrefMAC;

AeroCg = Aero;
AeroCg.Cmalpha = Aero.Cmalpha + Aero.CLalpha * DxOverC;
AeroCg.Cmdelta = Aero.Cmdelta + Aero.CLdelta * DxOverC;
AeroCg.Cm0 = Aero.Cm0 + Aero.CL0 * DxOverC;

EtaArea = CaseElevatorEta(Aero, AreaFraction);
CLdelta = Aero.CLdelta * EtaArea;
Cmdelta = AeroCg.Cmdelta * EtaArea;

DeltaTrim = -(AeroCg.Cm0 * Aero.CLalpha + AeroCg.Cmalpha * CLtrim) / ...
    (Cmdelta * Aero.CLalpha - AeroCg.Cmalpha * CLdelta);
AlphaTrim = (CLtrim - CLdelta * DeltaTrim) / Aero.CLalpha;

end

function [EtaArea] = CaseElevatorEta(Aero, AreaFraction) %#ok<INUSD>
% Keep elevator effectiveness centralized for CasADi expressions.

EtaArea = 0.85 * AreaFraction;

end

function [Segments] = BuildSegmentsFromPanels(Aircraft, Surface, Panels, ChordFractions, MaxStation, Name)
% Convert nonzero optimized panel chords into segment cells.

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Segments = {};

for ipanel = 1:length(ChordFractions)
    if ChordFractions(ipanel) <= 1.0e-4
        continue
    end

    Segment = Surface;
    Segment.Name = Name;
    Segment.YInboard = Panels.Inboard(ipanel) / MaxStation * HalfSpan;
    Segment.YOutboard = Panels.Outboard(ipanel) / MaxStation * HalfSpan;
    Segment.SpanFraction = (Panels.Outboard(ipanel) - Panels.Inboard(ipanel)) / MaxStation;
    Segment.ChordFraction = ChordFractions(ipanel);
    Segment.AreaFraction = PanelAreaCoefficients(Aircraft, Surface, struct( ...
        "Inboard", Panels.Inboard(ipanel), "Outboard", Panels.Outboard(ipanel)), MaxStation) * ChordFractions(ipanel);
    Segments{end + 1, 1} = Segment; %#ok<AGROW>
end

end

function [Trial] = CombineSegments(Surface, Segments, Name)
% Combine panel cells for existing authority-check functions.

if isempty(Segments)
    error("ERROR - OptimizeSharedElevons: %s has no selected panels.", Name);
end

Trial = Surface;
Trial.Name = Name;
Trial.Segments = Segments;
Trial.YInboard = min(cellfun(@(Segment) Segment.YInboard, Segments));
Trial.YOutboard = max(cellfun(@(Segment) Segment.YOutboard, Segments));
Trial.SpanFraction = max(cellfun(@(Segment) Segment.SpanFraction, Segments));
Trial.ChordFraction = max(cellfun(@(Segment) Segment.ChordFraction, Segments));
Trial.AreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, Segments));
Trial.EtaControl = Surface.EtaControl;

end

function [Rudder] = SizeRudder(Aircraft, Case, Surface)
% Size the separate winglet rudder directly from directional authority.

Lat = Aircraft.Specs.Dynamics.Lateral;
RequiredArea = abs(Case.RequiredCn / (Lat.Cndr * Surface.EtaControl * Case.MaxDeflection));
[SpanGrid, ChordGrid] = ndgrid(Surface.SpanFractions(:), Surface.ChordFractions(:));
AreaGrid = SpanGrid .* ChordGrid;
Feasible = AreaGrid >= RequiredArea;

if any(Feasible(:))
    CandidateArea = AreaGrid;
    CandidateArea(~Feasible) = Inf;
    [~, Index] = min(CandidateArea(:));
    Converged = true;
else
    [~, Index] = max(AreaGrid(:));
    Converged = false;
end

Rudder = Surface;
Rudder.SpanFraction = SpanGrid(Index);
Rudder.ChordFraction = ChordGrid(Index);
Rudder.AreaFraction = AreaGrid(Index);
Rudder.EtaControl = Surface.EtaControl;
Rudder.Converged = Converged;
Rudder.SpanFractions = Surface.SpanFractions(:);
Rudder.ChordFractions = Surface.ChordFractions(:);
Rudder.AreaFractions = AreaGrid;
Rudder.Feasible = Feasible;

end

function [Checks] = CheckElevator(Aircraft, Cases, Elevator)
% Group longitudinal checks for optimized shared elevons.

Checks.Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.LongitudinalTrim, Elevator);
Checks.Pullup = DynamicsPkg.CheckPullup(Aircraft, Cases.Pullup, Elevator);
Checks.Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, Cases.TakeoffRotation, Elevator);
Checks.Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.CruiseTrim, Elevator);
Checks.Feasible = Checks.Trim.Feasible && Checks.Pullup.Feasible && ...
    Checks.Rotation.Feasible && Checks.Cruise.Feasible;
Checks.MaxDeflection = max(abs([Checks.Trim.Delta; ...
    Checks.Pullup.DeltaFinal; Checks.Cruise.Delta; Cases.TakeoffRotation.DeltaElevator]));

end

function [Checks] = CheckAileron(Aircraft, Cases, Aileron)
% Check time-to-bank for optimized shared roll panels.

Checks.Bank = DynamicsPkg.CheckTimeToBank(Aircraft, Cases.TimeToBank, Aileron);
Checks.Feasible = Checks.Bank.Feasible;
Checks.MaxDeflection = abs(Checks.Bank.Delta);

end
