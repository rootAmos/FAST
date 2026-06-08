function [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% Optimize 0.2-station shared elevon panels with CasADi.
%

if ~isfield(Surfaces, "SharedTrailingEdge") || ~Surfaces.SharedTrailingEdge
    error("ERROR - OptimizeSharedElevons: Surfaces.SharedTrailingEdge must be true.");
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
[PitchClCoeff, PitchCmCoeff] = PanelPitchCoefficients(Aircraft, Surfaces.Elevator, PitchPanels, MaxStation);
[DualClCoeff, DualCmCoeff] = PanelPitchCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);

PitchMax = max(Surfaces.Elevator.ChordFractions);
DualMax = max(Surfaces.DualElevon.ChordFractions);
RollMax = max(Surfaces.Aileron.ChordFractions);
OutboardMax = max(DualMax, RollMax);

[PitchChordValue, DualChordValue, RollChordValue, SolverName, SolverValues] = SolvePanelChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation);

PitchOnlySegments = BuildSegmentsFromPanels(Aircraft, Surfaces.Elevator, PitchPanels, PitchChordValue, MaxStation, "Pitch-only elevon");
DualSegments = BuildSegmentsFromPanels(Aircraft, Surfaces.DualElevon, OutboardPanels, DualChordValue, MaxStation, "Dual-use elevon");
RollOnlySegments = BuildSegmentsFromPanels(Aircraft, Surfaces.Aileron, OutboardPanels, RollChordValue, MaxStation, "Roll-only elevon");

Elevator = CombineSegments(Surfaces.Elevator, [PitchOnlySegments; DualSegments], "Pitch Elevon");
Aileron = CombineSegments(Surfaces.Aileron, [DualSegments; RollOnlySegments], "Roll Elevon");
DualElevon = CombineSegments(Surfaces.DualElevon, DualSegments, "Dual-Use Elevon");
Rudder = SizeRudder(Aircraft, Cases.DirectionalTrim, Surfaces.Rudder);

Elevator.Checks = CheckElevator(Aircraft, Cases, Elevator);
Aileron.Checks = CheckAileron(Aircraft, Cases, Aileron);
Rudder.Checks.Direction = DynamicsPkg.CheckDirectionalTrim(Aircraft, Cases.DirectionalTrim, Rudder);

Elevator.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, PitchOnlySegments));
Aileron.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, RollOnlySegments));
Elevator.MaxDeflection = Elevator.Checks.MaxDeflection;
Aileron.MaxDeflection = Aileron.Checks.MaxDeflection;
Rudder.MaxDeflection = abs(Rudder.Checks.Direction.Delta);

Elevator.Converged = Elevator.Checks.Feasible;
Aileron.Converged = Aileron.Checks.Feasible;
Rudder.Converged = Rudder.Checks.Direction.Feasible;
DualElevon.Converged = true;

Sizing.Elevator = Elevator;
Sizing.Aileron = Aileron;
Sizing.DualElevon = DualElevon;
Sizing.Rudder = Rudder;
Sizing.Converged = Elevator.Converged && Aileron.Converged && Rudder.Converged;
Sizing.AreaFraction = Elevator.PhysicalAreaFraction + DualElevon.AreaFraction + ...
    Aileron.PhysicalAreaFraction + Rudder.AreaFraction;
Sizing.Casadi = struct( ...
    "Solver", SolverName, ...
    "PitchAreaFraction", SolverValues.PitchArea, ...
    "PitchCLdelta", SolverValues.PitchCLdelta, ...
    "PitchCmdelta", SolverValues.PitchCmdelta, ...
    "RollAreaFraction", SolverValues.RollArea, ...
    "PhysicalElevonAreaFraction", SolverValues.PhysicalArea);

end

function [PitchChordValue, DualChordValue, RollChordValue, SolverName, Values] = SolvePanelChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation)
% Use CasADi when available; otherwise solve the same continuous problem with fmincon.

if exist('Opti', 'file') == 2
    [PitchChordValue, DualChordValue, RollChordValue, Values] = SolvePanelChordsCasadi( ...
        Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
        PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation);
    SolverName = "casadi";
else
    [PitchChordValue, DualChordValue, RollChordValue, Values] = SolvePanelChordsFmincon( ...
        Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
        PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation);
    SolverName = "fmincon";
end

end

function [PitchChordValue, DualChordValue, RollChordValue, Values] = SolvePanelChordsFmincon( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation)
% Continuous fallback for systems where CasADi is not installed.

if exist('fmincon', 'file') ~= 2
    error("ERROR - OptimizeSharedElevons: neither CasADi nor fmincon is available.");
end

nPitch = length(PitchCoeff);
nOutboard = length(OutboardCoeff);
nvar = nPitch + 2 * nOutboard;
PitchIndex = 1:nPitch;
DualIndex = nPitch + (1:nOutboard);
RollIndex = nPitch + nOutboard + (1:nOutboard);

lb = zeros(nvar, 1);
ub = [PitchMax * ones(nPitch, 1); DualMax * ones(nOutboard, 1); RollMax * ones(nOutboard, 1)];
A = zeros(nOutboard, nvar);
for i = 1:nOutboard
    A(i, DualIndex(i)) = 1;
    A(i, RollIndex(i)) = 1;
end
b = OutboardMax * ones(nOutboard, 1);
x0 = 0.25 * ub;

Objective = @(x) PhysicalAreaValue(x, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchIndex, DualIndex, RollIndex, MaxStation);
Nonlinear = @(x) PanelConstraints(x, Aircraft, Cases, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchIndex, DualIndex, RollIndex);

Options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', ...
    'MaxFunctionEvaluations', 2.0e4, 'MaxIterations', 400);
x = fmincon(Objective, x0, A, b, [], [], lb, ub, Nonlinear, Options);

PitchChordValue = x(PitchIndex);
DualChordValue = x(DualIndex);
RollChordValue = x(RollIndex);
Values = PanelValues(PitchChordValue, DualChordValue, RollChordValue, PitchCoeff, OutboardCoeff, ...
    RollCoeff, PitchClCoeff, DualClCoeff, PitchCmCoeff, DualCmCoeff);

end

function [PitchChordValue, DualChordValue, RollChordValue, Values] = SolvePanelChordsCasadi( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, RollMax, OutboardMax, MaxStation)
% CasADi solve path for machines with CasADi installed on the MATLAB path.

import casadi.*

OptiProblem = Opti();
PitchChord = OptiProblem.variable(length(PitchPanels.Inboard), 1);
DualChord = OptiProblem.variable(length(OutboardPanels.Inboard), 1);
RollChord = OptiProblem.variable(length(OutboardPanels.Inboard), 1);

OptiProblem.subject_to(PitchChord >= 0);
OptiProblem.subject_to(PitchChord <= PitchMax);
OptiProblem.subject_to(DualChord >= 0);
OptiProblem.subject_to(DualChord <= DualMax);
OptiProblem.subject_to(RollChord >= 0);
OptiProblem.subject_to(RollChord <= RollMax);
OptiProblem.subject_to(DualChord + RollChord <= OutboardMax);

PitchCLdelta = PitchClCoeff' * PitchChord + DualClCoeff' * DualChord;
PitchCmdelta = PitchCmCoeff' * PitchChord + DualCmCoeff' * DualChord;
RollIntegral = RollCoeff' * (DualChord + RollChord);

ApplyPitchConstraints(OptiProblem, Aircraft, Cases.LongitudinalTrim, PitchCLdelta, PitchCmdelta);
ApplyPullupConstraints(OptiProblem, Aircraft, Cases.Pullup, PitchCLdelta, PitchCmdelta);
ApplyPitchConstraints(OptiProblem, Aircraft, Cases.CruiseTrim, PitchCLdelta, PitchCmdelta);
ApplyRollConstraint(OptiProblem, Aircraft, Cases.TimeToBank, RollIntegral);

PhysicalArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord + OutboardCoeff' * RollChord;
CenterBias = 1.0e-5 * (PitchPanels.Center' * PitchChord + OutboardPanels.Center' * DualChord) / MaxStation;
OptiProblem.minimize(PhysicalArea + CenterBias);
OptiProblem.solver('ipopt', struct('print_time', false), struct('print_level', 0));
OptiProblem.set_initial(PitchChord, 0.05);
OptiProblem.set_initial(DualChord, 0.05);
OptiProblem.set_initial(RollChord, 0.05);

Sol = OptiProblem.solve();
PitchChordValue = full(Sol.value(PitchChord));
DualChordValue = full(Sol.value(DualChord));
RollChordValue = full(Sol.value(RollChord));
Values = PanelValues(PitchChordValue, DualChordValue, RollChordValue, PitchCoeff, OutboardCoeff, ...
    RollCoeff, PitchClCoeff, DualClCoeff, PitchCmCoeff, DualCmCoeff);

end

function [CLCoeff, CmCoeff] = PanelPitchCoefficients(Aircraft, Surface, Panels, MaxStation)
% Per-unit-chord pitch derivatives from local trailing-edge moment arms.

Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Geom.b / 2;

CenterLeadingEdgeX = interp1(Surface.ChordEta, Surface.ChordLeadingEdgeX, 0, "linear", "extrap");
Xref = CenterLeadingEdgeX + Aero.XrefMAC * Geom.cbar;

CLCoeff = zeros(length(Panels.Inboard), 1);
CmCoeff = zeros(length(Panels.Inboard), 1);

for ipanel = 1:length(CLCoeff)
    y = linspace(Panels.Inboard(ipanel), Panels.Outboard(ipanel), 5) / MaxStation * HalfSpan;
    Eta = y / HalfSpan;
    LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, Eta, "linear", "extrap");
    LocalTrailingEdgeX = interp1(Surface.ChordEta, Surface.ChordTrailingEdgeX, Eta, "linear", "extrap");
    UnitControlChord = LocalChord;
    ControlCenterX = LocalTrailingEdgeX - 0.5 * UnitControlChord;
    if isfield(Surface, 'SectionClDelta')
        SectionClDelta = Surface.SectionClDelta;
    else
        SectionClDelta = Aero.CLdelta;
    end
    SectionLift = SectionClDelta * Surface.EtaControl * LocalChord;

    CLCoeff(ipanel) = 2 * trapz(y, SectionLift) / Sref;
    CmCoeff(ipanel) = -2 * trapz(y, SectionLift .* (ControlCenterX - Xref)) / (Sref * Geom.cbar);
end

end

function [Objective] = PhysicalAreaValue(x, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchIndex, DualIndex, RollIndex, MaxStation)
% Objective shared by the continuous panel solvers.

PitchChord = x(PitchIndex);
DualChord = x(DualIndex);
RollChord = x(RollIndex);
PhysicalArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord + OutboardCoeff' * RollChord;
CenterBias = 1.0e-5 * (PitchPanels.Center' * PitchChord + OutboardPanels.Center' * DualChord) / MaxStation;
Objective = PhysicalArea + CenterBias;

end

function [c, ceq] = PanelConstraints(x, Aircraft, Cases, RollCoeff, PitchClCoeff, DualClCoeff, ...
    PitchCmCoeff, DualCmCoeff, PitchIndex, DualIndex, RollIndex)
% Nonlinear authority constraints for the fmincon panel solve.

PitchChord = x(PitchIndex);
DualChord = x(DualIndex);
RollChord = x(RollIndex);
PitchCLdelta = PitchClCoeff' * PitchChord + DualClCoeff' * DualChord;
PitchCmdelta = PitchCmCoeff' * PitchChord + DualCmCoeff' * DualChord;
RollIntegral = RollCoeff' * (DualChord + RollChord);

c = [PitchConstraintValues(Aircraft, Cases.LongitudinalTrim, PitchCLdelta, PitchCmdelta); ...
     PullupConstraintValues(Aircraft, Cases.Pullup, PitchCLdelta, PitchCmdelta); ...
     PitchConstraintValues(Aircraft, Cases.CruiseTrim, PitchCLdelta, PitchCmdelta); ...
     RollConstraintValue(Aircraft, Cases.TimeToBank, RollIntegral)];
ceq = [];

end

function [Values] = PanelValues(PitchChord, DualChord, RollChord, PitchCoeff, OutboardCoeff, ...
    RollCoeff, PitchClCoeff, DualClCoeff, PitchCmCoeff, DualCmCoeff)
% Report optimized panel aggregate values.

Values.PitchArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord;
Values.PitchCLdelta = PitchClCoeff' * PitchChord + DualClCoeff' * DualChord;
Values.PitchCmdelta = PitchCmCoeff' * PitchChord + DualCmCoeff' * DualChord;
Values.RollArea = OutboardCoeff' * (DualChord + RollChord);
Values.RollIntegral = RollCoeff' * (DualChord + RollChord);
Values.PhysicalArea = PitchCoeff' * PitchChord + OutboardCoeff' * DualChord + OutboardCoeff' * RollChord;

end

function [c] = PitchConstraintValues(Aircraft, Case, CLdelta, CmdeltaRef)
% Convert trim limits into fmincon c <= 0 form.

[Delta, Alpha] = PitchTrimExpressions(Aircraft, Case, CLdelta, CmdeltaRef);
c = [Delta - Case.MaxDeflection; ...
    -Delta - Case.MaxDeflection; ...
    Alpha - Case.AlphaMax; ...
    -Alpha - Case.AlphaMax];

end

function [c] = PullupConstraintValues(Aircraft, Case, CLdelta, CmdeltaRef)
% Convert pull-up limits into fmincon c <= 0 form.

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = PitchTrimExpressions(Aircraft, Case, CLdelta, CmdeltaRef);
DeltaCL = (Case.NzFinal - 1) * Case.Mass * g / (qbar * Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g / (2 * TAS ^ 2);

b1 = DeltaCL - Aero.CLq * qhat;
b2 = -Aero.Cmq * qhat;
DetA = Aero.CLalpha * Cmdelta - AeroCg.Cmalpha * CLdelta;
DeltaAlpha = (b1 * Cmdelta - CLdelta * b2) / DetA;
DeltaElevator = (Aero.CLalpha * b2 - AeroCg.Cmalpha * b1) / DetA;

AlphaFinal = AlphaTrim + DeltaAlpha;
DeltaFinal = DeltaTrim + DeltaElevator;
c = [DeltaFinal - Case.MaxDeflection; ...
    -DeltaFinal - Case.MaxDeflection; ...
    AlphaFinal - Case.AlphaMax; ...
    -AlphaFinal - Case.AlphaMax];

end

function [c] = RollConstraintValue(Aircraft, Case, RollIntegral)
% Convert time-to-bank authority into fmincon c <= 0 form.

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
c = RequiredRollIntegral - RollIntegral;

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

function ApplyPitchConstraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply trim deflection and alpha constraints for one longitudinal case.

[Delta, Alpha] = PitchTrimExpressions(Aircraft, Case, CLdelta, CmdeltaRef);
Opti.subject_to(Delta <= Case.MaxDeflection);
Opti.subject_to(-Delta <= Case.MaxDeflection);
Opti.subject_to(Alpha <= Case.AlphaMax);
Opti.subject_to(-Alpha <= Case.AlphaMax);

end

function ApplyPullupConstraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply pull-up final deflection and alpha constraints.

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = PitchTrimExpressions(Aircraft, Case, CLdelta, CmdeltaRef);

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

function [DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = PitchTrimExpressions(Aircraft, Case, CLdelta, CmdeltaRef)
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

Cmdelta = CmdeltaRef + CLdelta * DxOverC;

DeltaTrim = -(AeroCg.Cm0 * Aero.CLalpha + AeroCg.Cmalpha * CLtrim) / ...
    (Cmdelta * Aero.CLalpha - AeroCg.Cmalpha * CLdelta);
AlphaTrim = (CLtrim - CLdelta * DeltaTrim) / Aero.CLalpha;

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
    [CLCoeff, CmCoeff] = PanelPitchCoefficients(Aircraft, Surface, struct( ...
        "Inboard", Panels.Inboard(ipanel), "Outboard", Panels.Outboard(ipanel)), MaxStation);
    Segment.CLdeltaEffective = CLCoeff * ChordFractions(ipanel);
    Segment.CmdeltaEffective = CmCoeff * ChordFractions(ipanel);
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
Trial.CLdeltaEffective = sum(cellfun(@(Segment) Segment.CLdeltaEffective, Segments));
Trial.CmdeltaEffective = sum(cellfun(@(Segment) Segment.CmdeltaEffective, Segments));
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
