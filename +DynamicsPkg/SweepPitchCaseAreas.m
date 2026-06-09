function [PitchSweep] = SweepPitchCaseAreas(Aircraft, Cases, Surfaces, XcgMAC)
%
% [PitchSweep] = SweepPitchCaseAreas(Aircraft, Cases, Surfaces, XcgMAC)
%
% Re-size pitch authority case-by-case across CG, without the roll constraint.
%

MaxStation = Surfaces.MaxModelHalfSpanStation;
PitchPanels = DynamicsPkg.BuildStationPanels(Surfaces.PitchStationRange(1), ...
    Surfaces.PitchStationRange(2), Surfaces.PanelStationWidth);
OutboardPanels = DynamicsPkg.BuildStationPanels(Surfaces.OutboardStationRange(1), ...
    Surfaces.OutboardStationRange(2), Surfaces.PanelStationWidth);

PitchCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.Elevator, PitchPanels, MaxStation);
DualCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);
PitchMax = max(Surfaces.Elevator.ChordFractions);
DualMax = max(Surfaces.DualElevon.ChordFractions);

XcgMAC = XcgMAC(:);
Area = nan(length(XcgMAC), 4);
Converged = false(size(Area));

for icg = 1:length(XcgMAC)
    SweepCases = Cases;
    SweepCases.LongitudinalTrim.XcgMAC = XcgMAC(icg);
    SweepCases.Pullup.XcgMAC = XcgMAC(icg);
    SweepCases.CruiseTrim.XcgMAC = XcgMAC(icg);
    SweepCases.TakeoffRotation.XcgMAC = XcgMAC(icg);

    [Area(icg, 1), Converged(icg, 1)] = solve_pitch_case(Aircraft, SweepCases.LongitudinalTrim, ...
        "trim", PitchCoeff, DualCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, MaxStation);
    [Area(icg, 2), Converged(icg, 2)] = solve_pitch_case(Aircraft, SweepCases.Pullup, ...
        "pullup", PitchCoeff, DualCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, MaxStation);
    [Area(icg, 3), Converged(icg, 3)] = solve_pitch_case(Aircraft, SweepCases.CruiseTrim, ...
        "trim", PitchCoeff, DualCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, MaxStation);
    [Area(icg, 4), Converged(icg, 4)] = solve_pitch_case(Aircraft, SweepCases.TakeoffRotation, ...
        "rotation", PitchCoeff, DualCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, MaxStation);
end

PitchSweep.XcgMAC = XcgMAC;
PitchSweep.Trim = Area(:, 1);
PitchSweep.Pullup = Area(:, 2);
PitchSweep.Cruise = Area(:, 3);
PitchSweep.Rotation = Area(:, 4);
PitchSweep.MaxAreaFraction = max(Area, [], 2);
PitchSweep.Converged = all(Converged, 2);

end

function [AreaValue, Converged] = solve_pitch_case(Aircraft, Case, CaseType, ...
    PitchCoeff, DualCoeff, PitchPanels, OutboardPanels, PitchMax, DualMax, MaxStation)
% Solve minimum pitch-capable area for one longitudinal case.

DynamicsPkg.SetupCasadi();
import casadi.*

OptiProblem = Opti();
PitchChord = OptiProblem.variable(length(PitchPanels.Inboard), 1);
DualChord = OptiProblem.variable(length(OutboardPanels.Inboard), 1);

OptiProblem.subject_to(PitchChord >= 0);
OptiProblem.subject_to(PitchChord <= PitchMax);
OptiProblem.subject_to(DualChord >= 0);
OptiProblem.subject_to(DualChord <= DualMax);

CLdelta = PitchCoeff.CL' * PitchChord + DualCoeff.CL' * DualChord;
CmdeltaRef = pitch_moment(PitchCoeff, PitchChord) + pitch_moment(DualCoeff, DualChord);

if CaseType == "pullup"
    apply_pullup_constraints(OptiProblem, Aircraft, Case, CLdelta, CmdeltaRef);
elseif CaseType == "rotation"
    apply_rotation_constraint(OptiProblem, Aircraft, Case, CLdelta, CmdeltaRef);
else
    apply_pitch_constraints(OptiProblem, Aircraft, Case, CLdelta, CmdeltaRef);
end

Area = PitchCoeff.Area' * PitchChord + DualCoeff.Area' * DualChord;
OptiProblem.minimize(Area);
OptiProblem.solver('ipopt', struct('print_time', false), struct('print_level', 0));
OptiProblem.set_initial(PitchChord, 0.05);
OptiProblem.set_initial(DualChord, 0.05);

try
    Sol = OptiProblem.solve();
    AreaValue = full(Sol.value(Area));
    Converged = true;
catch
    AreaValue = NaN;
    Converged = false;
end

end

function [Cmdelta] = pitch_moment(Coeff, ChordFraction)
% Moment derivative for a trailing-edge control strip with selected chord
% fraction f: the lift term is linear in f and the center shift adds f^2.

Cmdelta = Coeff.CmLinear' * ChordFraction + Coeff.CmQuadratic' * (ChordFraction .* ChordFraction);

end

function apply_pitch_constraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply trim deflection and alpha limits for one steady pitch case.

[Delta, Alpha] = pitch_trim_expressions(Aircraft, Case, CLdelta, CmdeltaRef);
Opti.subject_to(Delta <= Case.MaxDeflection);
Opti.subject_to(-Delta <= Case.MaxDeflection);
Opti.subject_to(Alpha <= Case.AlphaMax);
Opti.subject_to(-Alpha <= Case.AlphaMax);

end

function apply_pullup_constraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply final pull-up deflection and alpha limits.

g = 9.81; % [m/s^2] standard gravitational acceleration.
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = pitch_trim_expressions(Aircraft, Case, CLdelta, CmdeltaRef);
DeltaCL = (Case.NzFinal - 1) * Case.Mass * g / (qbar * Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g / (2 * TAS ^ 2);

b1 = DeltaCL - Aero.CLq * qhat;
b2 = -Aero.Cmq * qhat;
DetA = Aero.CLalpha * Cmdelta - AeroCg.Cmalpha * CLdelta;
DeltaAlpha = (b1 * Cmdelta - CLdelta * b2) / DetA;
DeltaElevator = (Aero.CLalpha * b2 - AeroCg.Cmalpha * b1) / DetA;

Opti.subject_to(DeltaTrim + DeltaElevator <= Case.MaxDeflection);
Opti.subject_to(-DeltaTrim - DeltaElevator <= Case.MaxDeflection);
Opti.subject_to(AlphaTrim + DeltaAlpha <= Case.AlphaMax);
Opti.subject_to(-AlphaTrim - DeltaAlpha <= Case.AlphaMax);

end

function apply_rotation_constraint(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Require main-gear rotation speed to stay below the target speed.

g = 9.81; % [m/s^2] standard gravitational acceleration.
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Gear = Aircraft.Specs.Dynamics.Gear;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
[~, ~, Rho] = MissionSegsPkg.StdAtm(Case.Alt);

DeltaElevator = Case.DeltaElevator;
DxOverC = Gear.XmlgMAC - Aero.XrefMAC;
CL = Aero.CL0 + Aero.CLalpha * Case.AlphaGround + CLdelta * DeltaElevator;
CmMLG = Aero.Cm0 + Aero.Cmalpha * Case.AlphaGround + ...
    CmdeltaRef * DeltaElevator + CL * DxOverC;

RotationSpeedLimit = Case.V2min - Case.Margin;
MomentArmWeight = Case.Mass * g * (Case.XcgMAC - Gear.XmlgMAC);
Opti.subject_to(MomentArmWeight + RotationSpeedLimit ^ 2 * Rho * Sref * CmMLG >= 0);

end

function [DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = pitch_trim_expressions(Aircraft, Case, CLdelta, CmdeltaRef)
% Express linear trim equations as CasADi-compatible scalar algebra.

g = 9.81; % [m/s^2] standard gravitational acceleration.
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
