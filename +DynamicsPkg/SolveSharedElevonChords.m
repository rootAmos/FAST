function [PitchChordValue, DualChordValue, RollChordValue, Values] = SolveSharedElevonChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchMax, DualMax, RollMax, OutboardMax, MaxStation)
%
% [PitchChordValue, DualChordValue, RollChordValue, Values] = SolveSharedElevonChords(...)
%
% Solve the shared pitch/dual/roll panel chord allocation with CasADi.
%

DynamicsPkg.SetupCasadi();
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

PitchCLdelta = PitchCoeff.CL' * PitchChord + OutboardCoeff.CL' * DualChord;
PitchCmdelta = PitchCoeff.Cm' * PitchChord + OutboardCoeff.Cm' * DualChord;
RollIntegral = OutboardCoeff.Roll' * (DualChord + RollChord);

apply_pitch_constraints(OptiProblem, Aircraft, Cases.LongitudinalTrim, PitchCLdelta, PitchCmdelta);
apply_pullup_constraints(OptiProblem, Aircraft, Cases.Pullup, PitchCLdelta, PitchCmdelta);
apply_pitch_constraints(OptiProblem, Aircraft, Cases.CruiseTrim, PitchCLdelta, PitchCmdelta);
apply_roll_constraint(OptiProblem, Aircraft, Cases.TimeToBank, RollIntegral);

PhysicalArea = PitchCoeff.Area' * PitchChord + OutboardCoeff.Area' * DualChord + OutboardCoeff.Area' * RollChord;
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
Values.PitchArea = PitchCoeff.Area' * PitchChordValue + OutboardCoeff.Area' * DualChordValue;
Values.PitchCLdelta = PitchCoeff.CL' * PitchChordValue + OutboardCoeff.CL' * DualChordValue;
Values.PitchCmdelta = PitchCoeff.Cm' * PitchChordValue + OutboardCoeff.Cm' * DualChordValue;
Values.RollArea = OutboardCoeff.Area' * (DualChordValue + RollChordValue);
Values.RollIntegral = OutboardCoeff.Roll' * (DualChordValue + RollChordValue);
Values.PhysicalArea = PitchCoeff.Area' * PitchChordValue + OutboardCoeff.Area' * DualChordValue + OutboardCoeff.Area' * RollChordValue;

end

function apply_pitch_constraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply trim deflection and alpha constraints for one longitudinal case.

[Delta, Alpha] = pitch_trim_expressions(Aircraft, Case, CLdelta, CmdeltaRef);
Opti.subject_to(Delta <= Case.MaxDeflection);
Opti.subject_to(-Delta <= Case.MaxDeflection);
Opti.subject_to(Alpha <= Case.AlphaMax);
Opti.subject_to(-Alpha <= Case.AlphaMax);

end

function apply_pullup_constraints(Opti, Aircraft, Case, CLdelta, CmdeltaRef)
% Apply pull-up final deflection and alpha constraints.

g = 9.81;
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

function apply_roll_constraint(Opti, Aircraft, Case, RollIntegral)
% Require the bank target to be reachable within the time limit.

Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Case.Alt, 0, Case.VelType, Case.Vel);
qbar = 0.5 * Rho * V ^ 2;
Lp = qbar * Sref * Geom.b ^ 2 * Lat.Clp / (2 * V * Inertia.Ixx);
BankGain = abs((2 * V / Geom.b) * ((2 * 0.85 / (Sref * Geom.b)) / Lat.Clp) * ...
    (Case.TimeLimit + (1 / Lp) * (1 - exp(Lp * Case.TimeLimit))));
Opti.subject_to(RollIntegral >= Case.BankTarget / (Case.MaxDeflection * BankGain));

end

function [DeltaTrim, AlphaTrim, AeroCg, TAS, qbar, Cmdelta] = pitch_trim_expressions(Aircraft, Case, CLdelta, CmdeltaRef)
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
