function [PitchChordValue, OutboardChordValue, Values] = SolveSharedElevonChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchMax, OutboardMax, OutboardTauControlEff)
%
% [PitchChordValue, OutboardChordValue, Values] = SolveSharedElevonChords(...)
%
% Solve the pitch-only and outboard dual-use panel chord allocation with CasADi.
%

DynamicsPkg.SetupCasadi();
import casadi.*

% OptiProblem is CasADi's nonlinear programming model. The variables below
% are the unknown chord fractions for each allowed spanwise panel.
OptiProblem = Opti();
PitchChord = OptiProblem.variable(length(PitchPanels.Inboard), 1);
OutboardChord = OptiProblem.variable(length(OutboardPanels.Inboard), 1);

% Decision variables are chord fractions for consecutive spanwise panels.
% Bounds enforce local chord limits. The outboard variable is one physical
% dual-use elevon surface: symmetric motion gives pitch, differential motion
% gives roll, so there is no separate roll-only chord allocation.
OptiProblem.subject_to(PitchChord >= 0);
OptiProblem.subject_to(PitchChord <= PitchMax);
OptiProblem.subject_to(OutboardChord >= 0);
OptiProblem.subject_to(OutboardChord <= OutboardMax);

PitchCLdelta = PitchCoeff.CL' * PitchChord + OutboardCoeff.CL' * OutboardChord;
PitchCmdelta = pitch_moment(PitchCoeff, PitchChord) + pitch_moment(OutboardCoeff, OutboardChord);

% The same outboard surface also supplies roll authority when left and right
% panels deflect differentially.
RollIntegral = OutboardCoeff.Roll' * OutboardChord;

% These constraints mirror the numeric Check* validators, but are written as
% CasADi scalar expressions so Ipopt can size the panel chord fractions.
apply_pitch_constraints(OptiProblem, Aircraft, Cases.LongitudinalTrim, PitchCLdelta, PitchCmdelta);
apply_pullup_constraints(OptiProblem, Aircraft, Cases.Pullup, PitchCLdelta, PitchCmdelta);
apply_pitch_constraints(OptiProblem, Aircraft, Cases.CruiseTrim, PitchCLdelta, PitchCmdelta);
apply_roll_constraint(OptiProblem, Aircraft, Cases.TimeToBank, RollIntegral, OutboardTauControlEff);

PhysicalArea = PitchCoeff.Area' * PitchChord + OutboardCoeff.Area' * OutboardChord;
OptiProblem.minimize(PhysicalArea);
OptiProblem.solver('ipopt', struct('print_time', false), struct('print_level', 0));
OptiProblem.set_initial(PitchChord, 0.05);
OptiProblem.set_initial(OutboardChord, 0.05);

% This line runs Ipopt through CasADi and produces the optimized chord
% fractions. Everything before this point only builds the symbolic problem.
Sol = OptiProblem.solve();
PitchChordValue = full(Sol.value(PitchChord));
OutboardChordValue = full(Sol.value(OutboardChord));
Values.PitchArea = PitchCoeff.Area' * PitchChordValue + OutboardCoeff.Area' * OutboardChordValue;
Values.PitchCLdelta = PitchCoeff.CL' * PitchChordValue + OutboardCoeff.CL' * OutboardChordValue;
Values.PitchCmdelta = pitch_moment(PitchCoeff, PitchChordValue) + pitch_moment(OutboardCoeff, OutboardChordValue);
Values.RollArea = OutboardCoeff.Area' * OutboardChordValue;
Values.RollIntegral = OutboardCoeff.Roll' * OutboardChordValue;
Values.PhysicalArea = PitchCoeff.Area' * PitchChordValue + OutboardCoeff.Area' * OutboardChordValue;

end

function [Cmdelta] = pitch_moment(Coeff, ChordFraction)
% Moment derivative for a trailing-edge strip: force scales with f, while
% strip-center x-location adds the f^2 correction.

Cmdelta = Coeff.CmLinear' * ChordFraction + Coeff.CmQuadratic' * (ChordFraction .* ChordFraction);

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

function apply_roll_constraint(Opti, Aircraft, Case, RollIntegral, TauControlEff)
% Require the bank target to be reachable within the time limit.

Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
if isfield(Lat, 'Crlp')
    Crlp = Lat.Crlp;
else
    Crlp = Lat.Clp;
end

[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Case.Alt, 0, Case.VelType, Case.Vel);
qbar = 0.5 * Rho * V ^ 2;
Lp = qbar * Sref * Geom.b ^ 2 * Crlp / (2 * V * Inertia.Ixx);
BankGain = abs((2 * V / Geom.b) * ((2 * TauControlEff / (Sref * Geom.b)) / Crlp) * ...
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
