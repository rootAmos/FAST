function [Sizing] = OptimizeElevonsCasadi(Aircraft, Cases, Elevon)
%
% [Sizing] = OptimizeElevonsCasadi(Aircraft, Cases, Elevon)
%
% Minimize grouped elevon area with CasADi/IPOPT.
%

if isempty(which("casadi.Opti"))
    CasadiPath = "C:\Program Files\MATLAB\casadi-3.7.2-windows64-matlab2018b";
    if isfolder(CasadiPath)
        addpath(CasadiPath);
    end
end

if isempty(which("casadi.Opti"))
    error("ERROR - OptimizeElevonsCasadi: CasADi is not on the MATLAB path.");
end

import casadi.*

g = 9.81;
DeflectionReserve = deg2rad(0.02);

Opt = Opti();
SpanFraction = Opt.variable();
ChordFraction = Opt.variable();
AreaFraction = SpanFraction * ChordFraction;
EtaArea = Elevon.EtaControl * AreaFraction;

Opt.subject_to(SpanFraction >= min(Elevon.SpanFractions));
Opt.subject_to(SpanFraction <= max(Elevon.SpanFractions));
Opt.subject_to(ChordFraction >= min(Elevon.ChordFractions));
Opt.subject_to(ChordFraction <= max(Elevon.ChordFractions));

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;
Gear = Aircraft.Specs.Dynamics.Gear;

TrimCases = [Cases.LongitudinalTrim, Cases.CruiseTrim];
for icase = 1:length(TrimCases)
    Case = TrimCases(icase);
    [~, TAS, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon( ...
        Case.Alt, 0, Case.VelType, Case.Vel);

    qbar = 0.5 * Rho * TAS ^ 2;
    CLtrim = Case.Mass * g * Case.Nz / (qbar * Sref);

    DxOverC = Case.XcgMAC - Aero.XrefMAC;
    Cmalpha = Aero.Cmalpha + Aero.CLalpha * DxOverC;
    Cmdelta = Aero.Cmdelta + Aero.CLdelta * DxOverC;
    Cm0 = Aero.Cm0 + Aero.CL0 * DxOverC;

    CLdelta = Aero.CLdelta * EtaArea;
    CmdeltaEff = Cmdelta * EtaArea;

    % Nelson Eq. 2.50-2.51, shifted to the case CG first.
    DeltaTrim = -(Cm0 * Aero.CLalpha + Cmalpha * CLtrim) / ...
                 (CmdeltaEff * Aero.CLalpha - Cmalpha * CLdelta);
    AlphaTrim = (CLtrim - CLdelta * DeltaTrim) / Aero.CLalpha;

    DeflectionLimit = Case.MaxDeflection - DeflectionReserve;
    Opt.subject_to(DeltaTrim <= DeflectionLimit);
    Opt.subject_to(DeltaTrim >= -DeflectionLimit);
    Opt.subject_to(AlphaTrim <= Case.AlphaMax);
    Opt.subject_to(AlphaTrim >= -Case.AlphaMax);
end

Case = Cases.Pullup;
[~, TAS, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon( ...
    Case.Alt, 0, Case.VelType, Case.Vel);
qbar = 0.5 * Rho * TAS ^ 2;
CLtrim = Case.Mass * g * Case.Nz / (qbar * Sref);

DxOverC = Case.XcgMAC - Aero.XrefMAC;
Cmalpha = Aero.Cmalpha + Aero.CLalpha * DxOverC;
Cmdelta = Aero.Cmdelta + Aero.CLdelta * DxOverC;
Cm0 = Aero.Cm0 + Aero.CL0 * DxOverC;
CLdelta = Aero.CLdelta * EtaArea;
CmdeltaEff = Cmdelta * EtaArea;

DeltaTrim = -(Cm0 * Aero.CLalpha + Cmalpha * CLtrim) / ...
             (CmdeltaEff * Aero.CLalpha - Cmalpha * CLdelta);
AlphaTrim = (CLtrim - CLdelta * DeltaTrim) / Aero.CLalpha;

DeltaCL = (Case.NzFinal - 1) * Case.Mass * g / (qbar * Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g / (2 * TAS ^ 2);
b1 = DeltaCL - Aero.CLq * qhat;
b2 = -Aero.Cmq * qhat;
detA = Aero.CLalpha * CmdeltaEff - CLdelta * Cmalpha;

% Paper Eq. 3.21-3.22, written explicitly for CasADi.
DeltaAlpha = (b1 * CmdeltaEff - CLdelta * b2) / detA;
DeltaElevon = (Aero.CLalpha * b2 - b1 * Cmalpha) / detA;
AlphaFinal = AlphaTrim + DeltaAlpha;
DeltaFinal = DeltaTrim + DeltaElevon;

DeflectionLimit = Case.MaxDeflection - DeflectionReserve;
Opt.subject_to(DeltaFinal <= DeflectionLimit);
Opt.subject_to(DeltaFinal >= -DeflectionLimit);
Opt.subject_to(AlphaFinal <= Case.AlphaMax);
Opt.subject_to(AlphaFinal >= -Case.AlphaMax);

Case = Cases.TimeToBank;
[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon( ...
    Case.Alt, 0, Case.VelType, Case.Vel);
qbar = 0.5 * Rho * V ^ 2;
Lp = qbar * Sref * Geom.b ^ 2 * Lat.Clp / (2 * V * Inertia.Ixx);
BankScale = abs((2 * V / Geom.b) * (Lat.Clda / Lat.Clp) * ...
    Case.MaxDeflection * (Case.TimeLimit + (1 / Lp) * ...
    (1 - exp(-Lp * Case.TimeLimit))));

% Paper Eq. 3.6-3.8; area enters through Clda.
Opt.subject_to(BankScale * EtaArea >= Case.BankTarget);

Case = Cases.TakeoffRotation;
[~, ~, Rho] = MissionSegsPkg.StdAtm(Case.Alt);
DxOverC = Gear.XmlgMAC - Aero.XrefMAC;
CL = Aero.CL0 + Aero.CLalpha * Case.AlphaGround + ...
     Aero.CLdelta * EtaArea * Case.DeltaElevon;
CmMLG = Aero.Cm0 + Aero.Cmalpha * Case.AlphaGround + ...
        Aero.Cmdelta * EtaArea * Case.DeltaElevon + CL * DxOverC;
VR2 = Case.Mass * g * (Case.XcgMAC - Gear.XmlgMAC) / (-Rho * Sref * CmMLG);

% Paper Eq. 3.23.
Opt.subject_to(VR2 <= (Case.V2min - Case.Margin) ^ 2);

Opt.minimize(AreaFraction);
Opt.set_initial(SpanFraction, median(Elevon.SpanFractions));
Opt.set_initial(ChordFraction, median(Elevon.ChordFractions));
Opt.solver('ipopt', struct('print_time', 0), struct('print_level', 0));

Sol = Opt.solve();

Trial = Elevon;
Trial.SpanFraction = full(Sol.value(SpanFraction));
Trial.ChordFraction = full(Sol.value(ChordFraction));
Trial.AreaFraction = full(Sol.value(AreaFraction));

Checks.Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.LongitudinalTrim, Trial);
Checks.Pullup = DynamicsPkg.CheckPullup(Aircraft, Cases.Pullup, Trial);
Checks.Bank = DynamicsPkg.CheckTimeToBank(Aircraft, Cases.TimeToBank, Trial);
Checks.Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, Cases.TakeoffRotation, Trial);
Checks.Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.CruiseTrim, Trial);

Sizing.SpanFraction = Trial.SpanFraction;
Sizing.ChordFraction = Trial.ChordFraction;
Sizing.AreaFraction = Trial.AreaFraction;
Sizing.EtaControl = Elevon.EtaControl;
Sizing.Converged = Checks.Trim.Feasible && Checks.Pullup.Feasible && ...
                   Checks.Bank.Feasible && Checks.Rotation.Feasible && ...
                   Checks.Cruise.Feasible;
Sizing.Checks = Checks;
Sizing.SolverStats = Sol.stats();

end
