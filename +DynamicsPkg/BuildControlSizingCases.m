function [Cases] = BuildControlSizingCases(Aircraft)
%
% [Cases] = BuildControlSizingCases(Aircraft)
%
% Build the named feasibility cases used by the BWB control sizing method.
%

g = 9.81;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
AltCrs = Aircraft.Specs.Performance.Alts.Crs;
MTOW = Aircraft.Specs.Weight.MTOW;
MLW = Aircraft.Specs.Weight.MLW;

Aero = Aircraft.Specs.Dynamics.Longitudinal;

[~, ~, RhoSL] = MissionSegsPkg.StdAtm(0);
[~, Vcruise, ~, ~, ~, ~, ~] = MissionSegsPkg.ComputeFltCon( ...
    AltCrs, 0, "Mach", Aircraft.Specs.Performance.Vels.Crs);

Vapp = sqrt(2 * MLW * g / (RhoSL * Sref * (0.85 * Aero.CLmaxLnd)));
V2min = 1.13 * sqrt(2 * MTOW * g / (RhoSL * Sref * Aero.CLmaxTko));

Cases.LongitudinalTrim.Name = "Longitudinal trim";
Cases.LongitudinalTrim.Rationale = "Forward CG, approach speed, MLW.";
Cases.LongitudinalTrim.Alt = 0;
Cases.LongitudinalTrim.VelType = "TAS";
Cases.LongitudinalTrim.Vel = Vapp;
Cases.LongitudinalTrim.Mass = MLW;
Cases.LongitudinalTrim.Nz = 1.0;
Cases.LongitudinalTrim.XcgMAC = Aircraft.Specs.Dynamics.CG.ForwardMAC;
Cases.LongitudinalTrim.MaxDeflection = Aero.DeltaMax;
Cases.LongitudinalTrim.AlphaMax = Aero.AlphaMax;

Cases.Pullup = Cases.LongitudinalTrim;
Cases.Pullup.Name = "Longitudinal pull-up";
Cases.Pullup.Rationale = "1.3g pull-up at forward CG, approach speed, MLW.";
Cases.Pullup.NzFinal = 1.3;

Cases.TimeToBank.Name = "Time to bank";
Cases.TimeToBank.Rationale = "Aft CG, approach speed, reach 60 deg bank in 7 sec.";
Cases.TimeToBank.Alt = 0;
Cases.TimeToBank.VelType = "TAS";
Cases.TimeToBank.Vel = Vapp;
Cases.TimeToBank.Mass = MLW;
Cases.TimeToBank.XcgMAC = Aircraft.Specs.Dynamics.CG.AftMAC;
Cases.TimeToBank.BankTarget = deg2rad(60);
Cases.TimeToBank.TimeLimit = 7;
Cases.TimeToBank.MaxDeflection = Aero.DeltaMax;

Cases.TakeoffRotation.Name = "Takeoff rotation";
Cases.TakeoffRotation.Rationale = "Forward CG, MTOW, max nose-up elevator.";
Cases.TakeoffRotation.Alt = 0;
Cases.TakeoffRotation.Mass = MTOW;
Cases.TakeoffRotation.XcgMAC = Aircraft.Specs.Dynamics.CG.ForwardMAC;
if isfield(Aircraft.Specs.Dynamics.Gear, 'AlphaGround')
    Cases.TakeoffRotation.AlphaGround = Aircraft.Specs.Dynamics.Gear.AlphaGround;
else
    Cases.TakeoffRotation.AlphaGround = 0;
end
Cases.TakeoffRotation.DeltaElevator = -Aero.DeltaMax;
Cases.TakeoffRotation.V2min = V2min;
Cases.TakeoffRotation.Margin = 5;

Cases.DirectionalTrim.Name = "Directional trim";
Cases.DirectionalTrim.Rationale = "Rudder authority against required yawing moment.";
Cases.DirectionalTrim.RequiredCn = 0.015;
Cases.DirectionalTrim.MaxDeflection = Aero.DeltaMax;

Cases.CruiseTrim.Name = "Cruise trim";
Cases.CruiseTrim.Rationale = "Nominal cruise trim and drag check.";
Cases.CruiseTrim.Alt = AltCrs;
Cases.CruiseTrim.VelType = "TAS";
Cases.CruiseTrim.Vel = Vcruise;
Cases.CruiseTrim.Mass = 0.80 * MTOW;
Cases.CruiseTrim.Nz = 1.0;
Cases.CruiseTrim.XcgMAC = Aircraft.Specs.Dynamics.CG.AftMAC;
Cases.CruiseTrim.MaxDeflection = Aero.DeltaMax;
Cases.CruiseTrim.AlphaMax = Aero.AlphaMax;

end
