function [Check] = CheckTakeoffRotation(Aircraft, Case, Elevator)
%
% [Check] = CheckTakeoffRotation(Aircraft, Case, Elevator)
%
% Estimate rotation speed from the main-gear pitching moment balance.
%

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Gear = Aircraft.Specs.Dynamics.Gear;

[~, ~, Rho] = MissionSegsPkg.StdAtm(Case.Alt);
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

if isfield(Case, 'DeltaElevator')
    DeltaElevator = Case.DeltaElevator;
else
    DeltaElevator = Case.DeltaElevon;
end

DxOverC = Gear.XmlgMAC - Aero.XrefMAC;
if isfield(Elevator, 'CLdeltaEffective') && isfield(Elevator, 'CmdeltaEffective')
    CLdelta = Elevator.CLdeltaEffective;
    CmdeltaRef = Elevator.CmdeltaEffective;
else
    CLdelta = Aero.CLdelta * Elevator.TauControlEff * Elevator.AreaFraction;
    CmdeltaRef = Aero.Cmdelta * Elevator.TauControlEff * Elevator.AreaFraction;
end

CL = Aero.CL0 + Aero.CLalpha * Case.AlphaGround + ...
     CLdelta * DeltaElevator;
CmMLG = Aero.Cm0 + Aero.Cmalpha * Case.AlphaGround + ...
        CmdeltaRef * DeltaElevator + ...
        CL * DxOverC;

VR = sqrt(Case.Mass * g * (Case.XcgMAC - Gear.XmlgMAC) / (-Rho * Sref * CmMLG));

Check.Name = Case.Name;
Check.VR = VR;
Check.CmMLG = CmMLG;
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);
Check.Feasible = VR <= (Case.V2min - Case.Margin) + Tolerance;

end
