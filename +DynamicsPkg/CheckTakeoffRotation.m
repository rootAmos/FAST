function [Check] = CheckTakeoffRotation(Aircraft, Case, Elevon)
%
% [Check] = CheckTakeoffRotation(Aircraft, Case, Elevon)
%
% Estimate rotation speed using the paper's Eq. 3.23.
%

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Gear = Aircraft.Specs.Dynamics.Gear;

[~, ~, Rho] = MissionSegsPkg.StdAtm(Case.Alt);
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

DxOverC = Gear.XmlgMAC - Aero.XrefMAC;
CL = Aero.CL0 + Aero.CLalpha * Case.AlphaGround + ...
     Aero.CLdelta * Elevon.EtaControl * Elevon.AreaFraction * Case.DeltaElevon;
CmMLG = Aero.Cm0 + Aero.Cmalpha * Case.AlphaGround + ...
        Aero.Cmdelta * Elevon.EtaControl * Elevon.AreaFraction * Case.DeltaElevon + ...
        CL * DxOverC;

VR = sqrt(Case.Mass * g * (Case.XcgMAC - Gear.XmlgMAC) / (-Rho * Sref * CmMLG));

Check.Name = Case.Name;
Check.VR = VR;
Check.CmMLG = CmMLG;
Check.Feasible = VR < (Case.V2min - Case.Margin);

end

