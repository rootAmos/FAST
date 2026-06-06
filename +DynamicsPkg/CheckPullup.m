function [Check] = CheckPullup(Aircraft, Case, Elevon)
%
% [Check] = CheckPullup(Aircraft, Case, Elevon)
%
% Check 1.3g longitudinal pull-up using the paper's Eq. 3.19-3.22.
%

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;

TrimCheck = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Case, Elevon);
Trim = TrimCheck.Trim;

V = Trim.TAS;
CLdelta = Aero.CLdelta * Elevon.EtaControl * Elevon.AreaFraction;
Cmdelta = Trim.Aero.Cmdelta * Elevon.EtaControl * Elevon.AreaFraction;

DeltaCL = (Case.NzFinal - 1) * Case.Mass * g ./ (Trim.qbar .* Trim.Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g ./ (2 * V .^ 2);

% Eq. 3.21-3.22 solve for additional alpha and elevon deflection.
A = [Aero.CLalpha, CLdelta; Trim.Aero.Cmalpha, Cmdelta];
b = [DeltaCL - Aero.CLq * qhat; -Aero.Cmq * qhat];
x = A \ b;

Check.Name = Case.Name;
Check.DeltaAlpha = x(1);
Check.DeltaElevon = x(2);
Check.AlphaFinal = Trim.AlphaTrim + Check.DeltaAlpha;
Check.DeltaFinal = Trim.DeltaTrim + Check.DeltaElevon;
Check.Feasible = abs(Check.DeltaFinal) <= Case.MaxDeflection && ...
                 abs(Check.AlphaFinal) <= Case.AlphaMax;

end

