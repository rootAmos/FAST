function [Check] = CheckDirectionalTrim(Aircraft, Case, Rudder)
%
% [Check] = CheckDirectionalTrim(Aircraft, Case, Rudder)
%
% Check rudder authority against a required yawing-moment coefficient.
%

Lat = Aircraft.Specs.Dynamics.Lateral;

Cndr = Lat.Cndr * Rudder.TauControlEff * Rudder.AreaFraction;
Delta = -Case.RequiredCn / Cndr;

Check.Name = Case.Name;
Check.RequiredCn = Case.RequiredCn;
Check.Delta = Delta;
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);
Check.Feasible = abs(Check.Delta) <= Case.MaxDeflection + Tolerance;

end
