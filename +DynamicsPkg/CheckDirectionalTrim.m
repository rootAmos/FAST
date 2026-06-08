function [Check] = CheckDirectionalTrim(Aircraft, Case, Rudder)
%
% [Check] = CheckDirectionalTrim(Aircraft, Case, Rudder)
%
% Check rudder authority against a required yawing-moment coefficient.
%

Lat = Aircraft.Specs.Dynamics.Lateral;

Cndr = Lat.Cndr * Rudder.EtaControl * Rudder.AreaFraction;
Delta = -Case.RequiredCn / Cndr;

Check.Name = Case.Name;
Check.RequiredCn = Case.RequiredCn;
Check.Delta = Delta;
Check.Feasible = abs(Check.Delta) <= Case.MaxDeflection;

end
