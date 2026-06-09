function [Check] = CheckLongitudinalTrim(Aircraft, Case, Elevator)
%
% [Check] = CheckLongitudinalTrim(Aircraft, Case, Elevator)
%
% Check level-flight trim using the source-method trim equations.
%

TrimCase.Alt = Case.Alt;
TrimCase.VelType = Case.VelType;
TrimCase.Vel = Case.Vel;
TrimCase.Mass = Case.Mass;
TrimCase.Nz = Case.Nz;
TrimCase.XcgMAC = Case.XcgMAC;
TrimCase.MaxDeflection = Case.MaxDeflection;
TrimCase.Elevon = Elevator;

Trim = DynamicsPkg.TrimLongitudinal(Aircraft, TrimCase);
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);

Check.Name = Case.Name;
Check.Trim = Trim;
Check.Alpha = Trim.AlphaTrim;
Check.Delta = Trim.DeltaTrim;
Check.Feasible = all(abs(Check.Delta) <= Case.MaxDeflection + Tolerance) && ...
                 all(abs(Check.Alpha) <= Case.AlphaMax + Tolerance);

end
