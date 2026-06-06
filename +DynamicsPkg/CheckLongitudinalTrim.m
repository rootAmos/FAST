function [Check] = CheckLongitudinalTrim(Aircraft, Case, Elevon)
%
% [Check] = CheckLongitudinalTrim(Aircraft, Case, Elevon)
%
% Check level-flight trim using the paper's Eq. 3.1-3.5.
%

TrimCase.Alt = Case.Alt;
TrimCase.VelType = Case.VelType;
TrimCase.Vel = Case.Vel;
TrimCase.Mass = Case.Mass;
TrimCase.Nz = Case.Nz;
TrimCase.XcgMAC = Case.XcgMAC;
TrimCase.MaxDeflection = Case.MaxDeflection;
TrimCase.Elevon = Elevon;

Trim = DynamicsPkg.TrimLongitudinal(Aircraft, TrimCase);

Check.Name = Case.Name;
Check.Trim = Trim;
Check.Alpha = Trim.AlphaTrim;
Check.Delta = Trim.DeltaTrim;
Check.Feasible = all(abs(Check.Delta) <= Case.MaxDeflection) && ...
                 all(abs(Check.Alpha) <= Case.AlphaMax);

end

