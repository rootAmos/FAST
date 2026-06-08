function [Tolerance] = ControlFeasibilityTolerance(Case)
%
% [Tolerance] = ControlFeasibilityTolerance(Case)
%
% Return the feasibility tolerance for control-surface checks.
%

if isfield(Case, 'FeasibilityTolerance')
    Tolerance = Case.FeasibilityTolerance;
else
    Tolerance = 1.0e-4;
end

end
