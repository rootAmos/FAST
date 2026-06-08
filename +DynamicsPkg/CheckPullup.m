function [Check] = CheckPullup(Aircraft, Case, Elevator)
%
% [Check] = CheckPullup(Aircraft, Case, Elevator)
%
% Check 1.3g longitudinal pull-up using the paper's Eq. 3.19-3.22.
%

g = 9.81;
Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;

TrimCheck = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Case, Elevator);
Trim = TrimCheck.Trim;

V = Trim.TAS;
if isfield(Trim, 'Control')
    CLdelta = Trim.Control.CLdelta;
    Cmdelta = Trim.Control.Cmdelta;
else
    CLdelta = Aero.CLdelta * Elevator.EtaControl * Elevator.AreaFraction;
    Cmdelta = Trim.Aero.Cmdelta * Elevator.EtaControl * Elevator.AreaFraction;
end

DeltaCL = (Case.NzFinal - 1) * Case.Mass * g ./ (Trim.qbar .* Trim.Sref);
qhat = (Case.NzFinal - 1) * Geom.cbar * g ./ (2 * V .^ 2);

% Eq. 3.21-3.22 solve for additional alpha and elevator deflection.
A = [Aero.CLalpha, CLdelta; Trim.Aero.Cmalpha, Cmdelta];
b = [DeltaCL - Aero.CLq * qhat; -Aero.Cmq * qhat];
x = A \ b;

Check.Name = Case.Name;
Check.DeltaAlpha = x(1);
Check.DeltaElevator = x(2);
Check.AlphaFinal = Trim.AlphaTrim + Check.DeltaAlpha;
Check.DeltaFinal = Trim.DeltaTrim + Check.DeltaElevator;
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);
Check.Feasible = abs(Check.DeltaFinal) <= Case.MaxDeflection + Tolerance && ...
                 abs(Check.AlphaFinal) <= Case.AlphaMax + Tolerance;

end
