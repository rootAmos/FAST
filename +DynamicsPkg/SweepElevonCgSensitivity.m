function [Sensitivity] = SweepElevonCgSensitivity(Aircraft, Cases, Elevator, XcgMAC)
%
% [Sensitivity] = SweepElevonCgSensitivity(Aircraft, Cases, Elevator, XcgMAC)
%
% Evaluate the selected pitch-control geometry across a CG sweep.
%

XcgMAC = XcgMAC(:);
npoint = length(XcgMAC);
DeltaTrim = zeros(npoint, 1);
DeltaPullup = zeros(npoint, 1);
DeltaCruise = zeros(npoint, 1);
AlphaTrim = zeros(npoint, 1);
AlphaPullup = zeros(npoint, 1);
AlphaCruise = zeros(npoint, 1);
RotationSpeed = zeros(npoint, 1);
Feasible = false(npoint, 1);

for ipoint = 1:npoint
    TrimCase = Cases.LongitudinalTrim;
    PullupCase = Cases.Pullup;
    CruiseCase = Cases.CruiseTrim;
    RotationCase = Cases.TakeoffRotation;
    TrimCase.XcgMAC = XcgMAC(ipoint);
    PullupCase.XcgMAC = XcgMAC(ipoint);
    CruiseCase.XcgMAC = XcgMAC(ipoint);
    RotationCase.XcgMAC = XcgMAC(ipoint);

    Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, TrimCase, Elevator);
    Pullup = DynamicsPkg.CheckPullup(Aircraft, PullupCase, Elevator);
    Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, CruiseCase, Elevator);
    Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, RotationCase, Elevator);

    DeltaTrim(ipoint) = max(abs(Trim.Delta));
    DeltaPullup(ipoint) = abs(Pullup.DeltaFinal);
    DeltaCruise(ipoint) = max(abs(Cruise.Delta));
    AlphaTrim(ipoint) = max(abs(Trim.Alpha));
    AlphaPullup(ipoint) = abs(Pullup.AlphaFinal);
    AlphaCruise(ipoint) = max(abs(Cruise.Alpha));
    RotationSpeed(ipoint) = Rotation.VR;
    Feasible(ipoint) = Trim.Feasible && Pullup.Feasible && Cruise.Feasible && Rotation.Feasible;
end

Sensitivity.XcgMAC = XcgMAC;
Sensitivity.DeltaTrim = DeltaTrim;
Sensitivity.DeltaPullup = DeltaPullup;
Sensitivity.DeltaCruise = DeltaCruise;
Sensitivity.AlphaTrim = AlphaTrim;
Sensitivity.AlphaPullup = AlphaPullup;
Sensitivity.AlphaCruise = AlphaCruise;
Sensitivity.RotationSpeed = RotationSpeed;
Sensitivity.Feasible = Feasible;

end
