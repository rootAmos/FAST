function [TrimCase] = SweepTrimEnvelope(Aircraft)
%
% [TrimCase] = SweepTrimEnvelope(Aircraft)
%
% Build named trim cases from simple flight-mechanics constraints. This
% gives SizeElevon a repeatable envelope without requiring a mission table.
%
% INPUTS:
%     Aircraft - FAST aircraft structure.
%                size/type/units: 1-by-1 / struct / []
%
% OUTPUTS:
%     TrimCase - structure that can be passed into SizeElevon.
%                size/type/units: 1-by-1 / struct / []
%

g = 9.81;

AltCrs = Aircraft.Specs.Performance.Alts.Crs;
MTOW = Aircraft.Specs.Weight.MTOW;

if isfield(Aircraft.Specs.Aero, "S") && ~isnan(Aircraft.Specs.Aero.S)
    Sref = Aircraft.Specs.Aero.S;
else
    Sref = MTOW / Aircraft.Specs.Aero.W_S.SLS;
end

CLmaxTko = 1.8;
CLmaxLnd = 1.9;
if isfield(Aircraft.Specs, "Dynamics") && isfield(Aircraft.Specs.Dynamics, "Longitudinal")
    Aero = Aircraft.Specs.Dynamics.Longitudinal;
    if isfield(Aero, "CLmaxTko")
        CLmaxTko = Aero.CLmaxTko;
    end
    if isfield(Aero, "CLmaxLnd")
        CLmaxLnd = Aero.CLmaxLnd;
    end
end

Cases = struct( ...
    "Name", "", ...
    "Rationale", "", ...
    "Alt", 0, ...
    "Mass", 0, ...
    "Nz", 1, ...
    "CLtarget", NaN);

Cases(1).Name = "Takeoff rotation";
Cases(1).Rationale = "High weight, low speed, and nose-up authority.";
Cases(1).Alt = 0;
Cases(1).Mass = MTOW;
Cases(1).Nz = 1.00;
Cases(1).CLtarget = 0.80 * CLmaxTko;

Cases(2).Name = "Approach near stall";
Cases(2).Rationale = "Low-speed trim with limited remaining CL margin.";
Cases(2).Alt = 0;
Cases(2).Mass = 0.86 * MTOW;
Cases(2).Nz = 1.00;
Cases(2).CLtarget = 0.85 * CLmaxLnd;

Cases(3).Name = "Low-speed turn";
Cases(3).Rationale = "Loaded near-stall case, usually harder than 1g approach.";
Cases(3).Alt = 0;
Cases(3).Mass = 0.86 * MTOW;
Cases(3).Nz = 1.50;
Cases(3).CLtarget = 0.90 * CLmaxLnd;

Cases(4).Name = "Cruise trim";
Cases(4).Rationale = "Nominal design-point trim and trim-drag check.";
Cases(4).Alt = AltCrs;
Cases(4).Mass = 0.80 * MTOW;
Cases(4).Nz = 1.00;

Cases(5).Name = "Cruise maneuver";
Cases(5).Rationale = "Moderate load factor at cruise dynamic pressure.";
Cases(5).Alt = AltCrs;
Cases(5).Mass = 0.80 * MTOW;
Cases(5).Nz = 1.30;

CaseName = string({Cases.Name})';
Rationale = string({Cases.Rationale})';
Alt = [Cases.Alt]';
Mass = [Cases.Mass]';
Nz = [Cases.Nz]';
CLtarget = [Cases.CLtarget]';

[~, ~, RhoLow] = MissionSegsPkg.StdAtm(Alt(1:3));
Vel = zeros(size(Alt));
Vel(1:3) = sqrt(2 .* Mass(1:3) .* g .* Nz(1:3) ./ (RhoLow .* Sref .* CLtarget(1:3)));

[~, TASCrs, ~, ~, ~, ~, ~] = MissionSegsPkg.ComputeFltCon( ...
    AltCrs, 0, "Mach", Aircraft.Specs.Performance.Vels.Crs);
Vel(4:5) = TASCrs;

[~, ~, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Alt, 0, "TAS", Vel);
CLactual = Mass .* g .* Nz ./ (0.5 .* Rho .* Vel .^ 2 .* Sref);

TrimCase.CaseName = CaseName;
TrimCase.Rationale = Rationale;
TrimCase.Alt = Alt;
TrimCase.VelType = "TAS";
TrimCase.Vel = Vel;
TrimCase.Mass = Mass;
TrimCase.Nz = Nz;
TrimCase.CLtarget = CLactual;
TrimCase.Assumptions.CLmaxTko = CLmaxTko;
TrimCase.Assumptions.CLmaxLnd = CLmaxLnd;
TrimCase.MaxDeflection = deg2rad(25);

end
