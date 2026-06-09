function [Trim] = TrimLongitudinal(Aircraft, TrimCase)
%
% [Trim] = TrimLongitudinal(Aircraft, TrimCase)
%
% Compute longitudinal trim angle of attack and elevon/elevator deflection
% from Nelson's linear trim equations. This is the sizing-level bridge from
% FAST aircraft outputs to a later 6DOF or Simulink trim model.
%
% INPUTS:
%     Aircraft - FAST aircraft structure.
%                size/type/units: 1-by-1 / struct / []
%
%     TrimCase - trim condition and optional aero override structure.
%                size/type/units: 1-by-1 / struct / []
%
% OUTPUTS:
%     Trim     - structure with trim state, residuals, and feasibility.
%                size/type/units: 1-by-1 / struct / []
%

g = 9.81;

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
Mass = TrimCase.Mass(:);
Nz = TrimCase.Nz(:);
Alt = TrimCase.Alt(:);
VelType = TrimCase.VelType;
Vel = TrimCase.Vel(:);
MaxDeflection = TrimCase.MaxDeflection;

[~, TAS, Mach, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon(Alt, 0, VelType, Vel);

qbar = 0.5 .* Rho .* TAS .^ 2;
CLtrim = Mass .* g .* Nz ./ (qbar .* Sref);

Aero = Aircraft.Specs.Dynamics.Longitudinal;

XrefMAC = Aero.XrefMAC;
XcgMAC = TrimCase.XcgMAC;
DxOverC = XcgMAC - XrefMAC;

% Nelson trim equations hold for moments about the CG. If aero data is
% supplied about another reference point, shift it first using
% Cm_alpha,cg = Cm_alpha,ref + CL_alpha * (x_cg - x_ref) / cbar and
% Cm0,cg = Cm0,ref + CL0 * (x_cg - x_ref) / cbar. The control derivative
% is shifted the same way because Eq. 2.47 also uses Cm_delta.
AeroRef = Aero;
Aero.Cmalpha = AeroRef.Cmalpha + AeroRef.CLalpha .* DxOverC;
Aero.Cmdelta = AeroRef.Cmdelta + AeroRef.CLdelta .* DxOverC;
Aero.Cm0 = AeroRef.Cm0 + AeroRef.CL0 .* DxOverC;

Elevon = TrimCase.Elevon;
if isfield(Elevon, 'CLdeltaEffective') && isfield(Elevon, 'CmdeltaEffective')
    CLdelta = Elevon.CLdeltaEffective;
    Cmdelta = Elevon.CmdeltaEffective + Elevon.CLdeltaEffective .* DxOverC;
else
    CLdelta = Aero.CLdelta .* Elevon.TauControlEff .* Elevon.AreaFraction;
    Cmdelta = Aero.Cmdelta .* Elevon.TauControlEff .* Elevon.AreaFraction;
end

% Nelson, Eq. 2.51: elevator/elevon angle required to trim at CLtrim.
DeltaTrim = -(Aero.Cm0 .* Aero.CLalpha + Aero.Cmalpha .* CLtrim) ./ ...
             (Cmdelta .* Aero.CLalpha - Aero.Cmalpha .* CLdelta);

% Nelson, Eq. 2.50: angle of attack at trim after accounting for control lift.
% This enforces CLtrim = CLalpha * alpha_trim + CLdelta * delta_trim.
AlphaTrim = (CLtrim - CLdelta .* DeltaTrim) ./ Aero.CLalpha;

% Nelson, Eq. 2.47: trim requires the pitching-moment coefficient to be zero.
CmResidual = Aero.Cm0 + Aero.Cmalpha .* AlphaTrim + Cmdelta .* DeltaTrim;

% Trim drag is charged as a quadratic drag increment from elevon deflection.
CDclean = Aero.CD0 + Aero.K .* CLtrim .^ 2;
CDcontrol = Aero.CDdelta .* Elevon.AreaFraction .* DeltaTrim .^ 2;
CDtrim = CDclean + CDcontrol;

Trim.Alt = Alt;
Trim.TAS = TAS;
Trim.Mach = Mach;
Trim.Rho = Rho;
Trim.qbar = qbar;
Trim.Mass = Mass;
Trim.Sref = Sref;
Trim.CLtrim = CLtrim;
Trim.AlphaTrim = AlphaTrim;
Trim.DeltaTrim = DeltaTrim;
Trim.CmResidual = CmResidual;
Trim.CDclean = CDclean;
Trim.CDcontrol = CDcontrol;
Trim.CDtrim = CDtrim;
Trim.L_D_clean = CLtrim ./ CDclean;
Trim.L_D = CLtrim ./ CDtrim;
Trim.Feasible = abs(DeltaTrim) <= MaxDeflection;
Trim.MaxDeflection = MaxDeflection;
Trim.Elevon = Elevon;
Trim.Control.CLdelta = CLdelta;
Trim.Control.Cmdelta = Cmdelta;
Trim.Aero = Aero;
Trim.AeroRef = AeroRef;
Trim.XrefMAC = XrefMAC;
Trim.XcgMAC = XcgMAC;
Trim.DxOverC = DxOverC;

end
