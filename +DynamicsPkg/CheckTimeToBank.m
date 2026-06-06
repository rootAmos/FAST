function [Check] = CheckTimeToBank(Aircraft, Case, Elevon)
%
% [Check] = CheckTimeToBank(Aircraft, Case, Elevon)
%
% Check time-to-bank using the paper's Eq. 3.6-3.8.
%

Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;

[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon( ...
    Case.Alt, 0, Case.VelType, Case.Vel);

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
qbar = 0.5 * Rho * V ^ 2;
Clda = Lat.Clda * Elevon.EtaControl * Elevon.AreaFraction;
Lp = qbar * Sref * Geom.b ^ 2 * Lat.Clp / (2 * V * Inertia.Ixx);

DeltaA = Case.MaxDeflection;
Phi = (2 * V / Geom.b) * (Clda / Lat.Clp) * DeltaA * ...
      (Case.TimeLimit + (1 / Lp) * (1 - exp(-Lp * Case.TimeLimit)));

Check.Name = Case.Name;
Check.Phi = Phi;
Check.Feasible = abs(Phi) >= Case.BankTarget;

end

