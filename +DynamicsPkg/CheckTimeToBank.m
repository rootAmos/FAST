function [Check] = CheckTimeToBank(Aircraft, Case, Aileron)
%
% [Check] = CheckTimeToBank(Aircraft, Case, Aileron)
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
Clda = Lat.Clda * Aileron.EtaControl * Aileron.AreaFraction;
if isfield(Aileron, 'Segments')
    CldaIntegral = 0;
    for isegment = 1:length(Aileron.Segments)
        CldaIntegral = CldaIntegral + ControlRollIntegral(Aircraft, Aileron.Segments{isegment});
    end
    Clda = 2 * Aileron.EtaControl * CldaIntegral / (Sref * Geom.b);
elseif isfield(Aileron, 'YInboard') && isfield(Aileron, 'YOutboard')
    Clda = 2 * Aileron.EtaControl * ControlRollIntegral(Aircraft, Aileron) / (Sref * Geom.b);
end
Lp = qbar * Sref * Geom.b ^ 2 * Lat.Clp / (2 * V * Inertia.Ixx);

BankPerDeflection = (2 * V / Geom.b) * (Clda / Lat.Clp) * ...
    (Case.TimeLimit + (1 / Lp) * (1 - exp(Lp * Case.TimeLimit)));
DeltaA = Case.BankTarget / abs(BankPerDeflection);
Phi = abs(BankPerDeflection) * Case.MaxDeflection;

Check.Name = Case.Name;
Check.Delta = DeltaA;
Check.Phi = Phi;
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);
Check.Feasible = abs(Check.Delta) <= Case.MaxDeflection + Tolerance;
Check.Clda = Clda;

if isfield(Aileron, 'YInboard')
    Check.YInboard = Aileron.YInboard;
    Check.YOutboard = Aileron.YOutboard;
end

end

function [RollIntegral] = ControlRollIntegral(Aircraft, Surface)
% Integrate local chord times moment arm for placed roll-control segments.

Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;

    if isfield(Surface, 'SectionClDelta')
        SectionClDelta = Surface.SectionClDelta;
    else
        SectionClDelta = 2.5;
    end

    y = linspace(Surface.YInboard, Surface.YOutboard, 25);
    if isfield(Surface, 'ChordEta') && isfield(Surface, 'ChordLength')
        LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, y / (Geom.b / 2), 'linear', 'extrap');
    elseif isfield(Surface, 'ReferenceChord')
        LocalChord = Surface.ReferenceChord * ones(size(y));
    else
        LocalChord = (Sref / Geom.b) * ones(size(y));
    end

ControlChord = Surface.ChordFraction * LocalChord;
RollIntegral = SectionClDelta * trapz(y, ControlChord .* y);

end
