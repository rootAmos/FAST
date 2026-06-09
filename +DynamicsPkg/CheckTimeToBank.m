function [Check] = CheckTimeToBank(Aircraft, Case, Aileron)
%
% [Check] = CheckTimeToBank(Aircraft, Case, Aileron)
%
% Check time-to-bank using the source-method roll response equations.
%

Lat = Aircraft.Specs.Dynamics.Lateral;
Geom = Aircraft.Specs.Dynamics.Geometry;
Inertia = Aircraft.Specs.Dynamics.Inertia;

[~, V, ~, ~, ~, Rho, ~] = MissionSegsPkg.ComputeFltCon( ...
    Case.Alt, 0, Case.VelType, Case.Vel);

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
qbar = 0.5 * Rho * V ^ 2;
if isfield(Lat, 'CrlDeltaA')
    CrlDeltaA = Lat.CrlDeltaA * Aileron.TauControlEff * Aileron.AreaFraction;
elseif isfield(Lat, 'CrollDeltaA')
    CrlDeltaA = Lat.CrollDeltaA * Aileron.TauControlEff * Aileron.AreaFraction;
else
    CrlDeltaA = Lat.Clda * Aileron.TauControlEff * Aileron.AreaFraction;
end
if isfield(Aileron, 'Segments')
    CrollIntegral = 0;
    for isegment = 1:length(Aileron.Segments)
        CrollIntegral = CrollIntegral + ControlRollIntegral(Aircraft, Aileron.Segments{isegment});
    end
    CrlDeltaA = 2 * Aileron.TauControlEff * CrollIntegral / (Sref * Geom.b);
elseif isfield(Aileron, 'YInboard') && isfield(Aileron, 'YOutboard')
    CrlDeltaA = 2 * Aileron.TauControlEff * ControlRollIntegral(Aircraft, Aileron) / (Sref * Geom.b);
end
if isfield(Lat, 'Crlp')
    Crlp = Lat.Crlp;
else
    Crlp = Lat.Clp;
end
Lp = qbar * Sref * Geom.b ^ 2 * Crlp / (2 * V * Inertia.Ixx);

BankPerDeflection = (2 * V / Geom.b) * (CrlDeltaA / Crlp) * ...
    (Case.TimeLimit + (1 / Lp) * (1 - exp(Lp * Case.TimeLimit)));
DeltaA = Case.BankTarget / abs(BankPerDeflection);
Phi = abs(BankPerDeflection) * Case.MaxDeflection;

Check.Name = Case.Name;
Check.Delta = DeltaA;
Check.Phi = Phi;
Tolerance = DynamicsPkg.ControlFeasibilityTolerance(Case);
Check.Feasible = abs(Check.Delta) <= Case.MaxDeflection + Tolerance;
Check.CrlDeltaA = CrlDeltaA;

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
        SectionClDelta = 3.0;
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
% SectionClDelta is local 2D dcl/d(delta_elevon) [1/rad]; integrating
% c(y)*y gives the dimensional roll-control derivative before Sref*b scaling.
RollIntegral = SectionClDelta * trapz(y, ControlChord .* y);

end
