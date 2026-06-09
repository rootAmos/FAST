function [Coeff] = PanelCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% [Coeff] = PanelCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% Integrate area, roll, and pitch effectiveness for placed spanwise panels.
%

Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Geom.b / 2;

Y = panel_y_stations(Panels, MaxStation, HalfSpan);
Eta = Y / HalfSpan;

% Interpolate the scaled X-48 chord distribution at each quadrature station.
% This ties panel area and authority to the BWB planform instead of using a
% rectangular reference chord.
LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, Eta, "linear", "extrap");
Coeff.Area = 2 * trapz_rows(Y, LocalChord) / Sref;
Coeff.Roll = Surface.SectionClDelta * trapz_rows(Y, LocalChord .* Y);

% Pitch effectiveness uses a trailing-edge control strip. For chord fraction
% f, the strip center is x_te - 0.5*f*c, so the moment derivative is not
% purely linear in f. Coeff.CmLinear multiplies f; Coeff.CmQuadratic
% multiplies f^2.
CenterLeadingEdgeX = interp1(Surface.ChordEta, Surface.ChordLeadingEdgeX, 0, "linear", "extrap");
LocalTrailingEdgeX = interp1(Surface.ChordEta, Surface.ChordTrailingEdgeX, Eta, "linear", "extrap");
Xref = CenterLeadingEdgeX + Aero.XrefMAC * Geom.cbar;
if isfield(Surface, 'SectionClDelta')
    SectionClDelta = Surface.SectionClDelta;
else
    SectionClDelta = Aero.CLdelta;
end

% SectionClDelta is local 2D dcl/d(delta_elevon) [1/rad]. Multiplying by
% TauControlEff and chord gives a spanwise lift-derivative density [m/rad].
SectionLift = SectionClDelta * Surface.TauControlEff * LocalChord;

Coeff.CL = 2 * trapz_rows(Y, SectionLift) / Sref;
Coeff.CmLinear = -2 * trapz_rows(Y, SectionLift .* (LocalTrailingEdgeX - Xref)) / (Sref * Geom.cbar);
Coeff.CmQuadratic = trapz_rows(Y, SectionLift .* LocalChord) / (Sref * Geom.cbar);
Coeff.Cm = Coeff.CmLinear + Coeff.CmQuadratic;

end

function [Y] = panel_y_stations(Panels, MaxStation, HalfSpan)
% Shared quadrature stations for all spanwise panels.

PanelFraction = linspace(0, 1, 25);
Station = Panels.Inboard + (Panels.Outboard - Panels.Inboard) .* PanelFraction;
Y = Station / MaxStation * HalfSpan;

end

function [Integral] = trapz_rows(X, Y)
% Row-wise trapezoid integration for panel-specific station grids.

Integral = sum(0.5 * (X(:, 2:end) - X(:, 1:end - 1)) .* ...
    (Y(:, 1:end - 1) + Y(:, 2:end)), 2);

end
