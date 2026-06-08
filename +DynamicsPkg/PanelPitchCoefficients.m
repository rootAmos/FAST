function [CLCoeff, CmCoeff] = PanelPitchCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% [CLCoeff, CmCoeff] = PanelPitchCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% Per-unit-chord pitch derivatives from local trailing-edge moment arms.
%

Aero = Aircraft.Specs.Dynamics.Longitudinal;
Geom = Aircraft.Specs.Dynamics.Geometry;
Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Geom.b / 2;

CenterLeadingEdgeX = interp1(Surface.ChordEta, Surface.ChordLeadingEdgeX, 0, "linear", "extrap");
Xref = CenterLeadingEdgeX + Aero.XrefMAC * Geom.cbar;

Y = DynamicsPkg.PanelYStations(Panels, MaxStation, HalfSpan);
Eta = Y / HalfSpan;
LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, Eta, "linear", "extrap");
LocalTrailingEdgeX = interp1(Surface.ChordEta, Surface.ChordTrailingEdgeX, Eta, "linear", "extrap");
ControlCenterX = LocalTrailingEdgeX - 0.5 * LocalChord;
if isfield(Surface, 'SectionClDelta')
    SectionClDelta = Surface.SectionClDelta;
else
    SectionClDelta = Aero.CLdelta;
end
SectionLift = SectionClDelta * Surface.EtaControl * LocalChord;

CLCoeff = 2 * DynamicsPkg.TrapzRows(Y, SectionLift) / Sref;
CmCoeff = -2 * DynamicsPkg.TrapzRows(Y, SectionLift .* (ControlCenterX - Xref)) / (Sref * Geom.cbar);

end
