function [Coeff] = PanelRollCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% [Coeff] = PanelRollCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% Roll integral per unit chord fraction for each placed spanwise panel.
%

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Y = DynamicsPkg.PanelYStations(Panels, MaxStation, HalfSpan);
LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, Y / HalfSpan, "linear", "extrap");
Coeff = Surface.SectionClDelta * DynamicsPkg.TrapzRows(Y, LocalChord .* Y);

end
