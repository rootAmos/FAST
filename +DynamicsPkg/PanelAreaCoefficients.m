function [Coeff] = PanelAreaCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% [Coeff] = PanelAreaCoefficients(Aircraft, Surface, Panels, MaxStation)
%
% Area per unit chord fraction for each placed spanwise panel.
%

Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Y = DynamicsPkg.PanelYStations(Panels, MaxStation, HalfSpan);
LocalChord = interp1(Surface.ChordEta, Surface.ChordLength, Y / HalfSpan, "linear", "extrap");
Coeff = 2 * DynamicsPkg.TrapzRows(Y, LocalChord) / Sref;

end
