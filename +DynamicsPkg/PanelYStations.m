function [Y] = PanelYStations(Panels, MaxStation, HalfSpan)
%
% [Y] = PanelYStations(Panels, MaxStation, HalfSpan)
%
% Shared quadrature stations for all spanwise panels.
%

PanelFraction = linspace(0, 1, 25);
Station = Panels.Inboard + (Panels.Outboard - Panels.Inboard) .* PanelFraction;
Y = Station / MaxStation * HalfSpan;

end
