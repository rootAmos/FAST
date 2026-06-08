function [Panels] = BuildStationPanels(StationIn, StationOut, PanelWidth)
%
% [Panels] = BuildStationPanels(StationIn, StationOut, PanelWidth)
%
% Build short station panels and preserve the exact requested outer edge.
%

Edges = StationIn:PanelWidth:StationOut;
if Edges(end) < StationOut
    Edges = [Edges, StationOut];
end

Panels.Inboard = Edges(1:end - 1)';
Panels.Outboard = Edges(2:end)';
Panels.Center = 0.5 * (Panels.Inboard + Panels.Outboard);

end
