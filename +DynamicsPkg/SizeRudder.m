function [Rudder] = SizeRudder(Aircraft, Case, Surface)
%
% [Rudder] = SizeRudder(Aircraft, Case, Surface)
%
% Size the separate winglet rudder directly from directional authority.
%

Lat = Aircraft.Specs.Dynamics.Lateral;
RequiredArea = abs(Case.RequiredCn / (Lat.Cndr * Surface.TauControlEff * Case.MaxDeflection));
[SpanGrid, ChordGrid] = ndgrid(Surface.SpanFractions(:), Surface.ChordFractions(:));
AreaGrid = SpanGrid .* ChordGrid;
Feasible = AreaGrid >= RequiredArea;

if any(Feasible(:))
    CandidateArea = AreaGrid;
    CandidateArea(~Feasible) = Inf;
    [~, Index] = min(CandidateArea(:));
    Converged = true;
else
    [~, Index] = max(AreaGrid(:));
    Converged = false;
end

Rudder = Surface;
Rudder.SpanFraction = SpanGrid(Index);
Rudder.ChordFraction = ChordGrid(Index);
Rudder.AreaFraction = AreaGrid(Index);
Rudder.TauControlEff = Surface.TauControlEff;
Rudder.Converged = Converged;
Rudder.SpanFractions = Surface.SpanFractions(:);
Rudder.ChordFractions = Surface.ChordFractions(:);
Rudder.AreaFractions = AreaGrid;
Rudder.Feasible = Feasible;

end
