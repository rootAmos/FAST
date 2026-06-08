function [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% Optimize 0.2-station shared elevon panels with CasADi.
%

if ~isfield(Surfaces, "SharedTrailingEdge") || ~Surfaces.SharedTrailingEdge
    error("ERROR - OptimizeSharedElevons: Surfaces.SharedTrailingEdge must be true.");
end

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
MaxStation = Surfaces.MaxModelHalfSpanStation;
PanelWidth = Surfaces.PanelStationWidth;
if isfield(Surfaces, "PitchStationRange")
    PitchStationIn = Surfaces.PitchStationRange(1);
    PitchStationOut = Surfaces.PitchStationRange(2);
else
    PitchStationIn = 0;
    PitchStationOut = 5;
end
if isfield(Surfaces, "OutboardStationRange")
    OutboardStationIn = Surfaces.OutboardStationRange(1);
    OutboardStationOut = Surfaces.OutboardStationRange(2);
else
    OutboardStationIn = 10;
    OutboardStationOut = max(Surfaces.OutboardEtaStations) * MaxStation;
end

PitchPanels = DynamicsPkg.BuildStationPanels(PitchStationIn, PitchStationOut, PanelWidth);
OutboardPanels = DynamicsPkg.BuildStationPanels(OutboardStationIn, OutboardStationOut, PanelWidth);

PitchCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.Elevator, PitchPanels, MaxStation);
OutboardCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);

PitchMax = max(Surfaces.Elevator.ChordFractions);
DualMax = max(Surfaces.DualElevon.ChordFractions);
RollMax = max(Surfaces.Aileron.ChordFractions);
OutboardMax = max(DualMax, RollMax);

[PitchChordValue, DualChordValue, RollChordValue, SolverValues] = DynamicsPkg.SolveSharedElevonChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchMax, DualMax, RollMax, OutboardMax, MaxStation);

Sizing = DynamicsPkg.BuildSharedElevonSizing(Aircraft, Cases, Surfaces, PitchPanels, OutboardPanels, ...
    PitchChordValue, DualChordValue, RollChordValue, SolverValues);

end
