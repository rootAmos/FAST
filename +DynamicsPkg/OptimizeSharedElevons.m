function [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% [Sizing] = OptimizeSharedElevons(Aircraft, Cases, Surfaces)
%
% Prepare the shared elevon panel model, run the CasADi chord allocation,
% and package the result into the numeric check/report structure.
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

% Pre-integrate each candidate panel once. The CasADi problem then chooses
% panel chord fractions without carrying interpolation/integration logic.
PitchCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.Elevator, PitchPanels, MaxStation);
OutboardCoeff = DynamicsPkg.PanelCoefficients(Aircraft, Surfaces.DualElevon, OutboardPanels, MaxStation);

PitchMax = max(Surfaces.Elevator.ChordFractions);
OutboardMax = max(Surfaces.DualElevon.ChordFractions);

% This is the CasADi/Ipopt optimization call. It returns the optimized chord
% fraction for each pitch-only and outboard dual-use spanwise panel.
[PitchChordValue, OutboardChordValue, SolverValues] = DynamicsPkg.SolveSharedElevonChords( ...
    Aircraft, Cases, PitchCoeff, OutboardCoeff, PitchPanels, OutboardPanels, ...
    PitchMax, OutboardMax, Surfaces.DualElevon.TauControlEff);

% Convert the raw optimizer vectors into surface structs, then run the
% numeric Check* functions so the output contains readable margins.
Sizing = DynamicsPkg.BuildSharedElevonSizing(Aircraft, Cases, Surfaces, PitchPanels, OutboardPanels, ...
    PitchChordValue, OutboardChordValue, SolverValues);

end
