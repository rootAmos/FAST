function [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, OutputFile)
%
% [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, OutputFile)
%
% Plot selected pitch-only, dual-use, and roll-only elevon panels.
%

if nargin < 5
    OutputFile = fullfile("+DynamicsPkg", "outputs", "control_surface_areas.png");
end

AvailableChordStation = ChordEta * MaxModelHalfSpanStation;
AvailableChordFraction = ChordLength / max(ChordLength);
ChunkStation = 0.2;

PitchOnlySegments = FilterSegmentsByName(Sizing.Elevator.Segments, "Pitch-only elevon");
DualSegments = Sizing.DualElevon.Segments;
RollOnlySegments = FilterSegmentsByName(Sizing.Aileron.Segments, "Roll-only elevon");

HalfSpan = PitchOnlySegments{1}.YOutboard / PitchOnlySegments{1}.SpanFraction;

RudderStationOut = MaxModelHalfSpanStation;
RudderStationIn = RudderStationOut - Sizing.Rudder.SpanFraction * 0.10 * MaxModelHalfSpanStation;

figure;
hold on
patch([5, 10, 10, 5], [0, 0, 1, 1], ...
    [0.88, 0.88, 0.88], "FaceAlpha", 0.60, "EdgeColor", "none");
plot([0, MaxModelHalfSpanStation], [0, 0], "k", "LineWidth", 1.2);
area(AvailableChordStation, AvailableChordFraction, ...
    "FaceColor", [0.82, 0.86, 0.90], "FaceAlpha", 0.35, "EdgeColor", [0.45, 0.48, 0.52], "LineWidth", 1.0);

DrawSegmentCells(PitchOnlySegments, HalfSpan, MaxModelHalfSpanStation, [0.20, 0.45, 0.85], ChunkStation);
DrawSegmentCells(DualSegments, HalfSpan, MaxModelHalfSpanStation, [0.20, 0.70, 0.55], ChunkStation);
DrawSegmentCells(RollOnlySegments, HalfSpan, MaxModelHalfSpanStation, [0.95, 0.62, 0.05], ChunkStation);
patch([RudderStationIn, RudderStationOut, RudderStationOut, RudderStationIn], ...
    [0, 0, Sizing.Rudder.ChordFraction, Sizing.Rudder.ChordFraction], ...
    [0.55, 0.25, 0.70], "FaceAlpha", 0.85, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);

PitchLabel = SegmentLabelPoint(PitchOnlySegments, HalfSpan, MaxModelHalfSpanStation);
DualLabel = SegmentLabelPoint(DualSegments, HalfSpan, MaxModelHalfSpanStation);
RollLabel = SegmentLabelPoint(RollOnlySegments, HalfSpan, MaxModelHalfSpanStation);

text(PitchLabel(1), PitchLabel(2) + 0.015, ...
    "Pitch Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
text(DualLabel(1), DualLabel(2) + 0.015, ...
    "Dual-Use Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
text(RollLabel(1), RollLabel(2) + 0.015, ...
    "Roll Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
text(mean([RudderStationIn, RudderStationOut]), Sizing.Rudder.ChordFraction + 0.015, ...
    "Winglet Rudder", "HorizontalAlignment", "center", "FontWeight", "bold");
text(7.5, 0.94, "No elevons", "HorizontalAlignment", "center", "FontWeight", "bold", "Color", [0.35, 0.35, 0.35]);

grid on
xlabel("Half-span station");
ylabel("Chord fraction");
title("Selected Control Surface Placement on Right Half-Span");
xlim([0, MaxModelHalfSpanStation]);
ylim([0, 1]);
saveas(gcf, OutputFile);

end

function [Segments] = FilterSegmentsByName(SegmentsIn, Name)
% Select panel cells by role name.

Keep = cellfun(@(Segment) string(Segment.Name) == Name, SegmentsIn);
Segments = SegmentsIn(Keep);

end

function DrawSegmentCells(Segments, HalfSpan, MaxModelHalfSpanStation, FaceColor, ChunkStation)
% Draw each selected optimized panel separately.

for isegment = 1:length(Segments)
    StationIn = Segments{isegment}.YInboard / HalfSpan * MaxModelHalfSpanStation;
    StationOut = Segments{isegment}.YOutboard / HalfSpan * MaxModelHalfSpanStation;
    DrawChunkedSurface(StationIn, StationOut, Segments{isegment}.ChordFraction, FaceColor, ChunkStation);
end

end

function [Point] = SegmentLabelPoint(Segments, HalfSpan, MaxModelHalfSpanStation)
% Place label near the middle of selected panel cells.

StationIn = cellfun(@(Segment) Segment.YInboard / HalfSpan * MaxModelHalfSpanStation, Segments);
StationOut = cellfun(@(Segment) Segment.YOutboard / HalfSpan * MaxModelHalfSpanStation, Segments);
Chord = cellfun(@(Segment) Segment.ChordFraction, Segments);
Point = [0.5 * (min(StationIn) + max(StationOut)), max(Chord)];

end

function DrawChunkedSurface(StationIn, StationOut, ChordFraction, FaceColor, ChunkStation)
% Render selected control surfaces as short spanwise panels.

Edges = StationIn:ChunkStation:StationOut;
if isempty(Edges) || Edges(1) > StationIn
    Edges = [StationIn, Edges];
end
if Edges(end) < StationOut
    Edges = [Edges, StationOut];
end

for iedge = 1:length(Edges) - 1
    PanelIn = Edges(iedge) + 0.015;
    PanelOut = Edges(iedge + 1) - 0.015;
    if PanelOut <= PanelIn
        PanelIn = Edges(iedge);
        PanelOut = Edges(iedge + 1);
    end
    patch([PanelIn, PanelOut, PanelOut, PanelIn], ...
        [0, 0, ChordFraction, ChordFraction], ...
        FaceColor, "FaceAlpha", 0.85, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 0.8);
end

end
