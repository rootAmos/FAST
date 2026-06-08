function [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, OutputFile)
%
% [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, OutputFile)
%
% Plot selected pitch-only, dual-use, and roll-only elevon panels.
%

if nargin < 5
    OutputFile = fullfile("+DynamicsPkg", "outputs", "control_surface_areas.png");
end

AvailableChordFraction = ChordLength / max(ChordLength);
FtPerM = 3.280839895;

PitchOnlySegments = FilterSegmentsByName(Sizing.Elevator.Segments, "Pitch-only elevon");
DualSegments = Sizing.DualElevon.Segments;
RollOnlySegments = FilterSegmentsByName(Sizing.Aileron.Segments, "Roll-only elevon");

HalfSpan = PitchOnlySegments{1}.YOutboard / PitchOnlySegments{1}.SpanFraction;
AvailableHalfSpan = ChordEta * HalfSpan * FtPerM;
ChunkDistance = 0.2 / MaxModelHalfSpanStation * HalfSpan * FtPerM;
NoElevonIn = 5 / MaxModelHalfSpanStation * HalfSpan * FtPerM;
NoElevonOut = 10 / MaxModelHalfSpanStation * HalfSpan * FtPerM;

RudderOut = HalfSpan * FtPerM;
RudderIn = RudderOut - Sizing.Rudder.SpanFraction * 0.10 * HalfSpan * FtPerM;

figure;
hold on
patch([NoElevonIn, NoElevonOut, NoElevonOut, NoElevonIn], [0, 0, 1, 1], ...
    [0.88, 0.88, 0.88], "FaceAlpha", 0.60, "EdgeColor", "none");
plot([0, HalfSpan * FtPerM], [0, 0], "k", "LineWidth", 1.2);
area(AvailableHalfSpan, AvailableChordFraction, ...
    "FaceColor", [0.82, 0.86, 0.90], "FaceAlpha", 0.35, "EdgeColor", [0.45, 0.48, 0.52], "LineWidth", 1.0);

DrawSegmentCells(PitchOnlySegments, HalfSpan, [0.20, 0.45, 0.85], ChunkDistance, FtPerM);
DrawSegmentCells(DualSegments, HalfSpan, [0.20, 0.70, 0.55], ChunkDistance, FtPerM);
DrawSegmentCells(RollOnlySegments, HalfSpan, [0.95, 0.62, 0.05], ChunkDistance, FtPerM);
patch([RudderIn, RudderOut, RudderOut, RudderIn], ...
    [0, 0, Sizing.Rudder.ChordFraction, Sizing.Rudder.ChordFraction], ...
    [0.55, 0.25, 0.70], "FaceAlpha", 0.85, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);

PitchLabel = SegmentLabelPoint(PitchOnlySegments, FtPerM);
DualLabel = SegmentLabelPoint(DualSegments, FtPerM);
RollLabel = SegmentLabelPoint(RollOnlySegments, FtPerM);

text(PitchLabel(1), PitchLabel(2) + 0.015, ...
    "Pitch Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
text(DualLabel(1), DualLabel(2) + 0.015, ...
    "Dual-Use Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
if ~isempty(RollLabel)
    text(RollLabel(1), RollLabel(2) + 0.015, ...
        "Roll Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
end
text(mean([RudderIn, RudderOut]), Sizing.Rudder.ChordFraction + 0.015, ...
    "Winglet Rudder", "HorizontalAlignment", "center", "FontWeight", "bold");
text(mean([NoElevonIn, NoElevonOut]), 0.94, "No elevons", "HorizontalAlignment", "center", "FontWeight", "bold", "Color", [0.35, 0.35, 0.35]);

grid on
xlabel("Physical half-span distance [ft]");
ylabel("Chord fraction");
title("Selected Control Surface Placement, 100 ft Span BWB");
xlim([0, HalfSpan * FtPerM]);
ylim([0, 1]);
saveas(gcf, OutputFile);

end

function [Segments] = FilterSegmentsByName(SegmentsIn, Name)
% Select panel cells by role name.

Keep = cellfun(@(Segment) string(Segment.Name) == Name, SegmentsIn);
Segments = SegmentsIn(Keep);

end

function DrawSegmentCells(Segments, HalfSpan, FaceColor, ChunkDistance, FtPerM)
% Draw each selected optimized panel separately.

for isegment = 1:length(Segments)
    DistanceIn = Segments{isegment}.YInboard * FtPerM;
    DistanceOut = Segments{isegment}.YOutboard * FtPerM;
    DrawChunkedSurface(DistanceIn, DistanceOut, Segments{isegment}.ChordFraction, FaceColor, ChunkDistance);
end

end

function [Point] = SegmentLabelPoint(Segments, FtPerM)
% Place label near the middle of selected panel cells.

StationIn = cellfun(@(Segment) Segment.YInboard * FtPerM, Segments);
StationOut = cellfun(@(Segment) Segment.YOutboard * FtPerM, Segments);
Chord = cellfun(@(Segment) Segment.ChordFraction, Segments);
if isempty(StationIn)
    Point = [];
else
    Point = [0.5 * (min(StationIn) + max(StationOut)), max(Chord)];
end

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
