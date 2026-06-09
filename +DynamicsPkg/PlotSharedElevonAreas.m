function [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, HalfSpan, OutputFile)
%
% [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, HalfSpan, OutputFile)
%
% Plot selected pitch-only and outboard dual-use elevon panels.
%

if nargin < 6
    OutputFile = fullfile("+DynamicsPkg", "outputs", "control_surface_areas.png");
end

AvailableChordFraction = ChordLength / max(ChordLength);
FtPerM = 3.280839895;
MaxPanelSpanFt = Inf;
if isfield(Sizing, "MaxControlSurfaceSpan")
    MaxPanelSpanFt = Sizing.MaxControlSurfaceSpan * FtPerM;
end

PitchOnlySegments = FilterSegmentsByName(Sizing.Elevator.Segments, "Pitch-only elevon");
DualSegments = Sizing.DualElevon.Segments;

AvailableHalfSpan = ChordEta * HalfSpan * FtPerM;
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

DrawSegmentBand(PitchOnlySegments, [0.20, 0.45, 0.85], FtPerM, 0, MaxPanelSpanFt, 0.85);
DrawSegmentBand(DualSegments, [0.20, 0.70, 0.55], FtPerM, 0, MaxPanelSpanFt, 0.70);

patch([RudderIn, RudderOut, RudderOut, RudderIn], ...
    [0, 0, Sizing.Rudder.ChordFraction, Sizing.Rudder.ChordFraction], ...
    [0.55, 0.25, 0.70], "FaceAlpha", 0.85, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);

LimitLabel = sprintf("Max Chord Fraction");
DrawChordLimit(PitchOnlySegments, FtPerM, max(Sizing.Elevator.ChordFractions), LimitLabel, 0.025);
DrawChordLimit(DualSegments, FtPerM, max(Sizing.DualElevon.ChordFractions), LimitLabel, 0.055);
DrawChordLimitSpan(RudderIn, RudderOut, max(Sizing.Rudder.ChordFractions), LimitLabel, 0.025);

PitchLabel = SegmentLabelPoint(PitchOnlySegments, FtPerM, 0);
DualLabel = SegmentLabelPoint(DualSegments, FtPerM, 0);

if ~isempty(PitchLabel)
    text(PitchLabel(1), PitchLabel(2) + 0.015, ...
        "Pitch Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
end
if ~isempty(DualLabel)
    text(DualLabel(1), max(DualLabel(2) - 0.035, 0.04), ...
        "Dual-Use Elevon (Pitch/Roll)", "HorizontalAlignment", "center", ...
        "VerticalAlignment", "top", "FontWeight", "bold");
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

function DrawSegmentBand(Segments, FaceColor, FtPerM, ChordOffset, MaxPanelSpanFt, FaceAlpha)
% Draw each optimizer cell at its own selected chord fraction.

if isempty(Segments)
    return
end

StationIn = cellfun(@(Segment) Segment.YInboard * FtPerM, Segments);
StationOut = cellfun(@(Segment) Segment.YOutboard * FtPerM, Segments);
Chord = cellfun(@(Segment) Segment.ChordFraction, Segments);
[StationIn, Order] = sort(StationIn);
StationOut = StationOut(Order);
Chord = Chord(Order);

for isegment = 1:length(StationIn)
    PanelIn = StationIn(isegment);
    while PanelIn < StationOut(isegment)
        PanelOut = min(PanelIn + MaxPanelSpanFt, StationOut(isegment));
        patch([PanelIn, PanelOut, PanelOut, PanelIn], ...
            ChordOffset + [0, 0, Chord(isegment), Chord(isegment)], ...
            FaceColor, "FaceAlpha", FaceAlpha, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);
        PanelIn = PanelOut;
    end
end

end

function DrawChordLimit(Segments, FtPerM, ChordLimit, Label, LabelOffset)
% Draw the maximum allowed chord fraction over the selected station range.

if isempty(Segments)
    return
end

StationIn = min(cellfun(@(Segment) Segment.YInboard * FtPerM, Segments));
StationOut = max(cellfun(@(Segment) Segment.YOutboard * FtPerM, Segments));
DrawChordLimitSpan(StationIn, StationOut, ChordLimit, Label, LabelOffset);

end

function DrawChordLimitSpan(StationIn, StationOut, ChordLimit, Label, LabelOffset)
% Red dashed cap line shows the optimizer's local chord-fraction bound.

plot([StationIn, StationOut], [ChordLimit, ChordLimit], "r--", "LineWidth", 1.6);
text(0.5 * (StationIn + StationOut), ChordLimit + LabelOffset, Label, ...
    "HorizontalAlignment", "center", "Color", [0.75, 0.05, 0.05], ...
    "FontSize", 10, "FontWeight", "bold");

end

function [Point] = SegmentLabelPoint(Segments, FtPerM, ChordOffset)
% Place label near the middle of selected surface bands.

StationIn = cellfun(@(Segment) Segment.YInboard * FtPerM, Segments);
StationOut = cellfun(@(Segment) Segment.YOutboard * FtPerM, Segments);
Chord = cellfun(@(Segment) Segment.ChordFraction, Segments);
if isempty(StationIn)
    Point = [];
else
    Point = [0.5 * (min(StationIn) + max(StationOut)), ChordOffset + max(Chord)];
end

end
