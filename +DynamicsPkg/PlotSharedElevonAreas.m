function [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, HalfSpan, OutputFile)
%
% [] = PlotSharedElevonAreas(Sizing, ChordEta, ChordLength, MaxModelHalfSpanStation, HalfSpan, OutputFile)
%
% Plot selected pitch-only, dual-use, and roll-only elevon panels.
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
RollOnlySegments = FilterSegmentsByName(Sizing.Aileron.Segments, "Roll-only elevon");

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
DrawSegmentBand(DualSegments, [0.20, 0.70, 0.55], FtPerM, 0, MaxPanelSpanFt, 0.60);

% Overlay roll-only panels on the same chord-fraction axis. The optimizer
% still enforces DualChord + RollChord <= max chord; this plot shows role
% overlap/usage, not physically stacked chord.
DrawSegmentBand(RollOnlySegments, [0.95, 0.62, 0.05], FtPerM, 0, MaxPanelSpanFt, 0.45);
patch([RudderIn, RudderOut, RudderOut, RudderIn], ...
    [0, 0, Sizing.Rudder.ChordFraction, Sizing.Rudder.ChordFraction], ...
    [0.55, 0.25, 0.70], "FaceAlpha", 0.85, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);

PitchLabel = SegmentLabelPoint(PitchOnlySegments, FtPerM, 0);
DualLabel = SegmentLabelPoint(DualSegments, FtPerM, 0);
RollLabel = SegmentLabelPoint(RollOnlySegments, FtPerM, 0);

if ~isempty(PitchLabel)
    text(PitchLabel(1), PitchLabel(2) + 0.015, ...
        "Pitch Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
end
if ~isempty(DualLabel)
    text(DualLabel(1), DualLabel(2) + 0.015, ...
        "Dual-Use Elevon", "HorizontalAlignment", "center", "FontWeight", "bold");
end
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

function DrawSegmentBand(Segments, FaceColor, FtPerM, ChordOffset, MaxPanelSpanFt, FaceAlpha)
% Draw contiguous optimizer cells as one physical control surface band.

if isempty(Segments)
    return
end

StationIn = cellfun(@(Segment) Segment.YInboard * FtPerM, Segments);
StationOut = cellfun(@(Segment) Segment.YOutboard * FtPerM, Segments);
Chord = cellfun(@(Segment) Segment.ChordFraction, Segments);
[StationIn, Order] = sort(StationIn);
StationOut = StationOut(Order);
Chord = Chord(Order);
Breaks = [true; StationIn(2:end) > StationOut(1:end - 1) + 1.0e-6];
BandId = cumsum(Breaks);

for iband = 1:BandId(end)
    Keep = BandId == iband;
    BandIn = min(StationIn(Keep));
    BandOut = max(StationOut(Keep));
    BandChord = max(Chord(Keep));
    PanelIn = BandIn;
    while PanelIn < BandOut
        PanelOut = min(PanelIn + MaxPanelSpanFt, BandOut);
        patch([PanelIn, PanelOut, PanelOut, PanelIn], ...
            ChordOffset + [0, 0, BandChord, BandChord], ...
            FaceColor, "FaceAlpha", FaceAlpha, "EdgeColor", [0.05, 0.12, 0.18], "LineWidth", 1.2);
        PanelIn = PanelOut;
    end
end

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

function [Chord] = MaxSegmentChord(Segments)
% Maximum selected chord fraction for a segment group.

if isempty(Segments)
    Chord = 0;
else
    Chord = max(cellfun(@(Segment) Segment.ChordFraction, Segments));
end

end
