function [Sizing] = BuildSharedElevonSizing(Aircraft, Cases, Surfaces, PitchPanels, OutboardPanels, ...
    PitchChordValue, DualChordValue, RollChordValue, SolverValues)
%
% [Sizing] = BuildSharedElevonSizing(...)
%
% Convert solved panel chords into FAST-style sizing and check structs.
%

MaxStation = Surfaces.MaxModelHalfSpanStation;

% Convert optimized chord-fraction vectors back into FAST-style surface
% structs. Dual-use panels contribute to both pitch and roll checks.
PitchOnlySegments = build_segments(Aircraft, Surfaces.Elevator, PitchPanels, PitchChordValue, MaxStation, "Pitch-only elevon");
DualSegments = build_segments(Aircraft, Surfaces.DualElevon, OutboardPanels, DualChordValue, MaxStation, "Dual-use elevon");
RollOnlySegments = build_segments(Aircraft, Surfaces.Aileron, OutboardPanels, RollChordValue, MaxStation, "Roll-only elevon");

Elevator = combine_segments(Surfaces.Elevator, [PitchOnlySegments; DualSegments], "Pitch Elevon");
Aileron = combine_segments(Surfaces.Aileron, [DualSegments; RollOnlySegments], "Roll Elevon");
DualElevon = combine_segments(Surfaces.DualElevon, DualSegments, "Dual-Use Elevon");
Rudder = DynamicsPkg.SizeRudder(Aircraft, Cases.DirectionalTrim, Surfaces.Rudder);

Elevator.Checks = check_elevator(Aircraft, Cases, Elevator);
Aileron.Checks = check_aileron(Aircraft, Cases, Aileron);
Rudder.Checks.Direction = DynamicsPkg.CheckDirectionalTrim(Aircraft, Cases.DirectionalTrim, Rudder);

Elevator.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, PitchOnlySegments));
Aileron.PhysicalAreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, RollOnlySegments));
Elevator.MaxDeflection = Elevator.Checks.MaxDeflection;
Aileron.MaxDeflection = Aileron.Checks.MaxDeflection;
Rudder.MaxDeflection = abs(Rudder.Checks.Direction.Delta);
Elevator.Converged = Elevator.Checks.Feasible;
Aileron.Converged = Aileron.Checks.Feasible;
Rudder.Converged = Rudder.Checks.Direction.Feasible;
DualElevon.Converged = true;

Sizing.Elevator = Elevator;
Sizing.Aileron = Aileron;
Sizing.DualElevon = DualElevon;
Sizing.Rudder = Rudder;
Sizing.Converged = Elevator.Converged && Aileron.Converged && Rudder.Converged;
Sizing.AreaFraction = Elevator.PhysicalAreaFraction + DualElevon.AreaFraction + ...
    Aileron.PhysicalAreaFraction + Rudder.AreaFraction;
Sizing.Casadi = struct("Solver", "casadi", ...
    "PitchAreaFraction", SolverValues.PitchArea, ...
    "PitchCLdelta", SolverValues.PitchCLdelta, ...
    "PitchCmdelta", SolverValues.PitchCmdelta, ...
    "RollAreaFraction", SolverValues.RollArea, ...
    "PhysicalElevonAreaFraction", SolverValues.PhysicalArea);
if isfield(Surfaces, "MaxControlSurfaceSpan")
    Sizing.MaxControlSurfaceSpan = Surfaces.MaxControlSurfaceSpan;
end

end

function [Segments] = build_segments(Aircraft, Surface, Panels, ChordFractions, MaxStation, Name)
% Convert nonzero optimized panel chords into segment cells.

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Coeff = DynamicsPkg.PanelCoefficients(Aircraft, Surface, Panels, MaxStation);
Segments = {};
for ipanel = find(ChordFractions(:)' > 1.0e-4)
    Segment = Surface;
    Segment.Name = Name;
    Segment.YInboard = Panels.Inboard(ipanel) / MaxStation * HalfSpan;
    Segment.YOutboard = Panels.Outboard(ipanel) / MaxStation * HalfSpan;
    Segment.SpanFraction = (Panels.Outboard(ipanel) - Panels.Inboard(ipanel)) / MaxStation;
    Segment.ChordFraction = ChordFractions(ipanel);
    Segment.AreaFraction = Coeff.Area(ipanel) * ChordFractions(ipanel);
    Segment.CLdeltaEffective = Coeff.CL(ipanel) * ChordFractions(ipanel);
    Segment.CmdeltaEffective = Coeff.Cm(ipanel) * ChordFractions(ipanel);
    Segments{end + 1, 1} = Segment; %#ok<AGROW>
end

end

function [Trial] = combine_segments(Surface, Segments, Name)
% Combine panel cells for existing authority-check functions.

if isempty(Segments)
    error("ERROR - BuildSharedElevonSizing: %s has no selected panels.", Name);
end

Trial = Surface;
Trial.Name = Name;
Trial.Segments = Segments;
Trial.YInboard = min(cellfun(@(Segment) Segment.YInboard, Segments));
Trial.YOutboard = max(cellfun(@(Segment) Segment.YOutboard, Segments));
Trial.SpanFraction = max(cellfun(@(Segment) Segment.SpanFraction, Segments));
Trial.ChordFraction = max(cellfun(@(Segment) Segment.ChordFraction, Segments));
Trial.AreaFraction = sum(cellfun(@(Segment) Segment.AreaFraction, Segments));
Trial.CLdeltaEffective = sum(cellfun(@(Segment) Segment.CLdeltaEffective, Segments));
Trial.CmdeltaEffective = sum(cellfun(@(Segment) Segment.CmdeltaEffective, Segments));
Trial.TauControlEff = Surface.TauControlEff;

end

function [Checks] = check_elevator(Aircraft, Cases, Elevator)
% Group longitudinal checks for optimized shared elevons.

Checks.Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.LongitudinalTrim, Elevator);
Checks.Pullup = DynamicsPkg.CheckPullup(Aircraft, Cases.Pullup, Elevator);
Checks.Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, Cases.TakeoffRotation, Elevator);
Checks.Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.CruiseTrim, Elevator);
Checks.Feasible = Checks.Trim.Feasible && Checks.Pullup.Feasible && Checks.Rotation.Feasible && Checks.Cruise.Feasible;
Checks.MaxDeflection = max(abs([Checks.Trim.Delta; Checks.Pullup.DeltaFinal; ...
    Checks.Cruise.Delta; Cases.TakeoffRotation.DeltaElevator]));

end

function [Checks] = check_aileron(Aircraft, Cases, Aileron)
% Check time-to-bank for optimized shared roll panels.

Checks.Bank = DynamicsPkg.CheckTimeToBank(Aircraft, Cases.TimeToBank, Aileron);
Checks.Feasible = Checks.Bank.Feasible;
Checks.MaxDeflection = abs(Checks.Bank.Delta);

end
