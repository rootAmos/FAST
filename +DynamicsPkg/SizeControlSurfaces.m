function [Sizing] = SizeControlSurfaces(Aircraft, Cases, Surfaces)
%
% [Sizing] = SizeControlSurfaces(Aircraft, Cases, Surfaces)
%
% Size separate elevator, aileron, and rudder surfaces against their
% own authority requirements.
%

if isfield(Surfaces, 'SharedTrailingEdge') && Surfaces.SharedTrailingEdge
    [Sizing.Elevator, Sizing.Aileron, Sizing.DualElevon] = SizeSharedElevonGrid(Aircraft, Surfaces.Elevator, ...
        Surfaces.DualElevon, Surfaces.Aileron, ...
        @(Surface) CheckElevator(Aircraft, Cases, Surface), @(Surface) CheckAileron(Aircraft, Cases, Surface));
elseif isfield(Surfaces.Elevator, 'Options')
    Sizing.Elevator = SizeCoupledElevatorGrid(Aircraft, Surfaces.Elevator.Options, @(Surface) CheckElevator(Aircraft, Cases, Surface));
    Sizing.Aileron = SizeSurfaceGrid(Aircraft, Surfaces.Aileron, @(Surface) CheckAileron(Aircraft, Cases, Surface));
else
    Sizing.Elevator = SizeSurfaceGrid(Aircraft, Surfaces.Elevator, @(Surface) CheckElevator(Aircraft, Cases, Surface));
    Sizing.Aileron = SizeSurfaceGrid(Aircraft, Surfaces.Aileron, @(Surface) CheckAileron(Aircraft, Cases, Surface));
end
Sizing.Rudder = SizeSurfaceGrid(Aircraft, Surfaces.Rudder, @(Surface) CheckRudder(Aircraft, Cases, Surface));
Sizing.Converged = Sizing.Elevator.Converged && Sizing.Aileron.Converged && Sizing.Rudder.Converged;
if isfield(Sizing, 'DualElevon')
    Sizing.AreaFraction = Sizing.Elevator.PhysicalAreaFraction + Sizing.DualElevon.AreaFraction + ...
        Sizing.Aileron.PhysicalAreaFraction + Sizing.Rudder.AreaFraction;
else
    Sizing.AreaFraction = Sizing.Elevator.AreaFraction + Sizing.Aileron.AreaFraction + Sizing.Rudder.AreaFraction;
end

end

function [Sizing] = SizeSurfaceGrid(Aircraft, Surface, CheckFunction)
% Sweep one surface geometry grid and return the minimum feasible area.

SpanFractions = Surface.SpanFractions(:);
ChordFractions = Surface.ChordFractions(:);

nspan = length(SpanFractions);
nchord = length(ChordFractions);
Feasible = false(nspan, nchord);
AreaFraction = zeros(nspan, nchord);
MaxDeflection = zeros(nspan, nchord);
Results = cell(nspan, nchord);

for ichord = 1:nchord
    for ispan = 1:nspan
        Trial = Surface;
        Trial.SpanFraction = SpanFractions(ispan);
        Trial.ChordFraction = ChordFractions(ichord);
        Trial = SetTrialGeometry(Aircraft, Trial);

        Checks = CheckFunction(Trial);
        Feasible(ispan, ichord) = Checks.Feasible;
        AreaFraction(ispan, ichord) = Trial.AreaFraction;
        MaxDeflection(ispan, ichord) = Checks.MaxDeflection;
        Results{ispan, ichord} = Checks;
    end
end

CandidateArea = AreaFraction;
CandidateArea(~Feasible) = Inf;
[BestArea, BestIndex] = min(CandidateArea(:));

if isinf(BestArea)
    [~, BestIndex] = max(AreaFraction(:));
    Converged = 0;
else
    Converged = 1;
end

[BestSpanIndex, BestChordIndex] = ind2sub(size(AreaFraction), BestIndex);

Sizing.SpanFraction = SpanFractions(BestSpanIndex);
Sizing.ChordFraction = ChordFractions(BestChordIndex);
Sizing.AreaFraction = AreaFraction(BestSpanIndex, BestChordIndex);
Sizing.EtaControl = Surface.EtaControl;
if isfield(Surface, 'Name')
    Sizing.Name = Surface.Name;
end
Sizing.Converged = Converged;
Sizing.Checks = Results{BestSpanIndex, BestChordIndex};
Sizing.SpanFractions = SpanFractions;
Sizing.ChordFractions = ChordFractions;
Sizing.AreaFractions = AreaFraction;
Sizing.Feasible = Feasible;
Sizing.MaxDeflection = MaxDeflection;

end

function [ElevatorSizing, AileronSizing, DualSizing] = SizeSharedElevonGrid(Aircraft, Elevator, DualElevon, Aileron, CheckElevatorFunction, CheckAileronFunction)
% Use pitch-only, dual-use, and roll-only zones on the shared trailing edge.

PitchOutEtaStations = Elevator.EtaStations(:);
OutboardEtaStations = DualElevon.EtaStations(:);
HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
BestViolation = Inf;
BestFallbackArea = Inf;
Best = struct();
Converged = 0;

PitchSegments = BuildSharedSegmentCache(Aircraft, Elevator, 0, PitchOutEtaStations, ...
    Elevator.ChordFractions(:), "Pitch-only elevon");
DualSegments = BuildSharedSegmentCache(Aircraft, DualElevon, OutboardEtaStations, OutboardEtaStations, ...
    DualElevon.ChordFractions(:), "Dual-use elevon");
RollSegments = BuildSharedSegmentCache(Aircraft, Aileron, OutboardEtaStations, OutboardEtaStations, ...
    Aileron.ChordFractions(:), "Roll-only elevon");
Candidates = BuildSharedElevonCandidates(PitchSegments, DualSegments, RollSegments, HalfSpan);
[~, EvaluationOrder] = sort(Candidates.Objective);

for iorder = 1:length(EvaluationOrder)
    icandidate = EvaluationOrder(iorder);
    PitchOnly = PitchSegments{Candidates.PitchOutIndex(icandidate), Candidates.PitchChordIndex(icandidate)};
    Dual = DualSegments{Candidates.DualInIndex(icandidate), Candidates.DualOutIndex(icandidate), Candidates.DualChordIndex(icandidate)};
    RollOnly = RollSegments{Candidates.DualOutIndex(icandidate), Candidates.RollOutIndex(icandidate), Candidates.RollChordIndex(icandidate)};

    PitchTrial = CombineSegments(Elevator, {PitchOnly, Dual}, "Pitch Elevon");
    RollTrial = CombineSegments(Aileron, {Dual, RollOnly}, "Roll Elevon");
    PitchChecks = CheckElevatorFunction(PitchTrial);
    RollChecks = CheckAileronFunction(RollTrial);

    PhysicalArea = Candidates.PhysicalArea(icandidate);
    PairFeasible = PitchChecks.Feasible && RollChecks.Feasible;

    if PairFeasible
        Best = PackSharedBest(PitchTrial, RollTrial, Dual, PitchChecks, RollChecks, PhysicalArea);
        Converged = 1;
        break
    end

    PairViolation = max(PitchChecks.MaxDeflection, RollChecks.MaxDeflection);
    if PairViolation < BestViolation || ...
            (abs(PairViolation - BestViolation) < 1.0e-12 && PhysicalArea < BestFallbackArea)
        BestViolation = PairViolation;
        BestFallbackArea = PhysicalArea;
        Best = PackSharedBest(PitchTrial, RollTrial, Dual, PitchChecks, RollChecks, PhysicalArea);
    end
end

ElevatorSizing = Best.Elevator;
AileronSizing = Best.Aileron;
DualSizing = Best.Dual;
ElevatorSizing.Converged = Converged && Best.PitchChecks.Feasible;
AileronSizing.Converged = Converged && Best.RollChecks.Feasible;

end

function [Segments] = BuildSharedSegmentCache(Aircraft, Surface, EtaInValues, EtaOutValues, ChordFractions, Name)
% Cache the physical geometry for each unique station/chord segment.

if isscalar(EtaInValues)
    Segments = cell(length(EtaOutValues), length(ChordFractions));
    for iout = 1:length(EtaOutValues)
        for ichord = 1:length(ChordFractions)
            Segments{iout, ichord} = BuildPlacedSegment(Aircraft, Surface, ...
                EtaInValues, EtaOutValues(iout), ChordFractions(ichord), Name);
        end
    end
    return
end

Segments = cell(length(EtaInValues), length(EtaOutValues), length(ChordFractions));
for iin = 1:length(EtaInValues)
    for iout = 1:length(EtaOutValues)
        if EtaOutValues(iout) <= EtaInValues(iin)
            continue
        end
        for ichord = 1:length(ChordFractions)
            Segments{iin, iout, ichord} = BuildPlacedSegment(Aircraft, Surface, ...
                EtaInValues(iin), EtaOutValues(iout), ChordFractions(ichord), Name);
        end
    end
end

end

function [Candidates] = BuildSharedElevonCandidates(PitchSegments, DualSegments, RollSegments, HalfSpan)
% Build the valid shared-elevon candidate table before running authority checks.

[npitchOut, npitchChord] = size(PitchSegments);
[~, noutboard, ndualChord] = size(DualSegments);
[~, ~, nrollChord] = size(RollSegments);
nchordCombos = npitchChord * ndualChord * nrollChord;
nstationCombos = npitchOut * nchoosek(noutboard, 3);
ncombo = nstationCombos * nchordCombos;

Candidates.PitchOutIndex = zeros(ncombo, 1);
Candidates.DualInIndex = zeros(ncombo, 1);
Candidates.DualOutIndex = zeros(ncombo, 1);
Candidates.RollOutIndex = zeros(ncombo, 1);
Candidates.PitchChordIndex = zeros(ncombo, 1);
Candidates.DualChordIndex = zeros(ncombo, 1);
Candidates.RollChordIndex = zeros(ncombo, 1);
Candidates.PhysicalArea = zeros(ncombo, 1);
Candidates.Objective = zeros(ncombo, 1);

icandidate = 0;
for ipitchOut = 1:npitchOut
    for idualIn = 1:noutboard - 2
        for idualOut = idualIn + 1:noutboard - 1
            for irollOut = idualOut + 1:noutboard
                for ipitchChord = 1:npitchChord
                    PitchOnly = PitchSegments{ipitchOut, ipitchChord};
                    for idualChord = 1:ndualChord
                        Dual = DualSegments{idualIn, idualOut, idualChord};
                        for irollChord = 1:nrollChord
                            RollOnly = RollSegments{idualOut, irollOut, irollChord};
                            icandidate = icandidate + 1;

                            Candidates.PitchOutIndex(icandidate) = ipitchOut;
                            Candidates.DualInIndex(icandidate) = idualIn;
                            Candidates.DualOutIndex(icandidate) = idualOut;
                            Candidates.RollOutIndex(icandidate) = irollOut;
                            Candidates.PitchChordIndex(icandidate) = ipitchChord;
                            Candidates.DualChordIndex(icandidate) = idualChord;
                            Candidates.RollChordIndex(icandidate) = irollChord;
                            Candidates.PhysicalArea(icandidate) = PitchOnly.AreaFraction + Dual.AreaFraction + RollOnly.AreaFraction;
                            PitchCenterBias = 1.0e-3 * mean([PitchOnly.YInboard, Dual.YOutboard]) / HalfSpan;
                            Candidates.Objective(icandidate) = Candidates.PhysicalArea(icandidate) + PitchCenterBias;
                        end
                    end
                end
            end
        end
    end
end

end

function [Segment] = BuildPlacedSegment(Aircraft, Surface, EtaIn, EtaOut, ChordFraction, Name)
% Convert one spanwise zone into a physical segment.

HalfSpan = Aircraft.Specs.Dynamics.Geometry.b / 2;
Segment = Surface;
Segment.Name = Name;
Segment.YInboard = EtaIn * HalfSpan;
Segment.YOutboard = EtaOut * HalfSpan;
Segment.SpanFraction = EtaOut - EtaIn;
Segment.ChordFraction = ChordFraction;
Segment = SetTrialGeometry(Aircraft, Segment);

end

function [Trial] = CombineSegments(Surface, Segments, Name)
% Combine colocated physical segments for one authority check.

Trial = Surface;
Trial.Name = Name;
Trial.Segments = Segments;
YInboard = cellfun(@(Segment) Segment.YInboard, Segments);
YOutboard = cellfun(@(Segment) Segment.YOutboard, Segments);
ChordFraction = cellfun(@(Segment) Segment.ChordFraction, Segments);
AreaFraction = cellfun(@(Segment) Segment.AreaFraction, Segments);
Trial.YInboard = min(YInboard);
Trial.YOutboard = max(YOutboard);
Trial.SpanFraction = Trial.YOutboard - Trial.YInboard;
Trial.ChordFraction = max(ChordFraction);
Trial.AreaFraction = sum(AreaFraction);

end

function [Best] = PackSharedBest(PitchTrial, RollTrial, Dual, PitchChecks, RollChecks, PhysicalArea)
% Store selected shared-layout fields without double-counting the dual-use zone.

Best.Elevator = PitchTrial;
Best.Elevator.PhysicalAreaFraction = PitchTrial.AreaFraction - Dual.AreaFraction;
Best.Elevator.Checks = PitchChecks;
Best.Elevator.MaxDeflection = PitchChecks.MaxDeflection;
Best.Aileron = RollTrial;
Best.Aileron.PhysicalAreaFraction = RollTrial.AreaFraction - Dual.AreaFraction;
Best.Aileron.Checks = RollChecks;
Best.Aileron.MaxDeflection = RollChecks.MaxDeflection;
Best.Dual = Dual;
Best.Dual.Checks = struct();
Best.Dual.MaxDeflection = max(PitchChecks.MaxDeflection, RollChecks.MaxDeflection);
Best.PitchChecks = PitchChecks;
Best.RollChecks = RollChecks;
Best.PhysicalArea = PhysicalArea;

end

function [Sizing] = SizeCoupledElevatorGrid(Aircraft, Options, CheckFunction)
% Sweep inboard and outboard pitch elevons together as one pitch-control system.

Inboard = Options(1);
Outboard = Options(2);

InSpanFractions = Inboard.SpanFractions(:);
InChordFractions = Inboard.ChordFractions(:);
OutSpanFractions = Outboard.SpanFractions(:);
OutChordFractions = Outboard.ChordFractions(:);

Feasible = false(length(InSpanFractions), length(InChordFractions), length(OutSpanFractions), length(OutChordFractions));
AreaFraction = zeros(size(Feasible));
MaxDeflection = zeros(size(Feasible));
Results = cell(size(Feasible));
Trials = cell(size(Feasible));

for iinchord = 1:length(InChordFractions)
    for iinspan = 1:length(InSpanFractions)
        InTrial = Inboard;
        InTrial.SpanFraction = InSpanFractions(iinspan);
        InTrial.ChordFraction = InChordFractions(iinchord);
        InTrial = SetTrialGeometry(Aircraft, InTrial);

        for ioutchord = 1:length(OutChordFractions)
            for ioutspan = 1:length(OutSpanFractions)
                OutTrial = Outboard;
                OutTrial.SpanFraction = OutSpanFractions(ioutspan);
                OutTrial.ChordFraction = OutChordFractions(ioutchord);
                OutTrial = SetTrialGeometry(Aircraft, OutTrial);

                Trial = InTrial;
                Trial.Name = "Combined pitch elevons";
                Trial.AreaFraction = InTrial.AreaFraction + OutTrial.AreaFraction;
                Trial.Inboard = InTrial;
                Trial.Outboard = OutTrial;

                Checks = CheckFunction(Trial);
                Feasible(iinspan, iinchord, ioutspan, ioutchord) = Checks.Feasible;
                AreaFraction(iinspan, iinchord, ioutspan, ioutchord) = Trial.AreaFraction;
                MaxDeflection(iinspan, iinchord, ioutspan, ioutchord) = Checks.MaxDeflection;
                Results{iinspan, iinchord, ioutspan, ioutchord} = Checks;
                Trials{iinspan, iinchord, ioutspan, ioutchord} = Trial;
            end
        end
    end
end

CandidateArea = AreaFraction;
CandidateArea(~Feasible) = Inf;
[BestArea, BestIndex] = min(CandidateArea(:));

if isinf(BestArea)
    [~, BestIndex] = max(AreaFraction(:));
    Converged = 0;
else
    Converged = 1;
end

Trial = Trials{BestIndex};
Sizing.Name = Trial.Name;
Sizing.SpanFraction = Trial.Inboard.SpanFraction + Trial.Outboard.SpanFraction;
Sizing.ChordFraction = max(Trial.Inboard.ChordFraction, Trial.Outboard.ChordFraction);
Sizing.AreaFraction = Trial.AreaFraction;
Sizing.EtaControl = Trial.EtaControl;
Sizing.Converged = Converged;
Sizing.Checks = Results{BestIndex};
Sizing.Inboard = Trial.Inboard;
Sizing.Outboard = Trial.Outboard;
Sizing.Feasible = Feasible;
Sizing.AreaFractions = AreaFraction;
Sizing.MaxDeflection = MaxDeflection;
Sizing.Options = Options;

end

function [Trial] = SetTrialGeometry(Aircraft, Trial)
% Convert candidate fractions into physical geometry when placement is known.

if isfield(Trial, 'UsePhysicalArea') && Trial.UsePhysicalArea
    Sref = Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS;
    Geom = Aircraft.Specs.Dynamics.Geometry;

    if ~isfield(Trial, 'YInboard')
        AvailableSpan = Trial.YOutboard - Trial.YInboardMin;
        AileronSpan = Trial.SpanFraction * AvailableSpan;
        Trial.YInboard = Trial.YOutboard - AileronSpan;
    end

    y = linspace(Trial.YInboard, Trial.YOutboard, 31);
    if isfield(Trial, 'ChordEta') && isfield(Trial, 'ChordLength')
        LocalChord = interp1(Trial.ChordEta, Trial.ChordLength, y / (Geom.b / 2), 'linear', 'extrap');
    elseif isfield(Trial, 'ReferenceChord')
        LocalChord = Trial.ReferenceChord * ones(size(y));
    else
        LocalChord = (Sref / Geom.b) * ones(size(y));
    end

    Trial.AreaFraction = 2 * trapz(y, Trial.ChordFraction * LocalChord) / Sref;
else
    Trial.AreaFraction = Trial.SpanFraction * Trial.ChordFraction;
end

end

function [Checks] = CheckElevator(Aircraft, Cases, Elevator)
% Group the longitudinal requirements that consume elevator authority.

Checks.Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.LongitudinalTrim, Elevator);
Checks.Pullup = DynamicsPkg.CheckPullup(Aircraft, Cases.Pullup, Elevator);
Checks.Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, Cases.TakeoffRotation, Elevator);
Checks.Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.CruiseTrim, Elevator);
Checks.Feasible = Checks.Trim.Feasible && Checks.Pullup.Feasible && ...
                  Checks.Rotation.Feasible && Checks.Cruise.Feasible;
Checks.MaxDeflection = max(abs([Checks.Trim.Delta; ...
                                Checks.Pullup.DeltaFinal; ...
                                Checks.Cruise.Delta; ...
                                Cases.TakeoffRotation.DeltaElevator]));

end

function [Checks] = CheckAileron(Aircraft, Cases, Aileron)
% Keep the roll requirement on the aileron surface only.

Checks.Bank = DynamicsPkg.CheckTimeToBank(Aircraft, Cases.TimeToBank, Aileron);
Checks.Feasible = Checks.Bank.Feasible;
Checks.MaxDeflection = abs(Checks.Bank.Delta);

end

function [Checks] = CheckRudder(Aircraft, Cases, Rudder)
% Keep directional/yaw authority on the rudder surface only.

Checks.Direction = DynamicsPkg.CheckDirectionalTrim(Aircraft, Cases.DirectionalTrim, Rudder);
Checks.Feasible = Checks.Direction.Feasible;
Checks.MaxDeflection = abs(Checks.Direction.Delta);

end
