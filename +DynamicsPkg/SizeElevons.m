function [Sizing] = SizeElevons(Aircraft, Cases, Elevon)
%
% [Sizing] = SizeElevons(Aircraft, Cases, Elevon)
%
% Size grouped elevons against trim, pull-up, bank, and rotation checks.
%

SpanFractions = Elevon.SpanFractions(:);
ChordFractions = Elevon.ChordFractions(:);

nspan = length(SpanFractions);
nchord = length(ChordFractions);
Feasible = false(nspan, nchord);
AreaFraction = zeros(nspan, nchord);
MaxDeflection = zeros(nspan, nchord);
Results = cell(nspan, nchord);

for ichord = 1:nchord
    for ispan = 1:nspan
        Trial = Elevon;
        Trial.SpanFraction = SpanFractions(ispan);
        Trial.ChordFraction = ChordFractions(ichord);
        Trial.AreaFraction = Trial.SpanFraction * Trial.ChordFraction;

        Checks.Trim = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.LongitudinalTrim, Trial);
        Checks.Pullup = DynamicsPkg.CheckPullup(Aircraft, Cases.Pullup, Trial);
        Checks.Bank = DynamicsPkg.CheckTimeToBank(Aircraft, Cases.TimeToBank, Trial);
        Checks.Rotation = DynamicsPkg.CheckTakeoffRotation(Aircraft, Cases.TakeoffRotation, Trial);
        Checks.Cruise = DynamicsPkg.CheckLongitudinalTrim(Aircraft, Cases.CruiseTrim, Trial);

        Feasible(ispan, ichord) = Checks.Trim.Feasible && ...
                                  Checks.Pullup.Feasible && ...
                                  Checks.Bank.Feasible && ...
                                  Checks.Rotation.Feasible && ...
                                  Checks.Cruise.Feasible;
        AreaFraction(ispan, ichord) = Trial.AreaFraction;
        MaxDeflection(ispan, ichord) = max(abs([Checks.Trim.Delta; ...
                                                Checks.Pullup.DeltaFinal; ...
                                                Checks.Cruise.Delta]));
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
Sizing.EtaControl = Elevon.EtaControl;
Sizing.Converged = Converged;
Sizing.Checks = Results{BestSpanIndex, BestChordIndex};
Sizing.SpanFractions = SpanFractions;
Sizing.ChordFractions = ChordFractions;
Sizing.AreaFractions = AreaFraction;
Sizing.Feasible = Feasible;
Sizing.MaxDeflection = MaxDeflection;

end
