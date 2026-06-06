function [Sizing] = SizeElevon(Aircraft, TrimCase)
%
% [Sizing] = SizeElevon(Aircraft, TrimCase)
%
% Find the smallest elevon span fraction that trims all requested cases
% within a deflection limit.
%
% INPUTS:
%     Aircraft - FAST aircraft structure.
%                size/type/units: 1-by-1 / struct / []
%
%     TrimCase - trim condition and sizing settings.
%                size/type/units: 1-by-1 / struct / []
%
% OUTPUTS:
%     Sizing   - structure with required elevon geometry and trim result.
%                size/type/units: 1-by-1 / struct / []
%

SpanFractions = TrimCase.Elevon.SpanFractions(:);
ChordFraction = TrimCase.Elevon.ChordFraction;
EtaControl = TrimCase.Elevon.EtaControl;

% Rectangular approximation: Se / S = span_fraction * chord_fraction.
AreaFractions = SpanFractions .* ChordFraction;

% One feasibility flag per candidate elevon span.
Feasible = false(size(SpanFractions));
MaxAbsDeflection = zeros(size(SpanFractions));
TrimResults = cell(size(SpanFractions));

for isurf = 1:length(SpanFractions)
    % TrimLongitudinal applies eta_control * area_fraction to CLdelta/Cmdelta.
    TrimCase.Elevon.SpanFraction = SpanFractions(isurf);
    TrimCase.Elevon.AreaFraction = AreaFractions(isurf);
    Trim = DynamicsPkg.TrimLongitudinal(Aircraft, TrimCase);

    % Feasible means every trim case stays inside the deflection limit.
    Feasible(isurf) = all(Trim.Feasible);
    MaxAbsDeflection(isurf) = max(abs(Trim.DeltaTrim));
    TrimResults{isurf} = Trim;
end

% Select the smallest span that works. If none work, report the largest.
FirstFeasible = find(Feasible, 1, 'first');

if isempty(FirstFeasible)
    SelectedIndex = length(SpanFractions);
    Converged = 0;
else
    SelectedIndex = FirstFeasible;
    Converged = 1;
end

Sizing.SpanFraction = SpanFractions(SelectedIndex);
Sizing.ChordFraction = ChordFraction;
Sizing.AreaFraction = AreaFractions(SelectedIndex);
Sizing.EtaControl = EtaControl;
Sizing.Converged = Converged;
Sizing.Trim = TrimResults{SelectedIndex};
Sizing.SpanFractions = SpanFractions;
Sizing.AreaFractions = AreaFractions;
Sizing.Feasible = Feasible;
Sizing.MaxAbsDeflection = MaxAbsDeflection;

end
