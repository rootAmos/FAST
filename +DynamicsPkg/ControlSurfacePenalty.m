function [Aircraft] = ControlSurfacePenalty(Aircraft, Sizing)
%
% [Aircraft] = ControlSurfacePenalty(Aircraft, Sizing)
%
% Feed a low-order control-surface area penalty back into FAST by adjusting
% airframe weight calibration and mission L/D values.
%
% INPUTS:
%     Aircraft - FAST aircraft structure.
%                size/type/units: 1-by-1 / struct / []
%
%     Sizing   - output from DynamicsPkg.SizeElevon.
%                size/type/units: 1-by-1 / struct / []
%
% OUTPUTS:
%     Aircraft - aircraft structure with updated penalty fields.
%                size/type/units: 1-by-1 / struct / []
%

AreaFraction = Sizing.AreaFraction;

if isfield(Sizing, "WeightPenaltyFactor")
    WeightPenaltyFactor = Sizing.WeightPenaltyFactor;
else
    WeightPenaltyFactor = 0.08;
end

if isfield(Sizing, "TrimDragPenaltyFactor")
    TrimDragPenaltyFactor = Sizing.TrimDragPenaltyFactor;
else
    TrimDragPenaltyFactor = 0.06;
end

if isfield(Aircraft.Specs.Weight, "WairfCF") && ~isnan(Aircraft.Specs.Weight.WairfCF)
    WairfCF = Aircraft.Specs.Weight.WairfCF;
else
    WairfCF = 1;
end

WeightPenalty = 1 + WeightPenaltyFactor * AreaFraction;
DragPenalty = 1 + TrimDragPenaltyFactor * AreaFraction;

Aircraft.Specs.Weight.WairfCF = WairfCF * WeightPenalty;

if isfield(Aircraft.Specs.Aero, "L_D")
    Aircraft.Specs.Aero.L_D.Clb = Aircraft.Specs.Aero.L_D.Clb / DragPenalty;
    Aircraft.Specs.Aero.L_D.Crs = Aircraft.Specs.Aero.L_D.Crs / DragPenalty;
    Aircraft.Specs.Aero.L_D.Des = Aircraft.Specs.Aero.L_D.Des / DragPenalty;
end

Aircraft.Dynamics.ControlSurface.AreaFraction = AreaFraction;
Aircraft.Dynamics.ControlSurface.SpanFraction = Sizing.SpanFraction;
Aircraft.Dynamics.ControlSurface.ChordFraction = Sizing.ChordFraction;
Aircraft.Dynamics.ControlSurface.EtaControl = Sizing.EtaControl;
Aircraft.Dynamics.ControlSurface.WeightPenalty = WeightPenalty;
Aircraft.Dynamics.ControlSurface.DragPenalty = DragPenalty;

end
