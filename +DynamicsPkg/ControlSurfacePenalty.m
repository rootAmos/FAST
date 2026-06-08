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
%     Sizing   - output from DynamicsPkg.SizeElevon or SizeControlSurfaces.
%                size/type/units: 1-by-1 / struct / []
%
% OUTPUTS:
%     Aircraft - aircraft structure with updated penalty fields.
%                size/type/units: 1-by-1 / struct / []
%

AreaFraction = Sizing.AreaFraction;

if isfield(Sizing, 'WeightPenaltyFactor')
    WeightPenaltyFactor = Sizing.WeightPenaltyFactor;
else
    WeightPenaltyFactor = 0.08;
end

if isfield(Aircraft.Specs.Weight, 'WairfCF') && ~isnan(Aircraft.Specs.Weight.WairfCF)
    WairfCF = Aircraft.Specs.Weight.WairfCF;
else
    WairfCF = 1;
end

WeightPenalty = 1 + WeightPenaltyFactor * AreaFraction;

TrimDragPenalty = 0;
if isfield(Sizing, 'Elevator')
    CDcontrol = [Sizing.Elevator.Checks.Trim.Trim.CDcontrol; Sizing.Elevator.Checks.Cruise.Trim.CDcontrol];
    CDclean = [Sizing.Elevator.Checks.Trim.Trim.CDclean; Sizing.Elevator.Checks.Cruise.Trim.CDclean];
    TrimDragPenalty = max(CDcontrol ./ CDclean);
elseif isfield(Sizing, 'Checks')
    CDcontrol = [Sizing.Checks.Trim.Trim.CDcontrol; Sizing.Checks.Cruise.Trim.CDcontrol];
    CDclean = [Sizing.Checks.Trim.Trim.CDclean; Sizing.Checks.Cruise.Trim.CDclean];
    TrimDragPenalty = max(CDcontrol ./ CDclean);
end

DragPenalty = 1 + TrimDragPenalty;

Aircraft.Specs.Weight.WairfCF = WairfCF * WeightPenalty;

if isfield(Aircraft.Specs.Aero, 'L_D')
    Aircraft.Specs.Aero.L_D.Clb = Aircraft.Specs.Aero.L_D.Clb / DragPenalty;
    Aircraft.Specs.Aero.L_D.Crs = Aircraft.Specs.Aero.L_D.Crs / DragPenalty;
    Aircraft.Specs.Aero.L_D.Des = Aircraft.Specs.Aero.L_D.Des / DragPenalty;
end

Aircraft.Dynamics.ControlSurface.AreaFraction = AreaFraction;
if isfield(Sizing, 'Elevator')
    Aircraft.Dynamics.ControlSurface.Elevator = Sizing.Elevator;
    Aircraft.Dynamics.ControlSurface.Aileron = Sizing.Aileron;
    Aircraft.Dynamics.ControlSurface.Rudder = Sizing.Rudder;
else
    Aircraft.Dynamics.ControlSurface.SpanFraction = Sizing.SpanFraction;
    Aircraft.Dynamics.ControlSurface.ChordFraction = Sizing.ChordFraction;
    Aircraft.Dynamics.ControlSurface.EtaControl = Sizing.EtaControl;
end
Aircraft.Dynamics.ControlSurface.WeightPenalty = WeightPenalty;
Aircraft.Dynamics.ControlSurface.DragPenalty = DragPenalty;
Aircraft.Dynamics.ControlSurface.TrimDragPenalty = TrimDragPenalty;

end
