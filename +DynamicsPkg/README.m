function [] = README()
%
% Dynamics Package (+DynamicsPkg)
%
% This package contains early conceptual flight-dynamics checks that can be
% run around FAST's aircraft sizing loop. The main BWB workflow sizes
% elevator, aileron, and rudder surfaces separately, where the wing sizing,
% trim drag, and control-surface authority are coupled.
%
% The trim equations are from:
%
%     Nelson, R. C. Flight Stability and Automatic Control.
%     Section 2.4.2, Elevator Angle to Trim, Equations 2.47-2.51.
%
% BWB shared-elevon sizing workflow:
%
%     Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);
%     Surfaces.MaxModelHalfSpanStation = MaxModelHalfSpanStation;
%     Surfaces.PanelStationWidth = FtToStation(5);
%     Surfaces.PitchStationRange = FtToStation([0, 10]);
%     Surfaces.OutboardStationRange = FtToStation([20, 45]);
%     Planform.TauControlEff = 0.85;
%     Planform.ChordEta = ChordEta;
%     Planform.ChordLength = ChordLength;
%     Planform.ChordLeadingEdgeX = ChordLeadingEdgeX;
%     Planform.ChordTrailingEdgeX = ChordTrailingEdgeX;
%     Surfaces.Elevator = Planform;
%     Surfaces.Elevator.ChordFractions = 0.25;
%     Surfaces.DualElevon = Planform;
%     Surfaces.DualElevon.ChordFractions = 0.40;
%     Surfaces.Aileron = Surfaces.DualElevon; % same physical outboard surface, used differentially for roll.
%     Surfaces.Rudder.TauControlEff = 0.85;
%     Surfaces.Rudder.ChordFractions = linspace(0.10, 0.35, 20)';
%     Surfaces.Rudder.SpanFractions = linspace(0.05, 0.80, 152)';
%     Sizing = DynamicsPkg.OptimizeSharedElevons(Aircraft, Cases, Surfaces);
%     Aircraft = DynamicsPkg.ControlSurfacePenalty(Aircraft, Sizing);
%
% A standalone demo is available with:
%
%     [Sizing, TrimCase, CgSweep] = DynamicsPkg.BWB_Dynamics_Demo();
%
% By default, the demo runs the shared-elevon optimizer and regenerates the
% control-surface area plot only. Use BWB_Dynamics_Demo(true) for the full
% report plots, BWB_Dynamics_Demo(true, true) to also re-optimize across
% the CG envelope, or BWB_Dynamics_Demo(true, false, true) to plot the
% selected elevon layout's CG sensitivity without re-optimizing.
%
% The trim solver does not infer stability derivatives. Provide them in
% Aircraft.Specs.Dynamics.Longitudinal.
%
% Nelson's trim equations are applied only after shifting supplied pitching
% moments to the CG. The shift is applied to Cm0, Cmalpha, and Cmdelta
% before Eq. 2.47-2.51 are evaluated.
%
% Control-surface area is integrated from the scaled BWB chord distribution
% when physical panel placement is available.
%
% Trim drag is modeled as:
%
%     DeltaCDtrim = CDdelta * AreaFraction * delta_e^2
%
% The current implementation is intentionally low order. It is meant to
% expose control-authority feasibility during conceptual sizing, not replace
% a nonlinear 6DOF simulation or high-fidelity aero database.
%
% OptimizeSharedElevons is the current BWB sizing entry point for a
% pitch-only inboard section, one dual-use outboard pitch/roll section, and
% a separately sized winglet rudder.
%
% ControlSurfacePenalty is the optional FAST feedback hook: it records the
% selected control surfaces on Aircraft.Dynamics.ControlSurface, adjusts the
% airframe weight calibration, and reduces mission L/D by the trim-drag
% penalty from the selected elevon checks.
%
end
