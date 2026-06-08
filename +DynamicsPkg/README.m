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
% Simple trim workflow:
%
%     Aircraft = Main(AircraftSpecsPkg.Example, @MissionProfilesPkg.ParametricRegional);
%     Aircraft.Specs.Dynamics.Longitudinal.CLalpha = 4.8;
%     Aircraft.Specs.Dynamics.Longitudinal.CL0 = 0.15;
%     Aircraft.Specs.Dynamics.Longitudinal.CLdelta = 0.30;
%     Aircraft.Specs.Dynamics.Longitudinal.Cm0 = 0.015;
%     Aircraft.Specs.Dynamics.Longitudinal.Cmalpha = -0.35;
%     Aircraft.Specs.Dynamics.Longitudinal.Cmdelta = -0.85;
%     Aircraft.Specs.Dynamics.Longitudinal.CD0 = 0.019;
%     Aircraft.Specs.Dynamics.Longitudinal.K = 0.050;
%     Aircraft.Specs.Dynamics.Longitudinal.XrefMAC = 0.25;
%     TrimCase = DynamicsPkg.SweepTrimEnvelope(Aircraft);
%     TrimCase.XcgMAC = 0.32;
%     Trim = DynamicsPkg.TrimLongitudinal(Aircraft, TrimCase);
%
% BWB shared-elevon sizing workflow:
%
%     Cases = DynamicsPkg.BuildControlSizingCases(Aircraft);
%     Surfaces.Elevator.EtaControl = 0.85;
%     Surfaces.Elevator.ChordFractions = 0.25;
%     Surfaces.Aileron.EtaControl = 0.85;
%     Surfaces.DualElevon.ChordFractions = 0.40;
%     Surfaces.Aileron.ChordFractions = 0.40;
%     Surfaces.Aileron.ReferenceChord = ...
%         (Aircraft.Specs.Weight.MTOW / Aircraft.Specs.Aero.W_S.SLS) / Aircraft.Specs.Dynamics.Geometry.b;
%     Surfaces.Aileron.SectionClDelta = 2.5;
%     Surfaces.Rudder.EtaControl = 0.85;
%     Surfaces.Rudder.ChordFractions = linspace(0.10, 0.35, 20)';
%     Surfaces.Rudder.SpanFractions = linspace(0.05, 0.80, 152)';
%     Sizing = DynamicsPkg.OptimizeSharedElevons(Aircraft, Cases, Surfaces);
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
% pitch-only inboard section, a dual-use outboard section, a roll-only
% outboard section, and a separately sized winglet rudder.
%
end
