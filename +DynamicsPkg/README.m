function [] = README()
%
% Dynamics Package (+DynamicsPkg)
%
% This package contains early conceptual flight-dynamics checks that can be
% run around FAST's aircraft sizing loop. The first implemented use case is
% longitudinal trim and elevon sizing for BWB-style aircraft, where the
% wing sizing, trim drag, and control-surface authority are coupled.
%
% The trim equations are from:
%
%     Nelson, R. C. Flight Stability and Automatic Control.
%     Section 2.4.2, Elevator Angle to Trim, Equations 2.47-2.51.
%
% Typical workflow:
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
%     TrimCase.Elevon.EtaControl = 0.85;
%     TrimCase.Elevon.ChordFraction = 0.25;
%     Sizing = DynamicsPkg.SizeElevon(Aircraft, TrimCase);
%     Aircraft = DynamicsPkg.ControlSurfacePenalty(Aircraft, Sizing);
%
% A standalone demo is available with:
%
%     [Sizing, TrimCase, CgSweep] = DynamicsPkg.BWB_Dynamics_Demo();
%
% The trim solver does not infer stability derivatives. Provide them in
% Aircraft.Specs.Dynamics.Longitudinal.
%
% Nelson's trim equations are applied only after shifting supplied pitching
% moments to the CG. The shift is applied to Cm0, Cmalpha, and Cmdelta
% before Eq. 2.47-2.51 are evaluated.
%
% Elevon area is approximated as span fraction times chord fraction. The
% trim derivatives use EtaControl * AreaFraction directly.
%
% The current implementation is intentionally low order. It is meant to
% expose control-authority feasibility during conceptual sizing, not replace
% a nonlinear 6DOF simulation or high-fidelity aero database.
%
end
