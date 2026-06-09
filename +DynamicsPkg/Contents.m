% Contents of DynamicsPkg package:
%
% The BWB demo uses the NASA/Boeing X-48 planform outline to generate the
% spanwise chord data, then scales that outline to the 100 ft span sizing
% case used by DynamicsPkg.BWB_Dynamics_Demo.
%
% BWB sizing flow:
%   DynamicsPkg.BWB_Dynamics_Demo
%     -> DynamicsPkg.BuildControlSizingCases
%     -> DynamicsPkg.OptimizeSharedElevons
%        -> DynamicsPkg.BuildStationPanels
%        -> DynamicsPkg.PanelCoefficients
%        -> DynamicsPkg.SolveSharedElevonChords  (CasADi/Ipopt solve)
%        -> DynamicsPkg.BuildSharedElevonSizing  (numeric Check* reports)
%     -> DynamicsPkg.PlotSharedElevonAreas
%
% A Mermaid version of this workflow is in docs/dynamics_workflow.md.
%
% README               - DynamicsPkg.README is a function.
%
% Functions
% -------------------------------------------------------------------------
% BuildControlSizingCases - DynamicsPkg.BuildControlSizingCases is a function.
% BuildSharedElevonSizing - DynamicsPkg.BuildSharedElevonSizing is a function.
% BuildStationPanels   - DynamicsPkg.BuildStationPanels is a function.
% BWB_Dynamics_Demo    - DynamicsPkg.BWB_Dynamics_Demo is a function.
% ControlFeasibilityTolerance - DynamicsPkg.ControlFeasibilityTolerance is a function.
% CheckLongitudinalTrim - DynamicsPkg.CheckLongitudinalTrim is a function.
% CheckDirectionalTrim - DynamicsPkg.CheckDirectionalTrim is a function.
% CheckPullup          - DynamicsPkg.CheckPullup is a function.
% CheckTakeoffRotation - DynamicsPkg.CheckTakeoffRotation is a function.
% CheckTimeToBank      - DynamicsPkg.CheckTimeToBank is a function.
% TrimLongitudinal     - DynamicsPkg.TrimLongitudinal is a function.
% OptimizeSharedElevons - DynamicsPkg.OptimizeSharedElevons is a function.
% PanelCoefficients   - DynamicsPkg.PanelCoefficients is a function.
% PlotSharedElevonAreas - DynamicsPkg.PlotSharedElevonAreas is a function.
% SizeRudder           - DynamicsPkg.SizeRudder is a function.
% SetupCasadi          - DynamicsPkg.SetupCasadi is a function.
% SolveSharedElevonChords - DynamicsPkg.SolveSharedElevonChords is a function.
% SweepPitchCaseAreas - DynamicsPkg.SweepPitchCaseAreas is a function.
% ControlSurfacePenalty - DynamicsPkg.ControlSurfacePenalty is a function.
% SweepElevonCgSensitivity - DynamicsPkg.SweepElevonCgSensitivity is a function.
