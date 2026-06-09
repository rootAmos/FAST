# DynamicsPkg BWB Workflow

The BWB demo sizes shared elevon panels from the scaled X-48 planform. The
CasADi optimization chooses panel chord fractions; the `Check*` functions then
evaluate the selected geometry numerically for reporting and plots.

```mermaid
flowchart TD
    A["DynamicsPkg.BWB_Dynamics_Demo"] --> B["Load scaled X-48 chord table"]
    B --> C["BuildControlSizingCases"]
    C --> D["OptimizeSharedElevons"]

    D --> E["BuildStationPanels"]
    D --> F["PanelCoefficients"]
    F --> F1["Integrate panel area, CLdelta, Cmdelta, roll integral"]

    D --> G["SolveSharedElevonChords"]
    G --> G1["SetupCasadi"]
    G1 --> G2["Build CasADi Opti variables and constraints"]
    G2 --> G3["OptiProblem.solve with Ipopt"]

    G3 --> H["BuildSharedElevonSizing"]
    H --> I["CheckLongitudinalTrim"]
    H --> J["CheckPullup"]
    H --> K["CheckTakeoffRotation"]
    H --> L["CheckTimeToBank"]
    H --> M["SizeRudder and CheckDirectionalTrim"]

    H --> N["PlotSharedElevonAreas"]
    H --> O["Report selected design margins"]
    H --> S["Optional ControlSurfacePenalty FAST feedback"]
    A --> P{"RunCgSweep?"}
    P -- "true" --> Q["Re-run OptimizeSharedElevons across CG points"]
    P -- "false" --> R["Use baseline optimized sizing point"]
```

Run from the repository root:

```matlab
DynamicsPkg.BWB_Dynamics_Demo(true, true, false)
```
