# Auto CAD Design

Ultrasonic welding machine parametric CAD design system: machine frame generator, multi-body assembly manager, and 2D engineering drawing engine.

## Project Scope

This project extends the [weld-sim](https://github.com/ffsquall2000) ultrasonic welding simulation platform with production-grade CAD capabilities:

### Module 1: Machine Frame Generator
Parametric 3D modeling of welding machine frames with manufacturing-level detail.
- **Frame types:** C-frame, Gantry (portal), Cantilever (swing-arm)
- **Features:** T-slots, cylinder bores, guide pillar holes, mounting holes, chamfers, fillets, reinforcement ribs
- **Output:** CadQuery solid → STEP/STL export + mesh preview

### Module 2: CAD Assembly Manager
Multi-body assembly with rule-based auto-positioning.
- **Components:** Frame + Horn + Booster + Anvil + extras
- **Positioning:** Rule-based Z-axis stacking (no constraint solver)
- **Output:** CadQuery Assembly → STEP export + BOM + component tree

### Module 3: Frontend Assembly Visualization
VTK.js multi-actor 3D rendering in the browser.
- **Explosion view:** Animated disassembly with slider control
- **Assembly tree:** Hierarchical component panel with visibility toggles
- **Interaction:** Click-to-highlight, right-click-to-isolate

### Module 4: 2D Engineering Drawing Engine
OCP hidden-line-removal projection → multi-format output.

| Tier | Format | Content | Use Case |
|------|--------|---------|----------|
| Outline | SVG | Projected outlines + major dims | Communication, quoting |
| Reference | PDF | Three views + full dims + title block | Workshop reference |
| Formal | DXF | Three views + GD&T + layered + title block | CNC/workshop production |

## Tech Stack

All libraries are already available on the target server (no new dependencies):

| Library | Version | Role |
|---------|---------|------|
| CadQuery | 2.7.0 | Parametric CAD modeling, Assembly, STEP export |
| OCP HLRBRep | (bundled) | 3D → 2D projection |
| ezdxf | 1.4.3 | DXF engineering drawing with dimensions |
| reportlab | 4.4.10 | PDF generation |
| matplotlib | 3.10.8 | 2D rendering for reference-grade drawings |
| Gmsh | 4.15.1 | FEA mesh generation |
| VTK.js | 35.2.0 | Frontend 3D rendering |
| Vue 3 + TypeScript | latest | Frontend UI |
| FastAPI | latest | Backend API |

## Project Structure

```
docs/
  plans/
    2026-03-06-frame-assembly-drawing-design.md   # Full design document
  analysis/
    2026-03-06-weld-sim-3d-capability-analysis.md # Capability gap analysis
src/
  (implementation code - to be added)
```

## Implementation Phases

1. **Phase 1:** Machine Frame Generator (backend)
2. **Phase 2:** Assembly Manager (backend) + STEP export
3. **Phase 3:** Drawing Engine (projector + layout + renderers)
4. **Phase 4:** Frontend Assembly Visualization
5. **Phase 5:** Frontend Drawing Workbench
6. **Phase 6:** Integration testing + i18n

## Related Project

This project builds upon the existing [weld-sim](https://github.com/ffsquall2000) ultrasonic welding simulation platform deployed at `180.152.71.166:/opt/weld-sim`.

## License

Private / Internal Use
