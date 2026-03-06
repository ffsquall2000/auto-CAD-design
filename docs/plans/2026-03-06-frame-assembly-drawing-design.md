# Design: Machine Frame, Assembly, and Engineering Drawing System

**Date:** 2026-03-06
**Status:** Approved
**Approach:** Hybrid (CadQuery modeling + matplotlib/reportlab rendering + ezdxf DXF)

## 1. Overview

Extend the weld-sim platform with three missing capabilities:

1. **Machine Frame Generator** — Parametric 3D modeling of welding machine frames (C-frame, gantry, cantilever) with manufacturing-level detail
2. **CAD Assembly Manager** — Multi-body assembly with rule-based auto-positioning, STEP export, and BOM generation
3. **Frontend Assembly Visualization** — VTK.js multi-actor rendering with explosion view and assembly tree
4. **2D Drawing Engine** — OCP HLR projection → matplotlib/reportlab PDF + ezdxf DXF + SVG, with auto-dimensioning

### Tech Stack (all already installed on server)

| Library | Version | Role |
|---------|---------|------|
| CadQuery | 2.7.0 | Parametric CAD modeling, Assembly, STEP export |
| OCP HLRBRep | (bundled) | 3D → 2D hidden-line-removal projection |
| ezdxf | 1.4.3 | DXF engineering drawing output with dimensions |
| reportlab | 4.4.10 | PDF generation |
| matplotlib | 3.10.8 | 2D rendering for reference-grade drawings |
| Pillow | 12.1.1 | Image thumbnails |
| VTK.js | 35.2.0 | Frontend 3D multi-body rendering |
| Three.js | 0.183.2 | Frontend 3D (existing FEA viewer) |

**No new dependencies required.**

---

## 2. Module 1: Machine Frame Generator

### 2.1 File Structure

```
ultrasonic_weld_master/plugins/geometry_analyzer/
  frame_generator.py              # NEW: parametric frame geometry
web/routers/
  frame.py                        # NEW: frame API endpoints
web/services/
  frame_service.py                # NEW: frame business logic
```

### 2.2 Data Model

```python
@dataclass
class FrameParams:
    frame_type: str = "c_frame"    # c_frame | gantry | cantilever

    # Overall dimensions (mm)
    base_width: float = 400.0
    base_depth: float = 300.0
    base_height: float = 40.0
    column_height: float = 500.0
    column_width: float = 80.0
    column_depth: float = 80.0
    beam_length: float = 300.0     # beam overhang length
    beam_height: float = 60.0

    # Gantry-specific
    column_spacing: float = 350.0  # dual-column spacing

    # Functional features
    cylinder_bore_dia: float = 50.0    # pneumatic cylinder bore
    guide_bore_dia: float = 25.0       # guide pillar bore
    guide_bore_spacing: float = 120.0
    t_slot_width: float = 14.0
    t_slot_depth: float = 10.0
    t_slot_count: int = 3
    mounting_hole_dia: float = 12.0    # base mounting holes
    mounting_hole_count: int = 4

    # Manufacturing detail
    chamfer_mm: float = 1.0
    fillet_radius_mm: float = 3.0
    rib_thickness: float = 10.0        # reinforcement rib thickness
    rib_count: int = 2

    # Material
    material: str = "Steel AISI 1045"
```

### 2.3 Frame Types

Each frame type is composed of sub-parts built and combined via CadQuery boolean operations:

**C-Frame:**
- Base plate: rectangular + T-slots + mounting holes
- Single column: rectangular + reinforcement ribs + chamfers
- Cantilever beam: rectangular + cylinder bore + guide bores + fillets

**Gantry Frame:**
- Base plate: rectangular + T-slots + mounting holes
- Dual columns: symmetric, rectangular + ribs
- Bridge beam: spanning columns + cylinder bore + guide bores

**Cantilever Frame:**
- Base plate: circular/rectangular + mounting holes
- Column: cylindrical/rectangular + bearing seat
- Swing arm: beam + cylinder mount + rotation joint

### 2.4 Output

```python
@dataclass
class FrameGenerationResult:
    parts: dict[str, Any]        # CadQuery solids keyed by part name
    mesh_preview: dict           # {part_name: {vertices, faces}} for 3D preview
    step_data: bytes | None      # STEP file bytes (assembly)
    stl_data: bytes | None       # STL file bytes
    frame_type: str
    dimensions: dict
    volume_mm3: float
    mass_kg: float               # based on material density
    bom: list[dict]              # bill of materials per part
    has_cad_export: bool
```

### 2.5 API Endpoints

```
POST /api/v2/frame/generate          → FrameGenerationResult
GET  /api/v2/frame/{id}/step         → STEP file download
GET  /api/v2/frame/{id}/mesh         → mesh preview data
GET  /api/v2/frame/types             → list available frame types
GET  /api/v2/frame/defaults/{type}   → default params for frame type
```

### 2.6 Pattern Consistency

Follows the same pattern as `horn_generator.py` and `anvil_generator.py`:
- CadQuery primary path with NumPy fallback
- Guarded CadQuery import with `HAS_CADQUERY` flag
- `generate()` method as main entry point
- Dataclass for params and result

---

## 3. Module 2: CAD Assembly Manager

### 3.1 File Structure

```
ultrasonic_weld_master/plugins/geometry_analyzer/
  assembly_manager.py             # NEW: CAD-level assembly
web/routers/
  assembly_cad.py                 # NEW: assembly API (separate from FEA assembly)
web/services/
  assembly_cad_service.py         # NEW: assembly service
```

### 3.2 Data Model

```python
@dataclass
class AssemblyComponent:
    name: str                     # "horn", "booster", "frame_base", etc.
    solid: Any                    # CadQuery Workplane/Shape
    color: str = "#4488cc"
    position: tuple = (0, 0, 0)  # translation (x, y, z) mm
    rotation: tuple = (0, 0, 0)  # rotation (rx, ry, rz) degrees
    parent: str | None = None    # parent component name (for tree)
    category: str = "component"  # frame | tooling | workpiece

@dataclass
class WeldingMachineAssembly:
    frame: AssemblyComponent
    horn: AssemblyComponent
    booster: AssemblyComponent | None
    anvil: AssemblyComponent
    extras: list[AssemblyComponent] = field(default_factory=list)
```

### 3.3 Auto-Positioning Rules

Instead of CadQuery constraint solving (slow and unreliable), use **rule-based auto-positioning** along the Z axis:

```
Z-axis stacking (top to bottom):
  beam_bottom_z = base_height + column_height + beam_height
  ├── Cylinder (mounted in beam cylinder bore)
  ├── Horn top   = beam_bottom_z - stroke_length
  ├── Booster top = horn_bottom
  └── Anvil top  = base_top_z (on T-slot)

XY centering:
  All welding stack components centered at (0, 0)
  Frame base centered at (0, 0, 0)
```

### 3.4 Output

```python
@dataclass
class AssemblyResult:
    cq_assembly: Any              # CadQuery Assembly object
    step_path: str                # exported assembly STEP file
    component_tree: dict          # tree JSON for frontend
    mesh_data: dict               # per-component tessellation
    bom: list[dict]               # bill of materials
    total_mass_kg: float
    explosion_offsets: dict       # per-component explosion vectors
```

### 3.5 Explosion Vector Calculation

For each component, compute the offset direction from the assembly centroid:

```python
def compute_explosion_offsets(components):
    centroid = mean([c.position for c in components])
    offsets = {}
    for c in components:
        direction = normalize(c.position - centroid)
        distance = max_dimension * 0.3  # 30% of assembly size
        offsets[c.name] = direction * distance
    return offsets
```

### 3.6 API Endpoints

```
POST /api/v2/assembly-cad/build         → AssemblyResult
GET  /api/v2/assembly-cad/{id}/step     → STEP download
GET  /api/v2/assembly-cad/{id}/tree     → component tree JSON
GET  /api/v2/assembly-cad/{id}/mesh     → per-component mesh data
GET  /api/v2/assembly-cad/{id}/bom      → BOM list
POST /api/v2/assembly-cad/{id}/explode  → explosion offsets
```

---

## 4. Module 3: Frontend Assembly Visualization

### 4.1 File Structure

```
frontend/src/
  components/assembly/
    AssemblyViewer.vue            # VTK.js multi-actor 3D viewport
    AssemblyTree.vue              # tree panel with visibility toggles
    ExplosionSlider.vue           # explosion factor slider (0-1)
    BomTable.vue                  # BOM display table
    ComponentColorPicker.vue      # per-component color/opacity
  composables/
    useAssemblyViewer.ts          # VTK.js multi-actor management
  stores/
    assemblyCad.ts                # assembly state (Pinia)
  views/
    AssemblyWorkbench.vue         # full assembly workbench page
  api/
    assembly-cad.ts               # typed API client
```

### 4.2 Multi-Actor VTK.js Architecture

Extends existing `useVtkViewer.ts` pattern but manages multiple actors:

```typescript
interface AssemblyActor {
  name: string
  actor: vtkActor
  mapper: vtkMapper
  polyData: vtkPolyData
  originalPosition: [number, number, number]
  explosionOffset: [number, number, number]
  visible: boolean
  color: [number, number, number]
  opacity: number
}
```

Key behaviors:
- Each component → independent VTK Actor with own color
- Explosion: `actor.position = original + factor * offset` (animated via RAF)
- Highlight: click tree node → highlight actor (edge visibility + color shift)
- Isolation: right-click → make all other actors transparent

### 4.3 Assembly Tree Panel

```
📦 Welding Machine Assembly
  ├── 🔧 Frame
  │   ├── [👁] Base
  │   ├── [👁] Column
  │   └── [👁] Beam
  ├── 🔩 Welding Stack
  │   ├── [👁] Horn
  │   ├── [👁] Booster
  │   └── [👁] Transducer
  └── ⬛ [👁] Anvil
```

- Click node → highlight in 3D
- Eye icon → toggle visibility
- Right-click → isolate / make others transparent

### 4.4 Page Layout

```
┌──────────────────────────────────────────────────┐
│  TopBar: Assembly Name | Export STEP | Export DXF │
├──────────┬───────────────────────┬───────────────┤
│ Assembly │                       │  Properties   │
│ Tree     │    VTK.js 3D View     │  - Dimensions │
│          │                       │  - Material   │
│ BOM List │   [Explosion Slider]  │  - Color      │
├──────────┴───────────────────────┴───────────────┤
│  StatusBar: Parts | Total Mass | Assembly State  │
└──────────────────────────────────────────────────┘
```

### 4.5 Routing

Add to `frontend/src/router/index.ts`:
```typescript
{ path: '/assembly', name: 'Assembly', component: AssemblyWorkbench }
```

---

## 5. Module 4: 2D Drawing Engine

### 5.1 File Structure

```
ultrasonic_weld_master/plugins/drawing_engine/
  __init__.py
  projector.py                    # OCP HLR 3D→2D projection
  layout.py                       # drawing sheet layout engine
  dimension.py                    # dimensioning system
  renderer_matplotlib.py          # matplotlib → PDF/SVG/PNG
  renderer_dxf.py                 # ezdxf → DXF
  title_block.py                  # title block templates
  templates/
    a3_landscape.json             # A3 420×297mm landscape
    a4_portrait.json              # A4 297×210mm portrait
web/routers/
  drawing.py                      # drawing API endpoints
web/services/
  drawing_service.py              # drawing generation service
```

### 5.2 Pipeline

```
CadQuery Solid
    │
    ▼
projector.py (OCP HLRBRep)
    │  Input:  3D solid + view direction (front/top/right/iso)
    │  Output: 2D edge set {visible, hidden, outline}
    ▼
layout.py
    │  Input:  multiple view projections + paper size (A3/A4)
    │  Process: auto-arrange views, compute scale
    │  Output: DrawingLayout (view positions, scale, border)
    ▼
dimension.py
    │  Input:  DrawingLayout + original 3D dimensions
    │  Process: auto-dimension (envelope + features) + manual dims
    │  Output: DimensionSet (lines, text, arrows, tolerances)
    ▼
┌──────────────────┬──────────────────┬────────────────┐
│ renderer_mpl     │ renderer_dxf     │ SVG direct     │
│ → PDF (ref)      │ → DXF (formal)   │ → SVG (outline)│
└──────────────────┴──────────────────┴────────────────┘
```

### 5.3 Projector (OCP HLR)

```python
class ShapeProjector:
    STANDARD_VIEWS = {
        "front":  gp_Dir(0, -1, 0),
        "top":    gp_Dir(0, 0, 1),
        "right":  gp_Dir(1, 0, 0),
        "iso":    gp_Dir(1, -1, 1),
        "back":   gp_Dir(0, 1, 0),
        "bottom": gp_Dir(0, 0, -1),
    }

    def project(self, solid, view: str) -> ProjectionResult:
        """Project 3D solid to 2D edges using HLR."""
        # Returns: visible_lines, hidden_lines, outline as
        # lists of [(x1,y1), (x2,y2)] line segments

    def project_standard_three(self, solid) -> dict[str, ProjectionResult]:
        """Project front + top + right views."""
```

### 5.4 Drawing Layout

Standard three-view + isometric arrangement:

```
┌──────────────────────────────────────┐
│  Front View       │   Right View     │
│                   │                  │
│                   ├──────────────────┤
├───────────────────┤   Isometric View │
│  Top View         │                  │
│                   │                  │
├───────────────────┴──────────────────┤
│          Title Block                 │
│  Part Name | Material | Scale | Date │
└──────────────────────────────────────┘
```

Auto-scaling logic:
1. Compute bounding box of each view projection
2. Calculate maximum scale that fits all views within paper margins
3. Round down to standard scale (1:1, 1:2, 1:5, 1:10, 2:1, 5:1)

### 5.5 Dimensioning System

**Auto-dimensioning strategy:**

| Priority | Type | Description |
|----------|------|-------------|
| 1 | Envelope | Overall length, width, height |
| 2 | Holes | Diameter, center distance, pattern spacing |
| 3 | Slots | Width, depth, spacing |
| 4 | Steps | Step heights, widths |
| 5 | Fillets/Chamfers | Radius, chamfer dims |

**Manual annotation support:**
- Linear dimension (horizontal, vertical, aligned)
- Diameter / radius
- Angle
- Tolerance (bilateral ±, unilateral, fit class e.g. H7)
- Surface roughness (Ra values)
- Geometric tolerance (GD&T frames)
- Notes / leaders

### 5.6 Three Output Tiers

| Tier | Format | Content | Use Case |
|------|--------|---------|----------|
| **Outline** | SVG | Projected outlines + major dims | Communication, quoting |
| **Reference** | PDF (matplotlib) | Three views + full dims + title block | Workshop reference |
| **Formal** | DXF (ezdxf) | Three views + GD&T + layered + title block | CNC/workshop production |

### 5.7 DXF Layer Standard

```
Layer       Color    Linetype     Purpose
0           White    Continuous   Visible outline edges
HIDDEN      Green    DASHED       Hidden edges
CENTER      Red      CENTER       Center lines
DIM         Cyan     Continuous   Dimension lines and text
HATCH       Blue     Continuous   Section hatch patterns
TEXT        Yellow   Continuous   Text annotations
BORDER      White    Continuous   Drawing border (thick)
TITLE       White    Continuous   Title block
```

### 5.8 API Endpoints

```
POST /api/v2/drawing/generate           → DrawingResult
  body: {
    solid_id: str,
    views: ["front", "top", "right", "iso"],
    paper_size: "A3",
    scale: "auto",
    auto_dimension: true,
    title: str,
    material: str,
    tolerance_class: str
  }

GET  /api/v2/drawing/{id}/svg           → SVG file (outline tier)
GET  /api/v2/drawing/{id}/pdf           → PDF file (reference tier)
GET  /api/v2/drawing/{id}/dxf           → DXF file (formal tier)
GET  /api/v2/drawing/{id}/preview       → PNG thumbnail

POST /api/v2/drawing/{id}/dimensions    → add/modify dimensions
POST /api/v2/drawing/{id}/tolerance     → add tolerance annotations
```

---

## 6. Frontend Drawing Integration

### 6.1 Additional Components

```
frontend/src/
  components/drawing/
    DrawingPreview.vue            # SVG/PNG drawing preview
    DimensionEditor.vue           # interactive dimension editing
    PaperSelector.vue             # paper size/orientation picker
  views/
    DrawingWorkbench.vue          # drawing generation workbench
  api/
    drawing.ts                    # typed API client
```

### 6.2 Drawing Workbench

```
┌──────────────────────────────────────────────────┐
│  TopBar: Part Name | Paper Size | Scale | Export  │
├──────────┬───────────────────────────────────────┤
│ Settings │                                        │
│ - Views  │   Drawing Preview (SVG rendered)       │
│ - Scale  │                                        │
│ - Dims   │   ┌─────────┬──────────┐              │
│ - GD&T   │   │ Front   │ Right    │              │
│          │   │         ├──────────┤              │
│ Export   │   │ Top     │ Iso      │              │
│ [SVG]    │   └─────────┴──────────┘              │
│ [PDF]    │   ┌────────────────────┐              │
│ [DXF]    │   │ Title Block        │              │
│          │   └────────────────────┘              │
├──────────┴───────────────────────────────────────┤
│  StatusBar: Scale 1:2 | A3 Landscape | 12 dims   │
└──────────────────────────────────────────────────┘
```

---

## 7. i18n

All new user-facing strings must be added to both `zh-CN.json` and `en.json` following existing patterns with `$t('key')`.

---

## 8. Testing Strategy

| Module | Test Type | Framework |
|--------|-----------|-----------|
| frame_generator.py | Unit | pytest, mock CadQuery if needed |
| assembly_manager.py | Unit | pytest |
| projector.py | Unit | pytest (verify edge count, no crashes) |
| dimension.py | Unit | pytest (verify dimension values) |
| renderer_*.py | Integration | pytest (verify file output) |
| web/routers/frame.py | API | FastAPI TestClient |
| web/routers/drawing.py | API | FastAPI TestClient |
| Frontend components | Type check | vue-tsc --noEmit |

---

## 9. Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| OCP HLR edge extraction quality for complex shapes | Medium | Fall back to wireframe projection if HLR fails |
| Auto-dimensioning produces cluttered output | Medium | Implement collision detection, limit to top-N dimensions |
| CadQuery boolean operations slow for complex frames | Low | Cache generated solids, use NumPy preview fallback |
| VTK.js multi-actor performance with 10+ parts | Low | Merge small parts, use LOD |
| ezdxf DXF compatibility with various CAD software | Medium | Test with AutoCAD, SolidWorks, FreeCAD |

---

## 10. Implementation Order

Phase 1: Machine Frame Generator (backend only)
Phase 2: Assembly Manager (backend) + Assembly STEP export
Phase 3: Drawing Engine (projector + layout + renderers)
Phase 4: Frontend Assembly Visualization
Phase 5: Frontend Drawing Workbench
Phase 6: Integration testing + i18n
