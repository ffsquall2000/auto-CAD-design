# Weld-Sim 3D Capability Analysis

**Date:** 2026-03-06
**Server:** 180.152.71.166:/opt/weld-sim
**Purpose:** Assess existing 3D drawing capabilities and identify gaps for machine frame, assembly, and engineering drawing generation.

## 1. Existing Tech Stack

### Frontend 3D Rendering

| Component | Technology | File | Capability |
|-----------|-----------|------|------------|
| FEA Viewer | Three.js v0.183 | `FEAViewer.vue` | Full Three.js pipeline: WebGLRenderer + OrbitControls + custom GLSL shaders |
| VTK Viewport | VTK.js v35 | `VtkViewport.vue` | VTK.js render pipeline: GenericRenderWindow + Actor/Mapper + color transfer |
| Light Preview | Canvas 2D | `ThreeViewer.vue` | Software rendering: perspective projection + painter's algorithm |
| Colormap Shader | GLSL | `colormap.vert/frag.glsl` | Scalar field → colormap texture + Lambertian shading |

### Frontend 3D Composables

| Composable | File | Capability |
|-----------|------|------------|
| useThreeScene | `useThreeScene.ts` | Three.js scene setup, camera, controls, lighting, resize |
| useVtkViewer | `useVtkViewer.ts` | VTK.js pipeline: polydata, scalar coloring, display modes (4 presets) |
| useColormap | `useColormap.ts` | 5 colormaps (jet/viridis/coolwarm/rainbow/grayscale), LUT textures |
| useClipping | `useClipping.ts` | X/Y/Z axis clipping planes with invert |
| useAnimation | `useAnimation.ts` | Mode shape animation: play/pause/step, speed/amplitude control |
| useIsosurface | `useIsosurface.ts` | Marching Tetrahedra via Web Worker |
| useMeshLoader | `useMeshLoader.ts` | Binary mesh loading with cache |

### Backend Geometry

| Generator | File | Shapes | STEP Export |
|-----------|------|--------|-------------|
| Horn | `horn_generator.py` | flat, cylindrical, exponential, blade, stepped | Yes (CadQuery) |
| Anvil | `anvil_generator.py` | flat, groove, knurled, contour | Yes (CadQuery) |
| Booster | `booster_generator.py` | uniform, stepped, exponential, catenoidal | Via Gmsh mesh |
| Mesher | `mesher.py` | TET4/TET10 from parametric horns | N/A (mesh only) |

### Backend FEA Assembly

| Module | File | Purpose |
|--------|------|---------|
| AssemblyBuilder | `assembly_builder.py` | FEA matrix coupling (K/M) via penalty springs |
| Component Detector | `component_detector.py` | STEP file component auto-classification |
| FEA Process Runner | `fea_process_runner.py` | Subprocess isolation for heavy computation |

### Server Library Versions (Verified)

| Library | Version | Status |
|---------|---------|--------|
| CadQuery | 2.7.0 | Installed, Assembly + constraints work |
| Gmsh | 4.15.1 | Installed |
| NumPy | 2.4.2 | Installed |
| SciPy | 1.17.1 | Installed |
| OCP HLRBRep | bundled | Hidden line removal projection works |
| ezdxf | 1.4.3 | DXF with dimensions works |
| reportlab | 4.4.10 | PDF generation available |
| matplotlib | 3.10.8 | 2D plotting available |
| Pillow | 12.1.1 | Image processing available |

### Verified Capabilities (Tested)

- CadQuery SVG export: `cq.exporters.getSVG(solid.val())` → works (2420 bytes for box)
- CadQuery DXF export: `cq.exporters.exportDXF()` → works (18061 bytes)
- CadQuery Assembly: `Assembly().add().constrain()` → works
- Assembly STEP export: `assembly.save()` → works (33510 bytes)
- OCP HLR projection: `HLRBRep_Algo` + `HLRBRep_HLRToShape` → visible + hidden edges extracted
- ezdxf dimensioning: `msp.add_linear_dim()` → works (15968 bytes)

## 2. Capability Gaps

### What Can Be Done Now

| Capability | Status |
|-----------|--------|
| Horn parametric modeling → STEP | Yes |
| Anvil parametric modeling → STEP | Yes |
| Booster mesh generation | Yes |
| FEA scalar field visualization | Yes (VTK.js + Three.js) |
| Mode shape animation | Yes |
| Isosurface extraction | Yes (Web Worker) |
| Binary mesh transfer | Yes |
| Clipping planes | Yes |

### What Cannot Be Done (Gaps)

| Gap | Severity | Description |
|-----|----------|-------------|
| Machine frame modeling | **Critical** | No frame generator exists |
| CAD-level assembly | **Critical** | Only FEA matrix coupling, no geometric positioning |
| 2D engineering drawings | **Critical** | No projection, dimensioning, or drawing output |
| Multi-body 3D visualization | **High** | Frontend only supports single mesh |
| Explosion view | **High** | Not implemented |
| Assembly tree panel | **High** | Not implemented |
| DXF/DWG drawing output | **Critical** | Not implemented |
| Dimension/tolerance annotation | **Critical** | Not implemented |
| BOM generation | **Medium** | Not implemented |
| STEP file import for frames | **Low** | component_detector.py exists but limited |

## 3. Decision: Hybrid Approach (Option C)

After evaluating three approaches:
- **Option A:** CadQuery full-stack (ezdxf for DXF)
- **Option B:** FreeCAD headless service
- **Option C:** Hybrid (CadQuery + matplotlib/reportlab + ezdxf) ← **Selected**

**Rationale:** matplotlib provides more flexible 2D rendering for reference-grade PDF output while ezdxf handles formal DXF output. Both are already installed. No new dependencies needed.

## 4. Implementation Plan Summary

| Phase | Module | Key Deliverables |
|-------|--------|-----------------|
| 1 | Frame Generator | 3 frame types with mfg detail, STEP export |
| 2 | Assembly Manager | Auto-positioning, STEP assembly, BOM |
| 3 | Drawing Engine | Projector + layout + dimensioning + 3 renderers |
| 4 | Frontend Assembly | VTK.js multi-actor + explosion + tree |
| 5 | Frontend Drawing | Drawing preview + export workbench |
| 6 | Integration | Testing + i18n |

See `docs/plans/2026-03-06-frame-assembly-drawing-design.md` for full design.
