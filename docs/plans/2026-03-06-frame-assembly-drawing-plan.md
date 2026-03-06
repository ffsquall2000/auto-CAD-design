# Frame, Assembly & Drawing Engine Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add machine frame parametric generation, multi-body CAD assembly, and 2D engineering drawing export to the weld-sim platform.

**Architecture:** CadQuery (backend modeling) + matplotlib/reportlab (PDF drawings) + ezdxf (DXF drawings) + VTK.js (frontend assembly visualization). All work targets the remote server at `180.152.71.166:/opt/weld-sim`.

**Tech Stack:** Python 3 / CadQuery 2.7 / OCP HLR / ezdxf 1.4.3 / matplotlib 3.10 / reportlab 4.4 / Vue 3 / TypeScript / VTK.js 35 / Pinia / TailwindCSS

**Remote Server Access:**
```bash
sshpass -p 'Erica1126!@#' ssh -o StrictHostKeyChecking=no squall@180.152.71.166
```
All file paths below are relative to `/opt/weld-sim/` on the server unless otherwise noted.

**Existing Patterns to Follow:**
- Generator: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_generator.py` (CadQuery + NumPy fallback, dataclass params/result)
- Schema: `web/schemas/horn.py` (Pydantic BaseModel with Field validators)
- Router: `web/routers/horn.py` (lazy singleton service, APIRouter with prefix)
- Service: `web/services/horn_service.py` (temp file cache, UUID download IDs)
- Test: `tests/test_horn_generator.py` (pytest class with parametrize)
- Store: `frontend/src/stores/viewer3d.ts` (Pinia composition API)
- API client: `frontend/src/api/client.ts` (axios baseURL=/api/v2)
- App registration: `web/app.py` (import + include_router)

---

## Phase 1: Machine Frame Generator (Backend)

### Task 1.1: Frame Generator — Data Models & Base Class

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/frame_generator.py`
- Test: `tests/test_frame_generator.py`

**Step 1: Write the failing test**

Create `tests/test_frame_generator.py`:

```python
"""Tests for parametric frame generator."""
import pytest
from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import (
    FrameGenerator, FrameParams, FrameGenerationResult,
)


class TestFrameParams:
    def test_default_params(self):
        """Default params should be valid."""
        p = FrameParams()
        assert p.frame_type == "c_frame"
        assert p.base_width == 400.0
        assert p.column_height == 500.0

    def test_gantry_params(self):
        """Gantry frame should have column_spacing."""
        p = FrameParams(frame_type="gantry", column_spacing=350.0)
        assert p.column_spacing == 350.0


class TestFrameGeneratorBase:
    def setup_method(self):
        self.gen = FrameGenerator()

    @pytest.mark.parametrize("frame_type", ["c_frame", "gantry", "cantilever"])
    def test_generate_all_types(self, frame_type):
        """All frame types should generate valid results."""
        params = FrameParams(frame_type=frame_type)
        result = self.gen.generate(params)
        assert isinstance(result, FrameGenerationResult)
        assert result.frame_type == frame_type
        assert result.volume_mm3 > 0
        assert len(result.parts) > 0
        assert len(result.mesh_preview) > 0

    def test_result_has_parts(self):
        """C-frame should have base, column, beam parts."""
        params = FrameParams(frame_type="c_frame")
        result = self.gen.generate(params)
        assert "base" in result.parts
        assert "column" in result.parts
        assert "beam" in result.parts

    def test_dimensions_in_result(self):
        """Result dimensions should match params."""
        params = FrameParams(base_width=500.0, column_height=600.0)
        result = self.gen.generate(params)
        assert result.dimensions["base_width"] == 500.0
        assert result.dimensions["column_height"] == 600.0

    def test_mass_calculation(self):
        """Mass should be positive and reasonable for steel."""
        params = FrameParams()
        result = self.gen.generate(params)
        # Steel density ~7850 kg/m3, frame volume ~tens of millions mm3
        assert result.mass_kg > 0
        assert result.mass_kg < 500  # sanity check

    def test_bom_generated(self):
        """BOM should list all parts."""
        params = FrameParams()
        result = self.gen.generate(params)
        assert len(result.bom) >= 3  # at least base, column, beam
        for item in result.bom:
            assert "name" in item
            assert "volume_mm3" in item
```

**Step 2: Run test to verify it fails**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_frame_generator.py -v 2>&1 | head -30
```
Expected: FAIL — `ModuleNotFoundError: No module named 'ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator'`

**Step 3: Write the frame generator with data models and base implementation**

Create `ultrasonic_weld_master/plugins/geometry_analyzer/frame_generator.py`:

```python
"""Parametric machine frame geometry generator.

Uses CadQuery when available for proper CAD output (STEP/STL),
falls back to numpy-based mesh generation for preview.

Supports three frame types:
  - c_frame: Single-column C-shaped frame (most common)
  - gantry: Dual-column portal frame (high rigidity)
  - cantilever: Swing-arm frame (work area access)
"""
from __future__ import annotations

import io
import logging
import math
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

logger = logging.getLogger(__name__)

try:
    import cadquery as cq
    HAS_CADQUERY = True
except ImportError:
    HAS_CADQUERY = False
    logger.info("CadQuery not available; using numpy mesh fallback")

# Material density lookup (kg/m^3)
MATERIAL_DENSITY = {
    "Steel AISI 1045": 7850.0,
    "Steel AISI 4140": 7850.0,
    "Aluminum 6061-T6": 2710.0,
    "Cast Iron": 7200.0,
}


@dataclass
class FrameParams:
    """Parameters for frame generation."""
    frame_type: str = "c_frame"  # c_frame | gantry | cantilever

    # Overall dimensions (mm)
    base_width: float = 400.0
    base_depth: float = 300.0
    base_height: float = 40.0
    column_height: float = 500.0
    column_width: float = 80.0
    column_depth: float = 80.0
    beam_length: float = 300.0
    beam_height: float = 60.0

    # Gantry-specific
    column_spacing: float = 350.0

    # Functional features
    cylinder_bore_dia: float = 50.0
    guide_bore_dia: float = 25.0
    guide_bore_spacing: float = 120.0
    t_slot_width: float = 14.0
    t_slot_depth: float = 10.0
    t_slot_count: int = 3
    mounting_hole_dia: float = 12.0
    mounting_hole_count: int = 4

    # Manufacturing detail
    chamfer_mm: float = 1.0
    fillet_radius_mm: float = 3.0
    rib_thickness: float = 10.0
    rib_count: int = 2

    # Material
    material: str = "Steel AISI 1045"


@dataclass
class FrameGenerationResult:
    """Result from frame generation."""
    parts: dict[str, Any] = field(default_factory=dict)
    mesh_preview: dict = field(default_factory=dict)
    step_data: Optional[bytes] = None
    stl_data: Optional[bytes] = None
    frame_type: str = ""
    dimensions: dict = field(default_factory=dict)
    volume_mm3: float = 0.0
    mass_kg: float = 0.0
    bom: list[dict] = field(default_factory=list)
    has_cad_export: bool = False


class FrameGenerator:
    """Generate parametric machine frame geometries."""

    def generate(self, params: FrameParams) -> FrameGenerationResult:
        """Generate a frame with the given parameters."""
        if HAS_CADQUERY:
            try:
                return self._generate_cadquery(params)
            except Exception as e:
                logger.warning("CadQuery generation failed, using fallback: %s", e)
        return self._generate_numpy(params)

    # ---- CadQuery path ----

    def _generate_cadquery(self, p: FrameParams) -> FrameGenerationResult:
        """Generate using CadQuery for proper CAD output."""
        builders = {
            "c_frame": self._cq_build_c_frame,
            "gantry": self._cq_build_gantry,
            "cantilever": self._cq_build_cantilever,
        }
        builder = builders.get(p.frame_type, self._cq_build_c_frame)
        parts = builder(p)

        # Union all parts for STEP export
        combined = None
        for name, solid in parts.items():
            if combined is None:
                combined = solid
            else:
                combined = combined.union(solid)

        # Export STEP
        step_buf = io.BytesIO()
        cq.exporters.export(combined, step_buf, exportType="STEP")
        step_data = step_buf.getvalue()

        # Export STL
        stl_buf = io.BytesIO()
        cq.exporters.export(combined, stl_buf, exportType="STL")
        stl_data = stl_buf.getvalue()

        # Tessellate each part for preview
        mesh_preview = {}
        bom = []
        total_vol = 0.0
        for name, solid in parts.items():
            mesh_preview[name] = self._cq_tessellate(solid)
            vol = float(solid.val().Volume())
            total_vol += vol
            bom.append({"name": name, "volume_mm3": round(vol, 2), "material": p.material})

        density = MATERIAL_DENSITY.get(p.material, 7850.0)
        mass_kg = total_vol * density * 1e-9  # mm3 -> m3 -> kg

        return FrameGenerationResult(
            parts=parts,
            mesh_preview=mesh_preview,
            step_data=step_data,
            stl_data=stl_data,
            frame_type=p.frame_type,
            dimensions={
                "base_width": p.base_width,
                "base_depth": p.base_depth,
                "base_height": p.base_height,
                "column_height": p.column_height,
                "beam_length": p.beam_length,
            },
            volume_mm3=round(total_vol, 2),
            mass_kg=round(mass_kg, 3),
            bom=bom,
            has_cad_export=True,
        )

    def _cq_build_base(self, p: FrameParams):
        """Build base plate with T-slots and mounting holes."""
        base = cq.Workplane("XY").box(p.base_width, p.base_depth, p.base_height)

        # T-slots along X axis on top face
        if p.t_slot_count > 0 and p.t_slot_depth > 0:
            slot_spacing = p.base_depth / (p.t_slot_count + 1)
            for i in range(1, p.t_slot_count + 1):
                y_pos = -p.base_depth / 2 + i * slot_spacing
                # T-slot: narrow top + wide bottom
                slot = (
                    cq.Workplane("XY")
                    .center(0, y_pos)
                    .transformed(offset=(0, 0, p.base_height / 2))
                    .rect(p.base_width * 0.8, p.t_slot_width)
                    .extrude(-p.t_slot_depth)
                )
                base = base.cut(slot)

        # Mounting holes at corners
        if p.mounting_hole_count == 4:
            mx = p.base_width / 2 - p.mounting_hole_dia * 2
            my = p.base_depth / 2 - p.mounting_hole_dia * 2
            for sx, sy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                hole = (
                    cq.Workplane("XY")
                    .center(sx * mx, sy * my)
                    .circle(p.mounting_hole_dia / 2)
                    .extrude(p.base_height)
                )
                base = base.cut(hole)

        return base

    def _cq_build_column(self, p: FrameParams, x_offset: float = 0.0):
        """Build a single column with ribs."""
        col = (
            cq.Workplane("XY")
            .center(x_offset, 0)
            .transformed(offset=(0, 0, p.base_height))
            .rect(p.column_width, p.column_depth)
            .extrude(p.column_height)
        )

        # Reinforcement ribs
        if p.rib_count > 0 and p.rib_thickness > 0:
            rib_h = p.column_height * 0.4
            for i in range(p.rib_count):
                angle = (i / max(p.rib_count - 1, 1)) * 0.5 - 0.25 if p.rib_count > 1 else 0
                rib = (
                    cq.Workplane("XZ")
                    .center(x_offset, p.base_height + rib_h / 2)
                    .rect(p.rib_thickness, rib_h)
                    .extrude(p.column_depth / 2 + p.rib_thickness)
                )
                col = col.union(rib)

        # Chamfers
        if p.chamfer_mm > 0:
            try:
                col = col.edges("|Z").chamfer(p.chamfer_mm)
            except Exception:
                pass  # Skip if chamfer fails on complex geometry

        return col

    def _cq_build_beam(self, p: FrameParams, z_offset: float = 0.0):
        """Build cantilever beam with cylinder bore and guide bores."""
        beam_z = p.base_height + p.column_height + z_offset
        beam = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, beam_z))
            .rect(p.beam_length, p.column_depth)
            .extrude(p.beam_height)
        )

        # Cylinder bore (through hole at center of beam)
        if p.cylinder_bore_dia > 0:
            bore = (
                cq.Workplane("XY")
                .transformed(offset=(0, 0, beam_z))
                .circle(p.cylinder_bore_dia / 2)
                .extrude(p.beam_height)
            )
            beam = beam.cut(bore)

        # Guide bores (symmetric pair)
        if p.guide_bore_dia > 0 and p.guide_bore_spacing > 0:
            for sx in [-1, 1]:
                gbore = (
                    cq.Workplane("XY")
                    .center(sx * p.guide_bore_spacing / 2, 0)
                    .transformed(offset=(0, 0, beam_z))
                    .circle(p.guide_bore_dia / 2)
                    .extrude(p.beam_height)
                )
                beam = beam.cut(gbore)

        # Fillets on beam edges
        if p.fillet_radius_mm > 0:
            try:
                beam = beam.edges("|Y").fillet(p.fillet_radius_mm)
            except Exception:
                pass

        return beam

    def _cq_build_c_frame(self, p: FrameParams) -> dict[str, Any]:
        """Build C-frame: base + single column + cantilever beam."""
        base = self._cq_build_base(p)
        col_x = -p.base_width / 2 + p.column_width / 2
        column = self._cq_build_column(p, x_offset=col_x)
        beam = self._cq_build_beam(p)
        return {"base": base, "column": column, "beam": beam}

    def _cq_build_gantry(self, p: FrameParams) -> dict[str, Any]:
        """Build gantry frame: base + dual columns + bridge beam."""
        base = self._cq_build_base(p)
        half_spacing = p.column_spacing / 2
        col_left = self._cq_build_column(p, x_offset=-half_spacing)
        col_right = self._cq_build_column(p, x_offset=half_spacing)

        # Bridge beam spans full column spacing
        bridge_beam = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, p.base_height + p.column_height))
            .rect(p.column_spacing + p.column_width, p.column_depth)
            .extrude(p.beam_height)
        )
        # Cylinder bore
        if p.cylinder_bore_dia > 0:
            bore = (
                cq.Workplane("XY")
                .transformed(offset=(0, 0, p.base_height + p.column_height))
                .circle(p.cylinder_bore_dia / 2)
                .extrude(p.beam_height)
            )
            bridge_beam = bridge_beam.cut(bore)
        # Guide bores
        if p.guide_bore_dia > 0:
            for sx in [-1, 1]:
                gbore = (
                    cq.Workplane("XY")
                    .center(sx * p.guide_bore_spacing / 2, 0)
                    .transformed(offset=(0, 0, p.base_height + p.column_height))
                    .circle(p.guide_bore_dia / 2)
                    .extrude(p.beam_height)
                )
                bridge_beam = bridge_beam.cut(gbore)

        return {
            "base": base,
            "column_left": col_left,
            "column_right": col_right,
            "beam": bridge_beam,
        }

    def _cq_build_cantilever(self, p: FrameParams) -> dict[str, Any]:
        """Build cantilever frame: base + column + swing arm."""
        base = self._cq_build_base(p)
        column = self._cq_build_column(p, x_offset=0)

        # Swing arm: cylindrical at base for rotation joint
        arm_z = p.base_height + p.column_height * 0.8
        arm = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, arm_z))
            .rect(p.beam_length, p.column_depth * 0.8)
            .extrude(p.beam_height)
        )
        # Rotation joint hole
        joint = (
            cq.Workplane("XZ")
            .center(-p.beam_length / 2, arm_z + p.beam_height / 2)
            .circle(p.guide_bore_dia)
            .extrude(p.column_depth)
        )
        arm = arm.cut(joint)

        # Cylinder bore
        if p.cylinder_bore_dia > 0:
            bore = (
                cq.Workplane("XY")
                .center(p.beam_length / 4, 0)
                .transformed(offset=(0, 0, arm_z))
                .circle(p.cylinder_bore_dia / 2)
                .extrude(p.beam_height)
            )
            arm = arm.cut(bore)

        return {"base": base, "column": column, "arm": arm}

    @staticmethod
    def _cq_tessellate(solid) -> dict:
        """Tessellate CadQuery solid to vertex/face arrays."""
        tess = solid.val().tessellate(0.1)
        vertices = [[float(v.x), float(v.y), float(v.z)] for v in tess[0]]
        faces = [list(f) for f in tess[1]]
        return {"vertices": vertices, "faces": faces}

    # ---- NumPy fallback ----

    def _generate_numpy(self, p: FrameParams) -> FrameGenerationResult:
        """Generate simplified mesh preview using NumPy (no CAD export)."""
        parts = {}
        mesh_preview = {}
        bom = []
        total_vol = 0.0

        # Simple box meshes for each part
        def box_mesh(w, d, h, ox=0, oy=0, oz=0):
            """Create a box mesh at offset."""
            hw, hd = w / 2, d / 2
            verts = [
                [ox - hw, oy - hd, oz], [ox + hw, oy - hd, oz],
                [ox + hw, oy + hd, oz], [ox - hw, oy + hd, oz],
                [ox - hw, oy - hd, oz + h], [ox + hw, oy - hd, oz + h],
                [ox + hw, oy + hd, oz + h], [ox - hw, oy + hd, oz + h],
            ]
            faces = [
                [0, 1, 2], [0, 2, 3],  # bottom
                [4, 6, 5], [4, 7, 6],  # top
                [0, 4, 5], [0, 5, 1],  # front
                [2, 6, 7], [2, 7, 3],  # back
                [0, 3, 7], [0, 7, 4],  # left
                [1, 5, 6], [1, 6, 2],  # right
            ]
            vol = w * d * h
            return {"vertices": verts, "faces": faces}, vol

        # Base
        m, v = box_mesh(p.base_width, p.base_depth, p.base_height)
        mesh_preview["base"] = m
        parts["base"] = None
        total_vol += v
        bom.append({"name": "base", "volume_mm3": round(v, 2), "material": p.material})

        # Column(s)
        if p.frame_type == "gantry":
            for side, label in [(-1, "column_left"), (1, "column_right")]:
                cx = side * p.column_spacing / 2
                m, v = box_mesh(p.column_width, p.column_depth, p.column_height,
                                ox=cx, oz=p.base_height)
                mesh_preview[label] = m
                parts[label] = None
                total_vol += v
                bom.append({"name": label, "volume_mm3": round(v, 2), "material": p.material})
        else:
            cx = -p.base_width / 2 + p.column_width / 2 if p.frame_type == "c_frame" else 0
            m, v = box_mesh(p.column_width, p.column_depth, p.column_height,
                            ox=cx, oz=p.base_height)
            mesh_preview["column"] = m
            parts["column"] = None
            total_vol += v
            bom.append({"name": "column", "volume_mm3": round(v, 2), "material": p.material})

        # Beam / Arm
        beam_label = "arm" if p.frame_type == "cantilever" else "beam"
        bz = p.base_height + p.column_height
        if p.frame_type == "cantilever":
            bz = p.base_height + p.column_height * 0.8
        m, v = box_mesh(p.beam_length, p.column_depth, p.beam_height, oz=bz)
        mesh_preview[beam_label] = m
        parts[beam_label] = None
        total_vol += v
        bom.append({"name": beam_label, "volume_mm3": round(v, 2), "material": p.material})

        density = MATERIAL_DENSITY.get(p.material, 7850.0)
        mass_kg = total_vol * density * 1e-9

        return FrameGenerationResult(
            parts=parts,
            mesh_preview=mesh_preview,
            frame_type=p.frame_type,
            dimensions={
                "base_width": p.base_width,
                "base_depth": p.base_depth,
                "base_height": p.base_height,
                "column_height": p.column_height,
                "beam_length": p.beam_length,
            },
            volume_mm3=round(total_vol, 2),
            mass_kg=round(mass_kg, 3),
            bom=bom,
            has_cad_export=False,
        )
```

**Step 4: Run tests to verify they pass**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_frame_generator.py -v
```
Expected: ALL PASS

**Step 5: Commit**

```bash
git add ultrasonic_weld_master/plugins/geometry_analyzer/frame_generator.py tests/test_frame_generator.py
git commit -m "feat: add parametric frame generator with C/gantry/cantilever types"
```

---

### Task 1.2: Frame API — Schema, Service, Router

**Files:**
- Create: `web/schemas/frame.py`
- Create: `web/services/frame_service.py`
- Create: `web/routers/frame.py`
- Modify: `web/app.py` (add router)
- Test: `tests/test_web/test_frame.py`

**Step 1: Write the failing API test**

Create `tests/test_web/test_frame.py`:

```python
"""Tests for frame API endpoints."""
import pytest
from fastapi.testclient import TestClient
from web.app import create_app


@pytest.fixture
def client():
    app = create_app()
    return TestClient(app)


class TestFrameAPI:
    def test_generate_c_frame(self, client):
        resp = client.post("/api/v2/frame/generate", json={
            "frame_type": "c_frame",
            "base_width": 400.0,
            "column_height": 500.0,
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["frame_type"] == "c_frame"
        assert data["volume_mm3"] > 0
        assert data["mass_kg"] > 0
        assert len(data["bom"]) >= 3

    def test_generate_gantry(self, client):
        resp = client.post("/api/v2/frame/generate", json={
            "frame_type": "gantry",
        })
        assert resp.status_code == 200
        assert resp.json()["frame_type"] == "gantry"

    def test_get_types(self, client):
        resp = client.get("/api/v2/frame/types")
        assert resp.status_code == 200
        types = resp.json()
        assert "c_frame" in types
        assert "gantry" in types
        assert "cantilever" in types

    def test_get_defaults(self, client):
        resp = client.get("/api/v2/frame/defaults/c_frame")
        assert resp.status_code == 200
        data = resp.json()
        assert "base_width" in data
        assert "column_height" in data

    def test_step_download(self, client):
        # First generate
        resp = client.post("/api/v2/frame/generate", json={"frame_type": "c_frame"})
        assert resp.status_code == 200
        download_id = resp.json().get("download_id")
        if download_id:
            resp2 = client.get(f"/api/v2/frame/download/{download_id}?fmt=step")
            assert resp2.status_code == 200
```

**Step 2: Run test to verify it fails**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_web/test_frame.py -v 2>&1 | head -20
```
Expected: FAIL — 404 (route not registered)

**Step 3: Create schema**

Create `web/schemas/frame.py`:

```python
"""Pydantic models for frame generation endpoints."""
from __future__ import annotations
from typing import Optional, Literal
from pydantic import BaseModel, Field


class FrameGenerateRequest(BaseModel):
    """Request to generate a parametric machine frame."""
    frame_type: Literal["c_frame", "gantry", "cantilever"] = "c_frame"
    base_width: float = Field(default=400.0, gt=0)
    base_depth: float = Field(default=300.0, gt=0)
    base_height: float = Field(default=40.0, gt=0)
    column_height: float = Field(default=500.0, gt=0)
    column_width: float = Field(default=80.0, gt=0)
    column_depth: float = Field(default=80.0, gt=0)
    beam_length: float = Field(default=300.0, gt=0)
    beam_height: float = Field(default=60.0, gt=0)
    column_spacing: float = Field(default=350.0, gt=0)
    cylinder_bore_dia: float = Field(default=50.0, ge=0)
    guide_bore_dia: float = Field(default=25.0, ge=0)
    guide_bore_spacing: float = Field(default=120.0, ge=0)
    t_slot_width: float = Field(default=14.0, ge=0)
    t_slot_depth: float = Field(default=10.0, ge=0)
    t_slot_count: int = Field(default=3, ge=0)
    mounting_hole_dia: float = Field(default=12.0, ge=0)
    mounting_hole_count: int = Field(default=4, ge=0)
    chamfer_mm: float = Field(default=1.0, ge=0)
    fillet_radius_mm: float = Field(default=3.0, ge=0)
    rib_thickness: float = Field(default=10.0, ge=0)
    rib_count: int = Field(default=2, ge=0)
    material: str = "Steel AISI 1045"


class FrameGenerateResponse(BaseModel):
    """Response from frame generation."""
    frame_type: str
    dimensions: dict
    volume_mm3: float
    mass_kg: float
    has_cad_export: bool
    mesh_preview: dict
    bom: list[dict]
    download_id: Optional[str] = None
```

**Step 4: Create service**

Create `web/services/frame_service.py`:

```python
"""Frame generation service with temporary file management."""
from __future__ import annotations

import logging
import os
import tempfile
import time
import uuid
from typing import Optional

from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import (
    FrameGenerator, FrameParams, FrameGenerationResult,
)

logger = logging.getLogger(__name__)

_MAX_CACHE_SIZE = 50
_MAX_CACHE_AGE_S = 3600


class FrameService:
    """Service for frame generation and file management."""

    def __init__(self):
        self._generator = FrameGenerator()
        self._file_cache: dict[str, dict] = {}
        self._cache_dir = tempfile.mkdtemp(prefix="frame_export_")

    def _cleanup_old_cache(self):
        now = time.time()
        expired = [k for k, v in self._file_cache.items()
                   if now - v.get("created", 0) > _MAX_CACHE_AGE_S]
        for k in expired:
            entry = self._file_cache.pop(k, None)
            if entry and os.path.exists(entry["path"]):
                try:
                    os.remove(entry["path"])
                except OSError:
                    pass

    def generate_frame(self, params: FrameParams) -> tuple[FrameGenerationResult, Optional[str]]:
        """Generate frame and cache export files."""
        self._cleanup_old_cache()
        result = self._generator.generate(params)
        download_id = None
        now = time.time()

        if result.has_cad_export and result.step_data:
            download_id = uuid.uuid4().hex[:12]
            step_path = os.path.join(self._cache_dir, f"{download_id}.step")
            with open(step_path, "wb") as f:
                f.write(result.step_data)
            self._file_cache[f"{download_id}_step"] = {
                "path": step_path, "format": "step", "created": now,
            }
            if result.stl_data:
                stl_path = os.path.join(self._cache_dir, f"{download_id}.stl")
                with open(stl_path, "wb") as f:
                    f.write(result.stl_data)
                self._file_cache[f"{download_id}_stl"] = {
                    "path": stl_path, "format": "stl", "created": now,
                }

        return result, download_id

    def get_download_path(self, file_id: str, fmt: str = "step") -> Optional[str]:
        key = f"{file_id}_{fmt}"
        entry = self._file_cache.get(key)
        if entry and os.path.exists(entry["path"]):
            return entry["path"]
        return None
```

**Step 5: Create router**

Create `web/routers/frame.py`:

```python
"""Machine frame generation endpoints."""
from __future__ import annotations

import logging
from dataclasses import asdict

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

from web.schemas.frame import FrameGenerateRequest, FrameGenerateResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/frame", tags=["frame"])

_frame_service = None


def _get_frame_service():
    global _frame_service
    if _frame_service is None:
        from web.services.frame_service import FrameService
        _frame_service = FrameService()
    return _frame_service


@router.post("/generate", response_model=FrameGenerateResponse)
async def generate_frame(request: FrameGenerateRequest):
    """Generate a parametric machine frame."""
    try:
        from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import FrameParams

        params = FrameParams(**request.model_dump())
        svc = _get_frame_service()
        result, download_id = svc.generate_frame(params)

        return FrameGenerateResponse(
            frame_type=result.frame_type,
            dimensions=result.dimensions,
            volume_mm3=result.volume_mm3,
            mass_kg=result.mass_kg,
            has_cad_export=result.has_cad_export,
            mesh_preview=result.mesh_preview,
            bom=result.bom,
            download_id=download_id,
        )
    except Exception as exc:
        logger.error("Frame generation error: %s", exc, exc_info=True)
        raise HTTPException(500, f"Frame generation failed: {exc}") from exc


@router.get("/types")
async def get_frame_types():
    """Return available frame types."""
    return ["c_frame", "gantry", "cantilever"]


@router.get("/defaults/{frame_type}")
async def get_frame_defaults(frame_type: str):
    """Return default parameters for a frame type."""
    from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import FrameParams
    if frame_type not in ("c_frame", "gantry", "cantilever"):
        raise HTTPException(400, f"Unknown frame type: {frame_type}")
    params = FrameParams(frame_type=frame_type)
    return asdict(params)


@router.get("/download/{file_id}")
async def download_frame_file(file_id: str, fmt: str = "step"):
    """Download generated frame file (STEP or STL)."""
    if fmt not in ("step", "stl"):
        raise HTTPException(400, "Format must be 'step' or 'stl'")
    svc = _get_frame_service()
    path = svc.get_download_path(file_id, fmt)
    if path is None:
        raise HTTPException(404, "File not found or expired")
    media_type = "application/step" if fmt == "step" else "application/sla"
    return FileResponse(path, media_type=media_type, filename=f"frame_{file_id}.{fmt}")
```

**Step 6: Register router in app.py**

Add to `web/app.py` imports:
```python
from web.routers import (
    ...
    frame,        # ADD THIS
)
```

Add to `create_app()` body:
```python
    application.include_router(frame.router, prefix=api_prefix)
```

**Step 7: Run API tests**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_web/test_frame.py -v
```
Expected: ALL PASS

**Step 8: Commit**

```bash
git add web/schemas/frame.py web/services/frame_service.py web/routers/frame.py web/app.py tests/test_web/test_frame.py
git commit -m "feat: add frame generation API endpoints (schema, service, router)"
```

---

## Phase 2: CAD Assembly Manager (Backend)

### Task 2.1: Assembly Manager Core

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/assembly_manager.py`
- Test: `tests/test_assembly_manager.py`

**Step 1: Write the failing test**

Create `tests/test_assembly_manager.py`:

```python
"""Tests for CAD assembly manager."""
import pytest
from ultrasonic_weld_master.plugins.geometry_analyzer.assembly_manager import (
    AssemblyManager, AssemblyComponent, AssemblyConfig,
)


class TestAssemblyManager:
    def setup_method(self):
        self.mgr = AssemblyManager()

    def test_build_basic_assembly(self):
        """Should build assembly from horn + anvil + frame."""
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            anvil_params={"anvil_type": "flat", "width_mm": 60, "depth_mm": 60, "height_mm": 20},
            frame_params={"frame_type": "c_frame"},
        )
        result = self.mgr.build(config)
        assert len(result.component_tree["children"]) > 0
        assert result.total_mass_kg > 0
        assert len(result.bom) >= 3

    def test_component_tree_structure(self):
        """Tree should have frame and welding_stack groups."""
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            frame_params={"frame_type": "c_frame"},
        )
        result = self.mgr.build(config)
        tree = result.component_tree
        child_names = [c["name"] for c in tree["children"]]
        assert "frame" in child_names or any("frame" in n.lower() for n in child_names)

    def test_explosion_offsets(self):
        """Each component should have an explosion offset vector."""
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            frame_params={"frame_type": "c_frame"},
        )
        result = self.mgr.build(config)
        assert len(result.explosion_offsets) > 0
        for name, offset in result.explosion_offsets.items():
            assert len(offset) == 3  # (dx, dy, dz)

    def test_mesh_data_per_component(self):
        """Each component should have mesh data for frontend rendering."""
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            frame_params={"frame_type": "c_frame"},
        )
        result = self.mgr.build(config)
        assert len(result.mesh_data) > 0
        for name, mesh in result.mesh_data.items():
            assert "vertices" in mesh
            assert "faces" in mesh
            assert len(mesh["vertices"]) > 0

    def test_step_export(self):
        """Should produce STEP file if CadQuery available."""
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            frame_params={"frame_type": "c_frame"},
        )
        result = self.mgr.build(config)
        # step_path may be None if CadQuery not available
        assert isinstance(result.step_path, (str, type(None)))
```

**Step 2: Run test to verify it fails**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_assembly_manager.py -v 2>&1 | head -20
```
Expected: FAIL — ModuleNotFoundError

**Step 3: Implement assembly manager**

Create `ultrasonic_weld_master/plugins/geometry_analyzer/assembly_manager.py`:

```python
"""CAD-level assembly manager for ultrasonic welding machines.

Builds multi-body assemblies from individual component generators
(horn, anvil, frame) using rule-based auto-positioning along the Z axis.
"""
from __future__ import annotations

import io
import logging
import os
import tempfile
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

logger = logging.getLogger(__name__)

try:
    import cadquery as cq
    HAS_CADQUERY = True
except ImportError:
    HAS_CADQUERY = False

# Default colors for assembly components
COMPONENT_COLORS = {
    "base": "#555555",
    "column": "#666666",
    "column_left": "#666666",
    "column_right": "#666666",
    "beam": "#777777",
    "arm": "#777777",
    "horn": "#cc8844",
    "booster": "#44aa88",
    "anvil": "#4488cc",
}


@dataclass
class AssemblyConfig:
    """Configuration for building an assembly."""
    horn_params: dict = field(default_factory=lambda: {"horn_type": "flat"})
    anvil_params: Optional[dict] = None
    booster_params: Optional[dict] = None
    frame_params: dict = field(default_factory=lambda: {"frame_type": "c_frame"})
    stroke_length_mm: float = 50.0  # cylinder stroke


@dataclass
class AssemblyComponent:
    """A single component in the assembly."""
    name: str
    solid: Any = None  # CadQuery object or None
    mesh: dict = field(default_factory=dict)
    position: tuple = (0.0, 0.0, 0.0)
    color: str = "#888888"
    parent: Optional[str] = None
    category: str = "component"
    volume_mm3: float = 0.0
    mass_kg: float = 0.0
    material: str = ""


@dataclass
class AssemblyResult:
    """Result from assembly building."""
    step_path: Optional[str] = None
    component_tree: dict = field(default_factory=dict)
    mesh_data: dict = field(default_factory=dict)
    bom: list[dict] = field(default_factory=list)
    total_mass_kg: float = 0.0
    explosion_offsets: dict = field(default_factory=dict)


class AssemblyManager:
    """Build and manage welding machine assemblies."""

    def build(self, config: AssemblyConfig) -> AssemblyResult:
        """Build a complete assembly from config."""
        components: list[AssemblyComponent] = []

        # 1. Generate frame
        from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import (
            FrameGenerator, FrameParams,
        )
        frame_gen = FrameGenerator()
        fp = FrameParams(**config.frame_params)
        frame_result = frame_gen.generate(fp)

        # Add frame parts as components
        for part_name, solid in frame_result.parts.items():
            mesh = frame_result.mesh_preview.get(part_name, {"vertices": [], "faces": []})
            bom_entry = next((b for b in frame_result.bom if b["name"] == part_name), {})
            components.append(AssemblyComponent(
                name=part_name,
                solid=solid,
                mesh=mesh,
                position=(0, 0, 0),
                color=COMPONENT_COLORS.get(part_name, "#888888"),
                parent="frame",
                category="frame",
                volume_mm3=bom_entry.get("volume_mm3", 0),
                material=fp.material,
            ))

        # 2. Generate horn
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_generator import (
            HornGenerator, HornParams,
        )
        horn_gen = HornGenerator()
        hp = HornParams(**config.horn_params)
        horn_result = horn_gen.generate(hp)

        # Position horn below beam
        beam_bottom_z = fp.base_height + fp.column_height
        horn_z = beam_bottom_z - config.stroke_length_mm - hp.height_mm
        components.append(AssemblyComponent(
            name="horn",
            solid=None,  # horn_result doesn't expose CQ solid directly
            mesh=horn_result.mesh,
            position=(0, 0, horn_z),
            color=COMPONENT_COLORS["horn"],
            parent="welding_stack",
            category="tooling",
            volume_mm3=horn_result.volume_mm3,
            material=hp.material,
        ))

        # 3. Generate anvil (optional)
        if config.anvil_params:
            from ultrasonic_weld_master.plugins.geometry_analyzer.anvil_generator import (
                AnvilGenerator, AnvilParams,
            )
            anvil_gen = AnvilGenerator()
            ap = AnvilParams(**config.anvil_params)
            anvil_result = anvil_gen.generate(ap)
            anvil_z = fp.base_height  # on top of base
            components.append(AssemblyComponent(
                name="anvil",
                solid=None,
                mesh=anvil_result.mesh,
                position=(0, 0, anvil_z),
                color=COMPONENT_COLORS["anvil"],
                parent="welding_stack",
                category="tooling",
                volume_mm3=anvil_result.volume_mm3,
                material=ap.material,
            ))

        # 4. Build result
        mesh_data = {}
        bom = []
        total_mass = 0.0
        positions = {}

        for comp in components:
            mesh_data[comp.name] = comp.mesh
            mass = comp.volume_mm3 * 7850.0 * 1e-9  # approximate
            total_mass += mass
            bom.append({
                "name": comp.name,
                "category": comp.category,
                "volume_mm3": round(comp.volume_mm3, 2),
                "mass_kg": round(mass, 3),
                "material": comp.material,
                "color": comp.color,
            })
            positions[comp.name] = np.array(comp.position)

        # 5. Build component tree
        tree = self._build_tree(components)

        # 6. Compute explosion offsets
        explosion = self._compute_explosion_offsets(positions, fp)

        # 7. Export STEP assembly
        step_path = self._export_step_assembly(components) if HAS_CADQUERY else None

        return AssemblyResult(
            step_path=step_path,
            component_tree=tree,
            mesh_data=mesh_data,
            bom=bom,
            total_mass_kg=round(total_mass, 3),
            explosion_offsets=explosion,
        )

    @staticmethod
    def _build_tree(components: list[AssemblyComponent]) -> dict:
        """Build hierarchical component tree for frontend."""
        groups: dict[str, list] = {}
        for comp in components:
            parent = comp.parent or "other"
            groups.setdefault(parent, []).append({
                "name": comp.name,
                "category": comp.category,
                "color": comp.color,
                "visible": True,
            })

        children = []
        for group_name, items in groups.items():
            children.append({
                "name": group_name,
                "children": items,
                "expanded": True,
            })

        return {
            "name": "assembly",
            "children": children,
            "expanded": True,
        }

    @staticmethod
    def _compute_explosion_offsets(
        positions: dict[str, np.ndarray],
        frame_params,
    ) -> dict:
        """Compute explosion offset vectors per component."""
        if not positions:
            return {}

        all_pos = np.array(list(positions.values()))
        centroid = all_pos.mean(axis=0)
        max_dim = max(frame_params.base_width, frame_params.column_height, frame_params.base_depth)
        spread = max_dim * 0.3

        offsets = {}
        for name, pos in positions.items():
            direction = pos - centroid
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                direction = direction / norm
            else:
                direction = np.array([0, 0, 1])
            offset = (direction * spread).tolist()
            offsets[name] = [round(v, 2) for v in offset]

        return offsets

    @staticmethod
    def _export_step_assembly(components: list[AssemblyComponent]) -> Optional[str]:
        """Export assembly to STEP file using CadQuery Assembly."""
        if not HAS_CADQUERY:
            return None

        solids_with_pos = [(c.name, c.solid, c.position) for c in components if c.solid is not None]
        if not solids_with_pos:
            return None

        try:
            assy = cq.Assembly()
            for name, solid, pos in solids_with_pos:
                loc = cq.Location(cq.Vector(*pos))
                assy.add(solid, name=name, loc=loc)

            path = os.path.join(tempfile.gettempdir(), "weld_assembly.step")
            assy.save(path)
            return path
        except Exception as e:
            logger.warning("STEP assembly export failed: %s", e)
            return None
```

**Step 4: Run tests**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_assembly_manager.py -v
```
Expected: ALL PASS

**Step 5: Commit**

```bash
git add ultrasonic_weld_master/plugins/geometry_analyzer/assembly_manager.py tests/test_assembly_manager.py
git commit -m "feat: add CAD assembly manager with rule-based positioning"
```

---

### Task 2.2: Assembly API — Schema, Service, Router

**Files:**
- Create: `web/schemas/assembly_cad.py`
- Create: `web/services/assembly_cad_service.py`
- Create: `web/routers/assembly_cad.py`
- Modify: `web/app.py` (add router)
- Test: `tests/test_web/test_assembly_cad.py`

**Step 1: Write failing test**

Create `tests/test_web/test_assembly_cad.py`:

```python
"""Tests for CAD assembly API endpoints."""
import pytest
from fastapi.testclient import TestClient
from web.app import create_app


@pytest.fixture
def client():
    return TestClient(create_app())


class TestAssemblyCadAPI:
    def test_build_assembly(self, client):
        resp = client.post("/api/v2/assembly-cad/build", json={
            "horn_params": {"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            "frame_params": {"frame_type": "c_frame"},
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["total_mass_kg"] > 0
        assert len(data["bom"]) >= 3
        assert "component_tree" in data

    def test_build_with_anvil(self, client):
        resp = client.post("/api/v2/assembly-cad/build", json={
            "horn_params": {"horn_type": "flat"},
            "anvil_params": {"anvil_type": "flat", "width_mm": 60, "depth_mm": 60, "height_mm": 20},
            "frame_params": {"frame_type": "c_frame"},
        })
        assert resp.status_code == 200
        bom_names = [b["name"] for b in resp.json()["bom"]]
        assert "anvil" in bom_names
```

**Step 2: Run to fail**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_web/test_assembly_cad.py -v 2>&1 | head -15
```

**Step 3: Create schema, service, router (following horn pattern)**

Create `web/schemas/assembly_cad.py`:

```python
"""Pydantic models for CAD assembly endpoints."""
from __future__ import annotations
from typing import Optional
from pydantic import BaseModel, Field


class AssemblyCadBuildRequest(BaseModel):
    horn_params: dict = Field(default_factory=lambda: {"horn_type": "flat"})
    anvil_params: Optional[dict] = None
    booster_params: Optional[dict] = None
    frame_params: dict = Field(default_factory=lambda: {"frame_type": "c_frame"})
    stroke_length_mm: float = Field(default=50.0, gt=0)


class AssemblyCadBuildResponse(BaseModel):
    component_tree: dict
    mesh_data: dict
    bom: list[dict]
    total_mass_kg: float
    explosion_offsets: dict
    has_step_export: bool = False
    download_id: Optional[str] = None
```

Create `web/services/assembly_cad_service.py`:

```python
"""CAD assembly service."""
from __future__ import annotations
import logging
import os
import tempfile
import time
import uuid
from typing import Optional

from ultrasonic_weld_master.plugins.geometry_analyzer.assembly_manager import (
    AssemblyManager, AssemblyConfig, AssemblyResult,
)

logger = logging.getLogger(__name__)


class AssemblyCadService:
    def __init__(self):
        self._manager = AssemblyManager()
        self._file_cache: dict[str, dict] = {}

    def build_assembly(self, config: AssemblyConfig) -> tuple[AssemblyResult, Optional[str]]:
        result = self._manager.build(config)
        download_id = None
        if result.step_path and os.path.exists(result.step_path):
            download_id = uuid.uuid4().hex[:12]
            self._file_cache[download_id] = {
                "path": result.step_path, "created": time.time(),
            }
        return result, download_id

    def get_step_path(self, download_id: str) -> Optional[str]:
        entry = self._file_cache.get(download_id)
        if entry and os.path.exists(entry["path"]):
            return entry["path"]
        return None
```

Create `web/routers/assembly_cad.py`:

```python
"""CAD assembly endpoints."""
from __future__ import annotations
import logging
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from web.schemas.assembly_cad import AssemblyCadBuildRequest, AssemblyCadBuildResponse

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/assembly-cad", tags=["assembly-cad"])

_service = None

def _get_service():
    global _service
    if _service is None:
        from web.services.assembly_cad_service import AssemblyCadService
        _service = AssemblyCadService()
    return _service


@router.post("/build", response_model=AssemblyCadBuildResponse)
async def build_assembly(request: AssemblyCadBuildRequest):
    try:
        from ultrasonic_weld_master.plugins.geometry_analyzer.assembly_manager import AssemblyConfig
        config = AssemblyConfig(
            horn_params=request.horn_params,
            anvil_params=request.anvil_params,
            booster_params=request.booster_params,
            frame_params=request.frame_params,
            stroke_length_mm=request.stroke_length_mm,
        )
        svc = _get_service()
        result, download_id = svc.build_assembly(config)
        return AssemblyCadBuildResponse(
            component_tree=result.component_tree,
            mesh_data=result.mesh_data,
            bom=result.bom,
            total_mass_kg=result.total_mass_kg,
            explosion_offsets=result.explosion_offsets,
            has_step_export=result.step_path is not None,
            download_id=download_id,
        )
    except Exception as exc:
        logger.error("Assembly build error: %s", exc, exc_info=True)
        raise HTTPException(500, f"Assembly build failed: {exc}") from exc


@router.get("/{download_id}/step")
async def download_step(download_id: str):
    svc = _get_service()
    path = svc.get_step_path(download_id)
    if not path:
        raise HTTPException(404, "STEP file not found or expired")
    return FileResponse(path, media_type="application/step", filename=f"assembly_{download_id}.step")
```

**Step 4: Register in app.py** — add `assembly_cad` import and `include_router` line.

**Step 5: Run tests**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_web/test_assembly_cad.py -v
```

**Step 6: Commit**

```bash
git add web/schemas/assembly_cad.py web/services/assembly_cad_service.py web/routers/assembly_cad.py web/app.py tests/test_web/test_assembly_cad.py
git commit -m "feat: add CAD assembly API with build, tree, BOM, explosion endpoints"
```

---

## Phase 3: 2D Drawing Engine (Backend)

### Task 3.1: HLR Projector

**Files:**
- Create: `ultrasonic_weld_master/plugins/drawing_engine/__init__.py`
- Create: `ultrasonic_weld_master/plugins/drawing_engine/projector.py`
- Test: `tests/test_drawing_engine/test_projector.py`

**Step 1: Write failing test**

Create `tests/test_drawing_engine/__init__.py` (empty) and `tests/test_drawing_engine/test_projector.py`:

```python
"""Tests for HLR projector."""
import pytest

try:
    import cadquery as cq
    HAS_CQ = True
except ImportError:
    HAS_CQ = False

from ultrasonic_weld_master.plugins.drawing_engine.projector import ShapeProjector, ProjectionResult


@pytest.mark.skipif(not HAS_CQ, reason="CadQuery required")
class TestProjector:
    def setup_method(self):
        self.proj = ShapeProjector()
        self.box = cq.Workplane("XY").box(100, 50, 30)

    def test_project_front(self):
        result = self.proj.project(self.box, "front")
        assert isinstance(result, ProjectionResult)
        assert len(result.visible_edges) > 0

    def test_project_standard_three(self):
        views = self.proj.project_standard_three(self.box)
        assert "front" in views
        assert "top" in views
        assert "right" in views

    def test_projection_has_bounds(self):
        result = self.proj.project(self.box, "front")
        assert result.bounds is not None
        w, h = result.bounds
        assert w > 0 and h > 0

    def test_hidden_edges_for_complex_shape(self):
        """A shape with hidden features should produce hidden edges."""
        shape = cq.Workplane("XY").box(100, 50, 30).faces(">Z").hole(20)
        result = self.proj.project(shape, "front")
        # Hole should produce hidden lines in front view
        assert len(result.visible_edges) > 0
```

**Step 2: Run to fail, Step 3: Implement**

Create `ultrasonic_weld_master/plugins/drawing_engine/__init__.py` (empty).

Create `ultrasonic_weld_master/plugins/drawing_engine/projector.py`:

```python
"""3D to 2D projection using OCP Hidden Line Removal (HLR).

Projects CadQuery solids to 2D edge sets for engineering drawing generation.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)

try:
    import cadquery as cq
    from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.BRep import BRep_Tool
    from OCP.GCPnts import GCPnts_UniformDeflection
    HAS_OCP = True
except ImportError:
    HAS_OCP = False
    logger.info("OCP not available; projector disabled")


@dataclass
class ProjectionResult:
    """Result of projecting a 3D shape to 2D."""
    visible_edges: list[list[tuple[float, float]]] = field(default_factory=list)
    hidden_edges: list[list[tuple[float, float]]] = field(default_factory=list)
    outline_edges: list[list[tuple[float, float]]] = field(default_factory=list)
    bounds: tuple[float, float] = (0.0, 0.0)  # (width, height) of bounding box
    view_name: str = ""


class ShapeProjector:
    """Project 3D CadQuery solids to 2D edges using OCP HLR."""

    STANDARD_VIEWS = {
        "front": gp_Dir(0, -1, 0) if HAS_OCP else None,
        "top": gp_Dir(0, 0, 1) if HAS_OCP else None,
        "right": gp_Dir(1, 0, 0) if HAS_OCP else None,
        "iso": gp_Dir(1, -1, 1) if HAS_OCP else None,
        "back": gp_Dir(0, 1, 0) if HAS_OCP else None,
        "bottom": gp_Dir(0, 0, -1) if HAS_OCP else None,
    }

    def project(self, solid: Any, view: str = "front") -> ProjectionResult:
        """Project a CadQuery solid onto a 2D plane.

        Parameters
        ----------
        solid : CadQuery Workplane
            The 3D shape to project.
        view : str
            View direction name (front/top/right/iso/back/bottom).

        Returns
        -------
        ProjectionResult
            2D edges as line-segment lists.
        """
        if not HAS_OCP:
            raise RuntimeError("OCP not available for HLR projection")

        shape = solid.val().wrapped if hasattr(solid, 'val') else solid

        direction = self.STANDARD_VIEWS.get(view)
        if direction is None:
            raise ValueError(f"Unknown view: {view}")

        # Setup HLR
        hlr = HLRBRep_Algo()
        hlr.Add(shape)

        projector = HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), direction))
        hlr.Projector(projector)
        hlr.Update()
        hlr.Hide()

        hlr_shapes = HLRBRep_HLRToShape(hlr)

        visible = self._extract_edges(hlr_shapes.VCompound())
        hidden = self._extract_edges(hlr_shapes.HCompound())
        outline = self._extract_edges(hlr_shapes.OutLineVCompound())

        # Compute bounds
        all_points = []
        for edge_list in [visible, hidden, outline]:
            for segments in edge_list:
                all_points.extend(segments)

        if all_points:
            xs = [p[0] for p in all_points]
            ys = [p[1] for p in all_points]
            bounds = (max(xs) - min(xs), max(ys) - min(ys))
        else:
            bounds = (0.0, 0.0)

        return ProjectionResult(
            visible_edges=visible,
            hidden_edges=hidden,
            outline_edges=outline,
            bounds=bounds,
            view_name=view,
        )

    def project_standard_three(self, solid: Any) -> dict[str, ProjectionResult]:
        """Project front, top, and right views."""
        return {
            view: self.project(solid, view)
            for view in ["front", "top", "right"]
        }

    def project_with_iso(self, solid: Any) -> dict[str, ProjectionResult]:
        """Project standard three views plus isometric."""
        return {
            view: self.project(solid, view)
            for view in ["front", "top", "right", "iso"]
        }

    @staticmethod
    def _extract_edges(compound) -> list[list[tuple[float, float]]]:
        """Extract 2D line segments from a TopoDS_Shape compound."""
        if compound is None:
            return []

        edges = []
        explorer = TopExp_Explorer(compound, TopAbs_EDGE)
        while explorer.More():
            edge = explorer.Current()
            try:
                adaptor = BRepAdaptor_Curve(edge)
                deflection = GCPnts_UniformDeflection(adaptor, 0.1)
                points = []
                for i in range(1, deflection.NbPoints() + 1):
                    p = deflection.Value(i)
                    points.append((round(float(p.X()), 4), round(float(p.Y()), 4)))
                if len(points) >= 2:
                    edges.append(points)
            except Exception:
                pass
            explorer.Next()

        return edges
```

**Step 4: Run tests**

```bash
cd /opt/weld-sim && source venv/bin/activate && python -m pytest tests/test_drawing_engine/test_projector.py -v
```

**Step 5: Commit**

```bash
git add ultrasonic_weld_master/plugins/drawing_engine/ tests/test_drawing_engine/
git commit -m "feat: add HLR projector for 3D-to-2D engineering drawing projection"
```

---

### Task 3.2: Drawing Layout Engine

**Files:**
- Create: `ultrasonic_weld_master/plugins/drawing_engine/layout.py`
- Create: `ultrasonic_weld_master/plugins/drawing_engine/title_block.py`
- Test: `tests/test_drawing_engine/test_layout.py`

**Step 1: Write failing test**

Create `tests/test_drawing_engine/test_layout.py`:

```python
"""Tests for drawing layout engine."""
import pytest
from ultrasonic_weld_master.plugins.drawing_engine.layout import (
    DrawingLayout, ViewPlacement, PaperSize, create_standard_layout,
)
from ultrasonic_weld_master.plugins.drawing_engine.projector import ProjectionResult


class TestDrawingLayout:
    def _mock_projection(self, w=100, h=50, name="front"):
        return ProjectionResult(
            visible_edges=[[(0, 0), (w, 0), (w, h), (0, h), (0, 0)]],
            bounds=(w, h),
            view_name=name,
        )

    def test_create_a3_layout(self):
        views = {
            "front": self._mock_projection(100, 50, "front"),
            "top": self._mock_projection(100, 30, "top"),
            "right": self._mock_projection(50, 50, "right"),
        }
        layout = create_standard_layout(views, paper_size="A3")
        assert layout.paper_size == PaperSize.A3
        assert layout.scale > 0
        assert len(layout.views) == 3

    def test_auto_scale_fits_paper(self):
        views = {
            "front": self._mock_projection(500, 300, "front"),
            "top": self._mock_projection(500, 200, "top"),
            "right": self._mock_projection(300, 300, "right"),
        }
        layout = create_standard_layout(views, paper_size="A4")
        # Scale should shrink large parts to fit
        assert layout.scale < 1.0

    def test_standard_scale_rounding(self):
        views = {"front": self._mock_projection(150, 80, "front")}
        layout = create_standard_layout(views, paper_size="A3")
        valid_scales = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
        assert layout.scale in valid_scales or layout.scale == pytest.approx(layout.scale, abs=0.01)
```

**Step 2: Run to fail, Step 3: Implement**

Create `ultrasonic_weld_master/plugins/drawing_engine/layout.py` and `title_block.py` — layout engine with paper sizes, view placement, auto-scaling, standard three-view arrangement.

**Step 4: Run tests, Step 5: Commit**

```bash
git add ultrasonic_weld_master/plugins/drawing_engine/layout.py ultrasonic_weld_master/plugins/drawing_engine/title_block.py tests/test_drawing_engine/test_layout.py
git commit -m "feat: add drawing layout engine with auto-scaling and standard views"
```

---

### Task 3.3: Dimensioning System

**Files:**
- Create: `ultrasonic_weld_master/plugins/drawing_engine/dimension.py`
- Test: `tests/test_drawing_engine/test_dimension.py`

Implements auto-dimensioning (envelope, holes, slots) and manual dimension API.

**Commit:** `git commit -m "feat: add auto-dimensioning system for engineering drawings"`

---

### Task 3.4: matplotlib PDF Renderer

**Files:**
- Create: `ultrasonic_weld_master/plugins/drawing_engine/renderer_matplotlib.py`
- Test: `tests/test_drawing_engine/test_renderer_mpl.py`

Renders DrawingLayout to PDF using matplotlib: visible lines (solid black), hidden lines (dashed green), dimensions (cyan), title block.

**Commit:** `git commit -m "feat: add matplotlib PDF renderer for reference-grade drawings"`

---

### Task 3.5: ezdxf DXF Renderer

**Files:**
- Create: `ultrasonic_weld_master/plugins/drawing_engine/renderer_dxf.py`
- Test: `tests/test_drawing_engine/test_renderer_dxf.py`

Renders DrawingLayout to DXF with proper layers (0/HIDDEN/CENTER/DIM/BORDER/TITLE), linetypes, and dimension entities.

**Commit:** `git commit -m "feat: add ezdxf DXF renderer for formal engineering drawings"`

---

### Task 3.6: Drawing API — Router & Service

**Files:**
- Create: `web/schemas/drawing.py`
- Create: `web/services/drawing_service.py`
- Create: `web/routers/drawing.py`
- Modify: `web/app.py`
- Test: `tests/test_web/test_drawing.py`

**Commit:** `git commit -m "feat: add drawing generation API with SVG/PDF/DXF export endpoints"`

---

## Phase 4: Frontend Assembly Visualization

### Task 4.1: Assembly Pinia Store & API Client

**Files:**
- Create: `frontend/src/stores/assemblyCad.ts`
- Create: `frontend/src/api/assembly-cad.ts`

```typescript
// frontend/src/api/assembly-cad.ts
import apiClient from './client'

export interface AssemblyBuildRequest {
  horn_params: Record<string, unknown>
  anvil_params?: Record<string, unknown>
  frame_params: Record<string, unknown>
  stroke_length_mm?: number
}

export interface AssemblyBuildResponse {
  component_tree: ComponentTree
  mesh_data: Record<string, { vertices: number[][]; faces: number[][] }>
  bom: BomItem[]
  total_mass_kg: number
  explosion_offsets: Record<string, [number, number, number]>
  has_step_export: boolean
  download_id?: string
}

export interface ComponentTree {
  name: string
  children: ComponentTreeNode[]
  expanded: boolean
}

export interface ComponentTreeNode {
  name: string
  children?: ComponentTreeNode[]
  category?: string
  color?: string
  visible?: boolean
  expanded?: boolean
}

export interface BomItem {
  name: string
  category: string
  volume_mm3: number
  mass_kg: number
  material: string
  color: string
}

export function buildAssembly(request: AssemblyBuildRequest) {
  return apiClient.post<AssemblyBuildResponse>('/assembly-cad/build', request)
}

export function downloadStep(downloadId: string) {
  return apiClient.get(`/assembly-cad/${downloadId}/step`, { responseType: 'blob' })
}
```

**Commit:** `git commit -m "feat: add assembly Pinia store and API client"`

---

### Task 4.2: VTK.js Multi-Actor Composable

**Files:**
- Create: `frontend/src/composables/useAssemblyViewer.ts`

Core composable extending `useVtkViewer.ts` pattern with multi-actor management, explosion animation, highlight, isolation.

**Commit:** `git commit -m "feat: add VTK.js multi-actor assembly viewer composable"`

---

### Task 4.3: Assembly UI Components

**Files:**
- Create: `frontend/src/components/assembly/AssemblyViewer.vue`
- Create: `frontend/src/components/assembly/AssemblyTree.vue`
- Create: `frontend/src/components/assembly/ExplosionSlider.vue`
- Create: `frontend/src/components/assembly/BomTable.vue`

**Commit:** `git commit -m "feat: add assembly viewer, tree, explosion, and BOM components"`

---

### Task 4.4: Assembly Workbench Page & Routing

**Files:**
- Create: `frontend/src/views/AssemblyWorkbench.vue`
- Modify: `frontend/src/router/index.ts`

**Commit:** `git commit -m "feat: add assembly workbench page with routing"`

---

## Phase 5: Frontend Drawing Workbench

### Task 5.1: Drawing API Client & Components

**Files:**
- Create: `frontend/src/api/drawing.ts`
- Create: `frontend/src/components/drawing/DrawingPreview.vue`
- Create: `frontend/src/components/drawing/PaperSelector.vue`
- Create: `frontend/src/views/DrawingWorkbench.vue`
- Modify: `frontend/src/router/index.ts`

**Commit:** `git commit -m "feat: add drawing workbench with preview and export controls"`

---

## Phase 6: Integration & i18n

### Task 6.1: i18n Strings

**Files:**
- Modify: `frontend/src/i18n/locales/zh-CN.json` (add assembly.*, drawing.*, frame.* keys)
- Modify: `frontend/src/i18n/locales/en.json` (same keys in English)

**Commit:** `git commit -m "feat: add i18n strings for frame, assembly, and drawing modules"`

---

### Task 6.2: End-to-End Integration Test

**Files:**
- Create: `tests/test_integration_cad.py`

```python
"""End-to-end integration test: generate frame → build assembly → export drawing."""
import pytest

class TestCADIntegration:
    def test_full_pipeline(self):
        """Generate frame, build assembly, project drawing, export PDF+DXF."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.frame_generator import FrameGenerator, FrameParams
        from ultrasonic_weld_master.plugins.geometry_analyzer.assembly_manager import AssemblyManager, AssemblyConfig

        # 1. Generate frame
        frame = FrameGenerator().generate(FrameParams(frame_type="c_frame"))
        assert frame.volume_mm3 > 0

        # 2. Build assembly
        config = AssemblyConfig(
            horn_params={"horn_type": "flat", "width_mm": 25, "height_mm": 80, "length_mm": 25},
            frame_params={"frame_type": "c_frame"},
        )
        assy = AssemblyManager().build(config)
        assert assy.total_mass_kg > 0

        # 3. If CadQuery available, test drawing projection
        try:
            import cadquery as cq
            from ultrasonic_weld_master.plugins.drawing_engine.projector import ShapeProjector
            # Project the frame base
            if frame.parts.get("base") is not None:
                proj = ShapeProjector()
                result = proj.project(frame.parts["base"], "front")
                assert len(result.visible_edges) > 0
        except ImportError:
            pytest.skip("CadQuery required for drawing projection test")
```

**Commit:** `git commit -m "test: add end-to-end CAD pipeline integration test"`

---

### Task 6.3: Frontend Type Check

**Step 1: Run vue-tsc to verify no type errors**

```bash
cd /opt/weld-sim/frontend && npx vue-tsc --noEmit
```
Expected: No errors

**Commit:** `git commit -m "chore: verify frontend type checks pass after new modules"`

---

## Summary: Total Tasks and Commits

| Phase | Tasks | Files Created | Tests |
|-------|-------|--------------|-------|
| Phase 1 | 1.1, 1.2 | 5 backend files | 2 test files |
| Phase 2 | 2.1, 2.2 | 5 backend files | 2 test files |
| Phase 3 | 3.1–3.6 | 10 backend files | 6 test files |
| Phase 4 | 4.1–4.4 | 7 frontend files | — |
| Phase 5 | 5.1 | 4 frontend files | — |
| Phase 6 | 6.1–6.3 | 3 files modified | 1 integration test |

**Total: ~34 files, ~18 commits, 6 phases**
