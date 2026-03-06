# Horn Instruction System Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a 9-layer structured instruction system that can precisely describe production-grade ultrasonic welding horns, drive CadQuery 3D modeling, and generate engineering drawings — validated against the real `lab 5-40.stp` horn.

**Architecture:** Three-layer stack: (1) Instruction Schema (`horn_materials.yaml` + `horn_instruction_schema.json`) defines the spec; (2) CadQuery Driver (`horn_instruction_driver.py`) translates instructions into 3D solids via a pipeline of ProfileBuilder → FeatureApplicator → EdgeProcessor; (3) Validation script compares generated horn against the reference STEP file. All new code goes under `/opt/weld-sim/` following existing patterns (dataclass params, CadQuery primary + numpy fallback, pytest TDD).

**Tech Stack:** CadQuery 2.7 (OCP kernel), JSON Schema (jsonschema lib), PyYAML, pytest, existing drawing engine for output.

**Server:** `squall@180.152.71.166:/opt/weld-sim` (venv at `/opt/weld-sim/venv`)
**Run prefix:** `cd /opt/weld-sim && source venv/bin/activate`

---

## Phase 1: Material Library & Instruction Schema

### Task 1: Create Material Library YAML

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_materials.yaml`

**Step 1: Create the material library file**

```yaml
# horn_materials.yaml — Acoustic material properties for ultrasonic horn design
materials:
  ASSAB_88:
    name: "ASSAB 88 (Sleipner)"
    category: "cold_work_tool_steel"
    composition: {C: 0.90, Si: 0.90, Mn: 0.50, Cr: 7.80, Mo: 2.50, V: 0.50}
    density_kg_m3: 7800
    elastic_modulus_gpa: 210
    poisson_ratio: 0.29
    sound_speed_m_s: 5190
    hardness_hrc_range: [60, 64]
    heat_treatment:
      hardening_temp_c: 1050
      tempering_temp_c: 540
      tempering_cycles: 3
    application: ["metal_welding"]

  ASSAB_EM2:
    name: "ASSAB EM2 (M2/W6Mo5Cr4V2)"
    category: "high_speed_steel"
    composition: {C: 0.85, W: 6.40, Mo: 5.00, Cr: 4.20, V: 1.90}
    density_kg_m3: 8160
    elastic_modulus_gpa: 223
    poisson_ratio: 0.29
    sound_speed_m_s: 5160
    hardness_hrc_range: [62, 65]
    heat_treatment:
      hardening_temp_c: 1220
      tempering_temp_c: 560
      tempering_cycles: 3
    application: ["metal_welding"]

  Ti6Al4V:
    name: "Titanium Ti-6Al-4V (Grade 5)"
    category: "titanium_alloy"
    composition: {Ti: 89.5, Al: 6.0, V: 4.0, Fe: 0.3}
    density_kg_m3: 4430
    elastic_modulus_gpa: 114
    poisson_ratio: 0.34
    sound_speed_m_s: 5090
    hardness_hrc_range: [30, 36]
    heat_treatment: null
    application: ["plastic_welding"]

  Al7075_T6:
    name: "Aluminum 7075-T6"
    category: "aluminum_alloy"
    composition: {Al: 90.0, Zn: 5.6, Mg: 2.5, Cu: 1.6}
    density_kg_m3: 2810
    elastic_modulus_gpa: 72
    poisson_ratio: 0.33
    sound_speed_m_s: 5080
    hardness_hrc_range: null
    heat_treatment: null
    application: ["plastic_welding"]

  D2:
    name: "AISI D2 Tool Steel"
    category: "cold_work_tool_steel"
    composition: {C: 1.55, Cr: 11.80, Mo: 0.80, V: 0.80}
    density_kg_m3: 7670
    elastic_modulus_gpa: 208
    poisson_ratio: 0.28
    sound_speed_m_s: 5210
    hardness_hrc_range: [58, 62]
    heat_treatment:
      hardening_temp_c: 1020
      tempering_temp_c: 520
      tempering_cycles: 2
    application: ["metal_welding"]

  CPM_10V:
    name: "CPM 10V (Vanadium Carbide)"
    category: "powder_metallurgy_steel"
    composition: {C: 2.45, Cr: 5.25, V: 9.75, Mo: 1.30}
    density_kg_m3: 7418
    elastic_modulus_gpa: 221
    poisson_ratio: 0.30
    sound_speed_m_s: 5460
    hardness_hrc_range: [58, 60]
    heat_treatment:
      hardening_temp_c: 1120
      tempering_temp_c: 550
      tempering_cycles: 3
    application: ["metal_welding"]
```

**Step 2: Write test for material loader**

```python
# tests/test_horn_instruction/test_materials.py
"""Tests for horn material library loading."""
import pytest
import os
import yaml

MATERIALS_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..",
    "ultrasonic_weld_master", "plugins", "geometry_analyzer", "horn_materials.yaml"
)


class TestMaterialLibrary:
    def test_load_yaml(self):
        with open(MATERIALS_PATH) as f:
            data = yaml.safe_load(f)
        assert "materials" in data
        assert len(data["materials"]) >= 6

    def test_assab_88_properties(self):
        with open(MATERIALS_PATH) as f:
            data = yaml.safe_load(f)
        m = data["materials"]["ASSAB_88"]
        assert m["density_kg_m3"] == 7800
        assert m["sound_speed_m_s"] == 5190
        assert m["elastic_modulus_gpa"] == 210

    def test_all_materials_have_required_fields(self):
        with open(MATERIALS_PATH) as f:
            data = yaml.safe_load(f)
        required = ["density_kg_m3", "elastic_modulus_gpa", "poisson_ratio",
                     "sound_speed_m_s", "application"]
        for name, mat in data["materials"].items():
            for field in required:
                assert field in mat, f"{name} missing {field}"

    def test_half_wavelength_calculation(self):
        """ASSAB 88 at 20kHz: lambda/2 = 5190/(2*20000)*1000 = 129.75mm"""
        with open(MATERIALS_PATH) as f:
            data = yaml.safe_load(f)
        c = data["materials"]["ASSAB_88"]["sound_speed_m_s"]
        half_wl = c / (2 * 20000) * 1000
        assert 129.0 < half_wl < 130.5
```

**Step 3:** Run test → verify FAIL (no file yet)
**Step 4:** Create the YAML file
**Step 5:** Run test → verify PASS
**Step 6:** Commit: `git commit -m "feat: add horn material library with acoustic properties"`

---

### Task 2: Create Material Loader Module

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_materials.py`
- Test: `tests/test_horn_instruction/test_materials.py` (extend)

**Step 1: Write test for MaterialLibrary class**

```python
# Append to tests/test_horn_instruction/test_materials.py

from ultrasonic_weld_master.plugins.geometry_analyzer.horn_materials import (
    MaterialLibrary, HornMaterial,
)


class TestMaterialLibraryClass:
    def setup_method(self):
        self.lib = MaterialLibrary()

    def test_get_known_material(self):
        mat = self.lib.get("ASSAB_88")
        assert isinstance(mat, HornMaterial)
        assert mat.density_kg_m3 == 7800

    def test_unknown_material_raises(self):
        with pytest.raises(KeyError):
            self.lib.get("NONEXISTENT")

    def test_half_wavelength(self):
        mat = self.lib.get("ASSAB_88")
        hw = mat.half_wavelength_mm(20000)
        assert 129.0 < hw < 130.5

    def test_mass_from_volume(self):
        mat = self.lib.get("ASSAB_88")
        # 1000 mm³ of ASSAB 88 = 7800 * 1e-9 * 1e6 = 7.8 g
        mass = mat.mass_kg(1e6)  # 1 cm³ = 1e6 mm³? No, 1cm³=1000mm³
        # 1000 mm³ * 7800 kg/m³ * 1e-9 m³/mm³ = 0.0078 kg = 7.8 g
        mass = mat.mass_kg(1000)
        assert abs(mass - 0.0078) < 0.001

    def test_list_materials(self):
        names = self.lib.list_materials()
        assert "ASSAB_88" in names
        assert "ASSAB_EM2" in names

    def test_filter_by_application(self):
        metal = self.lib.filter_by_application("metal_welding")
        assert all("metal_welding" in m.application for m in metal)
        assert len(metal) >= 4
```

**Step 2: Implement MaterialLibrary**

```python
# ultrasonic_weld_master/plugins/geometry_analyzer/horn_materials.py
"""Material library for ultrasonic horn design.

Loads material properties from YAML and provides acoustic calculations.
"""
from __future__ import annotations

import os
import math
from dataclasses import dataclass, field
from typing import Optional

import yaml


@dataclass
class HornMaterial:
    """Material properties for horn design."""
    id: str
    name: str
    category: str
    density_kg_m3: float
    elastic_modulus_gpa: float
    poisson_ratio: float
    sound_speed_m_s: float
    hardness_hrc_range: Optional[list[float]] = None
    heat_treatment: Optional[dict] = None
    application: list[str] = field(default_factory=list)
    composition: dict = field(default_factory=dict)

    def half_wavelength_mm(self, frequency_hz: float) -> float:
        """Calculate half-wavelength resonant length in mm."""
        return self.sound_speed_m_s / (2 * frequency_hz) * 1000

    def mass_kg(self, volume_mm3: float) -> float:
        """Calculate mass from volume in mm³."""
        return volume_mm3 * self.density_kg_m3 * 1e-9


_LIB_PATH = os.path.join(os.path.dirname(__file__), "horn_materials.yaml")


class MaterialLibrary:
    """Load and query horn material database."""

    def __init__(self, path: str = _LIB_PATH):
        with open(path) as f:
            data = yaml.safe_load(f)
        self._materials: dict[str, HornMaterial] = {}
        for mid, props in data.get("materials", {}).items():
            self._materials[mid] = HornMaterial(
                id=mid,
                name=props["name"],
                category=props.get("category", ""),
                density_kg_m3=props["density_kg_m3"],
                elastic_modulus_gpa=props["elastic_modulus_gpa"],
                poisson_ratio=props["poisson_ratio"],
                sound_speed_m_s=props["sound_speed_m_s"],
                hardness_hrc_range=props.get("hardness_hrc_range"),
                heat_treatment=props.get("heat_treatment"),
                application=props.get("application", []),
                composition=props.get("composition", {}),
            )

    def get(self, material_id: str) -> HornMaterial:
        """Get material by ID. Raises KeyError if not found."""
        if material_id not in self._materials:
            raise KeyError(f"Unknown material: {material_id}. "
                         f"Available: {list(self._materials.keys())}")
        return self._materials[material_id]

    def list_materials(self) -> list[str]:
        return list(self._materials.keys())

    def filter_by_application(self, app: str) -> list[HornMaterial]:
        return [m for m in self._materials.values() if app in m.application]
```

**Step 3:** Run tests → verify PASS
**Step 4:** Commit: `git commit -m "feat: add MaterialLibrary class with acoustic calculations"`

---

### Task 3: Create JSON Schema for Horn Instructions

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_instruction_schema.json`
- Test: `tests/test_horn_instruction/test_schema.py`

**Step 1: Write test**

```python
# tests/test_horn_instruction/test_schema.py
"""Tests for horn instruction JSON schema validation."""
import pytest
import json
import os
import yaml

try:
    import jsonschema
    HAS_JSONSCHEMA = True
except ImportError:
    HAS_JSONSCHEMA = False

SCHEMA_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..",
    "ultrasonic_weld_master", "plugins", "geometry_analyzer",
    "horn_instruction_schema.json"
)


@pytest.mark.skipif(not HAS_JSONSCHEMA, reason="jsonschema not installed")
class TestHornInstructionSchema:
    def setup_method(self):
        with open(SCHEMA_PATH) as f:
            self.schema = json.load(f)

    def test_schema_loads(self):
        assert self.schema["type"] == "object"
        assert "identity" in self.schema["properties"]

    def test_minimal_valid_instruction(self):
        instruction = {
            "identity": {
                "part_number": "TEST-001",
                "description": "Test horn",
                "horn_type": "block",
                "application": "plastic"
            },
            "material_resonance": {
                "material": "Ti6Al4V",
                "frequency_khz": 20.0,
                "frequency_tolerance_hz": 50,
                "tuning_mode": "longitudinal"
            },
            "envelope_datum": {
                "vibration_axis": "Z",
                "origin": "face_center",
                "total_length_mm": 127.0,
                "body_width_mm": 25.0,
                "body_height_mm": 25.0,
                "datums": {"A": "welding_face", "B": "coupling_bore_axis"}
            },
            "profile_sections": [
                {
                    "name": "body",
                    "y_range": [0, 127.0],
                    "cross_section": {
                        "shape": "rectangle",
                        "width_mm": 25.0,
                        "height_mm": 25.0
                    }
                }
            ]
        }
        jsonschema.validate(instruction, self.schema)

    def test_invalid_horn_type_rejected(self):
        instruction = {
            "identity": {
                "part_number": "TEST",
                "description": "Bad",
                "horn_type": "invalid_type",
                "application": "plastic"
            },
            "material_resonance": {
                "material": "Ti6Al4V",
                "frequency_khz": 20.0,
                "frequency_tolerance_hz": 50,
                "tuning_mode": "longitudinal"
            },
            "envelope_datum": {
                "vibration_axis": "Z",
                "origin": "face_center",
                "total_length_mm": 127.0,
                "body_width_mm": 25.0,
                "body_height_mm": 25.0,
                "datums": {"A": "welding_face", "B": "axis"}
            },
            "profile_sections": []
        }
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(instruction, self.schema)

    def test_missing_required_field_rejected(self):
        instruction = {
            "identity": {
                "part_number": "TEST",
                # missing description
                "horn_type": "block",
                "application": "plastic"
            }
        }
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(instruction, self.schema)
```

**Step 2:** Create the JSON Schema file (full 9-layer schema, ~300 lines of JSON). The schema must define all 9 layers with proper types, enums, required fields, and nested objects matching the design document.

**Step 3:** Run tests → verify PASS
**Step 4:** Commit: `git commit -m "feat: add JSON schema for 9-layer horn instructions"`

---

### Task 4: Create YAML Template with Lab 5-40 Example

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_instruction_template.yaml`
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/examples/lab_5-40_instruction.yaml`
- Test: `tests/test_horn_instruction/test_schema.py` (extend)

**Step 1: Write test**

```python
# Append to test_schema.py
class TestLabHornInstruction:
    def test_lab_5_40_validates(self):
        """The lab 5-40 reference instruction must pass schema validation."""
        example_path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "ultrasonic_weld_master", "plugins", "geometry_analyzer",
            "examples", "lab_5-40_instruction.yaml"
        )
        with open(example_path) as f:
            instruction = yaml.safe_load(f)
        with open(SCHEMA_PATH) as f:
            schema = json.load(f)
        jsonschema.validate(instruction, schema)

    def test_lab_5_40_has_all_layers(self):
        example_path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "ultrasonic_weld_master", "plugins", "geometry_analyzer",
            "examples", "lab_5-40_instruction.yaml"
        )
        with open(example_path) as f:
            instruction = yaml.safe_load(f)
        required_layers = [
            "identity", "material_resonance", "envelope_datum",
            "profile_sections", "coupling", "welding_face",
            "nodal_features", "edge_treatment", "quality_inspection"
        ]
        for layer in required_layers:
            assert layer in instruction, f"Missing layer: {layer}"
```

**Step 2:** Create the lab 5-40 YAML instruction file with all 9 layers filled from the geometric analysis data.

**Step 3:** Create a blank template YAML with Chinese/English comments explaining each field.

**Step 4:** Run tests → verify PASS
**Step 5:** Commit: `git commit -m "feat: add YAML template and lab 5-40 reference instruction"`

---

## Phase 2: CadQuery Driver

### Task 5: Instruction Parser Module

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/horn_instruction_driver.py`
- Test: `tests/test_horn_instruction/test_driver.py`

**Step 1: Write test**

```python
# tests/test_horn_instruction/test_driver.py
"""Tests for horn instruction driver."""
import pytest
import yaml
import os

from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
    InstructionParser, HornInstruction,
)


class TestInstructionParser:
    def setup_method(self):
        self.parser = InstructionParser()

    def test_parse_minimal(self):
        data = {
            "identity": {
                "part_number": "T-001", "description": "Test",
                "horn_type": "block", "application": "plastic"
            },
            "material_resonance": {
                "material": "Ti6Al4V", "frequency_khz": 20.0,
                "frequency_tolerance_hz": 50, "tuning_mode": "longitudinal"
            },
            "envelope_datum": {
                "vibration_axis": "Z", "origin": "face_center",
                "total_length_mm": 127.0, "body_width_mm": 25.0,
                "body_height_mm": 25.0,
                "datums": {"A": "welding_face", "B": "axis"}
            },
            "profile_sections": [
                {"name": "body", "y_range": [0, 127],
                 "cross_section": {"shape": "rectangle", "width_mm": 25, "height_mm": 25}}
            ]
        }
        inst = self.parser.parse(data)
        assert isinstance(inst, HornInstruction)
        assert inst.identity.horn_type == "block"
        assert inst.material.density_kg_m3 == 4430  # Ti6Al4V

    def test_parse_resolves_material_properties(self):
        data = {
            "identity": {"part_number": "T", "description": "T",
                        "horn_type": "blade", "application": "metal"},
            "material_resonance": {"material": "ASSAB_88", "frequency_khz": 20.0,
                                   "frequency_tolerance_hz": 50, "tuning_mode": "longitudinal"},
            "envelope_datum": {"vibration_axis": "Y", "origin": "face_center",
                             "total_length_mm": 224.0, "body_width_mm": 87.0,
                             "body_height_mm": 157.0,
                             "datums": {"A": "welding_face", "B": "axis"}},
            "profile_sections": [
                {"name": "body", "y_range": [0, 224],
                 "cross_section": {"shape": "rectangle", "width_mm": 87, "height_mm": 80}}
            ]
        }
        inst = self.parser.parse(data)
        assert inst.material.density_kg_m3 == 7800
        assert inst.material.sound_speed_m_s == 5190

    def test_parse_lab_5_40(self):
        path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "ultrasonic_weld_master", "plugins", "geometry_analyzer",
            "examples", "lab_5-40_instruction.yaml"
        )
        with open(path) as f:
            data = yaml.safe_load(f)
        inst = self.parser.parse(data)
        assert inst.identity.horn_type == "blade"
        assert len(inst.profile_sections) >= 4
```

**Step 2: Implement InstructionParser**

The parser should:
1. Validate against JSON Schema
2. Resolve material ID to full HornMaterial properties
3. Return a structured HornInstruction dataclass tree
4. Validate physical consistency (e.g., total_length ≈ half_wavelength)

**Step 3:** Run tests → PASS
**Step 4:** Commit: `git commit -m "feat: add InstructionParser with schema validation and material resolution"`

---

### Task 6: ProfileBuilder — Multi-Section Solid Generation

**Files:**
- Modify: `horn_instruction_driver.py`
- Test: `tests/test_horn_instruction/test_driver.py` (extend)

**Step 1: Write test**

```python
class TestProfileBuilder:
    def test_single_rectangular_section(self):
        """A single constant rectangle section produces a box solid."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            ProfileBuilder,
        )
        builder = ProfileBuilder()
        sections = [
            {"name": "body", "y_range": [0, 100],
             "cross_section": {"shape": "rectangle", "width_mm": 25, "height_mm": 25}}
        ]
        solid = builder.build(sections, vibration_axis="Y")
        assert solid is not None
        # Check bounding box
        bb = solid.val().BoundingBox()
        assert abs(bb.ylen - 100.0) < 1.0
        assert abs(bb.xlen - 25.0) < 1.0

    def test_circular_section(self):
        builder = ProfileBuilder()
        sections = [
            {"name": "neck", "y_range": [0, 50],
             "cross_section": {"shape": "circular", "diameter_mm": 40}}
        ]
        solid = builder.build(sections, vibration_axis="Y")
        bb = solid.val().BoundingBox()
        assert abs(bb.ylen - 50.0) < 1.0
        assert abs(bb.xlen - 40.0) < 1.0

    def test_multi_section_union(self):
        """Two sections with different cross-sections produce a union."""
        builder = ProfileBuilder()
        sections = [
            {"name": "face", "y_range": [0, 50],
             "cross_section": {"shape": "rectangle", "width_mm": 80, "height_mm": 80}},
            {"name": "neck", "y_range": [50, 100],
             "cross_section": {"shape": "circular", "diameter_mm": 40}},
        ]
        solid = builder.build(sections, vibration_axis="Y")
        bb = solid.val().BoundingBox()
        assert abs(bb.ylen - 100.0) < 1.0
        assert bb.xlen >= 79.0  # at least face width
```

**Step 2: Implement ProfileBuilder**

ProfileBuilder takes the `profile_sections` list and builds a CadQuery solid by:
1. Creating each section as an extruded cross-section along the vibration axis
2. Positioning sections at the correct Y offsets
3. Unioning all sections into a single solid
4. For transition sections (`profile_type != "constant"`), using CadQuery `loft` between start and end cross-sections

**Step 3:** Run tests → PASS
**Step 4:** Commit: `git commit -m "feat: add ProfileBuilder for multi-section horn solid generation"`

---

### Task 7: FeatureApplicator — Coupling, Knurl, Holes, Slots

**Files:**
- Modify: `horn_instruction_driver.py`
- Test: `tests/test_horn_instruction/test_driver.py` (extend)

**Step 1: Write test**

```python
class TestFeatureApplicator:
    def test_coupling_bore(self):
        """Apply a threaded bore to the coupling end."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            ProfileBuilder, FeatureApplicator,
        )
        builder = ProfileBuilder()
        sections = [
            {"name": "body", "y_range": [0, 100],
             "cross_section": {"shape": "circular", "diameter_mm": 40}}
        ]
        solid = builder.build(sections, vibration_axis="Y")

        applicator = FeatureApplicator()
        coupling = {
            "type": "threaded_bore",
            "bore": {"diameter_mm": 20, "depth_mm": 15},
        }
        result = applicator.apply_coupling(solid, coupling, vibration_axis="Y")
        # Volume should decrease (bore removed material)
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        p1 = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid.val().wrapped, p1)
        p2 = GProp_GProps()
        BRepGProp.VolumeProperties_s(result.val().wrapped, p2)
        assert p2.Mass() < p1.Mass()

    def test_through_hole(self):
        """Apply a through-hole at the node."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            ProfileBuilder, FeatureApplicator,
        )
        builder = ProfileBuilder()
        sections = [
            {"name": "body", "y_range": [0, 100],
             "cross_section": {"shape": "rectangle", "width_mm": 40, "height_mm": 80}}
        ]
        solid = builder.build(sections, vibration_axis="Y")

        applicator = FeatureApplicator()
        holes = [
            {"axis": "X", "diameter_mm": 10, "type": "through",
             "z_position_mm": 40, "y_position_mm": 50, "count": 1}
        ]
        result = applicator.apply_mounting_holes(solid, holes)
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        p1 = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid.val().wrapped, p1)
        p2 = GProp_GProps()
        BRepGProp.VolumeProperties_s(result.val().wrapped, p2)
        assert p2.Mass() < p1.Mass()
```

**Step 2: Implement FeatureApplicator**

Methods:
- `apply_coupling(solid, coupling_dict, vibration_axis)` → CadQuery solid with bore/thread
- `apply_knurl(solid, knurl_dict)` → CadQuery solid with knurl teeth (cone features)
- `apply_mounting_holes(solid, holes_list)` → CadQuery solid with drilled holes
- `apply_slots(solid, slots_list)` → CadQuery solid with relief slots

**Step 3:** Run tests → PASS
**Step 4:** Commit: `git commit -m "feat: add FeatureApplicator for coupling, holes, and slots"`

---

### Task 8: EdgeProcessor & Full Pipeline

**Files:**
- Modify: `horn_instruction_driver.py`
- Test: `tests/test_horn_instruction/test_driver.py` (extend)

**Step 1: Write test**

```python
class TestEdgeProcessor:
    def test_chamfer_reduces_volume(self):
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            ProfileBuilder, EdgeProcessor,
        )
        builder = ProfileBuilder()
        sections = [
            {"name": "body", "y_range": [0, 50],
             "cross_section": {"shape": "rectangle", "width_mm": 25, "height_mm": 25}}
        ]
        solid = builder.build(sections, vibration_axis="Y")
        processor = EdgeProcessor()
        edge_spec = {
            "chamfers": [
                {"location": "all_edges", "size_mm": 0.5, "angle_deg": 45}
            ],
            "general_deburr": True
        }
        result = processor.apply(solid, edge_spec)
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        p1 = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid.val().wrapped, p1)
        p2 = GProp_GProps()
        BRepGProp.VolumeProperties_s(result.val().wrapped, p2)
        assert p2.Mass() < p1.Mass()


class TestFullPipeline:
    def test_simple_block_horn_pipeline(self):
        """End-to-end: YAML → solid → STEP."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        instruction = {
            "identity": {"part_number": "PIPE-001", "description": "Pipeline test",
                        "horn_type": "block", "application": "plastic"},
            "material_resonance": {"material": "Ti6Al4V", "frequency_khz": 20.0,
                                   "frequency_tolerance_hz": 50, "tuning_mode": "longitudinal"},
            "envelope_datum": {"vibration_axis": "Z", "origin": "face_center",
                             "total_length_mm": 127.0, "body_width_mm": 25.0,
                             "body_height_mm": 25.0,
                             "datums": {"A": "welding_face", "B": "axis"}},
            "profile_sections": [
                {"name": "body", "y_range": [0, 127],
                 "cross_section": {"shape": "rectangle", "width_mm": 25, "height_mm": 25}}
            ],
            "coupling": {"type": "threaded_bore",
                        "bore": {"diameter_mm": 10, "depth_mm": 15}},
            "edge_treatment": {"chamfers": [{"location": "all_edges", "size_mm": 0.3}],
                             "general_deburr": True}
        }
        driver = HornInstructionDriver()
        result = driver.execute(instruction)
        assert result.solid is not None
        assert result.step_data is not None
        assert len(result.step_data) > 1000
        assert result.volume_mm3 > 0
        assert result.mass_kg > 0
```

**Step 2: Implement HornInstructionDriver** (orchestrator)

```python
class HornInstructionDriver:
    """Main driver: translates instruction dict → CadQuery solid + outputs."""

    def __init__(self):
        self._parser = InstructionParser()
        self._profile = ProfileBuilder()
        self._features = FeatureApplicator()
        self._edges = EdgeProcessor()

    def execute(self, instruction: dict) -> HornBuildResult:
        inst = self._parser.parse(instruction)
        solid = self._profile.build(inst.profile_sections, inst.envelope.vibration_axis)
        if inst.coupling:
            solid = self._features.apply_coupling(solid, inst.coupling, inst.envelope.vibration_axis)
        if inst.welding_face and inst.welding_face.get("knurl", {}).get("type", "none") != "none":
            solid = self._features.apply_knurl(solid, inst.welding_face["knurl"])
        if inst.nodal_features:
            if "mounting_holes" in inst.nodal_features:
                solid = self._features.apply_mounting_holes(solid, inst.nodal_features["mounting_holes"])
            if "slots" in inst.nodal_features:
                solid = self._features.apply_slots(solid, inst.nodal_features["slots"])
        if inst.edge_treatment:
            solid = self._edges.apply(solid, inst.edge_treatment)

        # Export
        import tempfile, cadquery as cq
        step_path = os.path.join(tempfile.gettempdir(), f"horn_{inst.identity.part_number}.step")
        cq.exporters.export(solid, step_path)
        with open(step_path, "rb") as f:
            step_data = f.read()

        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid.val().wrapped, props)
        volume = props.Mass()

        return HornBuildResult(
            solid=solid,
            step_data=step_data,
            volume_mm3=volume,
            mass_kg=inst.material.mass_kg(volume),
            part_number=inst.identity.part_number,
        )
```

**Step 3:** Run tests → PASS
**Step 4:** Commit: `git commit -m "feat: add HornInstructionDriver full pipeline"`

---

## Phase 3: Validation

### Task 9: Lab 5-40 Reconstruction Validation

**Files:**
- Create: `tests/test_horn_instruction/test_validation.py`

**Step 1: Write validation test**

```python
# tests/test_horn_instruction/test_validation.py
"""Validate horn instruction driver against real production horn lab 5-40."""
import pytest
import os
import yaml

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box


# Reference values from lab 5-40.stp analysis
REF_VOLUME_MM3 = 603353.55
REF_BBOX = {"x": 86.60, "y": 223.95, "z": 157.20}
REF_MASS_KG = 2.673  # at ASSAB 88 density


class TestLab540Reconstruction:
    @pytest.fixture
    def lab_instruction(self):
        path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "ultrasonic_weld_master", "plugins", "geometry_analyzer",
            "examples", "lab_5-40_instruction.yaml"
        )
        with open(path) as f:
            return yaml.safe_load(f)

    def test_builds_without_error(self, lab_instruction):
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        driver = HornInstructionDriver()
        result = driver.execute(lab_instruction)
        assert result.solid is not None
        assert result.step_data is not None

    def test_volume_within_tolerance(self, lab_instruction):
        """Volume should be within 10% of reference."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        driver = HornInstructionDriver()
        result = driver.execute(lab_instruction)
        deviation = abs(result.volume_mm3 - REF_VOLUME_MM3) / REF_VOLUME_MM3
        assert deviation < 0.10, f"Volume deviation {deviation*100:.1f}% exceeds 10%"

    def test_bounding_box_within_tolerance(self, lab_instruction):
        """Bounding box should match within ±2mm per axis."""
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        driver = HornInstructionDriver()
        result = driver.execute(lab_instruction)
        bb = result.solid.val().BoundingBox()
        assert abs(bb.xlen - REF_BBOX["x"]) < 2.0, f"X: {bb.xlen} vs {REF_BBOX['x']}"
        assert abs(bb.ylen - REF_BBOX["y"]) < 2.0, f"Y: {bb.ylen} vs {REF_BBOX['y']}"
        assert abs(bb.zlen - REF_BBOX["z"]) < 2.0, f"Z: {bb.zlen} vs {REF_BBOX['z']}"

    def test_step_export_valid(self, lab_instruction):
        """Exported STEP file should be reimportable."""
        import tempfile
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        driver = HornInstructionDriver()
        result = driver.execute(lab_instruction)
        path = os.path.join(tempfile.gettempdir(), "lab540_test.step")
        with open(path, "wb") as f:
            f.write(result.step_data)
        reimported = cq.importers.importStep(path)
        assert reimported is not None

    def test_engineering_drawing_output(self, lab_instruction):
        """Should produce valid engineering drawing from reconstructed horn."""
        import tempfile
        from ultrasonic_weld_master.plugins.geometry_analyzer.horn_instruction_driver import (
            HornInstructionDriver,
        )
        from ultrasonic_weld_master.plugins.drawing_engine.projector import ShapeProjector
        from ultrasonic_weld_master.plugins.drawing_engine.layout import create_standard_layout
        from ultrasonic_weld_master.plugins.drawing_engine.renderer_matplotlib import MatplotlibRenderer

        driver = HornInstructionDriver()
        result = driver.execute(lab_instruction)

        projector = ShapeProjector()
        views = projector.project_with_iso(result.solid)
        assert len(views) == 4
        assert all(len(v.visible_edges) > 0 for v in views.values())

        layout = create_standard_layout(views=views, paper_size="A3",
                                       title=lab_instruction["identity"]["description"])
        renderer = MatplotlibRenderer()
        pdf_path = os.path.join(tempfile.gettempdir(), "lab540_drawing_test.pdf")
        renderer.render_pdf(layout, pdf_path)
        assert os.path.getsize(pdf_path) > 5000
```

**Step 2:** Run tests → verify how close we get. Iterate on the driver and instruction until validation passes.
**Step 3:** Commit: `git commit -m "test: add lab 5-40 reconstruction validation suite"`

---

### Task 10: Generate Final Output Files

**Files:**
- Create: `ultrasonic_weld_master/plugins/geometry_analyzer/examples/simple_block_horn.yaml`
- Create: `docs/horn-instruction-guide.md` (Chinese + English reference)

**Step 1:** Create a simple block horn example YAML as a starter template

**Step 2:** Write the reference documentation in Chinese explaining:
- How to fill each layer
- LLM conversation flow for engineers
- Material selection guide
- Common horn types and their parameter patterns
- Troubleshooting guide

**Step 3:** Commit: `git commit -m "docs: add horn instruction guide and simple example"`

---

## Phase Summary

| Phase | Tasks | Key Deliverable |
|-------|-------|----------------|
| **Phase 1** | Tasks 1-4 | Material library + JSON Schema + YAML templates |
| **Phase 2** | Tasks 5-8 | CadQuery driver (parser → profile → features → edges) |
| **Phase 3** | Tasks 9-10 | Validation against lab 5-40 + documentation |

**Total estimated commits:** 10
**Total estimated new files:** ~12
**Total estimated new test cases:** ~25
