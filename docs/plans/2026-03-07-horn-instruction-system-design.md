# Ultrasonic Welding Horn Instruction System Design

**Date:** 2026-03-07
**Purpose:** Define a 9-layer structured instruction specification that can precisely describe production-grade ultrasonic welding horns, drive CadQuery-based 3D modeling, and output engineering drawings.

## 1. Background

### 1.1 Problem Statement

The current `HornGenerator` on weld-sim produces primitive block-shaped horns (flat/cylindrical/exponential/blade/stepped) that lack the manufacturing detail required for real production. A real production horn (e.g., `lab 5-40.stp`) contains:

| Feature | Real Horn (lab 5-40) | Current System |
|---------|---------------------|----------------|
| Topology | 250 faces, 1023 edges, 6 BSpline surfaces | ~20 faces |
| Cross-section | Multi-stage profile with BSpline transitions | Simple prism |
| Welding face | 66 diamond knurl teeth (cone features) | No knurl |
| Coupling | Ø38/Ø45mm bore, flange, alignment pins | Simple hole |
| Nodal features | Precision relief slots, Ø10mm through-holes | None |
| Edge treatment | R2mm fillets ×4, R1mm ×2, 45°/67.5° chamfers | 0.5mm chamfer |
| Mass | 2.67 kg (ASSAB 88 steel) | N/A |

### 1.2 Objectives

1. Define a structured instruction format (JSON Schema + YAML templates) that covers ALL manufacturing features of production horns
2. Build a CadQuery driver layer that translates instructions into 3D solid models
3. Validate by reconstructing the `lab 5-40` horn from instructions and comparing geometry
4. Enable engineers to produce correct instructions through LLM-guided conversations

### 1.3 Reference Horn Analysis: lab 5-40.stp

**Type:** Blade horn (longitudinal vibration)
**Frequency:** 20 kHz
**Material:** Tool steel (ASSAB 88 or equivalent)
**Overall dimensions:** 86.6 × 224.0 × 157.2 mm
**Volume:** 603.35 cm³
**Mass estimate:** ~2.67 kg (ASSAB 88, ρ=7800 kg/m³)

#### Geometric Feature Inventory

**Cylindrical features (44 surfaces):**
- Ø3.4mm: Alignment pin holes (4 surfaces)
- Ø5.0mm: Bolt holes (4 surfaces)
- Ø5.6mm: Lateral alignment holes (2 surfaces)
- Ø6.0mm: Thread holes, top/bottom (6 surfaces)
- Ø10.0mm: Through mounting holes at node (4 surfaces)
- Ø10.8mm: Stepped bore features (13 surfaces)
- Ø38.0mm: Coupling bore (1 surface)
- Ø40.0mm: Coupling neck (2 surfaces)
- Ø45.0mm: Coupling flange (4 surfaces)

**Cone features (72 surfaces):**
- 45°: Entry chamfers at coupling bore (2 surfaces)
- 67.5°: Flange chamfers (4 surfaces)
- 46.55°: Diamond knurl teeth (66 surfaces, R_tip=0.17mm)

**Torus features (6 surfaces):**
- R_major=24.5mm, R_minor=2.0mm: Gain transition fillets (4)
- R_major=6.33mm, R_minor=1.0mm: Bore entry fillets (2)

**BSpline surfaces (6):**
- 2 large (483/502 mm²): Horn profile transition curves
- 4 small (7.9 mm² each): Knurl zone blend surfaces

**Z-level cross-section map:**

| Z (mm) | Description | Key Dimensions |
|--------|-------------|----------------|
| -77.1 | Coupling end face | Ø14.8 pilot, Ø38 bore |
| -76.6 | Bore shoulder | Ø38 → Ø45 transition |
| -67.1 | Flange start | Ø45 → Ø49 taper |
| -55.1 ~ -59.1 | Stepped bore features | Ø10.8, Ø12.6, Ø14.6 |
| -41.6 | Alignment holes level | Ø5.6 lateral holes |
| -2.7 | Neck-to-body transition | Ø40 → rectangular |
| 0.0 | Body bottom face | 86.6×80mm rectangle, area=4320mm² |
| 28.5 ~ 51.5 | Mounting holes | Ø10 through-holes |
| 33.5 ~ 46.5 | Knurl/slot zone | Knurl teeth + relief slots |
| 80.0 | Body top face | area=4477mm², Ø6 threads |

## 2. Material Library

### 2.1 Supported Materials

| Material ID | Grade | Standard | Density (kg/m³) | E (GPa) | ν | c (m/s) | HRC Range | Application |
|------------|-------|----------|-----------------|---------|---|---------|-----------|-------------|
| `ASSAB_88` | Sleipner | - | 7800 | 210 | 0.29 | 5190 | 60-64 | Metal welding |
| `ASSAB_EM2` | M2/W6Mo5Cr4V2 | AISI M2 | 8160 | 223 | 0.29 | 5160 | 62-65 | Metal welding |
| `Ti6Al4V` | Grade 5 | AMS 4928 | 4430 | 114 | 0.34 | 5090 | 30-36 | Plastic welding |
| `Al7075_T6` | 7075-T6 | AMS 4045 | 2810 | 72 | 0.33 | 5080 | - | Plastic welding |
| `D2` | AISI D2 | - | 7670 | 208 | 0.28 | 5210 | 58-62 | Metal welding |
| `CPM_10V` | CPM 10V | - | 7418 | 221 | 0.30 | 5460 | 58-60 | Metal welding |

### 2.2 Sound Speed Note

Thin-wire wave speed `c = sqrt(E/ρ)` is used for half-wavelength calculation:
- Half wavelength `λ/2 = c / (2 × f)` where f = frequency in Hz
- For ASSAB 88 at 20kHz: `λ/2 = 5190 / (2 × 20000) × 1000 = 129.75 mm`
- For ASSAB EM2 at 20kHz: `λ/2 = 5160 / (2 × 20000) × 1000 = 129.0 mm`

## 3. Instruction Schema: 9 Layers

### Layer 1: identity
Basic part identification and classification.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| part_number | string | yes | Unique part identifier |
| revision | string | no | Revision letter (default "A") |
| description | string | yes | Human-readable description |
| horn_type | enum | yes | `blade` \| `block` \| `radial` \| `compound` |
| application | enum | yes | `metal` \| `plastic` \| `fabric` \| `food` |

### Layer 2: material_resonance
Material selection and acoustic/resonance requirements.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| material | enum | yes | Material ID from library |
| frequency_khz | float | yes | Target operating frequency |
| frequency_tolerance_hz | float | yes | Allowable frequency deviation (±) |
| tuning_mode | enum | yes | `longitudinal` \| `lateral` \| `torsional` |
| amplitude_um | float | no | Peak-to-peak amplitude at face (μm) |
| gain_ratio | float | no | Amplitude gain (face/coupling) |
| hardness_spec.target_hrc | float | cond. | Required for steel materials |
| hardness_spec.tolerance_hrc | float | cond. | HRC tolerance band (±) |
| heat_treatment.hardening_temp_c | float | cond. | Austenitizing temperature |
| heat_treatment.tempering_temp_c | float | cond. | Tempering temperature |
| heat_treatment.tempering_cycles | int | cond. | Number of temper cycles |

### Layer 3: envelope_datum
Overall dimensions, coordinate system, and GD&T datums.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| vibration_axis | enum | yes | `X` \| `Y` \| `Z` |
| origin | enum | yes | `face_center` \| `coupling_center` \| `nodal_center` |
| total_length_mm | float | yes | Resonant length along vibration axis |
| body_width_mm | float | yes | Perpendicular width |
| body_height_mm | float | yes | Perpendicular height |
| datums.A | string | yes | Primary datum surface |
| datums.B | string | yes | Secondary datum |
| datums.C | string | no | Tertiary datum |
| weight_limit_kg | float | no | Maximum mass constraint |

### Layer 4: profile_sections
Multi-segment cross-section profile along the vibration axis. This is the core geometric definition.

Each section is an object with:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| name | string | yes | Section identifier |
| y_range | [float, float] | yes | Start/end along vibration axis |
| profile_type | enum | cond. | For transitions: `constant` \| `linear` \| `exponential` \| `catenoidal` \| `bspline` |
| cross_section | object | cond. | For constant sections |
| cross_section.shape | enum | yes | `rectangle` \| `circular` \| `ellipse` \| `custom` |
| cross_section.width_mm | float | cond. | For rectangle |
| cross_section.height_mm | float | cond. | For rectangle |
| cross_section.diameter_mm | float | cond. | For circular |
| cross_section.corner_radius_mm | float | no | For rounded rectangle |
| cross_section.tolerance | string | no | ISO tolerance grade |
| start_section | object | cond. | For transition start |
| end_section | object | cond. | For transition end |
| blend_radius_mm | float | no | Transition fillet radius |

### Layer 5: coupling
Booster coupling interface definition.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | enum | yes | `threaded_bore` \| `threaded_stud` \| `flange` |
| bore.diameter_mm | float | cond. | Coupling bore ID |
| bore.depth_mm | float | cond. | Bore depth |
| bore.tolerance | string | cond. | ISO tolerance (e.g., "H7") |
| bore.surface_finish_ra | float | cond. | Ra in μm |
| thread.spec | string | cond. | Thread designation (e.g., "M10x1.0") |
| thread.class | string | cond. | Thread class (e.g., "6H") |
| thread.depth_mm | float | cond. | Thread depth |
| pilot.diameter_mm | float | no | Pilot bore diameter |
| pilot.depth_mm | float | no | Pilot bore depth |
| flange.outer_diameter_mm | float | no | Flange OD |
| flange.height_mm | float | no | Flange thickness |
| alignment.type | enum | no | `pin_holes` \| `keyway` \| `flat` |
| alignment.hole_diameter_mm | float | no | Alignment hole diameter |
| alignment.count | int | no | Number of alignment features |
| alignment.angular_position_deg | [float] | no | Angular positions |
| alignment.tolerance | string | no | Hole tolerance |
| chamfer.entry_angle_deg | float | no | Bore entry chamfer angle |
| chamfer.entry_depth_mm | float | no | Bore entry chamfer depth |

### Layer 6: welding_face
Welding contact surface definition including knurl pattern.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| profile | enum | yes | `flat` \| `crowned` \| `channeled` \| `stepped` |
| width_mm | float | yes | Face width |
| contact_length_mm | float | yes | Contact length along vibration axis |
| surface_finish.face_ra | float | yes | Surface roughness Ra (μm) |
| surface_finish.flatness_mm | float | yes | Flatness tolerance |
| surface_finish.parallelism_mm | float | no | Parallelism to datum |
| knurl.type | enum | yes | `none` \| `diamond` \| `linear` \| `cross_hatch` \| `custom` |
| knurl.zone.z_start_mm | float | cond. | Knurl zone start |
| knurl.zone.z_end_mm | float | cond. | Knurl zone end |
| knurl.geometry.tooth_count | int | cond. | Total tooth count |
| knurl.geometry.rows | int | cond. | Number of rows |
| knurl.geometry.columns | int | cond. | Teeth per row |
| knurl.geometry.pitch_z_mm | float | cond. | Z-direction pitch |
| knurl.geometry.pitch_x_mm | float | cond. | X-direction pitch |
| knurl.geometry.tooth_half_angle_deg | float | cond. | Tooth cone half-angle |
| knurl.geometry.tooth_tip_radius_mm | float | cond. | Tooth tip radius |
| knurl.geometry.tooth_base_diameter_mm | float | cond. | Tooth base diameter |
| knurl.geometry.tooth_height_mm | float | cond. | Tooth protrusion height |

### Layer 7: nodal_features
Features at or near the vibration node (zero-displacement plane).

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| nodal_position_y_mm | float | yes | Node location along vibration axis |
| slots[].name | string | yes | Slot identifier |
| slots[].orientation | enum | yes | `width` \| `height` \| `diagonal` |
| slots[].z_center_mm | float | yes | Slot center Z position |
| slots[].width_mm | float | yes | Slot extent |
| slots[].depth_mm | float | yes | Cut depth |
| slots[].slot_width_mm | float | yes | Opening width |
| slots[].surface_finish_ra | float | no | Ra |
| slots[].count | int | no | Number of identical slots |
| mounting_holes[].name | string | yes | Hole identifier |
| mounting_holes[].axis | enum | yes | `X` \| `Y` \| `Z` |
| mounting_holes[].diameter_mm | float | yes | Hole diameter |
| mounting_holes[].type | enum | yes | `through` \| `blind` \| `blind_threaded` |
| mounting_holes[].thread_spec | string | cond. | For threaded holes |
| mounting_holes[].depth_mm | float | cond. | For blind holes |
| mounting_holes[].z_position_mm | float | yes | Z location |
| mounting_holes[].y_position_mm | float | no | Y location |
| mounting_holes[].y_positions_mm | [float] | no | Multiple Y locations |
| mounting_holes[].count | int | yes | Count |
| mounting_holes[].tolerance | string | no | Hole tolerance |
| mounting_holes[].surface_finish_ra | float | no | Ra |

### Layer 8: edge_treatment
Fillets, chamfers, and deburring specifications.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| fillets[].location | string | yes | Feature location description |
| fillets[].radius_mm | float | yes | Fillet radius |
| fillets[].count | int | no | Number of fillets |
| chamfers[].location | string | yes | Feature location description |
| chamfers[].size_mm | float | yes | Chamfer size |
| chamfers[].angle_deg | float | no | Chamfer angle (default 45°) |
| chamfers[].edges | string | no | Edge selector |
| general_deburr | bool | no | All sharp edges deburred |

### Layer 9: quality_inspection
Tolerancing, inspection, and acceptance criteria.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| general_tolerance | string | yes | ISO 2768 class |
| surface_finish_default_ra | float | yes | Default Ra for unmarked surfaces |
| critical_dimensions[].desc | string | yes | Dimension description |
| critical_dimensions[].spec | string | yes | Tolerance specification |
| critical_dimensions[].method | string | no | Inspection method |
| acceptance_criteria.frequency_pass | bool | yes | Frequency test required |
| acceptance_criteria.visual_inspection | bool | yes | Visual check |
| acceptance_criteria.dimensional_report | bool | no | CMM report |
| acceptance_criteria.hardness_test | bool | cond. | For hardened steels |

## 4. Implementation Plan

### 4.1 Deliverables

1. **JSON Schema** (`horn_instruction_schema.json`): Machine-readable validation schema
2. **YAML Template** (`horn_instruction_template.yaml`): Human-readable example with comments
3. **Material Library** (`horn_materials.yaml`): Material database with acoustic properties
4. **CadQuery Driver** (`horn_instruction_driver.py`): Translates YAML/JSON instructions to CadQuery operations
5. **Validation Script** (`validate_horn_instruction.py`): Validates instruction completeness and physical consistency
6. **Lab 5-40 Reconstruction** (`lab_5-40_instruction.yaml`): Full instruction set for the reference horn

### 4.2 CadQuery Driver Architecture

```
YAML/JSON Instruction
        │
        ▼
┌──────────────────┐
│ InstructionParser │  ← Validate schema, resolve material properties
│                  │
├──────────────────┤
│ ProfileBuilder   │  ← Build multi-section cross-section profile
│                  │     (constant/exponential/catenoidal/BSpline)
├──────────────────┤
│ FeatureApplicator│  ← Apply coupling, knurl, slots, holes
│                  │
├──────────────────┤
│ EdgeProcessor    │  ← Apply fillets, chamfers
│                  │
├──────────────────┤
│ QualityAnnotator │  ← Attach GD&T metadata for drawing output
└──────────────────┘
        │
        ▼
   CadQuery Solid + STEP File + Engineering Drawing
```

### 4.3 Validation Criteria

The system is considered validated when:
1. Lab 5-40 instruction produces a solid with ≤5% volume deviation from the original
2. Bounding box dimensions match within ±1mm
3. Face count is within ±20% (due to tessellation differences)
4. Engineering drawing shows correct three-view projection with dimensions
5. Frequency estimate from half-wavelength matches 20kHz ± 500Hz

## 5. LLM Conversation Guide

When an engineer wants to design a horn through LLM chat, the LLM should follow this questioning sequence:

### Stage 1: Classification (Layer 1-2)
- "What type of welding? (metal/plastic/fabric)"
- "What frequency? (15kHz/20kHz/30kHz/35kHz/40kHz)"
- "What horn type? (blade for long seam, block for spot weld, radial for circular)"
- "What material? (ASSAB 88/EM2 for metal, Ti6Al4V for plastic)"

### Stage 2: Geometry (Layer 3-4)
- "What are the welding face dimensions? (width × length)"
- "What is the body cross-section? (same as face, or larger for gain)"
- "How many profile sections along the vibration axis?"
- "What transition type between sections? (step/exponential/catenoidal)"

### Stage 3: Interface (Layer 5)
- "Coupling type? (threaded bore for standard, stud for special)"
- "Thread specification? (M10 for 20kHz, M8 for higher frequencies)"
- "Need alignment features? (pin holes/keyway)"

### Stage 4: Surface (Layer 6)
- "Welding face profile? (flat for general, crowned for wire bonding)"
- "Knurl pattern? (diamond for metal, linear for plastic, none for smooth)"
- "Knurl specifications? (pitch, depth, tooth angle)"

### Stage 5: Mounting (Layer 7)
- "Mounting method? (nodal clamp/through-bolt/ring mount)"
- "Need relief slots? (yes for high-amplitude, no for standard)"

### Stage 6: Finish (Layer 8-9)
- "Edge treatment? (chamfer/fillet/both)"
- "Surface finish class? (Ra 0.4 for face, Ra 1.6 for holes, Ra 3.2 general)"
- "General tolerance class? (ISO 2768-m for standard, -f for precision)"

After collecting all answers, the LLM assembles the complete 9-layer YAML instruction and presents it for engineer review.
