# Unity Scene Setup Guide for VIMA

## Coordinate System Notes
- **PyBullet**: Right-handed, Z-up, workspace center at (0.5, 0, 0)
- **Unity**: Left-handed, Y-up (we'll adapt coordinates)
- **1 Unity unit = 1 meter** (same as PyBullet)

## Workspace Bounds (PyBullet → Unity conversion)
- **X**: 0.25 to 0.75 meters (workspace width: 0.5m)
- **Y**: -0.5 to 0.5 meters (workspace depth: 1.0m) 
- **Z**: 0 to 0.3 meters (height)
- **Table center**: (0.5, 0, 0) in PyBullet → **(0, 0, 0)** in Unity (we'll center at origin)

## Scene Hierarchy

```
VIMAEnvironment (root, empty GameObject)
├── Workspace
│   ├── GroundPlane (Plane, scale 10x10, position Y=-0.001)
│   └── Table (Plane or imported mesh, position Y=0)
├── RobotBase (empty, position (0, 0, 0) - marker for UR5 base)
├── Objects (empty parent)
│   ├── BaseObject_1 (e.g., Bowl)
│   ├── DraggedObject_1 (e.g., Block)
│   └── DistractorObject_1 (optional)
└── Cameras (empty parent)
    ├── FrontCam (Camera)
    └── TopCam (Camera)
```

## Object Setup

### 1. Ground Plane
- **Type**: Unity Plane primitive
- **Position**: (0, -0.001, 0)
- **Scale**: (10, 1, 10) - large ground plane
- **Material**: Dark gray (RGB: 0.2, 0.2, 0.2)

### 2. Table/Workspace
- **Type**: Unity Plane primitive (or imported workspace mesh)
- **Position**: (0, 0, 0)
- **Scale**: (0.5, 1, 1.0) - matches workspace bounds
- **Material**: Dark gray (RGB: 0.2, 0.2, 0.2)
- **Rotation**: (0, 0, 0)

### 3. Base Object (Container)
**Default options**: BOWL, PAN, FRAME, SQUARE, CONTAINER, PALLET

**Example: BOWL**
- **Type**: Import from `VimaBench/vima_bench/tasks/assets/bowl/` or use Unity primitive
- **Position**: Random within workspace, e.g., **(0.45, 0.2, 0.05)**
- **Size**: Typical bowl size ~0.08m diameter, 0.04m height
- **Category**: Fixed (kinematic, doesn't move)
- **Material**: Colored (from texture palette)

**Example: PAN**
- **Position**: Random, e.g., **(0.55, -0.2, 0.02)**
- **Size**: ~0.12m x 0.12m x 0.02m

### 4. Dragged Object (Small Object)
**Default options**: BLOCK, L_BLOCK, CAPITAL_LETTER_A/E/G/M/R/T/V, CROSS, DIAMOND, TRIANGLE, FLOWER, HEART, HEXAGON, PENTAGON, RING, ROUND, STAR

**Example: BLOCK**
- **Type**: Unity Cube primitive (or imported mesh)
- **Position**: Random within workspace, e.g., **(0.35, -0.15, 0.03)**
- **Size**: ~0.04m x 0.04m x 0.04m (typical block size)
- **Category**: Rigid (physics enabled)
- **Material**: Colored (different from base object)

**Example: ROUND**
- **Type**: Unity Sphere primitive
- **Position**: Random, e.g., **(0.65, 0.1, 0.02)**
- **Size**: ~0.05m diameter

### 5. Distractor Object (Optional)
- **Type**: Any object (can be dragged or base type)
- **Position**: Random within workspace, e.g., **(0.4, 0.3, 0.03)**
- **Size**: Varies
- **Category**: Rigid or Fixed (depending on type)

## Example Scene Layout (Simple Case)

For a minimal `visual_manipulation` scene with seed=42:

```
Object          Position (X, Y, Z)    Size (W, H, D)    Type
------------------------------------------------------------------------
GroundPlane     (0, -0.001, 0)        (10, 1, 10)       Plane
Table           (0, 0, 0)             (0.5, 1, 1.0)     Plane
Bowl            (0.45, 0.2, 0.05)     (0.08, 0.04, 0.08) Bowl (fixed)
Block           (0.35, -0.15, 0.03)   (0.04, 0.04, 0.04) Cube (rigid)
Distractor      (0.4, 0.3, 0.03)      (0.05, 0.05, 0.05) Sphere (rigid)
```

## Camera Setup

### Front Camera
- **Position**: (0, 0.5, -1.0) - in front of table
- **Rotation**: (15, 0, 0) - looking down slightly
- **Field of View**: ~60 degrees
- **Near/Far**: 0.1 / 10

### Top Camera  
- **Position**: (0, 2.0, 0) - directly above table
- **Rotation**: (90, 0, 0) - looking straight down
- **Field of View**: ~45 degrees (to match workspace)
- **Near/Far**: 0.1 / 10

## Object Size Ranges (from VIMA code)

These are typical size ranges - objects are randomly sampled within these:

**Base Objects:**
- BOWL: ~0.06-0.10m diameter, 0.03-0.05m height
- PAN: ~0.10-0.15m x 0.10-0.15m x 0.02m
- CONTAINER: ~0.08-0.12m x 0.08-0.12m x 0.05m

**Dragged Objects:**
- BLOCK: ~0.03-0.05m x 0.03-0.05m x 0.03-0.05m
- ROUND: ~0.04-0.06m diameter
- Letters/Shapes: ~0.04-0.08m (varies by shape)

## Quick Start: Simple Test Scene

Create this minimal scene to get started:

1. **Ground Plane**: Unity Plane at (0, -0.001, 0), scale (10, 1, 10)
2. **Table**: Unity Plane at (0, 0, 0), scale (0.5, 1, 1.0)
3. **Bowl**: Unity Cylinder at (0.45, 0.2, 0.05), scale (0.08, 0.04, 0.08)
4. **Block**: Unity Cube at (0.35, -0.15, 0.03), scale (0.04, 0.04, 0.04)
5. **Front Camera**: Position (0, 0.5, -1.0), Rotation (15, 0, 0)
6. **Top Camera**: Position (0, 2.0, 0), Rotation (90, 0, 0)

## Notes

- Objects are randomly placed within workspace bounds in the actual task
- For testing, use fixed positions first, then add randomization
- Make sure objects sit ON the table (Z position = object height/2)
- Base objects are typically "fixed" (kinematic) - they don't move
- Dragged objects are "rigid" - they have physics and can be moved

## Next Steps

1. Set up this basic scene structure
2. Add physics components (Rigidbody, Colliders)
3. Test camera views match PyBullet
4. Add object spawning system for randomization

