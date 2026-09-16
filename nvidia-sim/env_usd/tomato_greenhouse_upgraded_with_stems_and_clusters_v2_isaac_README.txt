Isaac Sim USD conversion

Source
------
tomato_greenhouse_upgraded_with_stems_and_clusters_v2.glb

Outputs
-------
tomato_greenhouse_upgraded_with_stems_and_clusters_v2_isaac.usd
tomato_greenhouse_upgraded_with_stems_and_clusters_v2_isaac.usda

Conversion choices
------------------
- Converted the GLB scene into OpenUSD ASCII (USDA syntax).
- The .usd file is the same USD layer using the generic .usd extension.
- Converted GLB Y-up coordinates to Isaac-friendly Z-up coordinates.
- metersPerUnit = 1.0.
- Preserved PBR base color, metallic, roughness and opacity.
- Preserved object transforms and smooth vertex normals where available.
- Deduplicated repeated mesh geometry into USD prototype references.
- Grouped all tomato plants under /World/Plants.
- Each plant is grouped into Stem and Truss_01/02/03 children for easier selection.
- Greenhouse hardware is under /World/Environment.
- Original GLB node names are stored in userProperties:originalName.

Scene statistics
----------------
Source GLB nodes: 77614
Source GLB geometry entries: 2975
Unique USD mesh prototypes: 1440
Unique USD materials: 29
Plant groups: 54
Environment mesh nodes: 1096
Expected Z-up bounds min: [-3.27, -0.11, -0.09]
Expected Z-up bounds max: [3.27, 10.859, 5.187]
USD size: 96.38 MB

Important for Isaac Sim
-----------------------
This is a visual/structural USD conversion. It does NOT automatically add
RigidBodyAPI, CollisionAPI, mass, joints, articulation, or tomato break logic.
Those should be authored separately for the plants/fruits that will actually
participate in harvesting physics.

Validation note
---------------
The conversion was checked structurally at the text layer level. The OpenUSD
Python runtime / usdchecker is not installed in this execution environment, so
final schema/runtime validation should be done when first opening the file in
Isaac Sim.
