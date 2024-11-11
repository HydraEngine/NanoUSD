add_subdirectory(ar) # Asset Resolution
add_subdirectory(kind) # Runtime-extensible Categorization
add_subdirectory(ndr) # Node Definition Registry
add_subdirectory(pcp) # PrimCache Population
add_subdirectory(sdf) # Scene Description Foundations
add_subdirectory(sdr) # Shader Definition Registry

add_subdirectory(usd)
add_subdirectory(usdUtils) # USD Utility Libraries

# Rendering Schema
add_subdirectory(usdGeom) # USD Geometry Schema ✅
add_subdirectory(usdLux) # USD Lighting Schema ✅
add_subdirectory(usdRender) # USD Render Schema  ✅
add_subdirectory(usdShade) # USD Shading Schema  ✅
add_subdirectory(usdVol) # USD Volume Schema  ✅

# Procedural Schema
add_subdirectory(usdHydra) # USD Hydra Schemas ✅
add_subdirectory(usdProc) # USD Schemas for Procedurals ✅

# Storage Schema
add_subdirectory(usdSkel) # USD Skeleton Schema  ✅
add_subdirectory(usdPhysics) # USD Physics Schema ✅
add_subdirectory(usdMedia) # USD Media Schema  ✅
add_subdirectory(usdUI) # USD UI Schemas  ✅
add_subdirectory(usdSemantics) # Semantic Labeling for Model Hierarchy  ✅

# External Schema
add_subdirectory(plugin)
add_subdirectory(usdMtlx) # MaterialX File Format and Shader Plugins
add_subdirectory(usdRi) # USD Renderman Schemas

# prim - entity
# schema - component
# stage - scene