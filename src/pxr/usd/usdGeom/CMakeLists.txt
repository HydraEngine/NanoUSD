#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usdGeom LANGUAGES C CXX)

set(SRC_FILES
        basisCurves.cpp
        bboxCache.cpp
        boundable.cpp
        boundableComputeExtent.cpp
        camera.cpp
        capsule.cpp
        capsule_1.cpp
        cone.cpp
        constraintTarget.cpp
        cube.cpp
        curves.cpp
        cylinder.cpp
        cylinder_1.cpp
        debugCodes.cpp
        examples_usdGeom.cpp
        gprim.cpp
        hermiteCurves.cpp
        imageable.cpp
        mesh.cpp
        metrics.cpp
        modelAPI.cpp
        motionAPI.cpp
        nurbsCurves.cpp
        nurbsPatch.cpp
        plane.cpp
        pointBased.cpp
        pointInstancer.cpp
        points.cpp
        primvar.cpp
        primvarsAPI.cpp
        samplingUtils.cpp
        scope.cpp
        sphere.cpp
        subset.cpp
        tetMesh.cpp
        tokens.cpp
        validators.cpp
        validatorTokens.cpp
        visibilityAPI.cpp
        xform.cpp
        xformable.cpp
        xformCache.cpp
        xformCommonAPI.cpp
        xformOp.cpp
)

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PXR_INCLUDE_PATH}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        fmt::fmt
        Eigen3::Eigen
        TBB::tbb
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)