#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usdPhysics LANGUAGES C CXX)

set(SRC_FILES
        articulationRootAPI.cpp
        collisionAPI.cpp
        collisionGroup.cpp
        distanceJoint.cpp
        driveAPI.cpp
        filteredPairsAPI.cpp
        fixedJoint.cpp
        joint.cpp
        limitAPI.cpp
        massAPI.cpp
        materialAPI.cpp
        meshCollisionAPI.cpp
        metrics.cpp
        prismaticJoint.cpp
        revoluteJoint.cpp
        rigidBodyAPI.cpp
        scene.cpp
        sphericalJoint.cpp
        tokens.cpp
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