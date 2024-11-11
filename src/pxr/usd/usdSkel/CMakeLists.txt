#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usdSkel LANGUAGES C CXX)

set(SRC_FILES
        animation.cpp
        animMapper.cpp
        animQuery.cpp
        animQueryImpl.cpp
        bakeSkinning.cpp
        bindingAPI.cpp
        blendShape.cpp
        blendShapeQuery.cpp
        cache.cpp
        cacheImpl.cpp
        debugCodes.cpp
        inbetweenShape.cpp
        root.cpp
        skelDefinition.cpp
        skeleton.cpp
        skeletonQuery.cpp
        skinningQuery.cpp
        tokens.cpp
        topology.cpp
        utils.cpp
        validators.cpp
        validatorTokens.cpp
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