#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usdShade LANGUAGES C CXX)

set(SRC_FILES
        connectableAPI.cpp
        connectableAPIBehavior.cpp
        coordSysAPI.cpp
        input.cpp
        material.cpp
        materialBindingAPI.cpp
        nodeDefAPI.cpp
        nodeGraph.cpp
        output.cpp
        shader.cpp
        shaderDefParser.cpp
        shaderDefUtils.cpp
        tokens.cpp
        udimUtils.cpp
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