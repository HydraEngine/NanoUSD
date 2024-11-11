#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usdLux LANGUAGES C CXX)

set(SRC_FILES
        blackbody.cpp
        boundableLightBase.cpp
        cylinderLight.cpp
        discoveryPlugin.cpp
        diskLight.cpp
        distantLight.cpp
        domeLight.cpp
        domeLight_1.cpp
        geometryLight.cpp
        lightAPI.cpp
        lightDefParser.cpp
        lightFilter.cpp
        lightListAPI.cpp
        listAPI.cpp
        meshLightAPI.cpp
        nonboundableLightBase.cpp
        pluginLight.cpp
        pluginLightFilter.cpp
        portalLight.cpp
        rectLight.cpp
        shadowAPI.cpp
        shapingAPI.cpp
        sphereLight.cpp
        tokens.cpp
        volumeLightAPI.cpp
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