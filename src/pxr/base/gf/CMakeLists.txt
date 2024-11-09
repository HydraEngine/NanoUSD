#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(gf LANGUAGES C CXX)

set(SRC_FILES
        bbox3d.cpp
        camera.cpp
        color.cpp
        colorSpace.cpp
        frustum.cpp
        gamma.cpp
        half.cpp
        homogeneous.cpp
        ilmbase_half.cpp
        interval.cpp
        line.cpp
        line2d.cpp
        lineSeg.cpp
        lineSeg2d.cpp
        math.cpp

        multiInterval.cpp
        ostreamHelpers.cpp
        plane.cpp
        ray.cpp
        rect2i.cpp
        rotation.cpp
        size2.cpp
        size3.cpp
        transform.cpp

        # quat.template.cpp
        quatd.cpp
        quatf.cpp
        quath.cpp
        quaternion.cpp

        # dualQuat.template.cpp
        dualQuatd.cpp
        dualQuatf.cpp
        dualQuath.cpp

        # range.template.cpp
        range1d.cpp
        range1f.cpp
        range2d.cpp
        range2f.cpp
        range3d.cpp
        range3f.cpp

        # matrix.template.cpp
        # matrix2.template.cpp
        matrix2d.cpp
        matrix2f.cpp

        # matrix3.template.cpp
        matrix3d.cpp
        matrix3f.cpp

        # matrix4.template.cpp
        matrix4d.cpp
        matrix4f.cpp
        matrixData.cpp

        # vec.template.cpp
        vec2d.cpp
        vec2f.cpp
        vec2h.cpp
        vec2i.cpp

        vec3d.cpp
        vec3f.cpp
        vec3h.cpp
        vec3i.cpp

        vec4d.cpp
        vec4f.cpp
        vec4h.cpp
        vec4i.cpp
)

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PXR_INCLUDE_PATH}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        fmt::fmt
        Eigen3::Eigen
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)