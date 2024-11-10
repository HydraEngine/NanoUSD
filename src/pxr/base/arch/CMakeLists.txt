#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(arch LANGUAGES C CXX)

set(SRC_FILES
        align.cpp
        assumptions.cpp
        attributes.cpp
        daemon.cpp
        debugger.cpp
        demangle.cpp
        env.cpp
        errno.cpp
        error.cpp
        fileSystem.cpp
        function.cpp
        hash.cpp
        initConfig.cpp
        library.cpp
        mallocHook.cpp
        regex.cpp
        stackTrace.cpp
        symbols.cpp
        systemInfo.cpp
        threads.cpp
        timing.cpp
        virtualMemory.cpp
        vsnprintf.cpp
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