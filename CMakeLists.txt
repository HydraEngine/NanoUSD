cmake_minimum_required(VERSION 3.15...3.26)

set(CMAKE_CXX_STANDARD 17)

project(nanoUSD LANGUAGES C CXX)

include(cmake/add_stub.cmake)

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)

execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

find_package(TBB REQUIRED)

find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(MaterialX CONFIG REQUIRED)

add_subdirectory(src)

# ==================================================================================================
# ADD LIBS
# ==================================================================================================
set(BASE_GF_BINDINGS
        src/py_binding/base/gf/py_bbox3d.cpp
)


set(BINDING_FILES
        ${BASE_GF_BINDINGS}
        src/py_binding/py_usd_ext.cpp
)

# We are now ready to compile the actual extension module
nanobind_add_module(
        # Name of the extension
        py_usd_ext

        # Target the stable ABI for Python 3.12+, which reduces
        # the number of binary wheels that must be built. This
        # does nothing on older Python versions
        STABLE_ABI

        # Build libnanobind statically and merge it into the
        # extension (which itself remains a shared library)
        #
        # If your project builds multiple extensions, you can
        # replace this flag by NB_SHARED to conserve space by
        # reusing a shared libnanobind across libraries
        NB_STATIC

        # Source code goes here
        ${BINDING_FILES}
)

# should modified nanobind to support recursive and output dir
# list(APPEND NB_STUBGEN_ARGS -O "${ARG_OUTPUT}")
# list(APPEND NB_STUBGEN_ARGS -r)
add_stub(
        py_usd_stub
        MODULE py_usd_ext
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/nanoUSD
        PYTHON_PATH $<TARGET_FILE_DIR:py_usd_ext>
        DEPENDS py_usd_ext
)

# ==================================================================================================
# LINK
# ==================================================================================================
target_include_directories(py_usd_ext PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/libs
)

target_link_libraries(py_usd_ext PRIVATE
        fmt::fmt
        arch
        tf
        gf
        js
        plug
        trace
        vt
        work
)

# ==================================================================================================
# TEST
# ==================================================================================================
add_subdirectory(tests/cpp)

# ==================================================================================================
# INSTALL
# ==================================================================================================

file(GLOB PY_FILES "./src/nanoUSD/*.py" "./src/nanoUSD/*.pyi")

# Install directive for scikit-build-core
install(TARGETS py_usd_ext LIBRARY DESTINATION nanoUSD)
install(FILES ${PY_FILES} DESTINATION nanoUSD)