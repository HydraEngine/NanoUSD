#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(ar LANGUAGES C CXX)

set(SRC_FILES
        asset.cpp
        assetInfo.cpp
        debugCodes.cpp
        defaultResolver.cpp
        defaultResolverContext.cpp
        definePackageResolver.cpp
        defineResolver.cpp
        filesystemAsset.cpp
        filesystemWritableAsset.cpp
        inMemoryAsset.cpp
        notice.cpp
        packageResolver.cpp
        packageUtils.cpp
        resolvedPath.cpp
        resolver.cpp
        resolverContext.cpp
        resolverContextBinder.cpp
        resolverScopedCache.cpp
        timestamp.cpp
        writableAsset.cpp
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