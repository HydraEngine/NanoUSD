#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(pcp LANGUAGES C CXX)

set(SRC_FILES
        arc.cpp
        cache.cpp
        changes.cpp
        composeSite.cpp
        debugCodes.cpp
        dependencies.cpp
        dependency.cpp
        dependentNamespaceEditUtils.cpp
        diagnostic.cpp
        dynamicFileFormatContext.cpp
        dynamicFileFormatDependencyData.cpp
        dynamicFileFormatInterface.cpp
        errors.cpp
        expressionVariables.cpp
        expressionVariablesDependencyData.cpp
        expressionVariablesSource.cpp
        instanceKey.cpp
        instancing.cpp
        iterator.cpp
        layerRelocatesEditBuilder.cpp
        layerStack.cpp
        layerStackIdentifier.cpp
        layerStackRegistry.cpp
        mapExpression.cpp
        mapFunction.cpp
        namespaceEdits.cpp
        node.cpp
        node_Iterator.cpp
        pathTranslation.cpp
        primIndex.cpp
        primIndex_Graph.cpp
        primIndex_StackFrame.cpp
        propertyIndex.cpp
        site.cpp
        statistics.cpp
        strengthOrdering.cpp
        targetIndex.cpp
        types.cpp
        utils.cpp
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