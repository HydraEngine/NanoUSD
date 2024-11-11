#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(sdf LANGUAGES C CXX)

set(SRC_FILES
        abstractData.cpp
        allowed.cpp
        assetPath.cpp
        assetPathResolver.cpp
        attributeSpec.cpp
        changeBlock.cpp
        changeList.cpp
        changeManager.cpp
        children.cpp
        childrenPolicies.cpp
        childrenProxy.cpp
        childrenUtils.cpp
        childrenView.cpp
        cleanupEnabler.cpp
        cleanupTracker.cpp
        connectionListEditor.cpp
        copyUtils.cpp
        data.cpp
        debugCodes.cpp
        declareHandles.cpp
        fileFormat.cpp
        fileFormatRegistry.cpp
        fileIO.cpp
        fileIO_Common.cpp
        identity.cpp
        layer.cpp
        layerOffset.cpp
        layerRegistry.cpp
        layerStateDelegate.cpp
        layerTree.cpp
        layerUtils.cpp
        listEditor.cpp
        listEditorProxy.cpp
        listOp.cpp
        listOpListEditor.cpp
        listProxy.cpp
        mapEditor.cpp
        namespaceEdit.cpp
        notice.cpp
        opaqueValue.cpp
        parserHelpers.cpp
        parserValueContext.cpp
        path.cpp
        pathExpression.cpp
        pathExpressionEval.cpp
        pathNode.cpp
        pathParser.cpp
        pathPattern.cpp
        pathTable.cpp
        payload.cpp
        pool.cpp
        predicateExpression.cpp
        predicateLibrary.cpp
        primSpec.cpp
        propertySpec.cpp
        proxyPolicies.cpp
        proxyTypes.cpp
        pseudoRootSpec.cpp
        reference.cpp
        relationshipSpec.cpp
        schema.cpp
        site.cpp
        siteUtils.cpp
        spec.cpp
        specType.cpp
        subLayerListEditor.cpp
        textFileFormat.cpp
        textFileFormatParser.cpp
        textParserContext.cpp
        textParserHelpers.cpp
        timeCode.cpp
        tokens.cpp
        types.cpp
        valueTypeName.cpp
        valueTypeRegistry.cpp
        variableExpression.cpp
        variableExpressionImpl.cpp
        variableExpressionParser.cpp
        variantSetSpec.cpp
        variantSpec.cpp
        vectorListEditor.cpp
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