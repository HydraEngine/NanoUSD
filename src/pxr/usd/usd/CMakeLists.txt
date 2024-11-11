#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(usd LANGUAGES C CXX)

set(SRC_FILES
        apiSchemaBase.cpp
        attribute.cpp
        attributeQuery.cpp
        clip.cpp
        clipCache.cpp
        clipsAPI.cpp
        clipSet.cpp
        clipSetDefinition.cpp
        collectionAPI.cpp
        collectionMembershipQuery.cpp
        collectionPredicateLibrary.cpp
        common.cpp
        coreValidators.cpp
        crateData.cpp
        crateFile.cpp
        crateInfo.cpp
        debugCodes.cpp
        editContext.cpp
        editTarget.cpp
        errors.cpp
#        examples_usd.cpp
        flattenUtils.cpp
        inherits.cpp
        instanceCache.cpp
        instanceKey.cpp
        integerCoding.cpp
        interpolation.cpp
        interpolators.cpp
        modelAPI.cpp
        namespaceEditor.cpp
        notice.cpp
        object.cpp
        payloads.cpp
        prim.cpp
        primCompositionQuery.cpp
        primData.cpp
        primDataHandle.cpp
        primDefinition.cpp
        primFlags.cpp
        primRange.cpp
        primTypeInfo.cpp
        primTypeInfoCache.cpp
        property.cpp
        references.cpp
        relationship.cpp
        resolveInfo.cpp
        resolver.cpp
        resolveTarget.cpp
        schemaBase.cpp
        schemaRegistry.cpp
        shared.cpp
        specializes.cpp
        stage.cpp
        stageCache.cpp
        stageCacheContext.cpp
        stageLoadRules.cpp
        stagePopulationMask.cpp
        timeCode.cpp
        tokens.cpp
        typed.cpp
        usdFileFormat.cpp
        usdcFileFormat.cpp
        usdaFileFormat.cpp
        usdzFileFormat.cpp
        usdzResolver.cpp
        validationContext.cpp
        validationError.cpp
        validationRegistry.cpp
        validator.cpp
        validatorTokens.cpp
        valueUtils.cpp
        variantSets.cpp
        zipFile.cpp
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