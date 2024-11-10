#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(tf LANGUAGES C CXX)

set(SRC_FILES
        anyUniquePtr.cpp
        anyWeakPtr.cpp
        atomicOfstreamWrapper.cpp
        atomicRenameUtil.cpp
        bigRWMutex.cpp
        bits.cpp
        bitUtils.cpp
        compressedBits.cpp
        debug.cpp
        debugCodes.cpp
        debugNotice.cpp
        denseHashMap.cpp
        denseHashSet.cpp
        diagnostic.cpp
        diagnosticBase.cpp
        diagnosticHelper.cpp
        diagnosticMgr.cpp
        dl.cpp
        enum.cpp
        envSetting.cpp
        error.cpp
        errorMark.cpp
        errorTransport.cpp
        exception.cpp
        expiryNotifier.cpp
        fastCompression.cpp
        fileUtils.cpp
        getenv.cpp
        hash.cpp
        initConfig.cpp
        iterator.cpp
        mallocTag.cpp
        notice.cpp
        noticeRegistry.cpp
        nullPtr.cpp
        ostreamMethods.cpp
        pathUtils.cpp
        patternMatcher.cpp
        pointerAndBits.cpp
        refBase.cpp
        refPtr.cpp
        refPtrTracker.cpp
        registryManager.cpp
        regTest.cpp
        safeOutputFile.cpp
        scoped.cpp
        scopeDescription.cpp
        scriptModuleLoader.cpp
        setenv.cpp
        singleton.cpp
        smallVector.cpp
        spinMutex.cpp
        spinRWMutex.cpp
        stacked.cpp
        stackTrace.cpp
        status.cpp
        stl.cpp
        stopwatch.cpp
        stringUtils.cpp
        templateString.cpp
        tf.cpp
        token.cpp
        type.cpp
        typeFunctions.cpp
        typeNotice.cpp
        unicodeCharacterClasses.cpp
        unicodeUtils.cpp
        warning.cpp
        weakBase.cpp
        weakPtr.cpp
        weakPtrFacade.cpp
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