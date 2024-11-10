#  Copyright (c) 2023 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

# create sample app project
project(trace LANGUAGES C CXX)

set(SRC_FILES
        aggregateNode.cpp
        aggregateTree.cpp
        aggregateTreeBuilder.cpp
        category.cpp
        collection.cpp
        collectionNotice.cpp
        collector.cpp
        counterAccumulator.cpp
        dataBuffer.cpp
        dynamicKey.cpp
        event.cpp
        eventContainer.cpp
        eventData.cpp
        eventList.cpp
        eventNode.cpp
        eventTree.cpp
        eventTreeBuilder.cpp
        jsonSerialization.cpp
        key.cpp
        reporter.cpp
        reporterBase.cpp
        reporterDataSourceBase.cpp
        reporterDataSourceCollection.cpp
        reporterDataSourceCollector.cpp
        serialization.cpp
        staticKeyData.cpp
        threads.cpp
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