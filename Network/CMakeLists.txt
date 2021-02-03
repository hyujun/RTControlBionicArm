cmake_minimum_required(VERSION 2.8)
project(Network)

set(SOURCE_FILES devMouseObject.cpp EcatDataSocket.cpp)

add_library(${PROJECT_NAME} SHARED STATIC
        ${SOURCE_FILES}
        )

add_library(sub::NETWORK ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Poco REQUIRED COMPONENTS Foundation Util Net)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    include_directories(${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})
endif ()

find_package(NRMKhelper REQUIRED)
if(NRMKhelper_FOUND)
    message(STATUS "NRMKhelper_INC_DIRS = ${NRMKhelper_INCLUDE_DIR}")
    message(STATUS "NRMKhelper_LIB_DIRS = ${NRMKhelper_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${NRMKhelper_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${NRMKhelper_LIBRARIES})
endif(NRMKhelper_FOUND)
