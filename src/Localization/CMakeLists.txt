file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")
file(GLOB_RECURSE common_srcs "${CMAKE_SOURCE_DIR}/src/common/*.cpp")
file(GLOB_RECURSE common_hdrs "${CMAKE_SOURCE_DIR}/src/common/*.h*")
include_directories(${CMAKE_SOURCE_DIR}/src/common)

if(NOT USE_PREBUILT_LIBRARIES)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

    find_package(G2O REQUIRED)
    find_package(PCL 1.7 REQUIRED COMPONENTS common io registration visualization)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    include_directories(${G2O_INCLUDE_DIR})
endif()

include_directories(${CMAKE_SOURCE_DIR}/3rdparty/DBoW2/include)

add_library(Localization SHARED ${srcs} ${hdrs} ${common_srcs} ${common_hdrs})
target_link_libraries(Localization DBoW2)

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(Localization ${PREBUILT_LIBS})
else()
    target_link_libraries(Localization ${OpenCV_LIBS})
    target_link_libraries(Localization ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D})
    target_link_libraries(Localization ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
endif()

set_target_properties(Localization PROPERTIES FOLDER "modules")
