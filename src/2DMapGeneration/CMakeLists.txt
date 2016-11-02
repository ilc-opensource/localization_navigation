file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")
file(GLOB_RECURSE common_srcs "${CMAKE_SOURCE_DIR}/src/common/*.cpp")
file(GLOB_RECURSE common_hdrs "${CMAKE_SOURCE_DIR}/src/common/*.h*")
include_directories(${CMAKE_SOURCE_DIR}/src/common)

if(NOT USE_PREBUILT_LIBRARIES)
    find_package(PCL 1.7 REQUIRED COMPONENTS common io registration visualization)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

add_executable(2DMapGeneration ${srcs} ${hdrs} ${common_srcs} ${common_hdrs})

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(2DMapGeneration ${PREBUILT_LIBS})
else()
    target_link_libraries(2DMapGeneration ${OpenCV_LIBS})
    target_link_libraries(2DMapGeneration ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
endif()

set_target_properties(2DMapGeneration PROPERTIES FOLDER "modules")
