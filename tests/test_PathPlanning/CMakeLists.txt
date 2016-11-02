file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")

include_directories(${CMAKE_SOURCE_DIR}/src/PathPlanning)

add_executable(test_PathPlanning ${srcs} ${hdrs})

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(test_PathPlanning ${PREBUILT_LIBS})
else()
    target_link_libraries(test_PathPlanning ${OpenCV_LIBS})
endif()

target_link_libraries(test_PathPlanning PathPlanning)
set_target_properties(test_PathPlanning PROPERTIES FOLDER "tests")
