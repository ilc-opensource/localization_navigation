file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")

add_library(PathPlanning SHARED ${srcs} ${hdrs})

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(PathPlanning ${PREBUILT_LIBS})
else()
    target_link_libraries(PathPlanning ${OpenCV_LIBS})
endif()

set_target_properties(PathPlanning PROPERTIES FOLDER "modules")
