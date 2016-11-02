file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")

include_directories(${CMAKE_SOURCE_DIR}/3rdparty/DBoW2/include)

if(WIN32 AND NOT USE_PREBUILT_LIBRARIES)
    include_directories($ENV{BOOST_ROOT})
endif()

add_executable(TreeGeneration ${srcs} ${hdrs})

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(TreeGeneration ${PREBUILT_LIBS})
else()
    target_link_libraries(TreeGeneration ${OpenCV_LIBS})
endif()

target_link_libraries(TreeGeneration DBoW2)
set_target_properties(TreeGeneration PROPERTIES FOLDER "modules")
