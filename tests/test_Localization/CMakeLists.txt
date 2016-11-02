file(GLOB_RECURSE srcs "*.cpp")
file(GLOB_RECURSE hdrs "*.h*")

if(WIN32 AND NOT USE_PREBUILT_LIBRARIES)
    include_directories($ENV{BOOST_ROOT})
endif()

include_directories(${CMAKE_SOURCE_DIR}/src/Localization)
add_executable(test_Localization ${srcs} ${hdrs})
target_link_libraries(test_Localization Localization)

if(USE_PREBUILT_LIBRARIES)
    target_link_libraries(test_Localization ${PREBUILT_LIBS})
else()
    target_link_libraries(test_Localization ${OpenCV_LIBS})
endif()

set_target_properties(test_Localization PROPERTIES FOLDER "tests")
