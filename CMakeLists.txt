cmake_minimum_required(VERSION 2.8)
project(DemoSLN)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")

if(WIN32)
    add_definitions(/wd4996)
    add_definitions(-DWIN32)
    option(USE_PREBUILT_LIBRARIES "Use prebuilt vs2013 3rdparty libraries on Windows; unset this if you have these libraries installed or build other versions." ON)
    option(X64_VERSION_BUILD "Generate vs2013 x64 files; unset this if you prefer x86 version." ON)

    # this macro is for prebuilt VTK configuration header, see 3rdparty/VTK_5.10.1/include/vtkConfigure.h
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        add_definitions(-DVTK64)
    endif()
else()
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
    endif(NOT CMAKE_BUILD_TYPE)
    add_compile_options(-Wall)

    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
	add_compile_options(-std=c++11)
    else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use higher version C++ compiler.")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(USE_PREBUILT_LIBRARIES)
    include(cmake/prebuilt.cmake)
else()
    find_package(OpenCV 3.1 REQUIRED)
    find_package(Boost COMPONENTS system filesystem iostreams thread date_time chrono REQUIRED)
endif()

add_subdirectory(3rdparty/DBoW2)
add_subdirectory(src)

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
