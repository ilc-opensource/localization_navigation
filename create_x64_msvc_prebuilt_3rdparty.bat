@echo off
echo Creating Microsoft Visual Studio 2013 x64 files...

cmake -G"Visual Studio 12 2013 Win64" -B".\build\Debug" --no-warn-unused-cli -DCMAKE_BUILD_TYPE:STRING="Debug" -DCMAKE_CONFIGURATION_TYPES:STRING="Debug" -DUSE_PREBUILT_LIBRARIES:BOOL="ON" -DX64_VERSION_BUILD:BOOL="ON" -DBUILD_TESTS:BOOL="ON" -H"."
cmake -G"Visual Studio 12 2013 Win64" -B".\build\Release" --no-warn-unused-cli -DCMAKE_BUILD_TYPE:STRING="Release" -DCMAKE_CONFIGURATION_TYPES:STRING="Release" -DUSE_PREBUILT_LIBRARIES:BOOL="ON" -DX64_VERSION_BUILD:BOOL="ON" -DBUILD_TESTS:BOOL="ON" -H"."

echo Copy 3rdparty prebuilt DLLs to bin

xcopy 3rdparty\g2o\bin\x64\*.dll bin\Release\ /Y
xcopy 3rdparty\g2o\bin\x64\*_d.dll bin\Debug\ /Y
xcopy 3rdparty\opencv\bin\x64\*310_64.dll bin\Release\ /Y
xcopy 3rdparty\opencv\bin\x64\*310_64.dll bin\Debug\ /Y
xcopy 3rdparty\opencv\bin\x64\*310.dll bin\Release\ /Y
xcopy 3rdparty\opencv\bin\x64\*310d.dll bin\Debug\ /Y
xcopy 3rdparty\PCL_1.7.2\bin\x64\*_release.dll bin\Release\ /Y
xcopy 3rdparty\PCL_1.7.2\bin\x64\*_debug.dll bin\Debug\ /Y
xcopy 3rdparty\librealsense\bin\x64\realsense.dll bin\Release\ /Y
xcopy 3rdparty\librealsense\bin\x64\realsense-d.dll bin\Debug\ /Y
xcopy 3rdparty\robotcontrol\bin\x64\*.dll bin\Release\ /Y

echo Done.
pause
