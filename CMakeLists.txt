cmake_minimum_required(VERSION 3.24)
project(PCLDemo VERSION 1.0.0 LANGUAGES C CXX)

# PCL 的 Windows 安装包由 MSVC 编译，不能与 MinGW 版 Qt 混用。
if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Use MSVC x64 and the MSVC edition of Qt.")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set(PCL_DIR "D:/PCL/cmake" CACHE PATH "PCLConfig.cmake directory")
list(PREPEND CMAKE_PREFIX_PATH "D:/software/Qt/6.12.0/msvc2022_64")
find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent)
set(OpenCV_DIR "D:/opencv/5.0.0/opencv/build" CACHE PATH "OpenCVConfig.cmake directory")
find_package(OpenCV 5 REQUIRED CONFIG COMPONENTS core imgproc imgcodecs features photo geometry)
find_package(PCL 1.15 REQUIRED COMPONENTS common io filters features search kdtree octree sample_consensus segmentation surface registration visualization)

add_executable(PCLDemo WIN32
    resources/PCLDemo.rc resources/PCLDemo.qrc
    src/main.cpp src/Algorithms.cpp src/Algorithms.h
    src/PointCloudView.cpp src/PointCloudView.h
    src/MainWindow.cpp src/MainWindow.h src/SelfTest.cpp src/SelfTest.h
    src/RgbdAlgorithms.cpp src/RgbdAlgorithms.h src/RgbdPanel.cpp src/RgbdPanel.h src/CvSelfTest.cpp)
target_include_directories(PCLDemo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/resources")
target_include_directories(PCLDemo SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(PCLDemo SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_directories(PCLDemo PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(PCLDemo PRIVATE Qt6::Widgets Qt6::Concurrent ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_compile_options(PCLDemo PRIVATE /utf-8 /bigobj /arch:AVX2 /W3 ${PCL_COMPILE_OPTIONS})
target_compile_definitions(PCLDemo PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN ${PCL_DEFINITIONS})
set_target_properties(PCLDemo PROPERTIES VS_DEBUGGER_ENVIRONMENT
    "PATH=D:/opencv/5.0.0/opencv/build/x64/vc16/bin;D:/PCL/bin;D:/PCL/3rdParty/VTK/bin;D:/PCL/3rdParty/FLANN/bin;D:/PCL/3rdParty/Qhull/bin;C:/Program Files/OpenNI2/Redist;D:/software/Qt/6.12.0/msvc2022_64/bin;%PATH%")
if(COMMAND vtk_module_autoinit)
    set(demo_vtk_modules ${PCL_LIBRARIES})
    list(FILTER demo_vtk_modules INCLUDE REGEX "^VTK::")
    vtk_module_autoinit(TARGETS PCLDemo MODULES ${demo_vtk_modules})
endif()
include(CTest)
if(BUILD_TESTING)
    add_test(NAME pcl_algorithms_and_ui COMMAND PCLDemo --self-test --report "${CMAKE_BINARY_DIR}/self-test-$<CONFIG>.txt")
    set_tests_properties(pcl_algorithms_and_ui PROPERTIES TIMEOUT 180)
    set_tests_properties(pcl_algorithms_and_ui PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM_PLUGIN_PATH=D:/software/Qt/6.12.0/msvc2022_64/plugins/platforms"
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:D:/opencv/5.0.0/opencv/build/x64/vc16/bin;PATH=path_list_prepend:D:/PCL/bin;PATH=path_list_prepend:D:/PCL/3rdParty/VTK/bin;PATH=path_list_prepend:D:/PCL/3rdParty/FLANN/bin;PATH=path_list_prepend:D:/PCL/3rdParty/Qhull/bin;PATH=path_list_prepend:C:/Program Files/OpenNI2/Redist;PATH=path_list_prepend:D:/software/Qt/6.12.0/msvc2022_64/bin")
endif()
