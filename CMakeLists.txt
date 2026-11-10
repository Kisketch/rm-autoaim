cmake_minimum_required(VERSION 3.16)
project(rm_autoaim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d dnn)

add_library(autoaim_core STATIC
    src/config.cpp
    src/detector.cpp
    src/pose.cpp
    src/pose_buffer.cpp
    src/delay_calibration.cpp
    src/camera_source.cpp
    src/keypoint_detector.cpp
    src/tracker.cpp
    src/ballistics.cpp
    src/aimer.cpp
    src/evaluation.cpp
    src/draw.cpp)

# Optional Hikvision MVS camera source (G-02 step 1). Off by default so the
# offline pipeline builds anywhere; enable on the robot with the SDK path:
#   cmake -DAUTOAIM_WITH_MVS=ON -DMVS_ROOT=/opt/MVS ..
option(AUTOAIM_WITH_MVS "Build the Hikvision MVS camera source" OFF)
if (AUTOAIM_WITH_MVS)
    set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikvision MVS SDK root")
    find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS ${MVS_ROOT}/include)
    find_library(MVS_LIBRARY MvCameraControl HINTS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib)
    if (NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
        message(FATAL_ERROR "AUTOAIM_WITH_MVS=ON but the MVS SDK was not found under ${MVS_ROOT}")
    endif()
    target_include_directories(autoaim_core PUBLIC ${MVS_INCLUDE_DIR})
    target_link_libraries(autoaim_core PUBLIC ${MVS_LIBRARY})
    target_compile_definitions(autoaim_core PUBLIC AUTOAIM_WITH_MVS)
endif()
target_include_directories(autoaim_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(autoaim_core PUBLIC ${OpenCV_LIBS})
target_compile_definitions(autoaim_core PUBLIC AUTOAIM_ROOT="${CMAKE_CURRENT_SOURCE_DIR}" _USE_MATH_DEFINES
                           _CRT_SECURE_NO_WARNINGS)
if (MSVC)
    target_compile_options(autoaim_core PUBLIC /utf-8 /W3)
else()
    target_compile_options(autoaim_core PUBLIC -Wall -Wextra)
endif()

add_executable(autoaim_video apps/autoaim_video.cpp)
target_link_libraries(autoaim_video PRIVATE autoaim_core)

add_executable(autoaim_sim apps/autoaim_sim.cpp)
target_link_libraries(autoaim_sim PRIVATE autoaim_core)

add_executable(autoaim_lights apps/autoaim_lights.cpp)
target_link_libraries(autoaim_lights PRIVATE autoaim_core)

add_executable(autoaim_tests tests/test_main.cpp)
target_link_libraries(autoaim_tests PRIVATE autoaim_core)

enable_testing()
add_test(NAME autoaim_tests COMMAND autoaim_tests)
