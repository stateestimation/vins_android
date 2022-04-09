# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

set(cmakePath ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(cmakePath)

set(srcPath ${cmakePath}/src)
include_directories(srcPath)
add_library(src_lib INTERFACE)
target_include_directories(src_lib INTERFACE ${srcPath})

set(srcDelaunayPath ${cmakePath}/src/delaunay)
add_library(srcDelaunay_lib INTERFACE)
target_include_directories(srcDelaunay_lib INTERFACE ${srcDelaunayPath})

set(srcLoopPath ${cmakePath}/src/loop)
add_library(srcLoopPath_lib INTERFACE)
target_include_directories(srcLoopPath_lib INTERFACE ${srcLoopPath})

set(srcDBowPath ${cmakePath}/src/DBoW)
add_library(srcDBowPath_lib INTERFACE)
target_include_directories(srcDBowPath_lib INTERFACE ${srcDBowPath})

set(srcDUtilsPath ${cmakePath}/src/DUtils)
add_library(srcDUtilsPath_lib INTERFACE)
target_include_directories(srcDUtilsPath_lib INTERFACE ${srcDUtilsPath})

set(srcDvisionPath ${cmakePath}/src/DVision)
add_library(srcDvisionPath_lib INTERFACE)
target_include_directories(srcDvisionPath_lib INTERFACE ${srcDvisionPath})

set(thirdPath ${cmakePath}/thirdparty)
include_directories(thirdPath)

set(boostPath ${thirdPath}/boost_1_74_0)
add_library(boost_lib INTERFACE)
target_include_directories(boost_lib INTERFACE ${boostPath})

set(eigenPath ${thirdPath}/eigen-3.2.10)
include_directories(eigenPath)
add_library(eigen_lib INTERFACE)
target_include_directories(
        eigen_lib
        INTERFACE
        ${eigenPath}
        ${thirdPath}
)

set(opencvPath ${thirdPath}/opencv-4.1.0-android-sdk/OpenCV-android-sdk/sdk/native)
include_directories(${opencvPath}/jni/include)
add_library(opencv_lib SHARED IMPORTED)
set_target_properties(opencv_lib PROPERTIES IMPORTED_LOCATION
        ${opencvPath}/libs/${ANDROID_ABI}/libopencv_java4.so)

set(ceresPath ${thirdPath}/ceres-solver-1.13.0)
include_directories(${ceresPath}/include)
include_directories(${ceresPath}/internal)
include_directories(${ceresPath}/internal/ceres/miniglog)
include_directories(${ceresPath}/config)
add_library(ceres_lib STATIC IMPORTED)
set_target_properties(ceres_lib PROPERTIES IMPORTED_LOCATION ${ceresPath}/obj/local/${ANDROID_ABI}/libceres.a)


add_library(native-lib SHARED
native-lib.cpp
ViewController.cpp
src/delaunay/delaunay.cpp
src/delaunay/triangle.cpp
        src/DBoW/BowVector.cpp
        src/DBoW/FBrief.cpp
        src/DBoW/FeatureVector.cpp
        src/DBoW/QueryResults.cpp
        src/DBoW/ScoringObject.cpp
        src/DUtils/Random.cpp
        src/DUtils/Timestamp.cpp
        src/DVision/BRIEF.cpp
        src/loop/keyfame_database.cpp
        src/loop/keyframe.cpp
        src/loop/loop_closure.cpp
        src/loop/VocabularyBinary.cpp
        src/draw_result.cpp
        src/feature_manager.cpp
        src/feature_tracker.cpp
        src/global_param.cpp
        src/inital_sfm.cpp
        src/initial_aligment.cpp
        src/marginalization_factor.cpp
        src/motion_estimator.cpp
        src/perspective_factor.cpp
        src/pose_local_parameterization.cpp
        src/projection_facor.cpp
        src/utility.cpp
        src/VINS.cpp
        src/vins_pnp.cpp
)

find_library( # Sets the name of the path variable.
              log-lib
              log )

target_link_libraries( # Specifies the target library.
                       native-lib
                       ${log-lib}
                        android
        opencv_lib
        eigen_lib
        src_lib
        srcDelaunay_lib
        srcLoopPath_lib
        srcDBowPath_lib
        srcDUtilsPath_lib
        srcDvisionPath_lib
        boost_lib
        ceres_lib)
