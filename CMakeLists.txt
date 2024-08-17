cmake_minimum_required(VERSION 3.10)
project(revopoint)
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
)

## thirdparty
set(THIRDPARTY_DIR ${PROJECT_SOURCE_DIR}/thirdparty)
message(${THIRDPARTY_DIR})

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "--Current platform: linux ")
    set(PLATFORM "linux")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "--Current platform: windows")
    set(PLATFORM "windows")
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    message(STATUS "--Current platform: mac")
    set(PLATFORM "mac")
else()
    message(STATUS "--Other platform: ${CMAKE_SYSTEM_NAME}")
endif()

set(PLATFORM_BITS x64)

## 3DCamera
set(CAMERA_SDK_INC ${THIRDPARTY_DIR}/3DCamera/include)
set(CAMERA_SDK_LIB ${THIRDPARTY_DIR}/3DCamera/${PLATFORM}/${PLATFORM_BITS})

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CAMERA_SDK_INC}
)

# link_directories(${CAMERA_SDK_INC})

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/cscamera.cpp)

## Specify libraries to link a library or executable target against
target_include_directories(${PROJECT_NAME}
  PRIVATE ${CAMERA_SDK_INC})

add_library(3DCAMERA_LIBRARY SHARED IMPORTED)

if(PLATFORM MATCHES "linux")
    set_target_properties(3DCAMERA_LIBRARY PROPERTIES
        IMPORTED_IMPLIB "${CAMERA_SDK_LIB}/lib3DCamera.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CAMERA_SDK_INC}"
    )
elseif(PLATFORM MATCHES "windows")
    set_target_properties(3DCAMERA_LIBRARY PROPERTIES
        IMPORTED_IMPLIB "${CAMERA_SDK_LIB}/3DCamera.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${CAMERA_SDK_INC}"
    )
endif()

target_link_libraries(${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE 3DCAMERA_LIBRARY
)

if(PLATFORM MATCHES "linux")
    add_custom_command(TARGET ${PROJECT_NAME} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CAMERA_SDK_LIB}/lib3DCamera.so" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
elseif(PLATFORM MATCHES "windows")
    add_custom_command(TARGET ${PROJECT_NAME} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CAMERA_SDK_LIB}/3DCamera.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
elseif(PLATFORM MATCHES "mac")
    add_custom_command(TARGET ${PROJECT_NAME} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CAMERA_SDK_LIB}/lib3DCamera.dylib" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
else()
    add_custom_command(TARGET ${PROJECT_NAME} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CAMERA_SDK_LIB}/*" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()
