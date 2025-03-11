cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(INCLUDE_PATH "/usr/include")
set(LIB_PATH "/usr/local/lib")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------
message(${PROJECT_SOURCE_DIR})
message(${PROJECT_SOURCE_DIR})
message(${PROJECT_SOURCE_DIR})
message(${PROJECT_SOURCE_DIR})
file(GLOB_RECURSE native_srcs ${PROJECT_SOURCE_DIR}/code/*.cpp)
message(${native_srcs})
add_library(CODE SHARED ${native_srcs})

file(GLOB_RECURSE native_srcs ${PROJECT_SOURCE_DIR}/recognition/*.cpp)
message(${native_srcs})
add_library(RECOGNITION SHARED ${native_srcs})

file(GLOB_RECURSE native_srcs ${PROJECT_SOURCE_DIR}/detection/*.cpp)
message(${native_srcs})
add_library(DETECTION SHARED ${native_srcs})


set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR 	"${PROJECT_SOURCE_DIR}/code"
						"${PROJECT_SOURCE_DIR}/recognition"
                        "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/driver" 
                        "${PROJECT_SOURCE_DIR}/lib" 
                        "${PROJECT_SOURCE_DIR}/src"          )


link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

# find_package(V4L2 REQUIRED)
# include_directories( ${V4L2_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ tool ]
#---------------------------------------------------------------------

#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(BAIDU_PROJECT_NAME "baidu")
set(BAIDU_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp)
add_executable(${BAIDU_PROJECT_NAME} ${BAIDU_PROJECT_SOURCES})
target_link_libraries(${BAIDU_PROJECT_NAME}	CODE)
target_link_libraries(${BAIDU_PROJECT_NAME}	RECOGNITION)
target_link_libraries(${BAIDU_PROJECT_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${BAIDU_PROJECT_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${BAIDU_PROJECT_NAME}  ${OpenCV_LIBS})
# target_link_libraries(${BAIDU_PROJECT_NAME}  ${V4L2_LIBRARIES})
target_link_libraries(${BAIDU_PROJECT_NAME}  pthread )
target_link_libraries(${BAIDU_PROJECT_NAME}  ${SERIAL_LIBRARIES})


set(CAMERA_PROJECT_NAME "camera")
set(CAMERA_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/camera.cpp)

add_executable(${CAMERA_PROJECT_NAME} ${CAMERA_PROJECT_SOURCES})
target_link_libraries(${CAMERA_PROJECT_NAME}  ${OpenCV_LIBS})
