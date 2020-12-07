cmake_minimum_required(VERSION 3.14)
project(sdf-2-sdf)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
    message("OpenCV Found")
endif()
find_package(VTK REQUIRED)
if (VTK_FOUND)
    message("VTK Found")
endif()

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    message("Eigen3 Found")
endif()

find_package(Sophus REQUIRED)
if (Sophus_FOUND)
    message("Sophus Found")
endif()

message("${EIGEN3_INCLUDE_DIR}")
include_directories(include
                    ${EIGEN3_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    )
message("${VTK_USE_FILE}")
include(${VTK_USE_FILE})

aux_source_directory(src SOURCE_FILES)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES})
message("${VTK_LIBRARIES}")