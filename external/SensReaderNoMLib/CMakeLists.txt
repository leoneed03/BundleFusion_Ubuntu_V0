cmake_minimum_required(VERSION 3.5)

project(readerSens)

set(CMAKE_CXX_STANDARD 11)
set(sensorPath ${CMAKE_CURRENT_SOURCE_DIR}/src)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

set(sensorDataPath ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorData)

set(sensorDataIncludeDirs
        ${sensorDataPath}/stb_image.h
        ${sensorDataPath}/uplinksimple_windows-image-codecs.h
        ${sensorDataPath}/uplinksimple_shift2depth.h
        ${sensorDataPath}/uplinksimple_memory.h
        ${sensorDataPath}/uplinksimple_image-codecs.h
        ${sensorDataPath}/uplinksimple.h
        ${sensorDataPath}/stb_image_write.h
        ${sensorPath}/sensorData.h
        src/readerTUM.h src/readerTUM.cpp)

add_executable(readSens ${sensorPath}/main.cpp ${sensorDataIncludeDirs})
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")

target_include_directories(readSens PUBLIC ${sensorDataIncludeDirs} ${EIGEN3_INCLUDE_DIRS}
        )
target_link_libraries(readSens
        ${OpenCV_LIBS}

        ${Boost_LIBRARIES})