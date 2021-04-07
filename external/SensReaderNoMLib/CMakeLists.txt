cmake_minimum_required(VERSION 3.5)

project(readerSens)

set(CMAKE_CXX_STANDARD 11)
set(sensorPath ${CMAKE_CURRENT_SOURCE_DIR}/src)

find_package(OpenCV REQUIRED)

add_executable(readSens ${sensorPath}/main.cpp)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(sensorDataPath ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorData)

set(sensorDataIncludeDirs
        ${sensorDataPath}/stb_image.h
        ${sensorDataPath}/uplinksimple_windows-image-codecs.h
        ${sensorDataPath}/uplinksimple_shift2depth.h
        ${sensorDataPath}/uplinksimple_memory.h
        ${sensorDataPath}/uplinksimple_image-codecs.h
        ${sensorDataPath}/uplinksimple.h
        ${sensorDataPath}/stb_image_write.h
        ${sensorPath}/sensorData.h)

target_include_directories(readSens PUBLIC ${sensorDataIncludeDirs}
        )
target_link_libraries(readSens
        ${OpenCV_LIBS})