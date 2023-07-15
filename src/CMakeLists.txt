cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

project(
    ZNDDS
    VERSION 1.0
    DESCRIPTION "ZNDDS"
    LANGUAGES CXX
)

find_package(
    QT
    NAMES
    Qt6
    Qt5
)

message(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

find_package(
        Qt${QT_VERSION_MAJOR}
        COMPONENTS
        Core
        Gui
        Widgets
        Network
        REQUIRED
        )

find_package(CURL REQUIRED)

find_package(Threads)


set (target_name ${PROJECT_NAME})
add_subdirectory(../socket/src ${CMAKE_BINARY_DIR}/socket)


aux_source_directory(
    .
    DIR_SOURCE_SRCS
)
aux_source_directory(
    ./UI
    DIR_UI_SRCS
)

aux_source_directory(
    ./DataProcess
    DIR_DateProces_SRCS
)
add_executable(
    ${PROJECT_NAME}
    WIN32
    ${DIR_SOURCE_SRCS}
    ${DIR_UI_SRCS}
    ${DIR_DateProces_SRCS}
)


target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    socket
	${CMAKE_THREAD_LIBS_INIT}
        Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    CURL::libcurl
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    ${CMAKE_SOURCE_DIR}/../socket/include
)

