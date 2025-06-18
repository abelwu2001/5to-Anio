cmake_minimum_required(VERSION 3.16)
project(TP2_STM32 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

qt_standard_project_setup()

qt_add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt6::Widgets Qt6::SerialPort
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;
     -checks=-clazy-connect-by-name")


