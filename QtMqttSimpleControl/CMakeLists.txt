cmake_minimum_required(VERSION 3.21)

project(QtMqttSimpleControl VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Mqtt Widgets)

qt_add_executable(QtMqttSimpleControl WIN32
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mqttclient.cpp
    src/mqttclient.h
    src/plotrenderer.cpp
    src/plotrenderer.h
)

target_link_libraries(QtMqttSimpleControl PRIVATE
    Qt6::Mqtt
    Qt6::Widgets
)

target_compile_definitions(QtMqttSimpleControl PRIVATE
    QT_NO_CAST_FROM_ASCII
    QT_NO_CAST_TO_ASCII
)

if(WIN32)
    target_compile_definitions(QtMqttSimpleControl PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
endif()

include(CTest)
if(BUILD_TESTING)
    qt_add_executable(PlotRendererSmokeTest
        tests/plotrenderer_smoke.cpp
        src/plotrenderer.cpp
        src/plotrenderer.h
    )
    target_link_libraries(PlotRendererSmokeTest PRIVATE Qt6::Widgets)
    add_test(NAME PlotRendererSmokeTest COMMAND PlotRendererSmokeTest)
    set_tests_properties(PlotRendererSmokeTest PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    )
endif()
