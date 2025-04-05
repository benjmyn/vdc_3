cmake_minimum_required(VERSION 3.30)
project(vdc)

set(CMAKE_CXX_STANDARD 20)

set(ARMADILLO_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/armadillo-code/include)
set(ARMADILLO_LIBRARY ${CMAKE_SOURCE_DIR}/lib/armadillo-code/libarmadillo.so)

set(GLFW_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/glfw/include)
set(GLFW_LIBRARY ${CMAKE_SOURCE_DIR}/lib/glfw/build/src/libglfw3.a)

set(GLAD_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/glad/include)
set(GLAD_SOURCES ${CMAKE_SOURCE_DIR}/lib/glad/src/glad.c)

set(IMGUI_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/imgui)
set(IMPLOT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/implot)
set(IMGUI_SOURCES
        ${IMGUI_INCLUDE_DIR}/imgui.cpp
        ${IMGUI_INCLUDE_DIR}/imgui_draw.cpp
        ${IMGUI_INCLUDE_DIR}/imgui_widgets.cpp
        ${IMGUI_INCLUDE_DIR}/imgui_tables.cpp
        ${IMGUI_INCLUDE_DIR}/imgui_demo.cpp
        ${IMGUI_INCLUDE_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp
)

set(IMPLOT_SOURCES
        ${IMPLOT_INCLUDE_DIR}/implot.cpp
        ${IMPLOT_INCLUDE_DIR}/implot_items.cpp
        ${IMPLOT_INCLUDE_DIR}/implot_demo.cpp
)

include_directories(
        ${ARMADILLO_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
        ${GLAD_INCLUDE_DIR}
        ${IMGUI_INCLUDE_DIR}
        ${IMPLOT_INCLUDE_DIR})

add_executable(vdc main.cpp
        Vehicle.cpp
        Vehicle.h
        xform.h
        gui_backend.h
        ${IMGUI_SOURCES}
        ${IMPLOT_SOURCES}
        ${GLAD_SOURCES}
        gui_frontend_panels.h
        gui_color_themes.h
        Log.h
        VisualYmd.h
        diff.h
)

target_link_libraries(vdc ${ARMADILLO_LIBRARY} ${GLFW_LIBRARY})
