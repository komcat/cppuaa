# CMakeList.txt : CMake project for cppuaa
cmake_minimum_required(VERSION 3.12)
project(cppuaa)

# Define SDL2 path
set(SDL2_PATH "C:/Users/komgr/source/repos/SDL2")

# Define ImGui path
set(IMGUI_PATH "C:/Users/komgr/source/repos/imgui-master")

# Include directories for SDL2 and ImGui
include_directories(${SDL2_PATH}/include ${IMGUI_PATH} ${IMGUI_PATH}/backends)

# Link directories for SDL2
link_directories(${SDL2_PATH}/lib/x64)

# ImGui source files
set(IMGUI_SOURCES
    ${IMGUI_PATH}/imgui.cpp
    ${IMGUI_PATH}/imgui_demo.cpp
    ${IMGUI_PATH}/imgui_draw.cpp
    ${IMGUI_PATH}/imgui_tables.cpp
    ${IMGUI_PATH}/imgui_widgets.cpp
    ${IMGUI_PATH}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_PATH}/backends/imgui_impl_opengl3.cpp
)

# Add source to this project's executable
add_executable(cppuaa cppuaa.cpp cppuaa.h ${IMGUI_SOURCES})

# Link libraries
target_link_libraries(cppuaa SDL2 SDL2main opengl32)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET cppuaa PROPERTY CXX_STANDARD 20)
endif()

# Ensure SDL2.dll is copied to the output directory
add_custom_command(TARGET cppuaa POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL2_PATH}/lib/x64/SDL2.dll"
        $<TARGET_FILE_DIR:cppuaa>)