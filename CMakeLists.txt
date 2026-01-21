cmake_minimum_required(VERSION 3.20)
project(sensor_mapper VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
include(FetchContent)

# GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(glfw)

# Dear ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

# CPR (C++ Requests - Curl Wrapper) for tile fetching
FetchContent_Declare(
    cpr
    GIT_REPOSITORY https://github.com/libcpr/cpr.git
    GIT_TAG        1.10.4
)
# CPR options to minimize build time/deps if needed, but defaults are usually fine for desktop
set(CPR_USE_SYSTEM_CURL OFF CACHE BOOL "" FORCE) # Build curl from source to avoid system dep issues on Windows
FetchContent_MakeAvailable(cpr)

# stb_image
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(stb)

# Source Files
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/*.hpp"
)

# Executable
add_executable(sensor_mapper ${SOURCES})

# Include Directories
target_include_directories(sensor_mapper PRIVATE 
    src
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${stb_SOURCE_DIR}
)

# Link Libraries
target_link_libraries(sensor_mapper PRIVATE 
    glfw
    cpr::cpr
)

# ImGui Backend Files (need to be compiled)
target_sources(sensor_mapper PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

# OS Specific Linking
if(WIN32)
    target_link_libraries(sensor_mapper PRIVATE opengl32)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(sensor_mapper PRIVATE GL)
endif()

# Copy assets if we had any (none yet)
