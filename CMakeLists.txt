cmake_minimum_required(VERSION 3.20)
project(sensor_mapper VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix for M_PI on Windows
add_compile_definitions(_USE_MATH_DEFINES)
add_compile_definitions(NOMINMAX)

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
# Build curl from source
set(CPR_USE_SYSTEM_CURL OFF CACHE BOOL "" FORCE)
set(CPR_ENABLE_SSL ON CACHE BOOL "" FORCE)
set(OPENSSL_USE_STATIC_LIBS OFF)

if(WIN32)
    # Use native Windows SSL (Schannel)
    set(CURL_USE_SCHANNEL ON CACHE BOOL "" FORCE)
    set(CURL_USE_OPENSSL OFF CACHE BOOL "" FORCE)
else()
    # Use OpenSSL on Linux/Unix
    find_package(OpenSSL REQUIRED)
endif()

FetchContent_MakeAvailable(cpr)

# stb_image
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(stb)

# nlohmann_json
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.2
)
FetchContent_MakeAvailable(json)

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
    nlohmann_json::nlohmann_json
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
