cmake_minimum_required(VERSION 3.20)
project(sensor_mapper VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix for M_PI on Windows
add_compile_definitions(_USE_MATH_DEFINES)
add_compile_definitions(NOMINMAX)

# Compiler Warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
# CPR options to minimize build time/deps
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
    GIT_TAG master
)
FetchContent_MakeAvailable(stb)

# libtiff
FetchContent_Declare(
    tiff
    GIT_REPOSITORY https://gitlab.com/libtiff/libtiff.git
    GIT_TAG        v4.5.1
)
# Disable unnecessary parts of libtiff to speed up build
set(tiff-tools OFF CACHE BOOL "" FORCE)
set(tiff-tests OFF CACHE BOOL "" FORCE)
set(tiff-docs OFF CACHE BOOL "" FORCE)
set(tiff-contrib OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tiff)

# LAStools (LASlib)
FetchContent_Declare(
    lastools
    GIT_REPOSITORY https://github.com/LAStools/LAStools.git
    GIT_TAG        master
)
# Disable building tools, only library
set(LAS_TOOLS_WITH_LASZIP ON CACHE BOOL "" FORCE) # Use internal LASzip
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE) # Try to disable tools (might be named differently)
FetchContent_GetProperties(lastools)
if(NOT lastools_POPULATED)
  FetchContent_Populate(lastools)
  # Add include directories before including the subdirectory
  include_directories(SYSTEM
    ${lastools_SOURCE_DIR}/LASzip/src
    ${lastools_SOURCE_DIR}/LASzip/include/laszip
    ${lastools_SOURCE_DIR}/LASlib/inc
  )
  add_subdirectory(${lastools_SOURCE_DIR} ${lastools_BINARY_DIR})
endif()

# GLAD OpenGL Loader
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v0.1.36
)
FetchContent_MakeAvailable(glad)

# nlohmann_json
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.2
)
FetchContent_MakeAvailable(json)

# PROJ for coordinate transformations (optional)
find_package(PROJ QUIET)
if(PROJ_FOUND)
    message(STATUS "PROJ found: ${PROJ_VERSION}")
    set(HAVE_PROJ TRUE)
else()
    message(WARNING "PROJ not found - LIDAR will only work with lat/lon coordinates")
    message(WARNING "Install with: sudo apt install libproj-dev")
    set(HAVE_PROJ FALSE)
endif()

# Source Files
# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source Files
set(SOURCES_CORE
  src/core/antenna_pattern.hpp
  src/core/antenna_pattern_io.hpp
  src/core/antenna_pattern_library.hpp
  src/core/antenna_pattern_viz.hpp
  src/core/building_service.cpp
  src/core/building_service.hpp
  src/core/elevation_service.cpp
  src/core/elevation_service.hpp
  src/core/geo_math.hpp
  src/core/crs_transformer.cpp
  src/core/crs_transformer.hpp
  src/core/persistence.cpp
  src/core/persistence.hpp
  src/core/rf_engine.cpp
  src/core/rf_engine.hpp
  src/core/sensor.cpp
  src/core/sensor.hpp
  src/core/tdoa_solver.cpp
  src/core/tdoa_solver.hpp
  src/core/tile_service.cpp
  src/core/tile_service.hpp
  src/core/terrarium_source.cpp
  src/core/terrarium_source.hpp
  src/core/geotiff_source.cpp
  src/core/geotiff_source.hpp
  src/core/lidar_source.cpp
  src/core/lidar_source.hpp
)

set(SOURCES_RENDERER
  src/renderer/shader.cpp
  src/renderer/shader.hpp
  src/renderer/gpu_rf_engine.cpp
  src/renderer/gpu_rf_engine.hpp
)

set(SOURCES_UI
  src/ui/map_widget.cpp
  src/ui/map_widget.hpp
  src/ui/texture.cpp
  src/ui/texture.hpp
  src/ui/app_ui.cpp
  src/ui/app_ui.hpp
  src/ui/antenna_pattern_ui.hpp
  src/ui/antenna_pattern_3d_viewer.hpp
)

set(SOURCES
  src/main.cpp
  ${SOURCES_CORE}
  ${SOURCES_RENDERER}
  ${SOURCES_UI}
)

# Executable
add_executable(sensor_mapper ${SOURCES})

# Include Directories
target_include_directories(sensor_mapper PRIVATE
    src
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${stb_SOURCE_DIR}
    ${glad_SOURCE_DIR}/include
    ${lastools_SOURCE_DIR}/LASzip/src
    ${lastools_SOURCE_DIR}/LASlib/inc
)

# Link Libraries
target_link_libraries(sensor_mapper PRIVATE
    glfw
    cpr::cpr
    nlohmann_json::nlohmann_json
    glad
    tiff
    LASlib
)

# Add PROJ if available
if(HAVE_PROJ)
    target_link_libraries(sensor_mapper PRIVATE PROJ::proj)
    target_compile_definitions(sensor_mapper PRIVATE HAVE_PROJ)
endif()

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
