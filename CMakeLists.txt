cmake_minimum_required(VERSION 3.24)
# The build number shown in the window title. Bump it for every build handed over.
project(TwoWheelSimulator VERSION 0.2.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Static MSVC runtime: the .exe runs without installing any Visual C++ redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

option(TWS_BUILD_GAME "Build the game executable (needs SDL3, Dear ImGui)" ON)

# --- Simulation core: no SDL, no OpenGL, no ImGui. Shared by the game and the headless tests. ---
add_library(tws_sim STATIC
  sim/terrain.cpp
  sim/tyre.cpp
  sim/wheel_lab.cpp
  sim/articulated.cpp)
target_include_directories(tws_sim PUBLIC sim)
if(MSVC)
  # /fp:precise (the default), never /fp:fast: the sim must be deterministic (MSVC does not contract to FMA by default).
  target_compile_options(tws_sim PUBLIC /W4 /permissive- /fp:precise)
else()
  target_compile_options(tws_sim PUBLIC -Wall -Wextra -pedantic -ffp-contract=off)
endif()

enable_testing()
add_executable(sim_tests tests/tests.cpp)
target_link_libraries(sim_tests PRIVATE tws_sim)
add_test(NAME sim_tests COMMAND sim_tests)

if(TWS_BUILD_GAME)
  include(FetchContent)
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-3.4.18/SDL3-3.4.18.tar.gz
    URL_HASH SHA256=9c75cf16330322c217dedd2e0609f1124f1b54b8633e763467b4684d0f4334a3
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(SDL3)

  # Dear ImGui (MIT): debug views, the lab panel, later the setup screens and editor.
  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9b.tar.gz
    URL_HASH SHA256=8e1bbc76c71d74fef2fb85db7e7ca8eba13d6a86623c54992b60162db554ffdb
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(imgui)
  add_library(tws_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(tws_imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  target_link_libraries(tws_imgui PUBLIC SDL3::SDL3-static)
  if(MSVC)
    target_compile_options(tws_imgui PRIVATE /w)
  else()
    target_compile_options(tws_imgui PRIVATE -w)
  endif()

  add_executable(twowheel WIN32
    game/main.cpp
    game/gl.cpp
    game/render.cpp
    game/png.cpp
    game/pad.cpp)
  target_link_libraries(twowheel PRIVATE tws_sim tws_imgui SDL3::SDL3-static)
  target_compile_definitions(twowheel PRIVATE TWS_VERSION="${PROJECT_VERSION}" _CRT_SECURE_NO_WARNINGS)
  if(MSVC)
    target_link_options(twowheel PRIVATE /INCREMENTAL:NO)
  endif()
endif()
