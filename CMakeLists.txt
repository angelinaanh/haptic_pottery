cmake_minimum_required (VERSION 3.10)
project (VirtualPottery)

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------

# CHAI3D is located through the package registry entry written by its build
# tree. Set chai3d_DIR by hand if that fails, e.g.
#   -Dchai3d_DIR=".../chai3d-3.3.0/out/build/<preset>"
find_package (chai3d 3.3 REQUIRED)
include_directories (${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions (${CHAI3D_DEFINITIONS})

find_package (GLFW REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

#-------------------------------------------------------------------------------
# Platform
#-------------------------------------------------------------------------------

if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
    set (OS "win")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set (ARCH "x64")
    else ()
        set (ARCH "Win32")
    endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    set (OS "lin")
    set (ARCH "${CMAKE_SYSTEM_PROCESSOR}")
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    set (OS "mac")
    set (ARCH ${CMAKE_HOST_SYSTEM_PROCESSOR})
endif ()

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif ()

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

foreach (cfg "" _DEBUG _MINSIZEREL _RELEASE _RELWITHDEBINFO)
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY${cfg} ${CMAKE_CURRENT_SOURCE_DIR}/bin/${OS}-${ARCH})
endforeach ()

if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
    if (MSVC)
        add_definitions (-D_CRT_SECURE_NO_DEPRECATE -DNOMINMAX)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP")
        if (CMAKE_SIZEOF_VOID_P EQUAL 8)
            add_definitions (-DWIN64)
        else ()
            add_definitions (-DWIN32)
        endif ()
    elseif (MINGW)
        add_definitions (-DWIN32)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated")
    endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    add_definitions (-DLINUX)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated")
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    add_definitions (-DMACOSX)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -Wno-deprecated")
endif ()

#-------------------------------------------------------------------------------
# Sources, by architectural layer
#-------------------------------------------------------------------------------

set (CORE_SOURCES
    Core_Data_Structures/CircularSectorElement.h
    Core_Data_Structures/MethodConfig.h
    Core_Data_Structures/VirtualPottery.h
    Core_Data_Structures/VirtualPottery.cpp
)

set (HAPTIC_SOURCES
    Haptic_Thread/HapticDeviceController.h
    Haptic_Thread/HapticDeviceController.cpp
    Haptic_Thread/CollisionDetector.h
    Haptic_Thread/CollisionDetector.cpp
    Haptic_Thread/CollisionResponder.h
    Haptic_Thread/CollisionResponder.cpp
    Haptic_Thread/LocalForceModel.h
    Haptic_Thread/LocalForceModel.cpp
)

set (GRAPHIC_SOURCES
    Graphic_Thread/GraphicSynchronizer.h
    Graphic_Thread/GraphicSynchronizer.cpp
    Graphic_Thread/VisualizePottery.h
    Graphic_Thread/VisualizePottery.cpp
)

set (EVALUATION_SOURCES
    Evaluation/Benchmark.h
    Evaluation/Benchmark.cpp
)

set (SHADERS
    Graphic_Thread/Shaders/pottery.vert
    Graphic_Thread/Shaders/pottery.frag
)

source_group ("Core_Data_Structures" FILES ${CORE_SOURCES})
source_group ("Haptic_Thread"        FILES ${HAPTIC_SOURCES})
source_group ("Graphic_Thread"       FILES ${GRAPHIC_SOURCES})
source_group ("Graphic_Thread\\Shaders" FILES ${SHADERS})
source_group ("Evaluation"           FILES ${EVALUATION_SOURCES})

add_executable (${PROJECT_NAME} main.cpp ${CORE_SOURCES} ${HAPTIC_SOURCES} ${GRAPHIC_SOURCES}
                                ${EVALUATION_SOURCES} ${SHADERS})

target_include_directories (${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core_Data_Structures
    ${CMAKE_CURRENT_SOURCE_DIR}/Haptic_Thread
    ${CMAKE_CURRENT_SOURCE_DIR}/Graphic_Thread
    ${CMAKE_CURRENT_SOURCE_DIR}/Evaluation
)

target_link_libraries (${PROJECT_NAME} ${CHAI3D_LIBRARIES} ${GLFW_LIBRARIES})

# Shaders: copied next to the executable; the source folder is the fallback.
target_compile_definitions (${PROJECT_NAME} PRIVATE
    VP_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Graphic_Thread/Shaders")

# Benchmark output (VirtualPottery --bench).
file (MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Results)
target_compile_definitions (${PROJECT_NAME} PRIVATE
    VP_RESULTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Results")

add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/Shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/Graphic_Thread/Shaders/pottery.vert
                                                  ${CMAKE_CURRENT_SOURCE_DIR}/Graphic_Thread/Shaders/pottery.frag
                                                  "$<TARGET_FILE_DIR:${PROJECT_NAME}>/Shaders")

# CHAI3D loads the PHANToM driver wrapper (hdPhantom64.dll) at run time from
# the executable's folder or the PATH; without it the device is not detected
# and the program falls back to the mouse.
set (CHAI3D_RUNTIME_DIR "${chai3d_DIR}/../../../bin/${OS}-${ARCH}"
     CACHE PATH "Folder holding CHAI3D's device driver DLLs")
if (WIN32)
    foreach (dll hdPhantom64.dll hdPhantom32.dll)
        if (EXISTS "${CHAI3D_RUNTIME_DIR}/${dll}")
            add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${CHAI3D_RUNTIME_DIR}/${dll}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
        endif ()
    endforeach ()
endif ()
