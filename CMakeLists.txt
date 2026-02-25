cmake_minimum_required(VERSION 3.15...3.26)
project(finelpy LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)


# ================================
# FINELPY SOURCE FILES
# ================================

file(GLOB BINDING_SOURCES src/finelpy/bindings/*.cpp)
file(GLOB ASSEMBLY_SOURCES src/finelpy/cpp/assembly/*.cpp)
file(GLOB ELEMENT_GEOMETRY_SOURCES src/finelpy/cpp/element_geometry/*.cpp) 
file(GLOB ELEMENT_PHYSICS_SOURCES src/finelpy/cpp/element_physics/*.cpp)
file(GLOB ELEMENT_SOURCES src/finelpy/cpp/elements/*.cpp)
file(GLOB GEOMETRY_SOURCES src/finelpy/cpp/geometry/*.cpp)
file(GLOB MATERIAL_SOURCES src/finelpy/cpp/material/*.cpp)
file(GLOB MESH_SOURCES src/finelpy/cpp/mesh/*.cpp)
file(GLOB PHYSICS_SOURCES src/finelpy/cpp/physics/*.cpp)
file(GLOB ANALYSIS_SOURCES src/finelpy/cpp/analysis/*.cpp)
file(GLOB SOLVER_SOURCES src/finelpy/cpp/solver/*.cpp)
file(GLOB MATRIX_SOURCES src/finelpy/cpp/matrix/*.cpp)
file(GLOB RESULT_SOURCES src/finelpy/cpp/result/*.cpp)


set(SOURCE_FILES
    src/finelpy/binding.cpp
    src/finelpy/cpp/compatibility.cpp
    ${MATRIX_SOURCES}
    ${BINDING_SOURCES}
    ${ASSEMBLY_SOURCES}
    ${ELEMENT_GEOMETRY_SOURCES}
    ${ELEMENT_PHYSICS_SOURCES}
    ${ELEMENT_SOURCES}
    ${GEOMETRY_SOURCES}
    ${MATERIAL_SOURCES}
    ${MESH_SOURCES}
    ${PHYSICS_SOURCES}
    ${ANALYSIS_SOURCES}
    ${RESULT_SOURCES}
    ${SOLVER_SOURCES}
    )

pybind11_add_module(core ${SOURCE_FILES})
target_include_directories(core PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# macOS specific
if(APPLE)
    set_target_properties(core PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()


# ================================
# EIGEN
# ================================

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.1
    )
    FetchContent_MakeAvailable(eigen)
endif()


target_link_libraries(core PRIVATE Eigen3::Eigen)


# ================================
# Accelerate
# ================================

if(APPLE)
    find_library(ACCELERATE_FRAMEWORK Accelerate REQUIRED)
    target_link_libraries(core PRIVATE ${ACCELERATE_FRAMEWORK})
    target_compile_definitions(core PRIVATE MACOS=1)
endif()

if(APPLE)
    find_package(Boost CONFIG REQUIRED COMPONENTS math)
    target_link_libraries(core PRIVATE Eigen3::Eigen Boost::math)
endif()

# ================================
# PETSc
# ================================

find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)

    if(DEFINED ENV{CONDA_PREFIX})
        set(ENV{PKG_CONFIG_PATH} "$ENV{CONDA_PREFIX}/lib/pkgconfig")
        message(STATUS "Using PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}")
    else()
        message(STATUS "CONDA_PREFIX is not set — activate your conda environment first")
    endif()

    pkg_check_modules(PETSC QUIET PETSc)
endif()


# if(PETSC_FOUND AND PkgConfig_FOUND)

#     message(STATUS "PETSc found: ${PETSC_DIR}")

#     # Query PETSc's MPI compiler wrappers
#     execute_process(
#         COMMAND pkg-config --variable=mpicc PETSc
#         OUTPUT_VARIABLE PETSC_MPICC
#         OUTPUT_STRIP_TRAILING_WHITESPACE
#     )

#     execute_process(
#         COMMAND pkg-config --variable=mpicxx PETSc
#         OUTPUT_VARIABLE PETSC_MPICXX
#         OUTPUT_STRIP_TRAILING_WHITESPACE
#     )

#     if(PETSC_MPICC AND PETSC_MPICXX)
#         message(STATUS "PETSc MPI C compiler:  ${PETSC_MPICC}")
#         message(STATUS "PETSc MPI CXX compiler: ${PETSC_MPICXX}")

#         set(MPI_C_COMPILER   ${PETSC_MPICC}  CACHE FILEPATH "" FORCE)
#         set(MPI_CXX_COMPILER ${PETSC_MPICXX} CACHE FILEPATH "" FORCE)
#     endif()
    
#     find_package(MPI REQUIRED)

#     add_library(PETSc::PETSc INTERFACE IMPORTED)

#     target_include_directories(PETSc::PETSc INTERFACE
#         ${PETSC_INCLUDE_DIRS}
#     )

#     target_link_libraries(PETSc::PETSc INTERFACE
#         ${PETSC_LIBRARIES}
#         MPI::MPI_CXX
#     )

#     target_compile_options(PETSc::PETSc INTERFACE
#         ${PETSC_CFLAGS_OTHER}
#     )

#     target_link_libraries(core PRIVATE PETSc::PETSc)
#     target_compile_definitions(core PRIVATE USE_PETSC)

            
        
# endif()



# =================================
# Install Python extension + stubs
# =================================
install(TARGETS core 
    LIBRARY DESTINATION finelpy
    )
