cmake_minimum_required(VERSION 3.10)
project(OR_Learner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GUROBI_DIR "C:/gurobi1203/win64" CACHE PATH "Gurobi安装目录")

if(NOT EXISTS ${GUROBI_DIR})
    message(FATAL_ERROR "Gurobi目录不存在，请检查路径: ${GUROBI_DIR}")
endif()

if(MSVC)
    add_compile_options(/utf-8)
    set(CMAKE_VS_GLOBALS "VSLANG=1033")
endif()

set(GUROBI_INCLUDE_DIR "${GUROBI_DIR}/include")
set(GUROBI_LIB_DIR "${GUROBI_DIR}/lib")

set(GUROBI_CXX_LIB_RELEASE "${GUROBI_LIB_DIR}/gurobi_c++md2017.lib")
set(GUROBI_CXX_LIB_DEBUG "${GUROBI_LIB_DIR}/gurobi_c++mdd2017.lib")
set(GUROBI_CORE_LIB "${GUROBI_LIB_DIR}/gurobi120.lib")

if(NOT EXISTS ${GUROBI_CXX_LIB_RELEASE})
    message(FATAL_ERROR "Gurobi C++ Release库缺失: ${GUROBI_CXX_LIB_RELEASE}")
endif()
if(NOT EXISTS ${GUROBI_CXX_LIB_DEBUG})
    message(FATAL_ERROR "Gurobi C++ Debug库缺失: ${GUROBI_CXX_LIB_DEBUG}")
endif()
if(NOT EXISTS ${GUROBI_CORE_LIB})
    message(FATAL_ERROR "Gurobi核心库缺失: ${GUROBI_CORE_LIB}")
endif()

include_directories(${PROJECT_SOURCE_DIR}/src ${GUROBI_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out)

add_subdirectory(src)