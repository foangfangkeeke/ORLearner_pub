cmake_minimum_required(VERSION 3.10)
project(OR_Learner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GUROBI_DIR "C:/gurobi1203/win64" CACHE PATH "Gurobi安装目录")

if(NOT EXISTS ${GUROBI_DIR})
    message(FATAL_ERROR "Gurobi目录不存在，请检查路径: ${GUROBI_DIR}")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "/MT")
set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
add_compile_options("/source-charset:utf-8")

set(GUROBI_INCLUDE_DIR "${GUROBI_DIR}/include")
set(GUROBI_LIB_DIR "${GUROBI_DIR}/lib")

set(GUROBI_CXX_LIB "${GUROBI_LIB_DIR}/gurobi_c++mt2017.lib")
set(GUROBI_CORE_LIB "${GUROBI_LIB_DIR}/gurobi120.lib")

if(NOT EXISTS ${GUROBI_CXX_LIB})
    message(FATAL_ERROR "Gurobi C++库缺失: ${GUROBI_CXX_LIB}")
endif()
if(NOT EXISTS ${GUROBI_CORE_LIB})
    message(FATAL_ERROR "Gurobi核心库缺失: ${GUROBI_CORE_LIB}")
endif()

include_directories(${PROJECT_SOURCE_DIR}/src ${GUROBI_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out)

add_subdirectory(src)