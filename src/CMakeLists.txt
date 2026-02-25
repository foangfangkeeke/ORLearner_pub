set(MAIN_SOURCES main.cpp)
file(GLOB ALGORITHM_SOURCES algorithm/*.cpp)
file(GLOB UTILS_SOURCES utils/*.cpp)
file(GLOB_RECURSE PROBLEM_SOURCES problem/*.cpp)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/cutting_stock_problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/test_problem
    ${GUROBI_INCLUDE_DIR}
)

add_executable(main 
    ${MAIN_SOURCES} 
    ${ALGORITHM_SOURCES} 
    ${UTILS_SOURCES} 
    ${PROBLEM_SOURCES}
)

set_target_properties(main PROPERTIES
    OUTPUT_NAME "OR_Learner"
    CXX_STANDARD 17
)

target_link_libraries(main PRIVATE
    ${GUROBI_CXX_LIB}
    ${GUROBI_CORE_LIB}
    ws2_32.lib
)