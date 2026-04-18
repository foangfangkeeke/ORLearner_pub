set(MAIN_SOURCES main.cpp)
file(GLOB ALGORITHM_SOURCES CONFIGURE_DEPENDS algorithm/*.cpp)
file(GLOB UTILS_SOURCES CONFIGURE_DEPENDS utils/*.cpp)
file(GLOB_RECURSE PROBLEM_SOURCES CONFIGURE_DEPENDS problem/*.cpp)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/cutting_stock_problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/test_problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/fixed_charge_transportation_problem
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/BARP
    ${CMAKE_CURRENT_SOURCE_DIR}/problem/wireless_charge_bus
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
    $<$<CONFIG:Debug>:${GUROBI_CXX_LIB_DEBUG}>
    $<$<NOT:$<CONFIG:Debug>>:${GUROBI_CXX_LIB_RELEASE}>
    ${GUROBI_CORE_LIB}
    ws2_32.lib
)
