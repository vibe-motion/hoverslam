cmake_minimum_required(VERSION 3.18)
project(hoverslam_osqp_wasm C)

set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PROFILING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG OFF CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/osqp ${CMAKE_CURRENT_BINARY_DIR}/osqp)

add_executable(osqp_bindings ${CMAKE_CURRENT_SOURCE_DIR}/../../src/flight/qp/bridge.c)
target_link_libraries(osqp_bindings PRIVATE osqpstatic)
target_compile_options(osqp_bindings PRIVATE -O3 -fno-exceptions -fno-rtti)
target_link_options(osqp_bindings PRIVATE
  -O3
  "-sMODULARIZE=1"
  "-sEXPORT_ES6=1"
  "-sENVIRONMENT=web,node"
  "-sFILESYSTEM=0"
  "-sALLOW_MEMORY_GROWTH=1"
  "-sASSERTIONS=0"
  "-sEXPORTED_FUNCTIONS=['_malloc','_free','_qp_create','_qp_solve','_qp_status','_qp_iterations','_qp_primal_residual','_qp_dual_residual','_qp_destroy']"
  "-sEXPORTED_RUNTIME_METHODS=['HEAP32','HEAPF64']"
)
set_target_properties(osqp_bindings PROPERTIES
  SUFFIX ".mjs"
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../src/flight/qp
)
