cmake_minimum_required(VERSION 3.16)
project(lacam-project CXX)

add_subdirectory(./lacam3)
add_subdirectory(./third_party/argparse)

include(FetchContent)
FetchContent_Declare(
  absl
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        20240722.0
  GIT_SHALLOW    TRUE
)
set(ABSL_PROPAGATE_CXX_STD ON)
FetchContent_MakeAvailable(absl)

FetchContent_Declare(
  cereal
  GIT_REPOSITORY https://github.com/USCiLab/cereal.git
  GIT_TAG        v1.3.2
  GIT_SHALLOW    TRUE
)
set(BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_SANDBOX OFF CACHE BOOL "" FORCE)
set(SKIP_PERFORMANCE_COMPARISON ON CACHE BOOL "" FORCE)
set(CEREAL_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cereal)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
# target_compile_options(main PRIVATE -O3 -march=native -DNDEBUG -ffast-math -funroll-loops -fomit-frame-pointer)
# target_link_options(main PRIVATE -flto)
target_link_libraries(main lacam3 argparse)

add_executable(eval_runner eval_runner.cpp)
target_compile_features(eval_runner PUBLIC cxx_std_17)
target_link_libraries(eval_runner lacam3 cereal::cereal)

add_executable(eval_ui eval_ui.cpp)
target_compile_features(eval_ui PUBLIC cxx_std_17)
add_dependencies(eval_ui eval_runner)
target_link_libraries(eval_ui lacam3)

# test
enable_testing()
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file})
  target_link_libraries(${name} lacam3)
  add_test(${name} ${name})
endforeach()
