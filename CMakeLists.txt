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

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_compile_options(main PRIVATE -O3 -march=native -DNDEBUG -ffast-math -funroll-loops -fomit-frame-pointer)
target_link_options(main PRIVATE -flto)
target_link_libraries(main lacam3 argparse)

# test
enable_testing()
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file})
  target_link_libraries(${name} lacam3)
  add_test(${name} ${name})
endforeach()
