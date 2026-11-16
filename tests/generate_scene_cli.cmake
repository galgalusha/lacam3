file(MAKE_DIRECTORY "${TEST_DIR}")
file(WRITE "${TEST_DIR}/small.map" "type octile\nheight 2\nwidth 2\nmap\n..\n..\n")
file(REMOVE "${TEST_DIR}/out.scen")

execute_process(
  COMMAND "${GENERATOR}" "${TEST_DIR}/small.map" 9 1 2
  WORKING_DIRECTORY "${TEST_DIR}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error
  TIMEOUT 30)
if(NOT result STREQUAL "0")
  message(FATAL_ERROR "generator failed: ${result}\n${output}\n${error}")
endif()
foreach(expected "Current number of agents: 1. Running 9 seeds..."
                 "Current number of agents: 2. Running 9 seeds..."
                 "100%")
  string(FIND "${output}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "missing '${expected}' in output: ${output}")
  endif()
endforeach()

execute_process(
  COMMAND "${VERIFY}" "${TEST_DIR}/out.scen" "${TEST_DIR}/small.map" 2
  RESULT_VARIABLE result)
if(NOT result STREQUAL "0")
  message(FATAL_ERROR "generated scenario cannot be loaded with two unique agents")
endif()
file(STRINGS "${TEST_DIR}/out.scen" scene_lines)
list(LENGTH scene_lines line_count)
if(NOT line_count EQUAL 3)
  message(FATAL_ERROR "scenario should contain a header and exactly two rows")
endif()
file(READ "${TEST_DIR}/out.scen" saved_scene)

foreach(args "0;1;2" "1;0;2" "1;1;5" "1x;1;2")
  execute_process(
    COMMAND "${GENERATOR}" "${TEST_DIR}/small.map" ${args}
    WORKING_DIRECTORY "${TEST_DIR}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(result STREQUAL "0" OR error STREQUAL "")
    message(FATAL_ERROR "invalid arguments ${args} were not rejected: ${output}")
  endif()
  file(READ "${TEST_DIR}/out.scen" current_scene)
  if(NOT current_scene STREQUAL saved_scene)
    message(FATAL_ERROR "invalid arguments modified the existing scenario")
  endif()
endforeach()

file(WRITE "${TEST_DIR}/larger.map"
  "type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n")
execute_process(
  COMMAND "${GENERATOR}" "${TEST_DIR}/larger.map" 9 2 3
  WORKING_DIRECTORY "${TEST_DIR}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error
  TIMEOUT 30)
if(NOT result STREQUAL "0")
  message(FATAL_ERROR "generator failed on partial increment: ${result}\n${output}\n${error}")
endif()
foreach(expected "Current number of agents: 2. Running 9 seeds..."
                 "Current number of agents: 3. Running 9 seeds...")
  string(FIND "${output}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "missing '${expected}' in output: ${output}")
  endif()
endforeach()
execute_process(
  COMMAND "${VERIFY}" "${TEST_DIR}/out.scen" "${TEST_DIR}/larger.map" 3
  RESULT_VARIABLE result)
if(NOT result STREQUAL "0")
  message(FATAL_ERROR "partial increment did not produce a valid three-agent scenario")
endif()
