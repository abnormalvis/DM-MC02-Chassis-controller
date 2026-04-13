cmake_minimum_required(VERSION 3.16)
project(protocol_smoke C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(protocol_smoke_test
  protocol_smoke_test.c
  ../Core/Src/app/protocol_unified_v0.c
  ../Core/Src/app/protocol_legacy_v1.c
  ../Core/Src/app/protocol_adapter.c
  ../Core/Src/app/crsf_protocol.c
)

target_include_directories(protocol_smoke_test PRIVATE
  ../Core/Inc
  ../Core/Inc/app
)

add_executable(motor_control_task_test
  motor_control_task_test.c
  ../Core/Src/app/motor_control_task.c
)

target_include_directories(motor_control_task_test PRIVATE
  stubs
  ../Core/Inc
  ../Core/Inc/app
)

add_executable(app_tasks_chain_test
  app_tasks_chain_test.c
  ../Core/Src/app/app_tasks.c
)

target_include_directories(app_tasks_chain_test PRIVATE
  stubs
  ../Core/Inc
  ../Core/Inc/app
)
