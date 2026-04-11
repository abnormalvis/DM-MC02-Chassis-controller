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
