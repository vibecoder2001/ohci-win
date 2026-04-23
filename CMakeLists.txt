cmake_minimum_required(VERSION 3.22)
project(xhci_win C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if (MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

add_subdirectory(core)
enable_testing()
add_subdirectory(test/harness)
