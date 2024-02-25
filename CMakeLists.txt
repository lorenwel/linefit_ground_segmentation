cmake_minimum_required(VERSION 3.14)
project(linefit_py
	VERSION 1.0.0
	DESCRIPTION "linefit_py"
	LANGUAGES CXX
)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# NanoBind Things ===> 
# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(
COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# my desktop need comment above and uncomment below
# find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development)
# execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
#                         OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
# list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
# find_package(nanobind CONFIG REQUIRED)
# <=== NanoBind Things

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cpp)
add_subdirectory(python)