cmake_minimum_required(VERSION 3.11)
project(czAltitude)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)
find_package(CURL REQUIRED)
pkg_check_modules(libzip REQUIRED IMPORTED_TARGET libzip)

find_path(LASZIP_INCLUDE_DIR NAMES laszip/laszip_api.h)
find_library(LASZIP_LIBRARY NAMES laszip)
if(NOT LASZIP_INCLUDE_DIR OR NOT LASZIP_LIBRARY)
    message(FATAL_ERROR "LASzip not found! Install with: sudo apt install liblaszip-dev")
endif()


add_executable(${PROJECT_NAME} 
    src/main.cpp 
    src/DMR5GElevationGrid.cpp 
    include/CzAltitude/DMR5GElevationGrid.hpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
    include 
    ${LASZIP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    PkgConfig::PROJ
    CURL::libcurl
    PkgConfig::libzip
    ${LASZIP_LIBRARY}
    Threads::Threads
)
