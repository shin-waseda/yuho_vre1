cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME yuho)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# ---- CubeIDE の設定から引き継いだもの ----
# ここ(CubeMX が作り直さないファイル)にまとめ，cmake/ の下のファイルは手で変えない

# Debug でも -O2 にする(CubeIDE の Debug と同じにして，処理の速さを変えない)
set(CMAKE_C_FLAGS_DEBUG "-O2 -g3")

# CubeIDE の時に使っていたリンカースクリプトを使う．
# 最後のセクタ(0x080E0000〜)を迷路の地図の保存に空けてあり(FLASH 896K)，
# CubeMX が作った STM32F405XX_FLASH.ld(FLASH 1024K)だと地図の場所にプログラムが入り得る
string(REPLACE "STM32F405XX_FLASH.ld" "STM32F405RGTX_FLASH.ld"
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
if(NOT CMAKE_EXE_LINKER_FLAGS MATCHES "STM32F405RGTX_FLASH\\.ld")
    message(FATAL_ERROR "リンカースクリプトを STM32F405RGTX_FLASH.ld に替えられなかった: ${CMAKE_EXE_LINKER_FLAGS}")
endif()

# printf で float を出す(ログで使う)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# 自作のソース．CubeIDE と同じく，フォルダに置いた .c はすべてビルドする
# (CONFIGURE_DEPENDS で，ファイルを足したときにビルドの時点で拾い直す)
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Core/Src/app/*.c
    ${CMAKE_SOURCE_DIR}/Core/Src/interface/*.c
    ${CMAKE_SOURCE_DIR}/Core/Src/logic/*.c
)

# CubeIDE と同じく yuho.bin も作る
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
