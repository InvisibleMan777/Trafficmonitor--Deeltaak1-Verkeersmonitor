# Just a recent number
cmake_minimum_required(VERSION 3.18)

# Determine the Arduino path, with all Arduino related libraries and generic cmake stuff
if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(ARDUINO_PATH $ENV{HOME}/Arduino)
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
    set(ARDUINO_PATH "$ENV{HOMEDRIVE}$ENV{HOMEPATH}/Arduino")
endif()

# the toolchain to use
set(CMAKE_TOOLCHAIN_FILE ${ARDUINO_PATH}/etc/toolchain-avr.cmake)

# we need the assembler this time
enable_language(ASM)

# Project name
project("USART")

# Where to install
set(CMAKE_INSTALL_PREFIX ${ARDUINO_PATH})

# Target: the Arduino core library with all core source files to compile
# To get a list of .c and cpp files in Linux:
#       cd <project directory>
#       find src -name \*.[cSs]\*
add_library(${PROJECT_NAME} STATIC
        src/usart.c
)

# Here the compiler finds include files (-I flags)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# All .h files of Arduino core
# To get a list of .h files (in Linux):
#       cd <project directory>
#       find src -name \*.h
set(INCLUDES
        src/usart.h
)

# Install the library here (INSTALL_PREFIX_PATH is prepended)
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib)
install(FILES ${INCLUDES} DESTINATION include/arduino)
