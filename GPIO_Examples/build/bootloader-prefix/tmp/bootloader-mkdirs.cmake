# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Espressif/frameworks/esp-idf-v5.4/components/bootloader/subproject")
  file(MAKE_DIRECTORY "C:/Espressif/frameworks/esp-idf-v5.4/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader"
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix"
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/tmp"
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/src/bootloader-stamp"
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/src"
  "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "S:/Self_Learning/Programming/ESP32 Programming Tutorial Learning/GPIO_Examples/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
