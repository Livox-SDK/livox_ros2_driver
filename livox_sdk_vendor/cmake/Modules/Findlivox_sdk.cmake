# Copyright 2020 Autoware Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###################################################################################################
#
# CMake script for finding livox_sdk
#
# Input variables:
#
# - livox_sdk_ROOT_DIR (optional): When specified, header files and libraries will be searched for in
#     ${livox_sdk_ROOT_DIR}/include
#     ${livox_sdk_ROOT_DIR}/libs
#   respectively, and the default CMake search order will be ignored. When unspecified, the default
#   CMake search order is used.
#   This variable can be specified either as a CMake or environment variable. If both are set,
#   preference is given to the CMake variable.
#   Use this variable for finding packages installed in a nonstandard location, or for enforcing
#   that one of multiple package installations is picked up.
#
#
# Cache variables (not intended to be used in CMakeLists.txt files)
#
# - livox_sdk_INCLUDE_DIR: Absolute path to package headers.
# - livox_sdk_LIBRARY: Absolute path to library.
#
#
# Output variables:
#
# - livox_sdk_FOUND: Boolean that indicates if the package was found
# - livox_sdk_INCLUDE_DIRS: Paths to the necessary header files
# - livox_sdk_LIBRARIES: Package libraries
#
#
# Example usage:
#
#  find_package(livox_sdk)
#  if(NOT livox_sdk_FOUND)
#    # Error handling
#  endif()
#  ...
#  include_directories(${livox_sdk_INCLUDE_DIRS} ...)
#  ...
#  target_link_libraries(my_target ${livox_sdk_LIBRARIES})
#
###################################################################################################

# Get package location hint from environment variable (if any)
if(NOT livox_sdk_ROOT_DIR AND DEFINED ENV{livox_sdk_ROOT_DIR})
    set(livox_sdk_ROOT_DIR "$ENV{livox_sdk_ROOT_DIR}" CACHE PATH
        "livox base directory location (optional, used for nonstandard installation paths)")
endif()

if(livox_sdk_ROOT_DIR)
  set(livox_sdk_INCLUDE_PATH PATHS "${livox_sdk_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(livox_sdk_LIBRARY_PATH PATHS "${livox_sdk_ROOT_DIR}/lib" NO_DEFAULT_PATH)
else()
  set(livox_sdk_INCLUDE_PATH "")
  set(livox_sdk_LIBRARY_PATH "")
endif()

# Search for headers and the library
find_path(livox_sdk_INCLUDE_DIR NAMES "livox_sdk.h" ${livox_sdk_INCLUDE_PATH})
find_library(livox_sdk_LIBRARY NAMES livox_sdk_static ${livox_sdk_LIBRARY_PATH})

mark_as_advanced(livox_sdk_INCLUDE_DIR livox_sdk_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(livox_sdk DEFAULT_MSG livox_sdk_INCLUDE_DIR livox_sdk_LIBRARY)

if(${livox_sdk_FOUND})
  set(livox_sdk_INCLUDE_DIRS ${livox_sdk_INCLUDE_DIR})
  set(livox_sdk_LIBRARIES ${livox_sdk_LIBRARY})

  add_library(livox::livox UNKNOWN IMPORTED)
  set_property(TARGET livox::livox PROPERTY IMPORTED_LOCATION ${livox_sdk_LIBRARY})
  set_property(TARGET livox::livox PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${livox_sdk_INCLUDE_DIR})
  list(APPEND livox_sdk_TARGETS livox::livox)
endif()
