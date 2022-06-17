# Copyright 2021 PickNik Inc
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


function(enable_sanitizers project_name)

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" OFF)

    if(ENABLE_COVERAGE)
      target_compile_options(${project_name} INTERFACE --coverage -O0 -g -fno-omit-frame-pointer)
      target_link_libraries(${project_name} INTERFACE --coverage)
    endif()

    set(SANITIZERS "")

    option(ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" OFF)
    if(ENABLE_SANITIZER_ADDRESS)
      list(APPEND SANITIZERS "address")
    endif()

    option(ENABLE_SANITIZER_LEAK "Enable leak sanitizer" OFF)
    if(ENABLE_SANITIZER_LEAK)
      list(APPEND SANITIZERS "leak")
    endif()

    option(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR "Enable undefined behavior sanitizer" OFF)
    if(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR)
      list(APPEND SANITIZERS "undefined")
    endif()

    option(ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
    if(ENABLE_SANITIZER_THREAD)
      if("address" IN_LIST SANITIZERS OR "leak" IN_LIST SANITIZERS)
        message(WARNING "Thread sanitizer does not work with Address and Leak sanitizer enabled")
      else()
        list(APPEND SANITIZERS "thread")
        if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
          target_compile_options(
            ${project_name} INTERFACE
            -fsanitize-blacklist=${PROJECT_SOURCE_DIR}/.tsan-blacklist)
        else()
          message(WARNING "Thread sanitizer blacklisting only works with clang compiler")
        endif()
      endif()
    endif()

    option(ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)
    if(ENABLE_SANITIZER_MEMORY AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      message(WARNING "Memory sanitizer requires all the code (including libc++) \
        to be MSan-instrumented otherwise it reports false positives")
      if("address" IN_LIST SANITIZERS
          OR "thread" IN_LIST SANITIZERS
          OR "leak" IN_LIST SANITIZERS)
        message(WARNING "Memory sanitizer does not work with Address, Thread and Leak sanitizer enabled")
      else()
        list(APPEND SANITIZERS "memory")
      endif()
    endif()

    list(
      JOIN
      SANITIZERS
      ","
      LIST_OF_SANITIZERS)

  endif()

  if(LIST_OF_SANITIZERS)
    if(NOT
        "${LIST_OF_SANITIZERS}"
        STREQUAL
        "")
      target_compile_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
      target_link_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
    endif()
  endif()

endfunction()
