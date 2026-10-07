cmake_minimum_required(VERSION 3.14)

project(tinyctrl
    VERSION 0.1.0
    DESCRIPTION "Tested control toolbox for microcontrollers: PID, Tustin discretisation, biquad, lead/lag"
    LANGUAGES C)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(TINYCTRL_IS_TOP_LEVEL ON)
else()
    set(TINYCTRL_IS_TOP_LEVEL OFF)
endif()

option(TINYCTRL_USE_DOUBLE "Use double instead of float for tc_real" OFF)
option(TINYCTRL_BUILD_TESTS "Build the unit tests" ${TINYCTRL_IS_TOP_LEVEL})
option(TINYCTRL_BUILD_EXAMPLES "Build the examples" ${TINYCTRL_IS_TOP_LEVEL})
# Off by default so that a newer compiler with new warnings cannot break a user's build; CI turns it on.
option(TINYCTRL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(TINYCTRL_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer (GCC/Clang)" OFF)

# Strict warnings for tinyctrl's own targets (not for the consumer's code).
function(tinyctrl_set_warnings target)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion -Wshadow -Wfloat-equal
            -Wstrict-prototypes -Wmissing-prototypes -Wcast-qual -Wundef)
        if(TINYCTRL_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
        if(TINYCTRL_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    endif()
    if(TINYCTRL_SANITIZE AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
endfunction()

add_library(tinyctrl
    src/tc_biquad.c
    src/tc_discretize.c
    src/tc_leadlag.c
    src/tc_lowpass.c
    src/tc_pid.c)
add_library(tinyctrl::tinyctrl ALIAS tinyctrl)

target_include_directories(tinyctrl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(tinyctrl PUBLIC c_std_99)
set_target_properties(tinyctrl PROPERTIES C_EXTENSIONS OFF)
if(TINYCTRL_USE_DOUBLE)
    target_compile_definitions(tinyctrl PUBLIC TINYCTRL_USE_DOUBLE)
endif()

include(CheckLibraryExists)
check_library_exists(m exp "" TINYCTRL_HAVE_LIBM)
if(TINYCTRL_HAVE_LIBM)
    target_link_libraries(tinyctrl PUBLIC m)
endif()

tinyctrl_set_warnings(tinyctrl)

include(GNUInstallDirs)
install(TARGETS tinyctrl EXPORT tinyctrlTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/tinyctrl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# find_package(tinyctrl) support for installed copies.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/tinyctrlConfigVersion.cmake
    COMPATIBILITY SameMinorVersion)
install(EXPORT tinyctrlTargets
    FILE tinyctrlConfig.cmake
    NAMESPACE tinyctrl::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tinyctrl)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tinyctrlConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tinyctrl)

if(TINYCTRL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(TINYCTRL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
