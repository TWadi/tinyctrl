include(FetchContent)

FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.7.0
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(unity)

foreach(name pid discretize biquad lowpass leadlag)
    add_executable(test_${name} test_${name}.c)
    target_link_libraries(test_${name} PRIVATE tinyctrl::tinyctrl unity)
    target_include_directories(test_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    tinyctrl_set_warnings(test_${name})
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
