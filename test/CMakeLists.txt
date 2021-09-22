add_executable(test_eiger
    test_eiger.cpp
)

target_link_libraries(test_eiger PUBLIC limacore eiger)

add_test(
    NAME basic_test
    COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/test_int_trig_mult.py
)
