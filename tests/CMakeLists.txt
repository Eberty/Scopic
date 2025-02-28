add_executable(scopic_unit_test main.cpp)

target_link_libraries(scopic_unit_test scopic_test_task_lib)

if(MSVC)
    target_compile_options(scopic_unit_test PRIVATE $<$<CONFIG:Release>:/UNDEBUG>)
endif()

add_test(NAME scopic_unit_test COMMAND scopic_unit_test)
