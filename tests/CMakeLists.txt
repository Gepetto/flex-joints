if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
    add_unit_test(test-flex-estimator-cpp test-flex-estimator.cpp)
    target_link_libraries(
        test-flex-estimator-cpp
        PUBLIC ${PROJECT_NAME}::${PROJECT_NAME}
    )
    target_link_libraries(test-flex-estimator-cpp PRIVATE doctest::doctest)
endif()

if(BUILD_PYTHON_INTERFACE)
    add_python_unit_test("test-flex-estimator-py"
                         "tests/python/test_flexi_hips.py" "python"
    )
endif()
