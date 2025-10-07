add_unit_test(test-flex-estimator-cpp test-flex-estimator.cpp)
target_link_libraries(test-flex-estimator-cpp PUBLIC ${PROJECT_NAME})
target_link_libraries(test-flex-estimator-cpp PRIVATE doctest::doctest)

if(BUILD_PYTHON_INTERFACE)
  add_python_unit_test("test-flex-estimator-py"
                       "tests/python/test_flexi_hips.py" "python")
endif()
