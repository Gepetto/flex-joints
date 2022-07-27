add_unit_test(flex-estimator test-flex-estimator.cpp)
target_link_libraries(flex-estimator PUBLIC ${PROJECT_NAME})
target_link_libraries(flex-estimator PRIVATE Boost::unit_test_framework)

if(BUILD_PYTHON_INTERFACE)
  add_python_unit_test("py-flexi-hips" "tests/python/test_flexi_hips.py"
                       "python")
endif()
