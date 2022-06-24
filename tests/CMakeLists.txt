ADD_UNIT_TEST(flex-estimator test-flex-estimator.cpp)
target_link_libraries(flex-estimator PUBLIC ${PROJECT_NAME})
target_link_libraries(flex-estimator PRIVATE Boost::unit_test_framework)

if(BUILD_PYTHON_INTERFACE)
  ADD_PYTHON_UNIT_TEST("py-flexi-hips" "tests/python/test_flexi_hips.py" "python")
endif()
