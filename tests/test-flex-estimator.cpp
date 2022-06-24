#define BOOST_TEST_MODULE biped_stabilizer test boost
#include <boost/test/included/unit_test.hpp>

#include "flex-joints/flexi-hips.hpp"

flex::FlexSettings makeObservedSettings() {
  flex::FlexSettings observed_settings;
  observed_settings.dt = 0.005;
  observed_settings.left_stiffness << 2200, 2200;
  observed_settings.right_stiffness << 5000, 5000;
  observed_settings.left_damping =
      2 * observed_settings.left_stiffness.cwiseSqrt();
  observed_settings.right_damping =
      2 * observed_settings.right_stiffness.cwiseSqrt();
  return observed_settings;
}

BOOST_AUTO_TEST_CASE(test_flex_estimator_default_constructor) {
  flex::FlexSettings default_settings;
  flex::Flex stiff_hips;
  BOOST_CHECK_EQUAL(stiff_hips.getSettings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_flex_estimator_initialize_constructor) {
  flex::FlexSettings current_settings = makeObservedSettings();
  flex::Flex stiff_hips;
  flex::Flex flex_hips(current_settings);

  stiff_hips.initialize(current_settings);

  BOOST_CHECK_EQUAL(stiff_hips.getSettings(), flex_hips.getSettings());
}

// BOOST_AUTO_TEST_CASE(test_compute_deflection) {
//   flex::FlexSettings current_settings = makeObservedSettings();
//   flex::Flex flex_hips(current_settings);
//   flex::eVector2 tau = flex::eVector2::Zero();
//   flex::eVector2 delta0 = flex::eVector2::Zero();

//   flex::eVector2 delta1 = flex_hips.computeDeflection(tau, flex::LEFT);
//   flex::eVector2 delta2 = flex_hips.computeDeflection(tau, delta0,
//   current_settings.left_stiffness,
//                                                       current_settings.left_damping,
//                                                       current_settings.dt);

//   BOOST_CHECK_EQUAL(delta1, delta2);
// }

// BOOST_AUTO_TEST_CASE(test_correct_estimated_deflection) {
//   flex::FlexSettings observed_settings = makeObservedSettings();
//   flex::Flex postureEstimator(observed_settings);

//   flex::eVectorX tau = flex::eVectorX::Zero(6);
//   flex::eVectorX q = flex::eVectorX::Zero(6);
//   flex::eVectorX dq = flex::eVectorX::Zero(6);
//   flex::eVectorX zero_q = flex::eVectorX::Zero(6);
//   flex::eVectorX zero_dq = flex::eVectorX::Zero(6);

//   // Checking no deflection due to yawl torque
//   tau(0) = 55; tau(3) = 55;
//   postureEstimator.correctEstimatedDeflections(tau, q, dq, Eigen::Array3i(0,
//   1, 2), Eigen::Array3i(3, 4, 5));

//   BOOST_CHECK_EQUAL(q, zero_q);
//   BOOST_CHECK_EQUAL(dq, zero_dq);
//   postureEstimator.resetLeftFlex0(); postureEstimator.resetRightFlex0();
//   q = flex::eVectorX::Zero(6); dq = flex::eVectorX::Zero(6); tau =
//   flex::eVectorX::Zero(6);

//   // Checking deflection from to roll torque
//   tau(1) = 55; tau(4) = 55;
//   postureEstimator.correctEstimatedDeflections(tau, q, dq, Eigen::Array3i(0,
//   1, 2), Eigen::Array3i(3, 4, 5));

//   BOOST_CHECK_EQUAL(q(0), zero_q(0));
//   BOOST_CHECK(q(1) != zero_q(1));
//   BOOST_CHECK_EQUAL(q(2), zero_q(2));
//   BOOST_CHECK_EQUAL(q(3), zero_q(3));
//   BOOST_CHECK(q(4) != zero_q(4));
//   BOOST_CHECK_EQUAL(q(5), zero_q(5));
//   postureEstimator.resetLeftFlex0(); postureEstimator.resetRightFlex0();
//   q = flex::eVectorX::Zero(6); dq = flex::eVectorX::Zero(6); tau =
//   flex::eVectorX::Zero(6);

//   // Checking deflection from pitch torque
//   tau(2) = 55; tau(5) = 55;
//   postureEstimator.correctEstimatedDeflections(tau, q, dq, Eigen::Array3i(0,
//   1, 2), Eigen::Array3i(3, 4, 5));

//   BOOST_CHECK_EQUAL(q(0), zero_q(0));
//   BOOST_CHECK_EQUAL(q(1), zero_q(1));
//   BOOST_CHECK(q(2) != zero_q(2));
//   BOOST_CHECK_EQUAL(q(3), zero_q(3));
//   BOOST_CHECK_EQUAL(q(4), zero_q(4));
//   BOOST_CHECK(q(5) != zero_q(5));
// }
