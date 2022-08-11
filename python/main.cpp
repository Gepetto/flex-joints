
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include "flex-joints/flexi-hips.hpp"
#include "flex-joints/fwd.hpp"

namespace flex {
namespace python {
namespace bp = boost::python;

void initialize(Flex &self, const bp::dict &settings) {
  FlexSettings conf;

  conf.left_stiffness = bp::extract<eVector2>(settings["left_stiffness"]);
  conf.left_damping = bp::extract<eVector2>(settings["left_damping"]);
  conf.right_stiffness = bp::extract<eVector2>(settings["right_stiffness"]);
  conf.right_damping = bp::extract<eVector2>(settings["right_damping"]);
  conf.flexToJoint = bp::extract<eVector3>(settings["flexToJoint"]);
  conf.dt = bp::extract<double>(settings["dt"]);
  conf.MA_duration = bp::extract<double>(settings["MA_duration"]);
  eVector3 left_hip_indices =
      bp::extract<eVector3>(settings["left_hip_indices"]);
  eVector3 right_hip_indices =
      bp::extract<eVector3>(settings["right_hip_indices"]);
  conf.filtered = bp::extract<bool>(settings["filtered"]);

  conf.left_hip_indices = (left_hip_indices.array()).cast<int>();
  conf.right_hip_indices = (right_hip_indices.array()).cast<int>();

  self.initialize(conf);
}

bp::dict get_settings(Flex &self) {
  bp::dict settings;
  FlexSettings conf = self.getSettings();
  settings["left_stiffness"] = conf.left_stiffness;
  settings["left_damping"] = conf.left_damping;
  settings["right_stiffness"] = conf.right_stiffness;
  settings["right_damping"] = conf.right_damping;
  settings["flexToJoint"] = conf.flexToJoint;
  settings["dt"] = conf.dt;
  settings["filtered"] = conf.filtered;
  settings["MA_duration"] = conf.MA_duration;
  settings["left_hip_indices"] = Eigen::Vector3i(conf.left_hip_indices);
  settings["right_hip_indices"] = Eigen::Vector3i(conf.right_hip_indices);

  return settings;
}

bp::tuple correctEstimatedDeflections(Flex &self, const eVectorX &desiredTorque,
                                      const eVectorX &q, const eVectorX &dq) {
  eVectorX correct_q(q.size()), correct_dq(dq.size());
  correct_q << q;
  correct_dq << dq;
  self.correctEstimatedDeflections(desiredTorque, correct_q, correct_dq);
  return bp::make_tuple(correct_q, correct_dq);
}

bp::tuple correctEstimatedDeflections(Flex &self, const eVectorX &desiredTorque,
                                      const eVectorX &q, const eVectorX &dq,
                                      const eVector3 &leftForce,
                                      const eVector3 &rightForce) {
  eVectorX correct_q(q.size()), correct_dq(dq.size());
  correct_q << q;
  correct_dq << dq;
  self.correctEstimatedDeflections(desiredTorque, correct_q, correct_dq,
                                   leftForce, rightForce);
  return bp::make_tuple(correct_q, correct_dq);
}

bp::tuple correctDeflections(Flex &self, const eVector2 &leftFlexingTorque,
                             const eVector2 &rightFlexingTorque,
                             const eVectorX &q, const eVectorX &dq) {
  eVectorX correct_q(q.size()), correct_dq(dq.size());
  correct_q << q;
  correct_dq << dq;
  self.correctDeflections(leftFlexingTorque, rightFlexingTorque, correct_q,
                          correct_dq);
  return bp::make_tuple(correct_q, correct_dq);
}

const eVector2 &computeDeflection(Flex &self, const eVector2 &torques,
                                  const eVector2 &delta0,
                                  const eVector2 &stiffness,
                                  const eVector2 &damping, double dt) {
  return self.computeDeflection(torques.array(), delta0.array(),
                                stiffness.array(), damping.array(), dt);
}

eVector2 get_sum_LH(Flex &self) { return self.get_summation_LH().matrix(); }
eVector2 get_sum_RH(Flex &self) { return self.get_summation_RH().matrix(); }

void exposeFlex() {
  bp::class_<Flex>("Flex", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings"))
      .def("Settings", &get_settings, bp::args("self"))
      .def("computeDeflection",
           bp::make_function(
               &computeDeflection,
               bp::return_value_policy<bp::reference_existing_object>(),
               bp::args("self", "torques", "delta0", "stiffness", "damping",
                        "dt")))
      .def("currentFlexToJoint",
           bp::make_function(
               &Flex::currentFlexToJoint,
               bp::return_value_policy<bp::reference_existing_object>(),
               bp::args("self", "delta")))
      .def("estimateFlexingTorque",
           bp::make_function<const eVector2 &(Flex::*)(const eVector3 &,
                                                       const eVector3 &)>(
               &Flex::estimateFlexingTorque,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("estimateFlexingTorque",
           bp::make_function<const eVector2 &(
               Flex::*)(const eVector3 &, const eVector3 &, const eVector2 &,
                        const eVector3 &)>(
               &Flex::estimateFlexingTorque,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("correctDeflections", &correctDeflections,
           bp::args("self", "leftFlexingTorque", "rightFlexingTorque", "q",
                    "dq"))
      .def<bp::tuple(Flex &, const eVectorX &, const eVectorX &,
                     const eVectorX &)>(
          "correctEstimatedDeflections", &correctEstimatedDeflections,
          bp::args("self", "desiredTorque", "q", "dq"))
      .def<bp::tuple(Flex &, const eVectorX &, const eVectorX &,
                     const eVectorX &, const eVector3 &, const eVector3 &)>(
          "correctEstimatedDeflections", &correctEstimatedDeflections,
          bp::args("self", "desiredTorque", "q", "dq", "leftForce",
                   "rightForce"))
      .add_property(
          "leftFlex0",
          bp::make_function(
              &Flex::getLeftFlex0,
              bp::return_value_policy<bp::reference_existing_object>()),
          &Flex::setLeftFlex0)
      .add_property(
          "rightFlex0",
          bp::make_function(
              &Flex::getRightFlex0,
              bp::return_value_policy<bp::reference_existing_object>()),
          &Flex::setRightFlex0)
      .def("reset", &Flex::reset, bp::args("self"))
      .def("get_sum_LH", &get_sum_LH)
      .def("get_sum_RH", &get_sum_RH)
      // .def("get_queue_LH", bp::make_function(&Flex::get_queue_LH,
      //               bp::return_value_policy<bp::reference_existing_object>()))
      // .def("get_queue_RH", bp::make_function(&Flex::get_queue_RH,
      //               bp::return_value_policy<bp::reference_existing_object>()))
      ;
}
}  // namespace python
}  // namespace flex

BOOST_PYTHON_MODULE(flex_joints) {
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Vector3i);
  flex::python::exposeFlex();
}
