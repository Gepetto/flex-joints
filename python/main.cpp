
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
  conf.dt = bp::extract<double>(settings["dt"]);
  conf.MA_duration = bp::extract<double>(settings["MA_duration"]);
  eVector3 left_hip_indices =
      bp::extract<eVector3>(settings["left_hip_indices"]);
  eVector3 right_hip_indices =
      bp::extract<eVector3>(settings["right_hip_indices"]);

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
  settings["dt"] = conf.dt;
  settings["MA_duration"] = conf.MA_duration;
  settings["left_hip_indices"] = eVector3(conf.left_hip_indices.cast<double>());
  settings["right_hip_indices"] =
      eVector3(conf.right_hip_indices.cast<double>());

  ///@todo: Here I am casting the Eigen::Vector3i into eVector 3 for
  /// python to recognize it. Later implement the translation for
  /// Eigen::Vector3i. or use a different representation of indices.

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

void exposeFlex() {
  bp::class_<Flex>("Flex", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings"))
      .def("Settings", &get_settings, bp::args("self"))
      .def("correctEstimatedDeflections", &correctEstimatedDeflections,
           bp::args("self", "desiredTorque", "q", "dq"));
}
}  // namespace python
}  // namespace flex

BOOST_PYTHON_MODULE(flex_joints) {
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXi);  // (look at the @todo above)
  flex::python::exposeFlex();
}
