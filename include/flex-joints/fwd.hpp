
#ifndef FLEX_FWD_HPP_
#define FLEX_FWD_HPP_

#include <Eigen/Dense>
namespace flex {

typedef Eigen::Vector2d eVector2;
typedef Eigen::Array2d eArray2;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::Vector3d eVector3;
typedef Eigen::VectorXd eVectorX;
typedef Eigen::Matrix2d eMatrix2;

namespace python {
void exposeFlex();
}
}  // namespace flex

#endif  // FLEX_FWD_HPP_
