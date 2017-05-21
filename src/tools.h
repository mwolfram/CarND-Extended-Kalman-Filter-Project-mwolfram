#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * @brief Tools::h, h function for mapping the x state vector to radar measurements
   * @param x_state
   * @return VectorXd a 3D vector containing the converted values
   */
  Eigen::VectorXd h(const Eigen::VectorXd& x_state);

  /**
   * @brief Tools::normalize
   * @param angle_rad
   * @return angle in radians normalized to range between -pi and pi
   */
  float normalize(const float angle_rad);

};

#endif /* TOOLS_H_ */
