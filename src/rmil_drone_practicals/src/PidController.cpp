/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...
  double u = 0.0;
  double delta_time = 1e-6*timestampMicroseconds - lastTimestampMicroseconds_;
  lastTimestampMicroseconds_ = 1e-6*timestampMicroseconds;
  if (delta_time < 0.1) {
    integratedError_ += e*delta_time;
    u = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;
    if(u>maxOutput_) {
      integratedError_ = integratedError_ - e*delta_time;
    }
  }
    
  else {
      u = parameters_.k_p * e + parameters_.k_d * e_dot;
    }
  
  

  if (u > maxOutput_) {
      u = maxOutput_;
  }
  else if (u < -maxOutput_) {
      u = -maxOutput_;
  }
    return u;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
