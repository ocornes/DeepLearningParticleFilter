#include <iostream>

#include <Eigen/Dense>

#include "refill/distributions/gaussian_distribution.h"
#include "refill/filters/extended_kalman_filter.h"
#include "refill/measurement_models/linear_measurement_model.h"
#include "refill/system_models/linear_system_model.h"

int main(int argc, char **argv) {
    if(argc != 1) {
        std::cout << argv[0] <<  "takes no arguments.\n";
        return 1;
    }

    // This is just a temporary test implementation of a Kalman filter.
    refill::GaussianDistribution noise(2);
    refill::LinearSystemModel sys_model(Eigen::Matrix2d::Identity(), noise);
    refill::LinearMeasurementModel meas_model(Eigen::Matrix2d::Identity(), noise);
    refill::GaussianDistribution init_state(2);

    refill::ExtendedKalmanFilter filter(init_state);

    filter.predict(sys_model);

    std::cout << "Predicted one step." << std::endl;
    return 0;
}
