#pragma once

#include "utilities.h"

#include <eigen3/Eigen/Dense>

namespace sbmpc{

class Agent
{
    private:
        virtual Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) = 0;

    protected:
        Eigen::VectorXd state;
        IntegrationMethod integration_method;

    public:
        Agent();
        ~Agent();

        void setState(const Eigen::VectorXd& _state);
        Eigen::VectorXd getState();

        Eigen::MatrixXd simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd& control_input);

};


// BlueBoat specific code

enum control_method
{
    HEADING,
    YAW_RATE
};

class BB_MODEL: public Agent
{   
    private:
        control_method control_type;
        Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) override;

    public:
        BB_MODEL(double _x, double _y, double _heading, control_method _control_type);
        ~BB_MODEL();

};

} // namespace sbmpc