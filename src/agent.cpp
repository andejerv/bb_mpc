#include "agent.h"

namespace sbmpc{

Agent::Agent(): integration_method(EULER){
    // Constructor
}

Agent::~Agent(){
    // Destructor
}


void Agent::setState(const Eigen::VectorXd & _state){
    state = _state;
}

Eigen::VectorXd Agent::getState(){
    return state;
}


Eigen::MatrixXd Agent::simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd & control_input){
    Eigen::MatrixXd states = Eigen::MatrixXd::Zero(this->state.rows(), num_steps+1); //state matrix, rows=num_states, cols=initial+num_steps

    // Integrate the system
    states.col(0) = this->state;

    switch (this->integration_method)
    {
    case RK4:
        for(int i = 1; i < num_steps+1; i++){
            Eigen::VectorXd k1 = step_size * this->dxdt(states.col(i-1), control_input);
            Eigen::VectorXd k2 = step_size * this->dxdt(states.col(i-1) + 0.5*k1, control_input);
            Eigen::VectorXd k3 = step_size * this->dxdt(states.col(i-1) + 0.5*k2, control_input);
            Eigen::VectorXd k4 = step_size * this->dxdt(states.col(i-1) + k3, control_input);

            states.col(i) = states.col(i-1) + (k1 + 2*k2 + 2*k3 + k4)/6.0;
        }

        return states;
        break;
    
    case EULER:
        for(int i = 1; i < num_steps+1; i++){
            states.col(i) = states.col(i-1) + step_size * this->dxdt(state, control_input);
        }

        return states;
    
    default:
        std::cout << "Integration method not supported" << std::endl;
        return states;
        break;
    }
}


BB_MODEL::BB_MODEL(double _x, double _y, double _heading, control_method _control_type): Agent(), control_type(_control_type){
    switch(this->control_type){
        case HEADING:
            this->state = Eigen::Vector2d(_x, _y);
            break;

        case YAW_RATE:
            this->state = Eigen::Vector3d(_x, _y, _heading);
            break;

        default:
            break;
    }
}

BB_MODEL::~BB_MODEL(){
    // Destructor
}


// Dynamic equations of the system
Eigen::VectorXd BB_MODEL::dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input){
    Eigen::VectorXd dxdt(state.rows());

    switch (this->control_type)
    {
    case HEADING:
        dxdt(0) = control_input(0) * cos(control_input(1)); // x
        dxdt(1) = control_input(0) * sin(control_input(1)); // y
        return dxdt;
        break;

    case YAW_RATE:
        dxdt(0) = control_input(0) * cos(state(2)); // x
        dxdt(1) = control_input(0) * sin(state(2)); // y
        dxdt(2) = control_input(1); // yaw-rate rad/sec
        return dxdt;
        break;
    
    default:
        std::cout << "Control method not supported" << std::endl;
        return dxdt;
        break;
    }
}

} // namespace sbmpc