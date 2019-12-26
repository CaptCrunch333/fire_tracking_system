#include "PitchRate_PVProvider.hpp"

PitchRate_PVProvider::PitchRate_PVProvider() 
{

}

PitchRate_PVProvider::~PitchRate_PVProvider() 
{

}

Vector3D<float> PitchRate_PVProvider::getProcessVariable()
{
    Vector3D<float> t_process_variable;
    t_process_variable.x = (this->getBodyRate()).y;
    t_process_variable.y = 0.0; //TODO yaw_dot
    t_process_variable.z = 0.0; //TODO yaw_dot_dot

    return t_process_variable;
    
}