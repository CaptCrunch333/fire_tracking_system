// #include "PVProvider.hpp"

// PVProvider::PVProvider(AttitudeProvider* t_attitude_provider, 
//                                             PositioningProvider* t_position_provider,
//                                             HeadingProvider* t_heading_provider,
//                                             VelocityProvider* t_velocity_provider,
//                                             AccelerationProvider* t_acceleration_provider) {

//     _attitude_provider = t_attitude_provider;
//     _position_provider = t_position_provider;
//     _heading_provider = t_heading_provider;
//     _velocity_provider = t_velocity_provider;
//     _acceleration_provider = t_acceleration_provider;

// }

// PVProvider::~PVProvider() {

// }

// Vector3D<float> PVProvider::getProcessVariable(control_system t_control_system){

//     Vector3D<float> process_variable;

//     switch (t_control_system)
//     {
//     case control_system::x:
//     {
//         process_variable.x = _position_provider->getPosition().x;
//         process_variable.y = _velocity_provider->getVelocity().dx; 
//         process_variable.z = _acceleration_provider->getAcceleration().ddx; 
//         break;
//     }
//     case control_system::y:
//     {
//         process_variable.x = _position_provider->getPosition().y;
//         process_variable.y = _velocity_provider->getVelocity().dy; 
//         process_variable.z = _acceleration_provider->getAcceleration().ddy; 
//         break;
//     }
//     case control_system::z:
//     {
//         process_variable.x = _position_provider->getPosition().z;
//         process_variable.y = _velocity_provider->getVelocity().dz; 
//         process_variable.z = _acceleration_provider->getAcceleration().ddz;
//         break;
//     }
//     case control_system::roll:
//     {
//         process_variable.x = _attitude_provider->getAttitude().roll;
//         process_variable.y = 0.0; //TODO roll_dot
//         process_variable.z = 0.0; //TODO roll_dot_dot
//         break;
//     }
//     case control_system::pitch:
//     {
//         process_variable.x = _attitude_provider->getAttitude().pitch;
//         process_variable.y = 0.0; //TODO pitch_dot
//         process_variable.z = 0.0; //TODO pitch_dot_dot
//         break;
//     }
//     case control_system::yaw:
//     {
//         process_variable.x = _heading_provider->getHeading().yaw;
//         process_variable.y = 0.0; //TODO yaw_dot
//         process_variable.z = 0.0; //TODO yaw_dot_dot
//         break;
//     }
//     default:
//         break;
//     }

//     return process_variable;

// }