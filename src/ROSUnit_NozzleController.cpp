#include "ROSUnit_NozzleController.hpp"

ROSUnit_NozzleController::ROSUnit_NozzleController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)
{
    m_NozzleController_srv = t_main_handler.advertiseService("/water_ext/set_nozzle_offset", service_cb);
}

ROSUnit_NozzleController::~ROSUnit_NozzleController()
{

}

bool ROSUnit_NozzleController::service_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

}