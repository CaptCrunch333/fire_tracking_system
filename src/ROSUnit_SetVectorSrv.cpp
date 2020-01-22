#include "ROSUnit_SetVectorSrv.hpp"

//ROSUnit_SetVectorSrv* ROSUnit_SetVectorSrv::m_ptr;

ROSUnit_SetVectorSrv::ROSUnit_SetVectorSrv(std::string t_name, ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)
{
    //m_server = t_main_handler.advertiseService(t_name, srv_callback);
    //m_ptr = this;
}

ROSUnit_SetVectorSrv::~ROSUnit_SetVectorSrv()
{

}

bool ROSUnit_SetVectorSrv::srv_callback(common_srv::set_vector::Request& req, common_srv::set_vector::Response& res)
{
    VectorMsg t_msg;
    t_msg.p1.x = req.p1.x;
    t_msg.p1.y = req.p1.y;
    t_msg.p1.z = req.p1.z;
    t_msg.p2.x = req.p2.x;
    t_msg.p2.y = req.p2.y;
    t_msg.p2.z = req.p2.z;

    //m_ptr->emit_message((DataMessage*) &t_msg);
    return true;
}