#define USING_CPP
#include "linux_serial_comm_device.hpp"
#include "BaseCommunication.hpp"
#include "CommChecker.hpp"
#include "Lepton3_5.hpp"
#include "HeatCenterProvider.hpp"
#include "GyroOrientationObserver.hpp"
#include "ComplementaryFilter.hpp"
#include "DataMessage.hpp"
#include "ImageConverter.hpp"
#include "NozzlePitch_UserReference.hpp"
#include "NozzleYaw_UserReference.hpp"
#include "ControlSystem.hpp"
#include "PIDController.hpp"
#include "NozzleOrientationProvider.hpp"
#include "std_logger.hpp"
#include "ControllerActuationBridge.hpp"
#include "looper.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testing_node");

    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ************************************ LOGGER ************************************
    Logger::assignLogger(new StdLogger());
    Logger::getAssignedLogger()->log("start of logger", LoggerLevel::Info);
    Logger::getAssignedLogger()->enableFileLog(LoggerLevel::Error);
    // ********************************************************************************
    // ***************************** COMMUNICATION DEVICE *****************************
    LinuxSerialCommDevice* main_comm_dev = new LinuxSerialCommDevice;
    BaseCommunication* main_comm_stack = new BaseCommunication((CommDevice*) main_comm_dev);
    std::string port_add = "/dev/ttyACM0";
    CommChecker* main_checker = new CommChecker(main_comm_dev, (void*) &port_add);
    // ********************************************************************************
    // *************************** THERMAL IMAGE PROVIDERS ****************************
    ROSUnit* myImageConverter = new ImageConverter("/lepton_topic", nh);
    ThermalCam* main_thermal_camera = new Lepton3_5();
    HeatCenterProvider* main_heatcenter_prov = new HeatCenterProvider();
    main_heatcenter_prov->setCutOffTemperature(90.f);
    // ********************************************************************************
    // ******************************* USER REFERENCES ********************************
    NozzlePitch_UserReference* main_Pitch_UserRef = new NozzlePitch_UserReference;
    NozzleYaw_UserReference* main_Yaw_UserRef = new NozzleYaw_UserReference;
    // ********************************************************************************
    // ****************************  ORIENTATION PROVIDER *****************************
    NozzleOrientationProvider* main_orientation_provider = new NozzleOrientationProvider();
    Pitch_PVProvider* cam_pitch_provider = (Pitch_PVProvider*) main_orientation_provider;
    Yaw_PVProvider* cam_yaw_provider = (Yaw_PVProvider*) main_orientation_provider;
    DataFilter* pitch_filter = new ComplementaryFilter();
    DataFilter* roll_filter = new ComplementaryFilter();
    DataFilter* yaw_filter = new ComplementaryFilter();
    FilterSettings* common_filter_settings = new ComplementaryFilterSettings(false, 0.01, 0.98);
    main_orientation_provider->setFilterType(pitch_filter, roll_filter, yaw_filter);
    main_orientation_provider->updateSettings(common_filter_settings);
    // ********************************************************************************
    // *****************************  PID'S & REFERENCES ******************************
    Block* PID_cam_pitch = new PIDController(block_id::PID_PITCH, block_type::controller);
    Block* PV_Ref_cam_pitch = new ProcessVariableReference(block_id::REF_PITCH, block_type::reference);
    Block* PID_cam_yaw = new PIDController(block_id::PID_YAW, block_type::controller);
    Block* PV_Ref_cam_yaw = new ProcessVariableReference(block_id::REF_YAW, block_type::reference);
    // ********************************************************************************
    // *******************************  CONTROL SYSTEM ********************************
    ControlSystem* CamPitch_ControlSystem = new ControlSystem(control_system::pitch, cam_pitch_provider, block_frequency::hz100);
    CamPitch_ControlSystem->addBlock(PID_cam_pitch);
    CamPitch_ControlSystem->addBlock(PV_Ref_cam_pitch);
    ControlSystem* CamYaw_ControlSystem = new ControlSystem(control_system::yaw, cam_yaw_provider, block_frequency::hz100);
    CamYaw_ControlSystem->addBlock(PID_cam_yaw);
    CamYaw_ControlSystem->addBlock(PV_Ref_cam_yaw);
    // ********************************************************************************
    // *******************************  PID PARAMETERS ********************************
    PID_parameters* pid_para_cam_pitch = new PID_parameters;
    pid_para_cam_pitch->kp = 0.0;
    pid_para_cam_pitch->ki = 0.8;
    pid_para_cam_pitch->kd = 0.0;
    pid_para_cam_pitch->kdd = 0.0;
    pid_para_cam_pitch->anti_windup = 0.55;
    pid_para_cam_pitch->en_pv_derivation = 0;
    CamPitch_ControlSystem->changePIDSettings(pid_para_cam_pitch);
    
    PID_parameters* pid_para_cam_yaw = new PID_parameters;
    pid_para_cam_yaw->kp = 0.0;
    pid_para_cam_yaw->ki = 1.5;
    pid_para_cam_yaw->kd = 0.0;
    pid_para_cam_yaw->kdd = 0.0;
    pid_para_cam_yaw->anti_windup = 0.55;
    pid_para_cam_yaw->en_pv_derivation = 0;
    CamYaw_ControlSystem->changePIDSettings(pid_para_cam_yaw);
    // ********************************************************************************
    // **************************  CONTROLLER OUTPUT BRIDGE ***************************
    //TODO: this needs to be removed after the controller output msg has been adjusted
    ControllerActuationBridge* main_ctrl_actuation_bridge = new ControllerActuationBridge();
    // ********************************************************************************
    // *****************************  SYSTEM CONNECTIONS ******************************
    main_checker->add_callback_msg_receiver((msg_receiver*) main_comm_stack);
    main_comm_stack->add_callback_msg_receiver((msg_receiver*) main_checker);
    myImageConverter->add_callback_msg_receiver((msg_receiver*) main_thermal_camera);
    main_thermal_camera->add_callback_msg_receiver((msg_receiver*) main_heatcenter_prov);
    main_heatcenter_prov->add_callback_msg_receiver((msg_receiver*) main_orientation_provider);
    main_comm_stack->add_callback_msg_receiver((msg_receiver*) main_orientation_provider);
    main_Pitch_UserRef->add_callback_msg_receiver((msg_receiver*) CamPitch_ControlSystem);
    main_Yaw_UserRef->add_callback_msg_receiver((msg_receiver*) CamYaw_ControlSystem);
    CamPitch_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    CamYaw_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    main_ctrl_actuation_bridge->add_callback_msg_receiver((msg_receiver*) main_comm_stack);
    // ********************************************************************************
    // ************************ Initialize Reference To Zero **************************
    msg_emitter tmp_emitter;
    UpdateNozzlePoseMessage tmp_user_ref_msg;
    tmp_user_ref_msg.setNozzlePoseMessage(0,0);
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) main_Yaw_UserRef);
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) main_Pitch_UserRef);
    tmp_emitter.emit_message((DataMessage*) &tmp_user_ref_msg);
    // ***********************************  LOOPER ************************************
    // ********************************************************************************
    pthread_t loop100hz_func_id;
    Looper* main_looper = new Looper();
    main_looper->addTimedBlock((TimedBlock*) CamPitch_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) CamYaw_ControlSystem);
    pthread_create(&loop100hz_func_id, NULL, &Looper::Loop100Hz, NULL);
    // ********************************************************************************
    
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}