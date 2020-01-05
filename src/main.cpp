#define USING_CPP
#include "linux_serial_comm_device.hpp"
#include "BaseCommunication.hpp"
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
    Logger::getAssignedLogger()->log("start of logger", LoggerLevel::Warning);
    Logger::getAssignedLogger()->enableFileLog(LoggerLevel::Error);
    // ********************************************************************************
    // ***************************** COMMUNICATION DEVICE *****************************
    LinuxSerialCommDevice* main_comm_dev = new LinuxSerialCommDevice;
    BaseCommunication* main_comm_stack = new BaseCommunication((CommDevice*) main_comm_dev);
    std::string port_add = "/dev/ttyACM0";
    main_comm_dev->attach_hardware_sender((void*) &(port_add));
    // ********************************************************************************
    // *************************** THERMAL IMAGE PROVIDERS ****************************
    ROSUnit* myImageConverter = new ImageConverter("/lepton_topic", nh);
    ThermalCam* main_thermal_camera = new Lepton3_5();
    HeatCenterProvider* main_heatcenter_prov = new HeatCenterProvider();
    main_heatcenter_prov->setCutOffTemperature(90.f);
    // ********************************************************************************
    // ******************************* USER REFERENCES ********************************
    msg_emitter tmp_emitter;
    UpdatePoseMessage tmp_user_ref_msg;
    NozzlePitch_UserReference* main_Pitch_UserRef = new NozzlePitch_UserReference;
    tmp_user_ref_msg.setPoseMessage(0,0,0,0);
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) main_Pitch_UserRef);
    NozzleYaw_UserReference* main_Yaw_UserRef = new NozzleYaw_UserReference;
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) main_Yaw_UserRef);
    // ********************************************************************************
    // *****************************  GYRO ANGLES PROVIDER *******************************
    GyroOrientationObserver* main_gyro_angles_provider = new GyroOrientationObserver();
    DataFilter* pitch_filter = new ComplementaryFilter();
    DataFilter* roll_filter = new ComplementaryFilter();
    DataFilter* yaw_filter = new ComplementaryFilter();
    FilterSettings* common_filter_settings = new ComplementaryFilterSettings(false, 0.01, 1);
    main_gyro_angles_provider->setFilterType(pitch_filter, roll_filter, yaw_filter);
    main_gyro_angles_provider->updateSettings(common_filter_settings);
    // ********************************************************************************
    // ****************************  ORIENTATION PROVIDER *****************************
    NozzleOrientationProvider* main_orientation_provider = new NozzleOrientationProvider();
    Pitch_PVProvider* cam_pitch_provider = (Pitch_PVProvider*) main_orientation_provider;
    Pitch_PVProvider* nozzle_pitch_provider = (Pitch_PVProvider*) main_gyro_angles_provider;
    Yaw_PVProvider* cam_yaw_provider = (Yaw_PVProvider*) main_orientation_provider;
    Yaw_PVProvider* nozzle_yaw_provider = (Yaw_PVProvider*) main_gyro_angles_provider;
    // ********************************************************************************
    // *****************************  PID'S & REFERENCES ******************************
    Block* PID_cam_pitch = new PIDController(block_id::PID_PITCH, block_type::controller);
    Block* PV_Ref_cam_pitch = new ProcessVariableReference(block_id::REF_PITCH, block_type::reference);
    Block* PID_nozzle_pitch = new PIDController(block_id::PID_PITCH_RATE, block_type::controller);
    Block* PV_Ref_nozzle_pitch = new ProcessVariableReference(block_id::REF_PITCH_RATE, block_type::reference);
    Block* PID_cam_yaw = new PIDController(block_id::PID_YAW, block_type::controller);
    Block* PV_Ref_cam_yaw = new ProcessVariableReference(block_id::REF_YAW, block_type::reference);
    Block* PID_nozzle_yaw = new PIDController(block_id::PID_YAW_RATE, block_type::controller);
    Block* PV_Ref_nozzle_yaw = new ProcessVariableReference(block_id::REF_YAW_RATE, block_type::reference);
    // ********************************************************************************
    // *******************************  CONTROL SYSTEM ********************************
    ControlSystem* CamPitch_ControlSystem = new ControlSystem(control_system::pitch, cam_pitch_provider, block_frequency::hz10);
    CamPitch_ControlSystem->addBlock(PID_cam_pitch);
    CamPitch_ControlSystem->addBlock(PV_Ref_cam_pitch);
    ControlSystem* NozzlePitch_ControlSystem = new ControlSystem(control_system::pitch, nozzle_pitch_provider, block_frequency::hz100);
    NozzlePitch_ControlSystem->addBlock(PID_nozzle_pitch);
    NozzlePitch_ControlSystem->addBlock(PV_Ref_nozzle_pitch);
    ControlSystem* CamYaw_ControlSystem = new ControlSystem(control_system::yaw, cam_yaw_provider, block_frequency::hz10);
    CamYaw_ControlSystem->addBlock(PID_cam_yaw);
    CamYaw_ControlSystem->addBlock(PV_Ref_cam_yaw);
    ControlSystem* NozzleYaw_ControlSystem = new ControlSystem(control_system::yaw, nozzle_yaw_provider, block_frequency::hz100);
    NozzleYaw_ControlSystem->addBlock(PID_nozzle_yaw);
    NozzleYaw_ControlSystem->addBlock(PV_Ref_nozzle_yaw);
    // ********************************************************************************
    // *******************************  PID PARAMETERS ********************************
    PID_parameters* pid_para_cam_pitch = new PID_parameters;
    pid_para_cam_pitch->kp = 0.0;
    pid_para_cam_pitch->ki = 03.0;
    pid_para_cam_pitch->kd = 0.0;
    pid_para_cam_pitch->kdd = 0.0;
    pid_para_cam_pitch->anti_windup = 0;
    pid_para_cam_pitch->en_pv_derivation = 0;
    CamPitch_ControlSystem->changePIDSettings(pid_para_cam_pitch);

    PID_parameters* pid_para_nozzle_pitch = new PID_parameters;
    pid_para_nozzle_pitch->kp = 0.05;
    pid_para_nozzle_pitch->ki = 0;
    pid_para_nozzle_pitch->kd = 0.0;
    pid_para_nozzle_pitch->kdd = 0.0;
    pid_para_nozzle_pitch->anti_windup = 0;
    pid_para_nozzle_pitch->en_pv_derivation = 0;
    NozzlePitch_ControlSystem->changePIDSettings(pid_para_nozzle_pitch);
    
    PID_parameters* pid_para_cam_yaw = new PID_parameters;
    pid_para_cam_yaw->kp = 0.0;
    pid_para_cam_yaw->ki = 03.0;
    pid_para_cam_yaw->kd = 0.0;
    pid_para_cam_yaw->kdd = 0.0;
    pid_para_cam_yaw->anti_windup = 0;
    pid_para_cam_yaw->en_pv_derivation = 0;
    CamYaw_ControlSystem->changePIDSettings(pid_para_cam_yaw);

    PID_parameters* pid_para_nozzle_yaw = new PID_parameters;
    pid_para_nozzle_yaw->kp = 0.05;
    pid_para_nozzle_yaw->ki = 0.0;
    pid_para_nozzle_yaw->kd = 0.0;
    pid_para_nozzle_yaw->kdd = 0.0;
    pid_para_nozzle_yaw->anti_windup = 0;
    pid_para_nozzle_yaw->en_pv_derivation = 0;
    NozzleYaw_ControlSystem->changePIDSettings(pid_para_nozzle_yaw);  
    // ********************************************************************************
    // **************************  CONTROLLER OUTPUT BRIDGE ***************************
    //TODO: this needs to be removed after the controller output msg has been adjusted
    ControllerActuationBridge* main_ctrl_actuation_bridge = new ControllerActuationBridge();
    // ********************************************************************************
    // *****************************  SYSTEM CONNECTIONS ******************************
    myImageConverter->add_callback_msg_receiver((msg_receiver*) main_thermal_camera);
    main_thermal_camera->add_callback_msg_receiver((msg_receiver*) main_heatcenter_prov);
    main_heatcenter_prov->add_callback_msg_receiver((msg_receiver*) main_orientation_provider);
    main_comm_stack->add_callback_msg_receiver((msg_receiver*) main_gyro_angles_provider);
    main_Pitch_UserRef->add_callback_msg_receiver((msg_receiver*) CamPitch_ControlSystem);
    main_Yaw_UserRef->add_callback_msg_receiver((msg_receiver*) CamYaw_ControlSystem);
    CamPitch_ControlSystem->add_callback_msg_receiver((msg_receiver*) NozzlePitch_ControlSystem);
    CamYaw_ControlSystem->add_callback_msg_receiver((msg_receiver*) NozzleYaw_ControlSystem);
    NozzlePitch_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    NozzleYaw_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    //CamPitch_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    //CamRate_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    main_ctrl_actuation_bridge->add_callback_msg_receiver((msg_receiver*) main_comm_stack);
    // ********************************************************************************
    // ***********************************  LOOPER ************************************
    pthread_t loop100hz_func_id, loop10hz_func_id;
    Looper* main_looper = new Looper();
    main_looper->addTimedBlock((TimedBlock*) CamPitch_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) CamYaw_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) NozzlePitch_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) NozzleYaw_ControlSystem);
    pthread_create(&loop100hz_func_id, NULL, &Looper::Loop100Hz, NULL);
    pthread_create(&loop10hz_func_id, NULL, &Looper::Loop10Hz, NULL);
    // ********************************************************************************
    
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}