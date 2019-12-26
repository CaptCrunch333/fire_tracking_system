#define USING_CPP
#include "linux_serial_comm_device.hpp"
#include "BaseCommunication.hpp"
#include "Lepton3_5.hpp"
#include "HeatCenterProvider.hpp"
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
    NozzlePitch_UserReference* main_Pitch_UserRef = new NozzlePitch_UserReference;
    NozzleYaw_UserReference* main_Yaw_UserRef = new NozzleYaw_UserReference;
    // ********************************************************************************
    // ****************************  ORIENTATION PROVIDER *****************************
    NozzleOrientationProvider* main_orientation_provider = new NozzleOrientationProvider();
    Pitch_PVProvider* main_pitch_provider = (Pitch_PVProvider*) main_orientation_provider;
    PitchRate_PVProvider* main_pitch_rate_provider = (PitchRate_PVProvider*) main_orientation_provider;
    Yaw_PVProvider* main_yaw_provider = (Yaw_PVProvider*) main_orientation_provider;
    YawRate_PVProvider* main_yaw_rate_provider = (YawRate_PVProvider*) main_orientation_provider;
    // ********************************************************************************
    // *****************************  PID'S & REFERENCES ******************************
    Block* PID_pitch = new PIDController(block_id::PID_PITCH, block_type::controller);
    Block* PV_Ref_pitch = new ProcessVariableReference(block_id::REF_PITCH, block_type::reference);
    Block* PID_pitch_rate = new PIDController(block_id::PID_PITCH_RATE, block_type::controller);
    Block* PV_Ref_pitch_rate = new ProcessVariableReference(block_id::REF_PITCH_RATE, block_type::reference);
    Block* PID_yaw = new PIDController(block_id::PID_YAW, block_type::controller);
    Block* PV_Ref_yaw = new ProcessVariableReference(block_id::REF_YAW, block_type::reference);
    Block* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE, block_type::controller);
    Block* PV_Ref_yaw_rate = new ProcessVariableReference(block_id::REF_YAW_RATE, block_type::reference);
    // ********************************************************************************
    // *******************************  CONTROL SYSTEM ********************************
    ControlSystem* Pitch_ControlSystem = new ControlSystem(control_system::pitch, main_pitch_provider, block_frequency::hz10);
    Pitch_ControlSystem->addBlock(PID_pitch);
    Pitch_ControlSystem->addBlock(PV_Ref_pitch);
    ControlSystem* Pitch_Rate_ControlSystem = new ControlSystem(control_system::pitch_rate, main_pitch_rate_provider, block_frequency::hz100);
    Pitch_Rate_ControlSystem->addBlock(PID_pitch_rate);
    Pitch_Rate_ControlSystem->addBlock(PV_Ref_pitch_rate);
    ControlSystem* Yaw_ControlSystem = new ControlSystem(control_system::yaw, main_yaw_provider, block_frequency::hz10);
    Yaw_ControlSystem->addBlock(PID_yaw);
    Yaw_ControlSystem->addBlock(PV_Ref_yaw);
    ControlSystem* Yaw_Rate_ControlSystem = new ControlSystem(control_system::yaw_rate, main_yaw_rate_provider, block_frequency::hz100);
    Yaw_Rate_ControlSystem->addBlock(PID_yaw_rate);
    Yaw_Rate_ControlSystem->addBlock(PV_Ref_yaw_rate);
    // ********************************************************************************
    // *******************************  PID PARAMETERS ********************************
    PID_parameters* pid_para_pitch = new PID_parameters;
    pid_para_pitch->kp = 2.0;
    pid_para_pitch->ki = 0.0;
    pid_para_pitch->kd = 0.0;
    pid_para_pitch->kdd = 0.0;
    pid_para_pitch->anti_windup = 0;
    pid_para_pitch->en_pv_derivation = 0;
    Pitch_ControlSystem->changePIDSettings(pid_para_pitch);

    PID_parameters* pid_para_pitch_rate = new PID_parameters;
    pid_para_pitch_rate->kp = 2.0;
    pid_para_pitch_rate->ki = 0.0;
    pid_para_pitch_rate->kd = 0.0;
    pid_para_pitch_rate->kdd = 0.0;
    pid_para_pitch_rate->anti_windup = 0;
    pid_para_pitch_rate->en_pv_derivation = 0;
    Pitch_Rate_ControlSystem->changePIDSettings(pid_para_pitch_rate);
    
    PID_parameters* pid_para_yaw = new PID_parameters;
    pid_para_yaw->kp = 2.0;
    pid_para_yaw->ki = 0.0;
    pid_para_yaw->kd = 0.0;
    pid_para_yaw->kdd = 0.0;
    pid_para_yaw->anti_windup = 0;
    pid_para_yaw->en_pv_derivation = 0;
    Yaw_ControlSystem->changePIDSettings(pid_para_yaw);

    PID_parameters* pid_para_yaw_rate = new PID_parameters;
    pid_para_yaw_rate->kp = 2.0;
    pid_para_yaw_rate->ki = 0.0;
    pid_para_yaw_rate->kd = 0.0;
    pid_para_yaw_rate->kdd = 0.0;
    pid_para_yaw_rate->anti_windup = 0;
    pid_para_yaw_rate->en_pv_derivation = 0;
    Yaw_Rate_ControlSystem->changePIDSettings(pid_para_yaw_rate);  
    // ********************************************************************************
    // **************************  CONTROLLER OUTPUT BRIDGE ***************************
    //TODO: this needs to be removed after the controller output msg has been adjusted
    ControllerActuationBridge* main_ctrl_actuation_bridge = new ControllerActuationBridge();
    // ********************************************************************************
    // *****************************  SYSTEM CONNECTIONS ******************************
    myImageConverter->add_callback_msg_receiver((msg_receiver*) main_thermal_camera);
    main_thermal_camera->add_callback_msg_receiver((msg_receiver*) main_heatcenter_prov);
    main_heatcenter_prov->add_callback_msg_receiver((msg_receiver*) main_orientation_provider);
    main_comm_stack->add_callback_msg_receiver((msg_receiver*) main_orientation_provider);
    main_Pitch_UserRef->add_callback_msg_receiver((msg_receiver*) Pitch_ControlSystem);
    main_Yaw_UserRef->add_callback_msg_receiver((msg_receiver*) Yaw_ControlSystem);
    Pitch_ControlSystem->add_callback_msg_receiver((msg_receiver*) Pitch_Rate_ControlSystem);
    Yaw_ControlSystem->add_callback_msg_receiver((msg_receiver*) Yaw_Rate_ControlSystem);
    Pitch_Rate_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    Yaw_Rate_ControlSystem->add_callback_msg_receiver((msg_receiver*) main_ctrl_actuation_bridge);
    main_ctrl_actuation_bridge->add_callback_msg_receiver((msg_receiver*) main_comm_stack);
    // ********************************************************************************
    // ***********************************  LOOPER ************************************
    pthread_t loop100hz_func_id, loop10hz_func_id;
    Looper* main_looper = new Looper();
    main_looper->addTimedBlock((TimedBlock*) Pitch_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) Yaw_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) Pitch_Rate_ControlSystem);
    main_looper->addTimedBlock((TimedBlock*) Yaw_Rate_ControlSystem);
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