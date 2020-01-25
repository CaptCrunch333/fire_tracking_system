#define USING_CPP
#include "linux_serial_comm_device.hpp"
#include "BaseCommunication.hpp"
#include "CommChecker.hpp"
#include "Lepton3_5.hpp"
#include "HeatCenterProvider.hpp"
#include "ComplementaryFilter.hpp"
#include "ImageConverter.hpp"
#include "NozzlePitch_UserReference.hpp"
#include "NozzleYaw_UserReference.hpp"
#include "ControlSystem.hpp"
#include "PIDController.hpp"
#include "NozzleOrientationProvider.hpp"
#include "std_logger.hpp"
#include "ControllerActuationBridge.hpp"
#include "looper.hpp"
#include "ROSUnit_Factory.hpp"
#include "PumpController.hpp"
#include "FireAssessor.hpp"
#include "PumpRosBridge.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testing_node");

    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ************************************ LOGGER ************************************
    Logger::assignLogger(new StdLogger());
    Logger::getAssignedLogger()->log("start of logger", LoggerLevel::Info);
    //Logger::getAssignedLogger()->enableFileLog(LoggerLevel::Error);
    // ********************************************************************************
    // ***************************** COMMUNICATION DEVICE *****************************
    LinuxSerialCommDevice* mainCommDevice = new LinuxSerialCommDevice;
    BaseCommunication* mainCommStack = new BaseCommunication((CommDevice*) mainCommDevice);
    std::string port_add = "/dev/NozzleMC";
    CommChecker* mainCommChecker = new CommChecker(mainCommDevice, (void*) &port_add, block_frequency::hz10);
    // ********************************************************************************
    // *************************** THERMAL IMAGE PROVIDERS ****************************
    ROSUnit* mainImageConverter = new ImageConverter("/lepton_topic", nh);
    ThermalCam* mainThermalCamera = new Lepton3_5();
    HeatCenterProvider* mainHeatcenterProv = new HeatCenterProvider();
    mainHeatcenterProv->setCutOffTemperature(90.f);
    // ********************************************************************************
    // ******************************* USER REFERENCES ********************************
    NozzlePitch_UserReference* mainPitchUserRef = new NozzlePitch_UserReference;
    NozzleYaw_UserReference* mainYawUserRef = new NozzleYaw_UserReference;
    // ********************************************************************************
    // ***************************** ORIENTATION PROVIDER *****************************
    NozzleOrientationProvider* mainOrientationProvider = new NozzleOrientationProvider();
    Pitch_PVProvider* camPitchProvider = (Pitch_PVProvider*) mainOrientationProvider;
    Yaw_PVProvider* camYawProvider = (Yaw_PVProvider*) mainOrientationProvider;
    DataFilter* pitchFilter = new ComplementaryFilter();
    DataFilter* rollFilter = new ComplementaryFilter();
    DataFilter* yawFilter = new ComplementaryFilter();
    FilterSettings* commonFilterSettings = new ComplementaryFilterSettings(false, 0.01, 0.98);
    mainOrientationProvider->setFilterType(pitchFilter, rollFilter, yawFilter);
    mainOrientationProvider->updateSettings(commonFilterSettings);
    // ********************************************************************************
    // ****************************** PID'S & REFERENCES ******************************
    Block* camPitchPID = new PIDController(block_id::PID_PITCH, block_type::controller);
    Block* camPitchPVRef = new ProcessVariableReference(block_id::REF_PITCH, block_type::reference);
    Block* camYawPID = new PIDController(block_id::PID_YAW, block_type::controller);
    Block* camYawPVRef = new ProcessVariableReference(block_id::REF_YAW, block_type::reference);
    // ********************************************************************************
    // ******************************** CONTROL SYSTEM ********************************
    ControlSystem* camPitchControlSystem = new ControlSystem(control_system::pitch, camPitchProvider, block_frequency::hz100);
    camPitchControlSystem->addBlock(camPitchPID);
    camPitchControlSystem->addBlock(camPitchPVRef);
    ControlSystem* camYawControlSystem = new ControlSystem(control_system::yaw, camYawProvider, block_frequency::hz100);
    camYawControlSystem->addBlock(camYawPID);
    camYawControlSystem->addBlock(camYawPVRef);
    // ********************************************************************************
    // ******************************** PID PARAMETERS ********************************
    PID_parameters* camPitchPIDPara = new PID_parameters;
    camPitchPIDPara->kp = 0.0;
    camPitchPIDPara->ki = 0.8;
    camPitchPIDPara->kd = 0.0;
    camPitchPIDPara->kdd = 0.0;
    camPitchPIDPara->anti_windup = 0.55;
    camPitchPIDPara->en_pv_derivation = 0;
    camPitchControlSystem->changePIDSettings(camPitchPIDPara);
    
    PID_parameters* camYawPIDPara = new PID_parameters;
    camYawPIDPara->kp = 0.0;
    camYawPIDPara->ki = 1.5;
    camYawPIDPara->kd = 0.0;
    camYawPIDPara->kdd = 0.0;
    camYawPIDPara->anti_windup = 0.55;
    camYawPIDPara->en_pv_derivation = 0;
    camYawControlSystem->changePIDSettings(camYawPIDPara);
    // ********************************************************************************
    // *************************** CONTROLLER OUTPUT BRIDGE ***************************
    //TODO: this needs to be removed after the controller output msg has been adjusted
    ControllerActuationBridge* mainCtrlActuationBridge = new ControllerActuationBridge();
    // ********************************************************************************
    // ******************************* PUMP CONTROLLER ********************************
    TimedSwitch* mainPump = new TimedSwitch;
    LUT2D* waterTankLUT = new LUT2D;
    waterTankLUT->setLUT("/home/ffuav01/MBZIRC_ws/src/water_ext/config/LUT.txt");
    PumpController* mainPumpController = new PumpController(mainPump, waterTankLUT);
    //TODO: use the new MsgEmitter Structure
    PumpRosBridge* mainPumpRosBridge = new PumpRosBridge;
    // ********************************************************************************
    // ******************************** FIRE ASSESSOR *********************************
    FireAssessor* mainFireAssessor = new FireAssessor;
    mainFireAssessor->setAngleTolerance(0.1745); //10 degrees
    mainFireAssessor->setExtinguishedTimeout(2000); //2 seconds
    mainFireAssessor->setMaximumTriggeringDistance(1);
    mainFireAssessor->setNeededWaterVolume(1);
    // ********************************************************************************
    // ********************************** ROS UNITS ***********************************
	ROSUnit_Factory main_ROSUnitFactory(nh);
	ROSUnit* InternalStateUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "/water_ext/set_mission_state");
    ROSUnit* EnvCondUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "/water_ext/set_environment_cond");
    ROSUnit* FireStateUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "/water_ext/update_fire_state");
    ROSUnit* WaterLevelRequesterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Empty, "/water_ext/get_water_level");
    ROSUnit* WaterLevelUpdaterClnt = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "/ex_bldg_fire_mm/update_water_level");
    ROSUnit* FireDistanceUpdaterSub = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, ROSUnit_msg_type::ROSUnit_Float, "/ugv_nav/distance_to_fire");
    ROSUnit* StateChangeUpdaterClnt = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "/gf_indoor_fire_mm/update_water_ext_state");
    // ********************************************************************************
    // ****************************** SYSTEM CONNECTIONS ******************************
    mainCommChecker->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) mainCommChecker);
    mainImageConverter->add_callback_msg_receiver((msg_receiver*) mainThermalCamera);
    mainThermalCamera->add_callback_msg_receiver((msg_receiver*) mainHeatcenterProv);
    mainHeatcenterProv->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    mainPitchUserRef->add_callback_msg_receiver((msg_receiver*) camPitchControlSystem);
    mainYawUserRef->add_callback_msg_receiver((msg_receiver*) camYawControlSystem);
    camPitchControlSystem->add_callback_msg_receiver((msg_receiver*) mainCtrlActuationBridge);
    camYawControlSystem->add_callback_msg_receiver((msg_receiver*) mainCtrlActuationBridge);
    mainCtrlActuationBridge->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) &waterExtMissionStateManager);
    (&waterExtMissionStateManager)->add_callback_msg_receiver((msg_receiver*) StateChangeUpdaterClnt);
    FireStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) mainFireAssessor);
    FireDistanceUpdaterSub->add_callback_msg_receiver((msg_receiver*) mainFireAssessor);
    WaterLevelRequesterSrv->add_callback_msg_receiver((msg_receiver*) mainPumpController);
    mainPumpController->add_callback_msg_receiver((msg_receiver*) mainPumpRosBridge);
    mainPumpRosBridge->add_callback_msg_receiver((msg_receiver*) WaterLevelUpdaterClnt);
    mainFireAssessor->add_callback_msg_receiver((msg_receiver*) mainPumpController);
    mainPumpController->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    // ********************************************************************************
    // ************************* Initialize Reference To Zero *************************
    msg_emitter tmp_emitter;
    UpdateNozzlePoseMessage tmp_user_ref_msg;
    tmp_user_ref_msg.setNozzlePoseMessage(0,0);
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) mainYawUserRef);
    tmp_emitter.add_callback_msg_receiver((msg_receiver*) mainPitchUserRef);
    tmp_emitter.emit_message((DataMessage*) &tmp_user_ref_msg);
    // ************************************ LOOPER ************************************
    // ********************************************************************************
    pthread_t loop10hz_func_id, loop100hz_func_id;
    Looper* main_looper = new Looper();
    main_looper->addTimedBlock((TimedBlock*) camPitchControlSystem);
    main_looper->addTimedBlock((TimedBlock*) camYawControlSystem);
    //main_looper->addTimedBlock((TimedBlock*) main_checker);
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