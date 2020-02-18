#define USING_CPP
#define HEAT_CENTER_PROV
//#define CIRCLE_CENTER_PROV
#define NEGATE_CAM
//#define NEGATE_GYRO
#define LDXPID

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
#include "Negator.hpp"
#include "CircleDetector.hpp"
#include "CameraScanner1D.hpp"
#include "UGVFireAssessor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "water_ext");

    ros::NodeHandle nh;
    //ros::Rate rate(100);
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
    #ifdef HEAT_CENTER_PROV
    HeatCenterProvider* mainHeatcenterProv = new HeatCenterProvider();
    #elif CIRCLE_CENTER_PROV
    CircleDetector* mainHeatcenterProv = new CircleDetector();
    #endif
    #ifdef NEGATE_CAM
    Negator* cam_negator = NULL;
    cam_negator = new Negator;
    #endif
    mainHeatcenterProv->setCutOffTemperature(60.f);
    // ********************************************************************************
    // ******************************** CAMERA SCANNER ********************************
    CameraScanner1D* mainThermalScanner = new CameraScanner1D;
    mainThermalScanner->setDelay(5000);
    // ******************************* USER REFERENCES ********************************
    NozzlePitch_UserReference* mainPitchUserRef = new NozzlePitch_UserReference;
    NozzleYaw_UserReference* mainYawUserRef = new NozzleYaw_UserReference;
    // ********************************************************************************
    // ***************************** ORIENTATION PROVIDER *****************************
    //TODO: add a "valve" that controls the flow of data between two blocks based on a logical expression(s)
    NozzleOrientationProvider* mainOrientationProvider = new NozzleOrientationProvider();
    #ifdef NEGATE_GYRO
    Negator* gyro_negator = NULL;
    gyro_negator = new Negator;
    #endif
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
    #ifdef LDXPID
    PID_parameters* camPitchPIDPara = new PID_parameters;
    camPitchPIDPara->kp = 0.0; 
    camPitchPIDPara->ki = 0.8;
    camPitchPIDPara->kd = 0.0;
    camPitchPIDPara->kdd = 0.0;
    camPitchPIDPara->anti_windup = 0.55;
    camPitchPIDPara->en_pv_derivation = 0;
    camPitchPIDPara->id = block_id::PID_PITCH;
    camPitchControlSystem->changePIDSettings(camPitchPIDPara);

    PID_parameters* camYawPIDPara = new PID_parameters;
    camYawPIDPara->kp = 0.;
    camYawPIDPara->ki = 1.5;
    camYawPIDPara->kd = 0.0;
    camYawPIDPara->kdd = 0.0;
    camYawPIDPara->anti_windup = 0.55;
    camYawPIDPara->en_pv_derivation = 0;
    camYawPIDPara->id = block_id::PID_YAW;
    camYawControlSystem->changePIDSettings(camYawPIDPara);

    #else
    PID_parameters* camPitchPIDPara = new PID_parameters;
    camPitchPIDPara->kp = 0.0; 
    camPitchPIDPara->ki = 3.2;
    camPitchPIDPara->kd = 0.0;
    camPitchPIDPara->kdd = 0.0;
    camPitchPIDPara->anti_windup = 0.55;
    camPitchPIDPara->en_pv_derivation = 0;
    camPitchPIDPara->id = block_id::PID_PITCH;
    camPitchControlSystem->changePIDSettings(camPitchPIDPara);

    PID_parameters* camYawPIDPara = new PID_parameters;
    camYawPIDPara->kp = 0.;
    camYawPIDPara->ki = 6.0;
    camYawPIDPara->kd = 0.0;
    camYawPIDPara->kdd = 0.0;
    camYawPIDPara->anti_windup = 0.55;
    camYawPIDPara->en_pv_derivation = 0;
    camYawPIDPara->id = block_id::PID_YAW;
    camYawControlSystem->changePIDSettings(camYawPIDPara);
    #endif
    // ********************************************************************************
    // *************************** CONTROLLER OUTPUT BRIDGE ***************************
    //TODO: this needs to be removed after the controller output msg has been adjusted
    ControllerActuationBridge* mainCtrlActuationBridge = new ControllerActuationBridge();
    // ********************************************************************************
    // ******************************* PUMP CONTROLLER ********************************
    TimedSwitch* mainPump = new TimedSwitch;
    LUT2D* waterTankLUT = new LUT2D;
    waterTankLUT->setLUT("/home/ch3uav4/catkin_wtr/src/water_ext/config/LUT.txt");
    PumpController* mainPumpController = new PumpController(mainPump, waterTankLUT);
    //TODO: use the new MsgEmitter Structure
    PumpRosBridge* mainPumpRosBridge = new PumpRosBridge;
    // ********************************************************************************
    // ******************************** FIRE ASSESSOR *********************************
    //TODO: add a "valve" that controls the flow of data between two blocks based on a logical expression(s)
    //FireAssessor* mainFireAssessor = new FireAssessor;
    UGVFireAssessor* mainFireAssessor = new UGVFireAssessor;
    mainFireAssessor->setAngleTolerance(0.1745/2); //5 degrees
    //mainFireAssessor->setExtinguishedTimeout(2000); //2 seconds
    //mainFireAssessor->setMaximumTriggeringDistance(1);
    //mainFireAssessor->setNeededWaterVolume(1);
    // ********************************************************************************
    // ********************************** ROS UNITS ***********************************
	ROSUnit_Factory main_ROSUnitFactory(nh);
	ROSUnit* InternalStateUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_mission_state");
    ROSUnit* EnvCondUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_environment_cond");
    ROSUnit* FireStateUpdaterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "water_ext/update_fire_state");
    ROSUnit* WaterLevelRequesterSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Empty, "water_ext/get_water_level");
    ROSUnit* WaterLevelUpdaterClnt = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_water_level");
    ROSUnit* FireDistanceUpdaterSub = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, ROSUnit_msg_type::ROSUnit_Float, "ugv_nav/distance_to_fire");
    ROSUnit* StateChangeUpdaterClnt = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_water_ext_state");
    ROSUnit* ThermalScannerSrv = main_ROSUnitFactory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_Empty, "water_ext/trigger_scan");
    // ********************************************************************************
    // ****************************** SYSTEM CONNECTIONS ******************************
    mainCommChecker->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) mainCommChecker);
    mainImageConverter->add_callback_msg_receiver((msg_receiver*) mainThermalCamera);
    mainThermalCamera->add_callback_msg_receiver((msg_receiver*) mainHeatcenterProv);
    #ifdef NEGATE_CAM
    Logger::getAssignedLogger()->log("Negating Camera", LoggerLevel::Info);
    mainHeatcenterProv->add_callback_msg_receiver((msg_receiver*) cam_negator);
    cam_negator->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    #else
    Logger::getAssignedLogger()->log("Not Negating Camera", LoggerLevel::Info);
    mainHeatcenterProv->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    #endif
    #ifdef NEGATE_GYRO
    Logger::getAssignedLogger()->log("Negating Gyro", LoggerLevel::Info);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) gyro_negator);
    gyro_negator->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    #else
    Logger::getAssignedLogger()->log("Not Negating Gyro", LoggerLevel::Info);
    mainCommStack->add_callback_msg_receiver((msg_receiver*) mainOrientationProvider);
    #endif
    mainPitchUserRef->add_callback_msg_receiver((msg_receiver*) camPitchControlSystem);
    mainYawUserRef->add_callback_msg_receiver((msg_receiver*) camYawControlSystem);
    camPitchControlSystem->add_callback_msg_receiver((msg_receiver*) mainCtrlActuationBridge);
    camYawControlSystem->add_callback_msg_receiver((msg_receiver*) mainCtrlActuationBridge);
    mainCtrlActuationBridge->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) &waterExtMissionStateManager);
    (&waterExtMissionStateManager)->add_callback_msg_receiver((msg_receiver*) StateChangeUpdaterClnt);
    mainHeatcenterProv->add_callback_msg_receiver((msg_receiver*) mainFireAssessor);
    FireStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) mainFireAssessor);
    //FireDistanceUpdaterSub->add_callback_msg_receiver((msg_receiver*) mainFireAssessor);
    mainPump->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    WaterLevelRequesterSrv->add_callback_msg_receiver((msg_receiver*) mainPumpController);
    mainPumpController->add_callback_msg_receiver((msg_receiver*) mainPumpRosBridge);
    mainPumpRosBridge->add_callback_msg_receiver((msg_receiver*) WaterLevelUpdaterClnt);
    mainFireAssessor->add_callback_msg_receiver((msg_receiver*) mainPumpController);
    //mainPumpController->add_callback_msg_receiver((msg_receiver*) mainCommStack);
    ThermalScannerSrv->add_callback_msg_receiver((msg_receiver*) mainThermalScanner);
    //mainThermalScanner->add_callback_msg_receiver((msg_receiver*) mainCommStack);
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
    main_looper->addTimedBlock((TimedBlock*) mainCommChecker);
    pthread_create(&loop100hz_func_id, NULL, &Looper::Loop100Hz, NULL);
    pthread_create(&loop10hz_func_id, NULL, &Looper::Loop10Hz, NULL);
    // ********************************************************************************
    while(ros::ok())
    {
        ros::spinOnce();
        //rate.sleep();
    }
    return 0;
}