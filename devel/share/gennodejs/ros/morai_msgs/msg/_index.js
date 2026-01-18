
"use strict";

let CollisionData = require('./CollisionData.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let CMDConveyor = require('./CMDConveyor.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let RadarDetection = require('./RadarDetection.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let Transforms = require('./Transforms.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let SaveSensorData = require('./SaveSensorData.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let GPSMessage = require('./GPSMessage.js');
let RobotState = require('./RobotState.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let VelocityCmd = require('./VelocityCmd.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let TrafficLight = require('./TrafficLight.js');
let GhostMessage = require('./GhostMessage.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let ExternalForce = require('./ExternalForce.js');
let ObjectStatus = require('./ObjectStatus.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let RobotOutput = require('./RobotOutput.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let EventInfo = require('./EventInfo.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let DillyCmd = require('./DillyCmd.js');
let IntscnTL = require('./IntscnTL.js');
let ShipState = require('./ShipState.js');
let Lamps = require('./Lamps.js');
let Conveyor = require('./Conveyor.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let TOF = require('./TOF.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let WheelControl = require('./WheelControl.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let Obstacle = require('./Obstacle.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let SensorPosControl = require('./SensorPosControl.js');
let RadarDetections = require('./RadarDetections.js');
let WaitForTick = require('./WaitForTick.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let Obstacles = require('./Obstacles.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let MapSpec = require('./MapSpec.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let PRStatus = require('./PRStatus.js');
let IntersectionControl = require('./IntersectionControl.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let ReplayInfo = require('./ReplayInfo.js');
let SVADC = require('./SVADC.js');
let CtrlCmd = require('./CtrlCmd.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let VehicleCollision = require('./VehicleCollision.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let GVStateCmd = require('./GVStateCmd.js');
let ERP42Info = require('./ERP42Info.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let VehicleSpec = require('./VehicleSpec.js');
let PREvent = require('./PREvent.js');

module.exports = {
  CollisionData: CollisionData,
  NpcGhostCmd: NpcGhostCmd,
  NpcGhostInfo: NpcGhostInfo,
  MoraiSimProcStatus: MoraiSimProcStatus,
  FaultInjection_Response: FaultInjection_Response,
  CMDConveyor: CMDConveyor,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  RadarDetection: RadarDetection,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  EgoVehicleStatus: EgoVehicleStatus,
  FaultInjection_Tire: FaultInjection_Tire,
  MultiEgoSetting: MultiEgoSetting,
  Transforms: Transforms,
  MoraiTLInfo: MoraiTLInfo,
  SaveSensorData: SaveSensorData,
  DdCtrlCmd: DdCtrlCmd,
  GeoVector3Message: GeoVector3Message,
  FaultInjection_Sensor: FaultInjection_Sensor,
  MultiPlayEventRequest: MultiPlayEventRequest,
  SyncModeResultResponse: SyncModeResultResponse,
  GPSMessage: GPSMessage,
  RobotState: RobotState,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  VelocityCmd: VelocityCmd,
  SyncModeRemoveObject: SyncModeRemoveObject,
  ObjectStatusListExtended: ObjectStatusListExtended,
  ObjectStatusExtended: ObjectStatusExtended,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  TrafficLight: TrafficLight,
  GhostMessage: GhostMessage,
  ObjectStatusList: ObjectStatusList,
  ScenarioLoad: ScenarioLoad,
  VehicleCollisionData: VehicleCollisionData,
  ExternalForce: ExternalForce,
  ObjectStatus: ObjectStatus,
  DillyCmdResponse: DillyCmdResponse,
  FaultStatusInfo: FaultStatusInfo,
  RobotOutput: RobotOutput,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  EventInfo: EventInfo,
  SyncModeCmdResponse: SyncModeCmdResponse,
  SyncModeAddObject: SyncModeAddObject,
  MultiPlayEventResponse: MultiPlayEventResponse,
  VehicleSpecIndex: VehicleSpecIndex,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  DillyCmd: DillyCmd,
  IntscnTL: IntscnTL,
  ShipState: ShipState,
  Lamps: Lamps,
  Conveyor: Conveyor,
  ShipCtrlCmd: ShipCtrlCmd,
  IntersectionStatus: IntersectionStatus,
  TOF: TOF,
  SyncModeCmd: SyncModeCmd,
  WheelControl: WheelControl,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  MoraiSimProcHandle: MoraiSimProcHandle,
  FaultInjection_Controller: FaultInjection_Controller,
  SyncModeSetGear: SyncModeSetGear,
  ManipulatorControl: ManipulatorControl,
  Obstacle: Obstacle,
  MoraiTLIndex: MoraiTLIndex,
  SensorPosControl: SensorPosControl,
  RadarDetections: RadarDetections,
  WaitForTick: WaitForTick,
  SkateboardStatus: SkateboardStatus,
  Obstacles: Obstacles,
  MoraiSrvResponse: MoraiSrvResponse,
  WoowaDillyStatus: WoowaDillyStatus,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  WaitForTickResponse: WaitForTickResponse,
  GVDirectCmd: GVDirectCmd,
  MapSpec: MapSpec,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  PRStatus: PRStatus,
  IntersectionControl: IntersectionControl,
  PRCtrlCmd: PRCtrlCmd,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  ReplayInfo: ReplayInfo,
  SVADC: SVADC,
  CtrlCmd: CtrlCmd,
  MapSpecIndex: MapSpecIndex,
  VehicleCollision: VehicleCollision,
  SyncModeInfo: SyncModeInfo,
  GVStateCmd: GVStateCmd,
  ERP42Info: ERP42Info,
  SetTrafficLight: SetTrafficLight,
  VehicleSpec: VehicleSpec,
  PREvent: PREvent,
};
