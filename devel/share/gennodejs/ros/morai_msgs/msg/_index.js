
"use strict";

let TrafficLightIndex = require('./TrafficLightIndex.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let SaveSensorData = require('./SaveSensorData.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let RadarDetection = require('./RadarDetection.js');
let GPSMessage = require('./GPSMessage.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let IntersectionControl = require('./IntersectionControl.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let VehicleSpec = require('./VehicleSpec.js');
let ObjectStatus = require('./ObjectStatus.js');
let EventInfo = require('./EventInfo.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let TrafficLightInfo = require('./TrafficLightInfo.js');
let ReplayInfo = require('./ReplayInfo.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let CollisionData = require('./CollisionData.js');
let SensorPosControl = require('./SensorPosControl.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let RadarDetections = require('./RadarDetections.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let GhostCmd = require('./GhostCmd.js');
let CtrlCmd = require('./CtrlCmd.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let Lamps = require('./Lamps.js');
let MapSpec = require('./MapSpec.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');

module.exports = {
  TrafficLightIndex: TrafficLightIndex,
  GetTrafficLightStatus: GetTrafficLightStatus,
  VehicleCollision: VehicleCollision,
  SaveSensorData: SaveSensorData,
  ScenarioLoad: ScenarioLoad,
  RadarDetection: RadarDetection,
  GPSMessage: GPSMessage,
  SetTrafficLight: SetTrafficLight,
  IntersectionControl: IntersectionControl,
  ObjectStatusList: ObjectStatusList,
  VehicleSpecIndex: VehicleSpecIndex,
  MoraiSimProcHandle: MoraiSimProcHandle,
  VehicleSpec: VehicleSpec,
  ObjectStatus: ObjectStatus,
  EventInfo: EventInfo,
  IntersectionStatus: IntersectionStatus,
  MoraiSrvResponse: MoraiSrvResponse,
  TrafficLightInfo: TrafficLightInfo,
  ReplayInfo: ReplayInfo,
  MoraiSimProcStatus: MoraiSimProcStatus,
  NpcGhostInfo: NpcGhostInfo,
  CollisionData: CollisionData,
  SensorPosControl: SensorPosControl,
  EgoVehicleStatus: EgoVehicleStatus,
  RadarDetections: RadarDetections,
  NpcGhostCmd: NpcGhostCmd,
  GhostCmd: GhostCmd,
  CtrlCmd: CtrlCmd,
  VehicleCollisionData: VehicleCollisionData,
  Lamps: Lamps,
  MapSpec: MapSpec,
  MultiEgoSetting: MultiEgoSetting,
};
