
"use strict";

let Call = require('./Call.js')
let GetInt32 = require('./GetInt32.js')
let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let SetAxis = require('./SetAxis.js')
let SetFloat32 = require('./SetFloat32.js')
let SetLoad = require('./SetLoad.js')
let SetToolModbus = require('./SetToolModbus.js')
let GetSetModbusData = require('./GetSetModbusData.js')
let GripperConfig = require('./GripperConfig.js')
let TCPOffset = require('./TCPOffset.js')
let SetMultipleInts = require('./SetMultipleInts.js')
let SetString = require('./SetString.js')
let MoveAxisAngle = require('./MoveAxisAngle.js')
let GripperMove = require('./GripperMove.js')
let GetFloat32List = require('./GetFloat32List.js')
let SetControllerAnalogIO = require('./SetControllerAnalogIO.js')
let PlayTraj = require('./PlayTraj.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')
let SetInt16 = require('./SetInt16.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let Move = require('./Move.js')
let GetErr = require('./GetErr.js')
let FtCaliLoad = require('./FtCaliLoad.js')
let MoveVelo = require('./MoveVelo.js')
let GripperState = require('./GripperState.js')
let SetModbusTimeout = require('./SetModbusTimeout.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let ClearErr = require('./ClearErr.js')
let FtIdenLoad = require('./FtIdenLoad.js')
let MoveVelocity = require('./MoveVelocity.js')

module.exports = {
  Call: Call,
  GetInt32: GetInt32,
  GetControllerDigitalIO: GetControllerDigitalIO,
  SetAxis: SetAxis,
  SetFloat32: SetFloat32,
  SetLoad: SetLoad,
  SetToolModbus: SetToolModbus,
  GetSetModbusData: GetSetModbusData,
  GripperConfig: GripperConfig,
  TCPOffset: TCPOffset,
  SetMultipleInts: SetMultipleInts,
  SetString: SetString,
  MoveAxisAngle: MoveAxisAngle,
  GripperMove: GripperMove,
  GetFloat32List: GetFloat32List,
  SetControllerAnalogIO: SetControllerAnalogIO,
  PlayTraj: PlayTraj,
  ConfigToolModbus: ConfigToolModbus,
  SetInt16: SetInt16,
  SetDigitalIO: SetDigitalIO,
  Move: Move,
  GetErr: GetErr,
  FtCaliLoad: FtCaliLoad,
  MoveVelo: MoveVelo,
  GripperState: GripperState,
  SetModbusTimeout: SetModbusTimeout,
  GetAnalogIO: GetAnalogIO,
  GetDigitalIO: GetDigitalIO,
  ClearErr: ClearErr,
  FtIdenLoad: FtIdenLoad,
  MoveVelocity: MoveVelocity,
};
