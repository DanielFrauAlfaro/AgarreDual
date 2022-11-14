
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let RawRequest = require('./RawRequest.js')
let AddToLog = require('./AddToLog.js')
let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetProgramState = require('./GetProgramState.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  RawRequest: RawRequest,
  AddToLog: AddToLog,
  GetRobotMode: GetRobotMode,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  GetSafetyMode: GetSafetyMode,
  GetProgramState: GetProgramState,
  IsInRemoteControl: IsInRemoteControl,
  GetLoadedProgram: GetLoadedProgram,
};
