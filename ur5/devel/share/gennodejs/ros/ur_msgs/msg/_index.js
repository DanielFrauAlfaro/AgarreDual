
"use strict";

let Analog = require('./Analog.js');
let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let Digital = require('./Digital.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let IOStates = require('./IOStates.js');

module.exports = {
  Analog: Analog,
  MasterboardDataMsg: MasterboardDataMsg,
  Digital: Digital,
  RobotStateRTMsg: RobotStateRTMsg,
  ToolDataMsg: ToolDataMsg,
  RobotModeDataMsg: RobotModeDataMsg,
  IOStates: IOStates,
};
