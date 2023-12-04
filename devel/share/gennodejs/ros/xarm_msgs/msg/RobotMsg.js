// Auto-generated. Do not edit!

// (in-package xarm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.mode = null;
      this.cmdnum = null;
      this.mt_brake = null;
      this.mt_able = null;
      this.err = null;
      this.warn = null;
      this.angle = null;
      this.pose = null;
      this.offset = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('cmdnum')) {
        this.cmdnum = initObj.cmdnum
      }
      else {
        this.cmdnum = 0;
      }
      if (initObj.hasOwnProperty('mt_brake')) {
        this.mt_brake = initObj.mt_brake
      }
      else {
        this.mt_brake = 0;
      }
      if (initObj.hasOwnProperty('mt_able')) {
        this.mt_able = initObj.mt_able
      }
      else {
        this.mt_able = 0;
      }
      if (initObj.hasOwnProperty('err')) {
        this.err = initObj.err
      }
      else {
        this.err = 0;
      }
      if (initObj.hasOwnProperty('warn')) {
        this.warn = initObj.warn
      }
      else {
        this.warn = 0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int16(obj.state, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int16(obj.mode, buffer, bufferOffset);
    // Serialize message field [cmdnum]
    bufferOffset = _serializer.int16(obj.cmdnum, buffer, bufferOffset);
    // Serialize message field [mt_brake]
    bufferOffset = _serializer.int16(obj.mt_brake, buffer, bufferOffset);
    // Serialize message field [mt_able]
    bufferOffset = _serializer.int16(obj.mt_able, buffer, bufferOffset);
    // Serialize message field [err]
    bufferOffset = _serializer.int16(obj.err, buffer, bufferOffset);
    // Serialize message field [warn]
    bufferOffset = _serializer.int16(obj.warn, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float32(obj.angle, buffer, bufferOffset, null);
    // Check that the constant length array field [pose] has the right length
    if (obj.pose.length !== 6) {
      throw new Error('Unable to serialize array field pose - length must be 6')
    }
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float32(obj.pose, buffer, bufferOffset, 6);
    // Check that the constant length array field [offset] has the right length
    if (obj.offset.length !== 6) {
      throw new Error('Unable to serialize array field offset - length must be 6')
    }
    // Serialize message field [offset]
    bufferOffset = _arraySerializer.float32(obj.offset, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotMsg
    let len;
    let data = new RobotMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [cmdnum]
    data.cmdnum = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [mt_brake]
    data.mt_brake = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [mt_able]
    data.mt_able = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [err]
    data.err = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [warn]
    data.warn = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [offset]
    data.offset = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.angle.length;
    return length + 66;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xarm_msgs/RobotMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b796af25d2a8912964fb162d2141ac39';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # feedback information of the controlled robot
    Header header
    
    # state of robot:
    #	1: RUNNING, executing motion command.
    #	2: SLEEPING, not in execution, but ready to move.
    #	3: PAUSED, paused in the middle of unfinished motion.
    #	4: STOPPED, not ready for any motion commands.
    #	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.
    int16 state
    
    # mode of robot:
    #	0 for POSITION mode.(position control by xarm controller box, execute api standard commands)
    #	1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)
    #	2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)
    int16 mode
    
    # cmdnum: number of commands waiting in the buffer.
    int16 cmdnum
    
    # mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled
    int16 mt_brake
    
    # mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled
    int16 mt_able
    
    # error code (if non-zero)
    int16 err
    
    # warning code (if non-zero)
    int16 warn
    
    # current joint angles expressed in radian.
    float32[] angle
    
    # current TCP Cartesian position expressed in mm(position), radian(orientation)  
    float32[6] pose
    
    # TCP offset from center of flange, with respect to tool frame.
    float32[6] offset
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.cmdnum !== undefined) {
      resolved.cmdnum = msg.cmdnum;
    }
    else {
      resolved.cmdnum = 0
    }

    if (msg.mt_brake !== undefined) {
      resolved.mt_brake = msg.mt_brake;
    }
    else {
      resolved.mt_brake = 0
    }

    if (msg.mt_able !== undefined) {
      resolved.mt_able = msg.mt_able;
    }
    else {
      resolved.mt_able = 0
    }

    if (msg.err !== undefined) {
      resolved.err = msg.err;
    }
    else {
      resolved.err = 0
    }

    if (msg.warn !== undefined) {
      resolved.warn = msg.warn;
    }
    else {
      resolved.warn = 0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = new Array(6).fill(0)
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = RobotMsg;
