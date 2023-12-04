// Auto-generated. Do not edit!

// (in-package xarm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MoveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.mvvelo = null;
      this.mvacc = null;
      this.mvtime = null;
      this.mvradii = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = [];
      }
      if (initObj.hasOwnProperty('mvvelo')) {
        this.mvvelo = initObj.mvvelo
      }
      else {
        this.mvvelo = 0.0;
      }
      if (initObj.hasOwnProperty('mvacc')) {
        this.mvacc = initObj.mvacc
      }
      else {
        this.mvacc = 0.0;
      }
      if (initObj.hasOwnProperty('mvtime')) {
        this.mvtime = initObj.mvtime
      }
      else {
        this.mvtime = 0.0;
      }
      if (initObj.hasOwnProperty('mvradii')) {
        this.mvradii = initObj.mvradii
      }
      else {
        this.mvradii = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveRequest
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float32(obj.pose, buffer, bufferOffset, null);
    // Serialize message field [mvvelo]
    bufferOffset = _serializer.float32(obj.mvvelo, buffer, bufferOffset);
    // Serialize message field [mvacc]
    bufferOffset = _serializer.float32(obj.mvacc, buffer, bufferOffset);
    // Serialize message field [mvtime]
    bufferOffset = _serializer.float32(obj.mvtime, buffer, bufferOffset);
    // Serialize message field [mvradii]
    bufferOffset = _serializer.float32(obj.mvradii, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveRequest
    let len;
    let data = new MoveRequest(null);
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mvvelo]
    data.mvvelo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mvacc]
    data.mvacc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mvtime]
    data.mvtime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mvradii]
    data.mvradii = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pose.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0e14760ef59968dcef4281098fe75b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request: command specification for motion executions.
    # Units:
    #	joint space/angles: radian, radian/s and radian/s^2.
    #	Cartesian space: mm, mm/s, and mm/s^2.
    #	time: sec
    
    # pose: target coordinate. 
    #	For Joint Space target，pose dimention is the number of joints. element as each target joint position.
    #	For Cartesian target: pose dimention is 6 for (x, y, z, roll, pitch, yaw)
    
    float32[] pose
    
    # mvvelo: specified maximum velocity during execution. linear or angular velocity 
    
    float32 mvvelo
    
    # mvacc: specified maximum acceleration during execution. linear or angular acceleration.
    
    float32 mvacc
    
    # mvtime: currently do not have any special meaning, please just give it 0. 
    # PLEASE NOTE: after firmware version 1.5, For servo_cartesian motion, mvtime will be used as indicator of coordinate system. (0 for BASE coordinate, non-zero for TOOL coordinate)  
    
    float32 mvtime
    
    # mvradii: this is special for move_ineb service, meaning the blending radius between 2 straight path trajectories, 0 for no blend.
    
    float32 mvradii
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = []
    }

    if (msg.mvvelo !== undefined) {
      resolved.mvvelo = msg.mvvelo;
    }
    else {
      resolved.mvvelo = 0.0
    }

    if (msg.mvacc !== undefined) {
      resolved.mvacc = msg.mvacc;
    }
    else {
      resolved.mvacc = 0.0
    }

    if (msg.mvtime !== undefined) {
      resolved.mvtime = msg.mvtime;
    }
    else {
      resolved.mvtime = 0.0
    }

    if (msg.mvradii !== undefined) {
      resolved.mvradii = msg.mvradii;
    }
    else {
      resolved.mvradii = 0.0
    }

    return resolved;
    }
};

class MoveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ret = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('ret')) {
        this.ret = initObj.ret
      }
      else {
        this.ret = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveResponse
    let len;
    let data = new MoveResponse(null);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76c68a2c5e109f4d6a4dc1cfc355f405';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # response: 
    #	ret is 0 for successful execution and others for errors or warnings occured
    #	message is a string returned by function, indicating execution status.
    
    int16 ret
    
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveResponse(null);
    if (msg.ret !== undefined) {
      resolved.ret = msg.ret;
    }
    else {
      resolved.ret = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: MoveRequest,
  Response: MoveResponse,
  md5sum() { return '149d0c53b84d9002e7f64f44b16335dd'; },
  datatype() { return 'xarm_msgs/Move'; }
};
