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

class MoveAxisAngleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.mvvelo = null;
      this.mvacc = null;
      this.mvtime = null;
      this.coord = null;
      this.relative = null;
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
      if (initObj.hasOwnProperty('coord')) {
        this.coord = initObj.coord
      }
      else {
        this.coord = 0;
      }
      if (initObj.hasOwnProperty('relative')) {
        this.relative = initObj.relative
      }
      else {
        this.relative = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveAxisAngleRequest
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float32(obj.pose, buffer, bufferOffset, null);
    // Serialize message field [mvvelo]
    bufferOffset = _serializer.float32(obj.mvvelo, buffer, bufferOffset);
    // Serialize message field [mvacc]
    bufferOffset = _serializer.float32(obj.mvacc, buffer, bufferOffset);
    // Serialize message field [mvtime]
    bufferOffset = _serializer.float32(obj.mvtime, buffer, bufferOffset);
    // Serialize message field [coord]
    bufferOffset = _serializer.int16(obj.coord, buffer, bufferOffset);
    // Serialize message field [relative]
    bufferOffset = _serializer.int16(obj.relative, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveAxisAngleRequest
    let len;
    let data = new MoveAxisAngleRequest(null);
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mvvelo]
    data.mvvelo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mvacc]
    data.mvacc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mvtime]
    data.mvtime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [coord]
    data.coord = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [relative]
    data.relative = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pose.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveAxisAngleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f222fa2eeda5e2b5482505e275dcd88';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request: command specification for axis-angle motion executions.
    # Units:
    #	Linear: mm, mm/s, and mm/s^2.
    #   Angular: radian, radian/s and radian/s^2.
    #	time: sec
    
    # pose: target coordinate. 
    #	For Cartesian target: pose dimention is 6 for (x, y, z, rx, ry, rz)
    
    float32[] pose
    
    # mvvelo: specified maximum velocity during execution. linear or angular velocity 
    
    float32 mvvelo
    
    # mvacc: specified maximum acceleration during execution. linear or angular acceleration.
    
    float32 mvacc
    
    # mvtime: currently do not have any special meaning, please just give it 0. 
    
    float32 mvtime
    
    # coord: motion coordinate system indicator, base (0) or tool(1) coordinate.
    
    int16 coord
    
    # relative: indicator of given target is relative (1) or not (0, absolute)
    
    int16 relative
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveAxisAngleRequest(null);
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

    if (msg.coord !== undefined) {
      resolved.coord = msg.coord;
    }
    else {
      resolved.coord = 0
    }

    if (msg.relative !== undefined) {
      resolved.relative = msg.relative;
    }
    else {
      resolved.relative = 0
    }

    return resolved;
    }
};

class MoveAxisAngleResponse {
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
    // Serializes a message object of type MoveAxisAngleResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveAxisAngleResponse
    let len;
    let data = new MoveAxisAngleResponse(null);
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
    return 'xarm_msgs/MoveAxisAngleResponse';
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
    const resolved = new MoveAxisAngleResponse(null);
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
  Request: MoveAxisAngleRequest,
  Response: MoveAxisAngleResponse,
  md5sum() { return 'e3ce015929065df69fd9351b8d2c8407'; },
  datatype() { return 'xarm_msgs/MoveAxisAngle'; }
};
