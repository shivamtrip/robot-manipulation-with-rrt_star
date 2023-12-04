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

class GetControllerDigitalIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.io_num = null;
    }
    else {
      if (initObj.hasOwnProperty('io_num')) {
        this.io_num = initObj.io_num
      }
      else {
        this.io_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerDigitalIORequest
    // Serialize message field [io_num]
    bufferOffset = _serializer.int16(obj.io_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerDigitalIORequest
    let len;
    let data = new GetControllerDigitalIORequest(null);
    // Deserialize message field [io_num]
    data.io_num = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetControllerDigitalIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d30b3ebd5a1d172f02950e37a7469f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Getting the controller DIGITAL input port status, io_num: from 1 to 16
    
    int16 io_num
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerDigitalIORequest(null);
    if (msg.io_num !== undefined) {
      resolved.io_num = msg.io_num;
    }
    else {
      resolved.io_num = 0
    }

    return resolved;
    }
};

class GetControllerDigitalIOResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ret = null;
      this.value = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('ret')) {
        this.ret = initObj.ret
      }
      else {
        this.ret = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
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
    // Serializes a message object of type GetControllerDigitalIOResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.int16(obj.value, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerDigitalIOResponse
    let len;
    let data = new GetControllerDigitalIOResponse(null);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetControllerDigitalIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '108e679b0ec7260e30f3ee2c752c2a2b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int16 ret
    
    int16 value
    
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerDigitalIOResponse(null);
    if (msg.ret !== undefined) {
      resolved.ret = msg.ret;
    }
    else {
      resolved.ret = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
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
  Request: GetControllerDigitalIORequest,
  Response: GetControllerDigitalIOResponse,
  md5sum() { return 'a10e26b4b2c853a8b561f9587bff33b2'; },
  datatype() { return 'xarm_msgs/GetControllerDigitalIO'; }
};
