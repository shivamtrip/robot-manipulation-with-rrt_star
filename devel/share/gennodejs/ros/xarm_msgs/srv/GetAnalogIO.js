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

class GetAnalogIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_num = null;
    }
    else {
      if (initObj.hasOwnProperty('port_num')) {
        this.port_num = initObj.port_num
      }
      else {
        this.port_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAnalogIORequest
    // Serialize message field [port_num]
    bufferOffset = _serializer.int16(obj.port_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAnalogIORequest
    let len;
    let data = new GetAnalogIORequest(null);
    // Deserialize message field [port_num]
    data.port_num = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetAnalogIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1c58d245d5dbcbc33afe76f9fc1dff4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Getting one of the 2 analog Input port at robot end connector or controller
    
    int16 port_num
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAnalogIORequest(null);
    if (msg.port_num !== undefined) {
      resolved.port_num = msg.port_num;
    }
    else {
      resolved.port_num = 0
    }

    return resolved;
    }
};

class GetAnalogIOResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.analog_value = null;
      this.ret = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('analog_value')) {
        this.analog_value = initObj.analog_value
      }
      else {
        this.analog_value = 0.0;
      }
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
    // Serializes a message object of type GetAnalogIOResponse
    // Serialize message field [analog_value]
    bufferOffset = _serializer.float32(obj.analog_value, buffer, bufferOffset);
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAnalogIOResponse
    let len;
    let data = new GetAnalogIOResponse(null);
    // Deserialize message field [analog_value]
    data.analog_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetAnalogIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14b69cf7f6c4030ec842bfd1c9d215d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 analog_value
    
    int16 ret
    
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAnalogIOResponse(null);
    if (msg.analog_value !== undefined) {
      resolved.analog_value = msg.analog_value;
    }
    else {
      resolved.analog_value = 0.0
    }

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
  Request: GetAnalogIORequest,
  Response: GetAnalogIOResponse,
  md5sum() { return 'be8d9a2c0ed50c727cbf098654568f97'; },
  datatype() { return 'xarm_msgs/GetAnalogIO'; }
};
