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

class SetControllerAnalogIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_num = null;
      this.analog_value = null;
    }
    else {
      if (initObj.hasOwnProperty('port_num')) {
        this.port_num = initObj.port_num
      }
      else {
        this.port_num = 0;
      }
      if (initObj.hasOwnProperty('analog_value')) {
        this.analog_value = initObj.analog_value
      }
      else {
        this.analog_value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControllerAnalogIORequest
    // Serialize message field [port_num]
    bufferOffset = _serializer.int16(obj.port_num, buffer, bufferOffset);
    // Serialize message field [analog_value]
    bufferOffset = _serializer.float32(obj.analog_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControllerAnalogIORequest
    let len;
    let data = new SetControllerAnalogIORequest(null);
    // Deserialize message field [port_num]
    data.port_num = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [analog_value]
    data.analog_value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/SetControllerAnalogIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f4df395f9657a7248b13463d3cf4caac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Setting the analog Output port value at robot controller, io_num: from 1 to 2
    
    int16 port_num
    
    float32 analog_value
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControllerAnalogIORequest(null);
    if (msg.port_num !== undefined) {
      resolved.port_num = msg.port_num;
    }
    else {
      resolved.port_num = 0
    }

    if (msg.analog_value !== undefined) {
      resolved.analog_value = msg.analog_value;
    }
    else {
      resolved.analog_value = 0.0
    }

    return resolved;
    }
};

class SetControllerAnalogIOResponse {
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
    // Serializes a message object of type SetControllerAnalogIOResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControllerAnalogIOResponse
    let len;
    let data = new SetControllerAnalogIOResponse(null);
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
    return 'xarm_msgs/SetControllerAnalogIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76c68a2c5e109f4d6a4dc1cfc355f405';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int16 ret
    
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControllerAnalogIOResponse(null);
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
  Request: SetControllerAnalogIORequest,
  Response: SetControllerAnalogIOResponse,
  md5sum() { return '2b78c87698704b2bc72cea876ba90e55'; },
  datatype() { return 'xarm_msgs/SetControllerAnalogIO'; }
};
