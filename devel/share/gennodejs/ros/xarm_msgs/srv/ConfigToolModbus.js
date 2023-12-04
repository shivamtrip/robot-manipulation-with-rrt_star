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

class ConfigToolModbusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.baud_rate = null;
      this.timeout_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('baud_rate')) {
        this.baud_rate = initObj.baud_rate
      }
      else {
        this.baud_rate = 0;
      }
      if (initObj.hasOwnProperty('timeout_ms')) {
        this.timeout_ms = initObj.timeout_ms
      }
      else {
        this.timeout_ms = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigToolModbusRequest
    // Serialize message field [baud_rate]
    bufferOffset = _serializer.int32(obj.baud_rate, buffer, bufferOffset);
    // Serialize message field [timeout_ms]
    bufferOffset = _serializer.int32(obj.timeout_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigToolModbusRequest
    let len;
    let data = new ConfigToolModbusRequest(null);
    // Deserialize message field [baud_rate]
    data.baud_rate = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [timeout_ms]
    data.timeout_ms = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/ConfigToolModbusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e41deb7d5a91bccb5610cc9acda38eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # configure the tool modbus communication baud rate, in bps:
    int32 baud_rate
    
    # configure the timeout parameter in modbus communication, in milliseconds
    int32 timeout_ms
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigToolModbusRequest(null);
    if (msg.baud_rate !== undefined) {
      resolved.baud_rate = msg.baud_rate;
    }
    else {
      resolved.baud_rate = 0
    }

    if (msg.timeout_ms !== undefined) {
      resolved.timeout_ms = msg.timeout_ms;
    }
    else {
      resolved.timeout_ms = 0
    }

    return resolved;
    }
};

class ConfigToolModbusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
      this.ret = null;
    }
    else {
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('ret')) {
        this.ret = initObj.ret
      }
      else {
        this.ret = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigToolModbusResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigToolModbusResponse
    let len;
    let data = new ConfigToolModbusResponse(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/ConfigToolModbusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d61b57e27c330e977d9f6b98b84ff3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string message
    int16 ret
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigToolModbusResponse(null);
    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.ret !== undefined) {
      resolved.ret = msg.ret;
    }
    else {
      resolved.ret = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ConfigToolModbusRequest,
  Response: ConfigToolModbusResponse,
  md5sum() { return '4641743544bd81148c56355ba50062be'; },
  datatype() { return 'xarm_msgs/ConfigToolModbus'; }
};
