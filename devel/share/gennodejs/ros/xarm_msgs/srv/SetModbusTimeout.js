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

class SetModbusTimeoutRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timeout_ms = null;
      this.is_transparent_transmission = null;
    }
    else {
      if (initObj.hasOwnProperty('timeout_ms')) {
        this.timeout_ms = initObj.timeout_ms
      }
      else {
        this.timeout_ms = 0;
      }
      if (initObj.hasOwnProperty('is_transparent_transmission')) {
        this.is_transparent_transmission = initObj.is_transparent_transmission
      }
      else {
        this.is_transparent_transmission = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetModbusTimeoutRequest
    // Serialize message field [timeout_ms]
    bufferOffset = _serializer.int32(obj.timeout_ms, buffer, bufferOffset);
    // Serialize message field [is_transparent_transmission]
    bufferOffset = _serializer.bool(obj.is_transparent_transmission, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetModbusTimeoutRequest
    let len;
    let data = new SetModbusTimeoutRequest(null);
    // Deserialize message field [timeout_ms]
    data.timeout_ms = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_transparent_transmission]
    data.is_transparent_transmission = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/SetModbusTimeoutRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac346b12c265f37a4849a9684c07403b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # configure the timeout parameter in modbus communication, in milliseconds
    int32 timeout_ms
    
    # whether the set timeout is the timeout of transparent transmission
    bool is_transparent_transmission
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetModbusTimeoutRequest(null);
    if (msg.timeout_ms !== undefined) {
      resolved.timeout_ms = msg.timeout_ms;
    }
    else {
      resolved.timeout_ms = 0
    }

    if (msg.is_transparent_transmission !== undefined) {
      resolved.is_transparent_transmission = msg.is_transparent_transmission;
    }
    else {
      resolved.is_transparent_transmission = false
    }

    return resolved;
    }
};

class SetModbusTimeoutResponse {
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
    // Serializes a message object of type SetModbusTimeoutResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetModbusTimeoutResponse
    let len;
    let data = new SetModbusTimeoutResponse(null);
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
    return 'xarm_msgs/SetModbusTimeoutResponse';
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
    const resolved = new SetModbusTimeoutResponse(null);
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
  Request: SetModbusTimeoutRequest,
  Response: SetModbusTimeoutResponse,
  md5sum() { return 'cd53862105e4494b607dc1eaae204c5f'; },
  datatype() { return 'xarm_msgs/SetModbusTimeout'; }
};
