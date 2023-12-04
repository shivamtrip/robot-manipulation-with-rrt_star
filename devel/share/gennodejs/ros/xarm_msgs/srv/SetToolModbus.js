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

class SetToolModbusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.send_data = null;
      this.respond_len = null;
    }
    else {
      if (initObj.hasOwnProperty('send_data')) {
        this.send_data = initObj.send_data
      }
      else {
        this.send_data = [];
      }
      if (initObj.hasOwnProperty('respond_len')) {
        this.respond_len = initObj.respond_len
      }
      else {
        this.respond_len = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetToolModbusRequest
    // Serialize message field [send_data]
    bufferOffset = _arraySerializer.uint8(obj.send_data, buffer, bufferOffset, null);
    // Serialize message field [respond_len]
    bufferOffset = _serializer.int16(obj.respond_len, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetToolModbusRequest
    let len;
    let data = new SetToolModbusRequest(null);
    // Deserialize message field [send_data]
    data.send_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [respond_len]
    data.respond_len = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.send_data.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/SetToolModbusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '199c3425393f03db695077c48d89982d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # unsigned char data to be sent to tool device through modbus
    uint8[] send_data
    
    # Specify the anticipated maximum respond data length in bytes
    int16 respond_len
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetToolModbusRequest(null);
    if (msg.send_data !== undefined) {
      resolved.send_data = msg.send_data;
    }
    else {
      resolved.send_data = []
    }

    if (msg.respond_len !== undefined) {
      resolved.respond_len = msg.respond_len;
    }
    else {
      resolved.respond_len = 0
    }

    return resolved;
    }
};

class SetToolModbusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ret = null;
      this.message = null;
      this.respond_data = null;
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
      if (initObj.hasOwnProperty('respond_data')) {
        this.respond_data = initObj.respond_data
      }
      else {
        this.respond_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetToolModbusResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [respond_data]
    bufferOffset = _arraySerializer.uint8(obj.respond_data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetToolModbusResponse
    let len;
    let data = new SetToolModbusResponse(null);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [respond_data]
    data.respond_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += object.respond_data.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/SetToolModbusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6758d9ca75742796b3adadfd14ca06e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int16 ret
    string message
    uint8[] respond_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetToolModbusResponse(null);
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

    if (msg.respond_data !== undefined) {
      resolved.respond_data = msg.respond_data;
    }
    else {
      resolved.respond_data = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SetToolModbusRequest,
  Response: SetToolModbusResponse,
  md5sum() { return '0b8299eb42a5262c78587a39fca90c6d'; },
  datatype() { return 'xarm_msgs/SetToolModbus'; }
};
