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

class GetSetModbusDataRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.send_data = null;
      this.respond_len = null;
      this.host_id = null;
      this.is_transparent_transmission = null;
      this.use_503_port = null;
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
      if (initObj.hasOwnProperty('host_id')) {
        this.host_id = initObj.host_id
      }
      else {
        this.host_id = 0;
      }
      if (initObj.hasOwnProperty('is_transparent_transmission')) {
        this.is_transparent_transmission = initObj.is_transparent_transmission
      }
      else {
        this.is_transparent_transmission = false;
      }
      if (initObj.hasOwnProperty('use_503_port')) {
        this.use_503_port = initObj.use_503_port
      }
      else {
        this.use_503_port = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSetModbusDataRequest
    // Serialize message field [send_data]
    bufferOffset = _arraySerializer.uint8(obj.send_data, buffer, bufferOffset, null);
    // Serialize message field [respond_len]
    bufferOffset = _serializer.int16(obj.respond_len, buffer, bufferOffset);
    // Serialize message field [host_id]
    bufferOffset = _serializer.uint8(obj.host_id, buffer, bufferOffset);
    // Serialize message field [is_transparent_transmission]
    bufferOffset = _serializer.bool(obj.is_transparent_transmission, buffer, bufferOffset);
    // Serialize message field [use_503_port]
    bufferOffset = _serializer.bool(obj.use_503_port, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSetModbusDataRequest
    let len;
    let data = new GetSetModbusDataRequest(null);
    // Deserialize message field [send_data]
    data.send_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [respond_len]
    data.respond_len = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [host_id]
    data.host_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [is_transparent_transmission]
    data.is_transparent_transmission = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [use_503_port]
    data.use_503_port = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.send_data.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetSetModbusDataRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0df4146f0963d1cf15d364518c202d50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # unsigned char data to be sent to tool device through modbus
    uint8[] send_data
    
    # Specify the anticipated maximum respond data length in bytes
    int16 respond_len
    
    # host id, 9: END RS485, 10: Controller RS485
    uint8 host_id
    
    # whether to choose transparent transmission
    bool is_transparent_transmission
    
    # whether to use port 503 for communication
    # if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission
    bool use_503_port
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSetModbusDataRequest(null);
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

    if (msg.host_id !== undefined) {
      resolved.host_id = msg.host_id;
    }
    else {
      resolved.host_id = 0
    }

    if (msg.is_transparent_transmission !== undefined) {
      resolved.is_transparent_transmission = msg.is_transparent_transmission;
    }
    else {
      resolved.is_transparent_transmission = false
    }

    if (msg.use_503_port !== undefined) {
      resolved.use_503_port = msg.use_503_port;
    }
    else {
      resolved.use_503_port = false
    }

    return resolved;
    }
};

class GetSetModbusDataResponse {
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
    // Serializes a message object of type GetSetModbusDataResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [respond_data]
    bufferOffset = _arraySerializer.uint8(obj.respond_data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSetModbusDataResponse
    let len;
    let data = new GetSetModbusDataResponse(null);
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
    return 'xarm_msgs/GetSetModbusDataResponse';
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
    const resolved = new GetSetModbusDataResponse(null);
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
  Request: GetSetModbusDataRequest,
  Response: GetSetModbusDataResponse,
  md5sum() { return 'dc342acff6bdfadcdcdb95fac36d565f'; },
  datatype() { return 'xarm_msgs/GetSetModbusData'; }
};
