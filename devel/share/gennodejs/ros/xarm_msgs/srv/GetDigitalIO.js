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

class GetDigitalIORequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDigitalIORequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDigitalIORequest
    let len;
    let data = new GetDigitalIORequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetDigitalIORequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Getting the 2 digital Input port at robot end connector
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDigitalIORequest(null);
    return resolved;
    }
};

class GetDigitalIOResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.digital_1 = null;
      this.digital_2 = null;
      this.ret = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('digital_1')) {
        this.digital_1 = initObj.digital_1
      }
      else {
        this.digital_1 = 0;
      }
      if (initObj.hasOwnProperty('digital_2')) {
        this.digital_2 = initObj.digital_2
      }
      else {
        this.digital_2 = 0;
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
    // Serializes a message object of type GetDigitalIOResponse
    // Serialize message field [digital_1]
    bufferOffset = _serializer.int32(obj.digital_1, buffer, bufferOffset);
    // Serialize message field [digital_2]
    bufferOffset = _serializer.int32(obj.digital_2, buffer, bufferOffset);
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDigitalIOResponse
    let len;
    let data = new GetDigitalIOResponse(null);
    // Deserialize message field [digital_1]
    data.digital_1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [digital_2]
    data.digital_2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/GetDigitalIOResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c2bfd923228c71a217e97a1d1747b99';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32 digital_1
    
    int32 digital_2
    
    int16 ret
    
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDigitalIOResponse(null);
    if (msg.digital_1 !== undefined) {
      resolved.digital_1 = msg.digital_1;
    }
    else {
      resolved.digital_1 = 0
    }

    if (msg.digital_2 !== undefined) {
      resolved.digital_2 = msg.digital_2;
    }
    else {
      resolved.digital_2 = 0
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
  Request: GetDigitalIORequest,
  Response: GetDigitalIOResponse,
  md5sum() { return '5c2bfd923228c71a217e97a1d1747b99'; },
  datatype() { return 'xarm_msgs/GetDigitalIO'; }
};
