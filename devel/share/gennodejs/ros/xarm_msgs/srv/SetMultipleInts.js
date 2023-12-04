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

class SetMultipleIntsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data_int = null;
    }
    else {
      if (initObj.hasOwnProperty('data_int')) {
        this.data_int = initObj.data_int
      }
      else {
        this.data_int = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMultipleIntsRequest
    // Serialize message field [data_int]
    bufferOffset = _arraySerializer.int16(obj.data_int, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMultipleIntsRequest
    let len;
    let data = new SetMultipleIntsRequest(null);
    // Deserialize message field [data_int]
    data.data_int = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.data_int.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/SetMultipleIntsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44acdf287a0679e58f3ebc0c5712b826';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request: set multiple integer parameters.
    # data_int: int values to be set.
    
    int16[] data_int
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMultipleIntsRequest(null);
    if (msg.data_int !== undefined) {
      resolved.data_int = msg.data_int;
    }
    else {
      resolved.data_int = []
    }

    return resolved;
    }
};

class SetMultipleIntsResponse {
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
    // Serializes a message object of type SetMultipleIntsResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMultipleIntsResponse
    let len;
    let data = new SetMultipleIntsResponse(null);
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
    return 'xarm_msgs/SetMultipleIntsResponse';
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
    const resolved = new SetMultipleIntsResponse(null);
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
  Request: SetMultipleIntsRequest,
  Response: SetMultipleIntsResponse,
  md5sum() { return '72a7c6b1ca8071c36cdc4733c55fc7f3'; },
  datatype() { return 'xarm_msgs/SetMultipleInts'; }
};
