// Auto-generated. Do not edit!

// (in-package xarm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class IOState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.digital_1 = null;
      this.digital_2 = null;
      this.analog_1 = null;
      this.analog_2 = null;
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
      if (initObj.hasOwnProperty('analog_1')) {
        this.analog_1 = initObj.analog_1
      }
      else {
        this.analog_1 = 0.0;
      }
      if (initObj.hasOwnProperty('analog_2')) {
        this.analog_2 = initObj.analog_2
      }
      else {
        this.analog_2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IOState
    // Serialize message field [digital_1]
    bufferOffset = _serializer.int32(obj.digital_1, buffer, bufferOffset);
    // Serialize message field [digital_2]
    bufferOffset = _serializer.int32(obj.digital_2, buffer, bufferOffset);
    // Serialize message field [analog_1]
    bufferOffset = _serializer.float32(obj.analog_1, buffer, bufferOffset);
    // Serialize message field [analog_2]
    bufferOffset = _serializer.float32(obj.analog_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IOState
    let len;
    let data = new IOState(null);
    // Deserialize message field [digital_1]
    data.digital_1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [digital_2]
    data.digital_2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [analog_1]
    data.analog_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [analog_2]
    data.analog_2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xarm_msgs/IOState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf35d5117b3f1964a89d007e947ecbee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # for indicating 2 digital and 2 analog Input port state
    
    int32 digital_1
    
    int32 digital_2
    
    float32 analog_1
    
    float32 analog_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IOState(null);
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

    if (msg.analog_1 !== undefined) {
      resolved.analog_1 = msg.analog_1;
    }
    else {
      resolved.analog_1 = 0.0
    }

    if (msg.analog_2 !== undefined) {
      resolved.analog_2 = msg.analog_2;
    }
    else {
      resolved.analog_2 = 0.0
    }

    return resolved;
    }
};

module.exports = IOState;
