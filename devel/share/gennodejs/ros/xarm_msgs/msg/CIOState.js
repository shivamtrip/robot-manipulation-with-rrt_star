// Auto-generated. Do not edit!

// (in-package xarm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CIOState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.code = null;
      this.input_digitals = null;
      this.output_digitals = null;
      this.input_analogs = null;
      this.output_analogs = null;
      this.input_conf = null;
      this.output_conf = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('input_digitals')) {
        this.input_digitals = initObj.input_digitals
      }
      else {
        this.input_digitals = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('output_digitals')) {
        this.output_digitals = initObj.output_digitals
      }
      else {
        this.output_digitals = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('input_analogs')) {
        this.input_analogs = initObj.input_analogs
      }
      else {
        this.input_analogs = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('output_analogs')) {
        this.output_analogs = initObj.output_analogs
      }
      else {
        this.output_analogs = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('input_conf')) {
        this.input_conf = initObj.input_conf
      }
      else {
        this.input_conf = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('output_conf')) {
        this.output_conf = initObj.output_conf
      }
      else {
        this.output_conf = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CIOState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int16(obj.state, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.int16(obj.code, buffer, bufferOffset);
    // Check that the constant length array field [input_digitals] has the right length
    if (obj.input_digitals.length !== 2) {
      throw new Error('Unable to serialize array field input_digitals - length must be 2')
    }
    // Serialize message field [input_digitals]
    bufferOffset = _arraySerializer.uint16(obj.input_digitals, buffer, bufferOffset, 2);
    // Check that the constant length array field [output_digitals] has the right length
    if (obj.output_digitals.length !== 2) {
      throw new Error('Unable to serialize array field output_digitals - length must be 2')
    }
    // Serialize message field [output_digitals]
    bufferOffset = _arraySerializer.uint16(obj.output_digitals, buffer, bufferOffset, 2);
    // Check that the constant length array field [input_analogs] has the right length
    if (obj.input_analogs.length !== 2) {
      throw new Error('Unable to serialize array field input_analogs - length must be 2')
    }
    // Serialize message field [input_analogs]
    bufferOffset = _arraySerializer.float32(obj.input_analogs, buffer, bufferOffset, 2);
    // Check that the constant length array field [output_analogs] has the right length
    if (obj.output_analogs.length !== 2) {
      throw new Error('Unable to serialize array field output_analogs - length must be 2')
    }
    // Serialize message field [output_analogs]
    bufferOffset = _arraySerializer.float32(obj.output_analogs, buffer, bufferOffset, 2);
    // Check that the constant length array field [input_conf] has the right length
    if (obj.input_conf.length !== 16) {
      throw new Error('Unable to serialize array field input_conf - length must be 16')
    }
    // Serialize message field [input_conf]
    bufferOffset = _arraySerializer.uint8(obj.input_conf, buffer, bufferOffset, 16);
    // Check that the constant length array field [output_conf] has the right length
    if (obj.output_conf.length !== 16) {
      throw new Error('Unable to serialize array field output_conf - length must be 16')
    }
    // Serialize message field [output_conf]
    bufferOffset = _arraySerializer.uint8(obj.output_conf, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CIOState
    let len;
    let data = new CIOState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [input_digitals]
    data.input_digitals = _arrayDeserializer.uint16(buffer, bufferOffset, 2)
    // Deserialize message field [output_digitals]
    data.output_digitals = _arrayDeserializer.uint16(buffer, bufferOffset, 2)
    // Deserialize message field [input_analogs]
    data.input_analogs = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [output_analogs]
    data.output_analogs = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [input_conf]
    data.input_conf = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    // Deserialize message field [output_conf]
    data.output_conf = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xarm_msgs/CIOState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a51ddec3163bc4c38eaa278474b2dc1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # contorller gpio module state
    int16 state
    
    # controller gpio module error code
    int16 code
    
    # input_digitals[0]: digital input functional gpio state
    # input_digitals[1]: digital input configuring gpio state
    #    CI0: (input_digitals[1] >> 0) & 0x0001
    #    CI1: (input_digitals[1] >> 1) & 0x0001
    #    CI7: (input_digitals[1] >> 7) & 0x0001
    #    DI0: (input_digitals[1] >> 8) & 0x0001
    #    DI1: (input_digitals[1] >> 9) & 0x0001
    #    DI7: (input_digitals[1] >> 15) & 0x0001
    uint16[2] input_digitals
    
    # output_digitals[0]: digital output functional gpio state
    # output_digitals[1]: digital output configuring gpio state
    #    CO0: (output_digitals[1] >> 0) & 0x0001
    #    CO1: (output_digitals[1] >> 1) & 0x0001
    #    CO7: (output_digitals[1] >> 7) & 0x0001
    #    DO0: (output_digitals[1] >> 8) & 0x0001
    #    DO1: (output_digitals[1] >> 9) & 0x0001
    #    DO7: (output_digitals[1] >> 15) & 0x0001
    uint16[2] output_digitals
    
    # input_analogs[0]: the value of AI0
    # input_analogs[1]: the value of AI1
    float32[2] input_analogs
    
    # output_analogs[0]: the value of AO0
    # output_analogs[1]: the value of AO1
    float32[2] output_analogs
    
    # digital input functional info
    uint8[16] input_conf
    
    # digital output functional info
    uint8[16] output_conf
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CIOState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.input_digitals !== undefined) {
      resolved.input_digitals = msg.input_digitals;
    }
    else {
      resolved.input_digitals = new Array(2).fill(0)
    }

    if (msg.output_digitals !== undefined) {
      resolved.output_digitals = msg.output_digitals;
    }
    else {
      resolved.output_digitals = new Array(2).fill(0)
    }

    if (msg.input_analogs !== undefined) {
      resolved.input_analogs = msg.input_analogs;
    }
    else {
      resolved.input_analogs = new Array(2).fill(0)
    }

    if (msg.output_analogs !== undefined) {
      resolved.output_analogs = msg.output_analogs;
    }
    else {
      resolved.output_analogs = new Array(2).fill(0)
    }

    if (msg.input_conf !== undefined) {
      resolved.input_conf = msg.input_conf;
    }
    else {
      resolved.input_conf = new Array(16).fill(0)
    }

    if (msg.output_conf !== undefined) {
      resolved.output_conf = msg.output_conf;
    }
    else {
      resolved.output_conf = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = CIOState;
