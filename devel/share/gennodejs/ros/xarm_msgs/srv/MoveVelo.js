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

class MoveVeloRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velocities = null;
      this.jnt_sync = null;
      this.coord = null;
    }
    else {
      if (initObj.hasOwnProperty('velocities')) {
        this.velocities = initObj.velocities
      }
      else {
        this.velocities = [];
      }
      if (initObj.hasOwnProperty('jnt_sync')) {
        this.jnt_sync = initObj.jnt_sync
      }
      else {
        this.jnt_sync = 0;
      }
      if (initObj.hasOwnProperty('coord')) {
        this.coord = initObj.coord
      }
      else {
        this.coord = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveVeloRequest
    // Serialize message field [velocities]
    bufferOffset = _arraySerializer.float32(obj.velocities, buffer, bufferOffset, null);
    // Serialize message field [jnt_sync]
    bufferOffset = _serializer.int16(obj.jnt_sync, buffer, bufferOffset);
    // Serialize message field [coord]
    bufferOffset = _serializer.int16(obj.coord, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveVeloRequest
    let len;
    let data = new MoveVeloRequest(null);
    // Deserialize message field [velocities]
    data.velocities = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [jnt_sync]
    data.jnt_sync = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [coord]
    data.coord = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.velocities.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveVeloRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de45efefc7a22e2ad261a65d9d8c2df1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request: command specification for velocity executions.
    # Units:
    #	joint space/angles: radian/s
    #	Cartesian space: mm/s, radian/s.
    
    # velocities: the velocity list of the joints/tcp
    #   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]
    #   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]
    float32[] velocities
    
    # jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.
    int16 jnt_sync
    
    # coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)
    int16 coord
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveVeloRequest(null);
    if (msg.velocities !== undefined) {
      resolved.velocities = msg.velocities;
    }
    else {
      resolved.velocities = []
    }

    if (msg.jnt_sync !== undefined) {
      resolved.jnt_sync = msg.jnt_sync;
    }
    else {
      resolved.jnt_sync = 0
    }

    if (msg.coord !== undefined) {
      resolved.coord = msg.coord;
    }
    else {
      resolved.coord = 0
    }

    return resolved;
    }
};

class MoveVeloResponse {
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
    // Serializes a message object of type MoveVeloResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveVeloResponse
    let len;
    let data = new MoveVeloResponse(null);
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
    return 'xarm_msgs/MoveVeloResponse';
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
    const resolved = new MoveVeloResponse(null);
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
  Request: MoveVeloRequest,
  Response: MoveVeloResponse,
  md5sum() { return '7ab1cadf314da821f631e86ead48e30d'; },
  datatype() { return 'xarm_msgs/MoveVelo'; }
};
