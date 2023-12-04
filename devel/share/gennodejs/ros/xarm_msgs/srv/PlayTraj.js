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

class PlayTrajRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.traj_file = null;
      this.repeat_times = null;
      this.speed_factor = null;
    }
    else {
      if (initObj.hasOwnProperty('traj_file')) {
        this.traj_file = initObj.traj_file
      }
      else {
        this.traj_file = '';
      }
      if (initObj.hasOwnProperty('repeat_times')) {
        this.repeat_times = initObj.repeat_times
      }
      else {
        this.repeat_times = 0;
      }
      if (initObj.hasOwnProperty('speed_factor')) {
        this.speed_factor = initObj.speed_factor
      }
      else {
        this.speed_factor = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlayTrajRequest
    // Serialize message field [traj_file]
    bufferOffset = _serializer.string(obj.traj_file, buffer, bufferOffset);
    // Serialize message field [repeat_times]
    bufferOffset = _serializer.int16(obj.repeat_times, buffer, bufferOffset);
    // Serialize message field [speed_factor]
    bufferOffset = _serializer.int16(obj.speed_factor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayTrajRequest
    let len;
    let data = new PlayTrajRequest(null);
    // Deserialize message field [traj_file]
    data.traj_file = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [repeat_times]
    data.repeat_times = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [speed_factor]
    data.speed_factor = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.traj_file);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/PlayTrajRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cc25d7fcc3dc3c27c76051b595cd231';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request: set multiple integer parameters.
    # traj_file: file name of trajectory record, do not forget ".traj" suffix
    # repeat_times: how many times will be played
    # speed_factor: supported values are 1 (original speed), 2 (2x speed), 4 (4x speed, not recommended). 
    
    string traj_file
    int16 repeat_times
    int16 speed_factor
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlayTrajRequest(null);
    if (msg.traj_file !== undefined) {
      resolved.traj_file = msg.traj_file;
    }
    else {
      resolved.traj_file = ''
    }

    if (msg.repeat_times !== undefined) {
      resolved.repeat_times = msg.repeat_times;
    }
    else {
      resolved.repeat_times = 0
    }

    if (msg.speed_factor !== undefined) {
      resolved.speed_factor = msg.speed_factor;
    }
    else {
      resolved.speed_factor = 0
    }

    return resolved;
    }
};

class PlayTrajResponse {
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
    // Serializes a message object of type PlayTrajResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayTrajResponse
    let len;
    let data = new PlayTrajResponse(null);
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
    return 'xarm_msgs/PlayTrajResponse';
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
    const resolved = new PlayTrajResponse(null);
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
  Request: PlayTrajRequest,
  Response: PlayTrajResponse,
  md5sum() { return '80c963774ceddb6d2f6133dfc972a9dd'; },
  datatype() { return 'xarm_msgs/PlayTraj'; }
};
