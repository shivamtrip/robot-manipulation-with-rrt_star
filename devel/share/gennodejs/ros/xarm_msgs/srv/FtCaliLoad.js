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

class FtCaliLoadRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.datas = null;
      this.association_setting_tcp_load = null;
    }
    else {
      if (initObj.hasOwnProperty('datas')) {
        this.datas = initObj.datas
      }
      else {
        this.datas = [];
      }
      if (initObj.hasOwnProperty('association_setting_tcp_load')) {
        this.association_setting_tcp_load = initObj.association_setting_tcp_load
      }
      else {
        this.association_setting_tcp_load = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FtCaliLoadRequest
    // Serialize message field [datas]
    bufferOffset = _arraySerializer.float32(obj.datas, buffer, bufferOffset, null);
    // Serialize message field [association_setting_tcp_load]
    bufferOffset = _serializer.bool(obj.association_setting_tcp_load, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FtCaliLoadRequest
    let len;
    let data = new FtCaliLoadRequest(null);
    // Deserialize message field [datas]
    data.datas = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [association_setting_tcp_load]
    data.association_setting_tcp_load = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.datas.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/FtCaliLoadRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1a744721d8bd85af94cc035f3af6940';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This format is suitable for the following services
    #   - ft_sensor_cali_load
    
    # iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
    float32[] datas
    
    # whether to convert the paramster to the corresponding tcp load and set
    bool association_setting_tcp_load
    # float32 m   0.325
    # float32 x   -17.0
    # float32 y   9.0
    # float32 z   11.8
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FtCaliLoadRequest(null);
    if (msg.datas !== undefined) {
      resolved.datas = msg.datas;
    }
    else {
      resolved.datas = []
    }

    if (msg.association_setting_tcp_load !== undefined) {
      resolved.association_setting_tcp_load = msg.association_setting_tcp_load;
    }
    else {
      resolved.association_setting_tcp_load = false
    }

    return resolved;
    }
};

class FtCaliLoadResponse {
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
    // Serializes a message object of type FtCaliLoadResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FtCaliLoadResponse
    let len;
    let data = new FtCaliLoadResponse(null);
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
    return 'xarm_msgs/FtCaliLoadResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76c68a2c5e109f4d6a4dc1cfc355f405';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int16 ret
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FtCaliLoadResponse(null);
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
  Request: FtCaliLoadRequest,
  Response: FtCaliLoadResponse,
  md5sum() { return '002aec91323cfbf6a70a5f91f59ae0cf'; },
  datatype() { return 'xarm_msgs/FtCaliLoad'; }
};
