// Auto-generated. Do not edit!

// (in-package landmarks_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Landmark_Msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.label = null;
      this.x = null;
      this.y = null;
      this.s_x = null;
      this.s_y = null;
    }
    else {
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('s_x')) {
        this.s_x = initObj.s_x
      }
      else {
        this.s_x = 0.0;
      }
      if (initObj.hasOwnProperty('s_y')) {
        this.s_y = initObj.s_y
      }
      else {
        this.s_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Landmark_Msg
    // Serialize message field [label]
    bufferOffset = _serializer.uint64(obj.label, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [s_x]
    bufferOffset = _serializer.float32(obj.s_x, buffer, bufferOffset);
    // Serialize message field [s_y]
    bufferOffset = _serializer.float32(obj.s_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Landmark_Msg
    let len;
    let data = new Landmark_Msg(null);
    // Deserialize message field [label]
    data.label = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s_x]
    data.s_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [s_y]
    data.s_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'landmarks_msg/Landmark_Msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37586ccc5f4ed3186510933e2fe487e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint64 label
    float32 x
    float32 y
    float32 s_x 
    float32 s_y 
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Landmark_Msg(null);
    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.s_x !== undefined) {
      resolved.s_x = msg.s_x;
    }
    else {
      resolved.s_x = 0.0
    }

    if (msg.s_y !== undefined) {
      resolved.s_y = msg.s_y;
    }
    else {
      resolved.s_y = 0.0
    }

    return resolved;
    }
};

module.exports = Landmark_Msg;
