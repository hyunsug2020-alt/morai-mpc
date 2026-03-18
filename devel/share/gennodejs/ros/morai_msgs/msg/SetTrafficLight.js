// Auto-generated. Do not edit!

// (in-package morai_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SetTrafficLight {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.traffic_light_index = null;
      this.traffic_light_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('traffic_light_index')) {
        this.traffic_light_index = initObj.traffic_light_index
      }
      else {
        this.traffic_light_index = '';
      }
      if (initObj.hasOwnProperty('traffic_light_status')) {
        this.traffic_light_status = initObj.traffic_light_status
      }
      else {
        this.traffic_light_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTrafficLight
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [traffic_light_index]
    bufferOffset = _serializer.string(obj.traffic_light_index, buffer, bufferOffset);
    // Serialize message field [traffic_light_status]
    bufferOffset = _serializer.int16(obj.traffic_light_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTrafficLight
    let len;
    let data = new SetTrafficLight(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [traffic_light_index]
    data.traffic_light_index = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [traffic_light_status]
    data.traffic_light_status = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.traffic_light_index);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'morai_msgs/SetTrafficLight';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d80dc8acfeb7a2e665a8844435f9a44';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    string traffic_light_index
    int16 traffic_light_status
    
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
    const resolved = new SetTrafficLight(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.traffic_light_index !== undefined) {
      resolved.traffic_light_index = msg.traffic_light_index;
    }
    else {
      resolved.traffic_light_index = ''
    }

    if (msg.traffic_light_status !== undefined) {
      resolved.traffic_light_status = msg.traffic_light_status;
    }
    else {
      resolved.traffic_light_status = 0
    }

    return resolved;
    }
};

module.exports = SetTrafficLight;
