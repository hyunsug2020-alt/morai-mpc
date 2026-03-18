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

class SaveSensorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.is_custom_file_name = null;
      this.custom_file_name = null;
      this.file_dir = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('is_custom_file_name')) {
        this.is_custom_file_name = initObj.is_custom_file_name
      }
      else {
        this.is_custom_file_name = false;
      }
      if (initObj.hasOwnProperty('custom_file_name')) {
        this.custom_file_name = initObj.custom_file_name
      }
      else {
        this.custom_file_name = '';
      }
      if (initObj.hasOwnProperty('file_dir')) {
        this.file_dir = initObj.file_dir
      }
      else {
        this.file_dir = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveSensorData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [is_custom_file_name]
    bufferOffset = _serializer.bool(obj.is_custom_file_name, buffer, bufferOffset);
    // Serialize message field [custom_file_name]
    bufferOffset = _serializer.string(obj.custom_file_name, buffer, bufferOffset);
    // Serialize message field [file_dir]
    bufferOffset = _serializer.string(obj.file_dir, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveSensorData
    let len;
    let data = new SaveSensorData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_custom_file_name]
    data.is_custom_file_name = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [custom_file_name]
    data.custom_file_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [file_dir]
    data.file_dir = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.custom_file_name);
    length += _getByteLength(object.file_dir);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'morai_msgs/SaveSensorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ea26c4b6cccd1ac8432c50be268450c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool is_custom_file_name
    string custom_file_name
    string file_dir
    
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
    const resolved = new SaveSensorData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.is_custom_file_name !== undefined) {
      resolved.is_custom_file_name = msg.is_custom_file_name;
    }
    else {
      resolved.is_custom_file_name = false
    }

    if (msg.custom_file_name !== undefined) {
      resolved.custom_file_name = msg.custom_file_name;
    }
    else {
      resolved.custom_file_name = ''
    }

    if (msg.file_dir !== undefined) {
      resolved.file_dir = msg.file_dir;
    }
    else {
      resolved.file_dir = ''
    }

    return resolved;
    }
};

module.exports = SaveSensorData;
