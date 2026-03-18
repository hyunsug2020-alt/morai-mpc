// Auto-generated. Do not edit!

// (in-package morai_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EgoVehicleStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.unique_id = null;
      this.acceleration = null;
      this.position = null;
      this.velocity = null;
      this.angular_velocity = null;
      this.heading = null;
      this.accel = null;
      this.brake = null;
      this.front_steer = null;
      this.rear_steer = null;
      this.tire_lateral_force_fl = null;
      this.tire_lateral_force_fr = null;
      this.tire_lateral_force_rl = null;
      this.tire_lateral_force_rr = null;
      this.side_slip_angle_fl = null;
      this.side_slip_angle_fr = null;
      this.side_slip_angle_rl = null;
      this.side_slip_angle_rr = null;
      this.tire_cornering_stiffness_fl = null;
      this.tire_cornering_stiffness_fr = null;
      this.tire_cornering_stiffness_rl = null;
      this.tire_cornering_stiffness_rr = null;
      this.distance_left_lane_boundary = null;
      this.distance_right_lane_boundary = null;
      this.cross_track_error = null;
      this.wheel_angle = null;
      this.gear = null;
      this.ctrl_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('unique_id')) {
        this.unique_id = initObj.unique_id
      }
      else {
        this.unique_id = 0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = 0.0;
      }
      if (initObj.hasOwnProperty('brake')) {
        this.brake = initObj.brake
      }
      else {
        this.brake = 0.0;
      }
      if (initObj.hasOwnProperty('front_steer')) {
        this.front_steer = initObj.front_steer
      }
      else {
        this.front_steer = 0.0;
      }
      if (initObj.hasOwnProperty('rear_steer')) {
        this.rear_steer = initObj.rear_steer
      }
      else {
        this.rear_steer = 0.0;
      }
      if (initObj.hasOwnProperty('tire_lateral_force_fl')) {
        this.tire_lateral_force_fl = initObj.tire_lateral_force_fl
      }
      else {
        this.tire_lateral_force_fl = 0.0;
      }
      if (initObj.hasOwnProperty('tire_lateral_force_fr')) {
        this.tire_lateral_force_fr = initObj.tire_lateral_force_fr
      }
      else {
        this.tire_lateral_force_fr = 0.0;
      }
      if (initObj.hasOwnProperty('tire_lateral_force_rl')) {
        this.tire_lateral_force_rl = initObj.tire_lateral_force_rl
      }
      else {
        this.tire_lateral_force_rl = 0.0;
      }
      if (initObj.hasOwnProperty('tire_lateral_force_rr')) {
        this.tire_lateral_force_rr = initObj.tire_lateral_force_rr
      }
      else {
        this.tire_lateral_force_rr = 0.0;
      }
      if (initObj.hasOwnProperty('side_slip_angle_fl')) {
        this.side_slip_angle_fl = initObj.side_slip_angle_fl
      }
      else {
        this.side_slip_angle_fl = 0.0;
      }
      if (initObj.hasOwnProperty('side_slip_angle_fr')) {
        this.side_slip_angle_fr = initObj.side_slip_angle_fr
      }
      else {
        this.side_slip_angle_fr = 0.0;
      }
      if (initObj.hasOwnProperty('side_slip_angle_rl')) {
        this.side_slip_angle_rl = initObj.side_slip_angle_rl
      }
      else {
        this.side_slip_angle_rl = 0.0;
      }
      if (initObj.hasOwnProperty('side_slip_angle_rr')) {
        this.side_slip_angle_rr = initObj.side_slip_angle_rr
      }
      else {
        this.side_slip_angle_rr = 0.0;
      }
      if (initObj.hasOwnProperty('tire_cornering_stiffness_fl')) {
        this.tire_cornering_stiffness_fl = initObj.tire_cornering_stiffness_fl
      }
      else {
        this.tire_cornering_stiffness_fl = 0.0;
      }
      if (initObj.hasOwnProperty('tire_cornering_stiffness_fr')) {
        this.tire_cornering_stiffness_fr = initObj.tire_cornering_stiffness_fr
      }
      else {
        this.tire_cornering_stiffness_fr = 0.0;
      }
      if (initObj.hasOwnProperty('tire_cornering_stiffness_rl')) {
        this.tire_cornering_stiffness_rl = initObj.tire_cornering_stiffness_rl
      }
      else {
        this.tire_cornering_stiffness_rl = 0.0;
      }
      if (initObj.hasOwnProperty('tire_cornering_stiffness_rr')) {
        this.tire_cornering_stiffness_rr = initObj.tire_cornering_stiffness_rr
      }
      else {
        this.tire_cornering_stiffness_rr = 0.0;
      }
      if (initObj.hasOwnProperty('distance_left_lane_boundary')) {
        this.distance_left_lane_boundary = initObj.distance_left_lane_boundary
      }
      else {
        this.distance_left_lane_boundary = 0.0;
      }
      if (initObj.hasOwnProperty('distance_right_lane_boundary')) {
        this.distance_right_lane_boundary = initObj.distance_right_lane_boundary
      }
      else {
        this.distance_right_lane_boundary = 0.0;
      }
      if (initObj.hasOwnProperty('cross_track_error')) {
        this.cross_track_error = initObj.cross_track_error
      }
      else {
        this.cross_track_error = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_angle')) {
        this.wheel_angle = initObj.wheel_angle
      }
      else {
        this.wheel_angle = 0.0;
      }
      if (initObj.hasOwnProperty('gear')) {
        this.gear = initObj.gear
      }
      else {
        this.gear = 0;
      }
      if (initObj.hasOwnProperty('ctrl_mode')) {
        this.ctrl_mode = initObj.ctrl_mode
      }
      else {
        this.ctrl_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EgoVehicleStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [unique_id]
    bufferOffset = _serializer.int32(obj.unique_id, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_velocity, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float64(obj.heading, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = _serializer.float64(obj.accel, buffer, bufferOffset);
    // Serialize message field [brake]
    bufferOffset = _serializer.float64(obj.brake, buffer, bufferOffset);
    // Serialize message field [front_steer]
    bufferOffset = _serializer.float64(obj.front_steer, buffer, bufferOffset);
    // Serialize message field [rear_steer]
    bufferOffset = _serializer.float64(obj.rear_steer, buffer, bufferOffset);
    // Serialize message field [tire_lateral_force_fl]
    bufferOffset = _serializer.float32(obj.tire_lateral_force_fl, buffer, bufferOffset);
    // Serialize message field [tire_lateral_force_fr]
    bufferOffset = _serializer.float32(obj.tire_lateral_force_fr, buffer, bufferOffset);
    // Serialize message field [tire_lateral_force_rl]
    bufferOffset = _serializer.float32(obj.tire_lateral_force_rl, buffer, bufferOffset);
    // Serialize message field [tire_lateral_force_rr]
    bufferOffset = _serializer.float32(obj.tire_lateral_force_rr, buffer, bufferOffset);
    // Serialize message field [side_slip_angle_fl]
    bufferOffset = _serializer.float32(obj.side_slip_angle_fl, buffer, bufferOffset);
    // Serialize message field [side_slip_angle_fr]
    bufferOffset = _serializer.float32(obj.side_slip_angle_fr, buffer, bufferOffset);
    // Serialize message field [side_slip_angle_rl]
    bufferOffset = _serializer.float32(obj.side_slip_angle_rl, buffer, bufferOffset);
    // Serialize message field [side_slip_angle_rr]
    bufferOffset = _serializer.float32(obj.side_slip_angle_rr, buffer, bufferOffset);
    // Serialize message field [tire_cornering_stiffness_fl]
    bufferOffset = _serializer.float32(obj.tire_cornering_stiffness_fl, buffer, bufferOffset);
    // Serialize message field [tire_cornering_stiffness_fr]
    bufferOffset = _serializer.float32(obj.tire_cornering_stiffness_fr, buffer, bufferOffset);
    // Serialize message field [tire_cornering_stiffness_rl]
    bufferOffset = _serializer.float32(obj.tire_cornering_stiffness_rl, buffer, bufferOffset);
    // Serialize message field [tire_cornering_stiffness_rr]
    bufferOffset = _serializer.float32(obj.tire_cornering_stiffness_rr, buffer, bufferOffset);
    // Serialize message field [distance_left_lane_boundary]
    bufferOffset = _serializer.float32(obj.distance_left_lane_boundary, buffer, bufferOffset);
    // Serialize message field [distance_right_lane_boundary]
    bufferOffset = _serializer.float32(obj.distance_right_lane_boundary, buffer, bufferOffset);
    // Serialize message field [cross_track_error]
    bufferOffset = _serializer.float32(obj.cross_track_error, buffer, bufferOffset);
    // Serialize message field [wheel_angle]
    bufferOffset = _serializer.float64(obj.wheel_angle, buffer, bufferOffset);
    // Serialize message field [gear]
    bufferOffset = _serializer.int32(obj.gear, buffer, bufferOffset);
    // Serialize message field [ctrl_mode]
    bufferOffset = _serializer.int32(obj.ctrl_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EgoVehicleStatus
    let len;
    let data = new EgoVehicleStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [unique_id]
    data.unique_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [brake]
    data.brake = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_steer]
    data.front_steer = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rear_steer]
    data.rear_steer = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tire_lateral_force_fl]
    data.tire_lateral_force_fl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_lateral_force_fr]
    data.tire_lateral_force_fr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_lateral_force_rl]
    data.tire_lateral_force_rl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_lateral_force_rr]
    data.tire_lateral_force_rr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [side_slip_angle_fl]
    data.side_slip_angle_fl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [side_slip_angle_fr]
    data.side_slip_angle_fr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [side_slip_angle_rl]
    data.side_slip_angle_rl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [side_slip_angle_rr]
    data.side_slip_angle_rr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_cornering_stiffness_fl]
    data.tire_cornering_stiffness_fl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_cornering_stiffness_fr]
    data.tire_cornering_stiffness_fr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_cornering_stiffness_rl]
    data.tire_cornering_stiffness_rl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tire_cornering_stiffness_rr]
    data.tire_cornering_stiffness_rr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance_left_lane_boundary]
    data.distance_left_lane_boundary = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance_right_lane_boundary]
    data.distance_right_lane_boundary = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cross_track_error]
    data.cross_track_error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wheel_angle]
    data.wheel_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gear]
    data.gear = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ctrl_mode]
    data.ctrl_mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 216;
  }

  static datatype() {
    // Returns string type for a message object
    return 'morai_msgs/EgoVehicleStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '805ccb1a6ed8e5235a26921c3f7a53d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int32 unique_id
    geometry_msgs/Vector3 acceleration
    geometry_msgs/Vector3 position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 angular_velocity
    
    float64 heading
    float64 accel
    float64 brake
    float64 front_steer
    float64 rear_steer
    
    float32 tire_lateral_force_fl
    float32 tire_lateral_force_fr
    float32 tire_lateral_force_rl
    float32 tire_lateral_force_rr
    
    float32 side_slip_angle_fl
    float32 side_slip_angle_fr
    float32 side_slip_angle_rl
    float32 side_slip_angle_rr
    
    float32 tire_cornering_stiffness_fl
    float32 tire_cornering_stiffness_fr
    float32 tire_cornering_stiffness_rl
    float32 tire_cornering_stiffness_rr
    
    float32 distance_left_lane_boundary
    float32 distance_right_lane_boundary
    float32 cross_track_error
    
    float64 wheel_angle
    int32 gear
    int32 ctrl_mode
    
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
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EgoVehicleStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.unique_id !== undefined) {
      resolved.unique_id = msg.unique_id;
    }
    else {
      resolved.unique_id = 0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = geometry_msgs.msg.Vector3.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity)
    }
    else {
      resolved.angular_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.accel !== undefined) {
      resolved.accel = msg.accel;
    }
    else {
      resolved.accel = 0.0
    }

    if (msg.brake !== undefined) {
      resolved.brake = msg.brake;
    }
    else {
      resolved.brake = 0.0
    }

    if (msg.front_steer !== undefined) {
      resolved.front_steer = msg.front_steer;
    }
    else {
      resolved.front_steer = 0.0
    }

    if (msg.rear_steer !== undefined) {
      resolved.rear_steer = msg.rear_steer;
    }
    else {
      resolved.rear_steer = 0.0
    }

    if (msg.tire_lateral_force_fl !== undefined) {
      resolved.tire_lateral_force_fl = msg.tire_lateral_force_fl;
    }
    else {
      resolved.tire_lateral_force_fl = 0.0
    }

    if (msg.tire_lateral_force_fr !== undefined) {
      resolved.tire_lateral_force_fr = msg.tire_lateral_force_fr;
    }
    else {
      resolved.tire_lateral_force_fr = 0.0
    }

    if (msg.tire_lateral_force_rl !== undefined) {
      resolved.tire_lateral_force_rl = msg.tire_lateral_force_rl;
    }
    else {
      resolved.tire_lateral_force_rl = 0.0
    }

    if (msg.tire_lateral_force_rr !== undefined) {
      resolved.tire_lateral_force_rr = msg.tire_lateral_force_rr;
    }
    else {
      resolved.tire_lateral_force_rr = 0.0
    }

    if (msg.side_slip_angle_fl !== undefined) {
      resolved.side_slip_angle_fl = msg.side_slip_angle_fl;
    }
    else {
      resolved.side_slip_angle_fl = 0.0
    }

    if (msg.side_slip_angle_fr !== undefined) {
      resolved.side_slip_angle_fr = msg.side_slip_angle_fr;
    }
    else {
      resolved.side_slip_angle_fr = 0.0
    }

    if (msg.side_slip_angle_rl !== undefined) {
      resolved.side_slip_angle_rl = msg.side_slip_angle_rl;
    }
    else {
      resolved.side_slip_angle_rl = 0.0
    }

    if (msg.side_slip_angle_rr !== undefined) {
      resolved.side_slip_angle_rr = msg.side_slip_angle_rr;
    }
    else {
      resolved.side_slip_angle_rr = 0.0
    }

    if (msg.tire_cornering_stiffness_fl !== undefined) {
      resolved.tire_cornering_stiffness_fl = msg.tire_cornering_stiffness_fl;
    }
    else {
      resolved.tire_cornering_stiffness_fl = 0.0
    }

    if (msg.tire_cornering_stiffness_fr !== undefined) {
      resolved.tire_cornering_stiffness_fr = msg.tire_cornering_stiffness_fr;
    }
    else {
      resolved.tire_cornering_stiffness_fr = 0.0
    }

    if (msg.tire_cornering_stiffness_rl !== undefined) {
      resolved.tire_cornering_stiffness_rl = msg.tire_cornering_stiffness_rl;
    }
    else {
      resolved.tire_cornering_stiffness_rl = 0.0
    }

    if (msg.tire_cornering_stiffness_rr !== undefined) {
      resolved.tire_cornering_stiffness_rr = msg.tire_cornering_stiffness_rr;
    }
    else {
      resolved.tire_cornering_stiffness_rr = 0.0
    }

    if (msg.distance_left_lane_boundary !== undefined) {
      resolved.distance_left_lane_boundary = msg.distance_left_lane_boundary;
    }
    else {
      resolved.distance_left_lane_boundary = 0.0
    }

    if (msg.distance_right_lane_boundary !== undefined) {
      resolved.distance_right_lane_boundary = msg.distance_right_lane_boundary;
    }
    else {
      resolved.distance_right_lane_boundary = 0.0
    }

    if (msg.cross_track_error !== undefined) {
      resolved.cross_track_error = msg.cross_track_error;
    }
    else {
      resolved.cross_track_error = 0.0
    }

    if (msg.wheel_angle !== undefined) {
      resolved.wheel_angle = msg.wheel_angle;
    }
    else {
      resolved.wheel_angle = 0.0
    }

    if (msg.gear !== undefined) {
      resolved.gear = msg.gear;
    }
    else {
      resolved.gear = 0
    }

    if (msg.ctrl_mode !== undefined) {
      resolved.ctrl_mode = msg.ctrl_mode;
    }
    else {
      resolved.ctrl_mode = 0
    }

    return resolved;
    }
};

module.exports = EgoVehicleStatus;
