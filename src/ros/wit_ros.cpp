#include "../../include/wit_node/wit_ros.hpp"
#include <sensor_msgs/MagneticField.h>
#include <float.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ecl/streams/string_stream.hpp>
#include <string>

namespace wit {

WitRos::WitRos(string &node_name)
    : name_(node_name), slot_stream_data_(&WitRos::processStreamData, *this) {}

WitRos::~WitRos() {
  ROS_INFO_STREAM("Wit: waiting for wd_ thread finish[" << name_ << " ].");
}

bool WitRos::init(NodeHandle &nh) {
  advertiseTopics(nh);
  subscribeTopics(nh);
  slot_stream_data_.connect(name_ + string("/stream_data"));
  
  Parameter parameter;

  parameter.ns = name_;

  /*
  if (!nh.getParam("port", parameter.port_)) {
    ROS_ERROR_STREAM(
        "Wit : no wit device port given on the parameter server (e.g. "
        "/dev/ttyUSB0)["
        << name_ << "].");
    return false;
  }
  */

  /*********************
   ** Driver Init
   **********************/
  try {
    wd_.init(parameter);

    ros::Duration(0.1).sleep();
  } catch (const ecl::StandardException &e) {
    switch (e.flag()) {
      case (ecl::OpenError): {
        ROS_ERROR_STREAM("Wit : could not open connection ["
                         << parameter.wit_node_port_ << "][" << name_ << "].");
        break;
      }
      default: {
        ROS_ERROR_STREAM("Wit : initialisation failed [" << name_ << "].");
        ROS_DEBUG_STREAM(e.what());
        break;
      }
    }
    return false;
  }

  ROS_INFO_STREAM("Wit : open connection OK [" << parameter.wit_node_port_ << "][" << name_ << "].");
  return true;
}

bool WitRos::update() {
  if (wd_.isShutdown()) {
    ROS_ERROR_STREAM("Wit : Driver has been shutdown. Stopping update loop. ["
                     << name_ << "].");
    return false;
  }
  if (!wd_.isConnected()) {
    ROS_ERROR_STREAM(
        "Wit : arm serial port is not connetced, please connect the arm.");
    return false;
  }
  //ROS_INFO("processStreamData");
  processStreamData();
  return true;
}

void WitRos::advertiseTopics(NodeHandle &nh) {
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/wit/imu_data", 1000);
  mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("/wit/mag_data", 1000);
  gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
  raw_data_pub_ = nh.advertise<wit_node::ImuGpsRaw>("raw_data", 1000);
  related_yaw_pub_ = nh.advertise<std_msgs::Float64>("related_yaw", 1000);
}

void WitRos::subscribeTopics(NodeHandle &nh) {
  reset_offset_sub_ = nh.subscribe(string("reset_offset"), 10,
                                   &WitRos::subscribeResetOffset, this);
}
void WitRos::subscribeResetOffset(const std_msgs::Empty msg) {
  wd_.resetYawOffset();
}

void WitRos::processStreamData() {
  if (ok()) {
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    sensor_msgs::NavSatFix gps_msg;
    wit_node::ImuGpsRaw raw_msg;
    std_msgs::Float64 yaw_msg;
    imu_msg.header.frame_id = "imu_link";
    mag_msg.header.stamp = ros::Time::now();
    imu_msg.header.stamp = ros::Time::now();
    gps_msg.header.stamp = ros::Time::now();
    raw_msg.header.stamp = ros::Time::now();
    Data::IMUGPS data = wd_.getData();

    // This is a message to hold data from an IMU (Inertial Measurement Unit)
    // 
    // Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    // 
    // If the covariance of the measurement is known, it should be filled in (if all you know is the 
    // variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    // A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    // data a covariance will have to be assumed or gotten from some other source
    // 
    // If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    // estimate), please set element 0 of the associated covariance matrix to -1
    // If you are interpreting this message, please check for a value of -1 in the first element of each 
    // covariance matrix, and disregard the associated estimate.

    // Wit 901B gives linar acceleration in G: Convert G to MeterPerSecond for ROS
    imu_msg.linear_acceleration.x = data.a[0]; //*9.80665;
    imu_msg.linear_acceleration.y = data.a[1]; //*9.80665;
    imu_msg.linear_acceleration.z = data.a[2];  //Remove gravity: data.a[2]; //*9.80665;

    // Wit 901B gives angular velocities in DegreesPerSeconde: Convert to RadsPerSecond for ROS
    imu_msg.angular_velocity.x = data.w[0]; //0; //Remove roll (balanceo) data.w[0]; //*0.0174;
    imu_msg.angular_velocity.y = data.w[1]; //0; //Remove pitch (inclinación, arriba y abajo) data.w[1]; //*0.0174;
    imu_msg.angular_velocity.z = data.w[2]; //User yaw (giro) //*0.0174;

    // Wit901B magnetic field already converted to Teslas for ROS:
    mag_msg.magnetic_field.x = data.mag[0];
    mag_msg.magnetic_field.y = data.mag[1];
    mag_msg.magnetic_field.z = data.mag[2];

    tf2::Quaternion quaternion_tf2_quat;
    quaternion_tf2_quat[0] = data.q[0];
    quaternion_tf2_quat[1] = data.q[1];
    quaternion_tf2_quat[2] = data.q[2];
    quaternion_tf2_quat[3] = data.q[3];

    geometry_msgs::Quaternion quaternionFromQuatMsg = tf2::toMsg(quaternion_tf2_quat);
    imu_msg.orientation = quaternionFromQuatMsg;

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(data.rpy[0], data.rpy[1], data.rpy[2]);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    imu_msg.orientation = quaternion;

    imu_msg.orientation_covariance =
    {0.001, 0.0,    0.0,
     0.0,   0.001,  0.0,
     0.0,   0.0,    0.001};

    imu_msg.angular_velocity_covariance =
    {0.00001, 0.0,      0.0,
     0.0,     0.00001,  0.0,
     0.0,     0.0,      0.00001};
     
    imu_msg.linear_acceleration_covariance =
    {0.0000001,  0.0,  0.0,
     0.0,   0.0000001, 0.0,
     0.0,   0.0,  0.0000001};

    mag_msg.magnetic_field_covariance =
    {0.01,  0.0,  0.0,
     0.0,   0.01, 0.0,
     0.0,   0.0,  0.01};  

    gps_msg.altitude = data.altitude;
    gps_msg.latitude = data.latitude;
    gps_msg.longitude = data.longtitude;
    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    for (int i = 0; i < 3; i++) {
      raw_msg.acc.push_back(data.a[i]);
      raw_msg.gyro.push_back(data.w[i]);
      raw_msg.rpy.push_back(data.rpy[i]);
      raw_msg.mag.push_back(data.mag[i]);
      raw_msg.dop.push_back(data.gpsa[i]);
    }

    for (int i = 0; i < 4; i++) {
      raw_msg.ps.push_back(data.d[i]);
      raw_msg.quarternion.push_back(data.q[i]);
    }

    raw_msg.sn = data.satelites;
    raw_msg.gpsh = data.gpsh;
    raw_msg.gpsy = data.gpsy;
    raw_msg.gpsv = data.gpsv;
    raw_msg.ap = data.pressure;
    raw_msg.longtitude = data.longtitude;
    raw_msg.altitude = data.altitude;
    raw_msg.latitude = data.latitude;
    raw_msg.time = data.timestamp;
    raw_msg.temperature = data.temperature;

    yaw_msg.data = wd_.getRelatedYaw();

    imu_pub_.publish(imu_msg);
    mag_pub_.publish(mag_msg);
    gps_pub_.publish(gps_msg);
    raw_data_pub_.publish(raw_msg);
    related_yaw_pub_.publish(yaw_msg);
  }
}
}
