/*
 * Copyright (c) 2018, Ensor Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>

template <typename T>
T getParamOrDefault(ros::NodeHandle nh, std::string parameter_name,
                    T default_val) {
    T value = default_val;
    if (!nh.getParam(parameter_name, value)) {
        nh.setParam(parameter_name, value);
    }
    return value;
}


class Sonar {
private:
  std::string m_id;
  float m_min_range;
  float m_max_range;
  float m_field_of_view;
  geometry_msgs::Vector3 m_translation;
  geometry_msgs::Quaternion m_rotation;

  std::string m_frame_id;
  ros::Publisher m_publisher;

public:
  Sonar(boost::property_tree::ptree & component_descriptor, ros::NodeHandle & nh, std::string attach_point);

  std::string & getId(void);

  std::string & getFrameId(void);
  ros::Publisher & getPublisher(void);

  float getMinRange(void);
  float getMaxRange(void);
  float getFieldOfView(void);


  geometry_msgs::Vector3 & getTranslation();
  geometry_msgs::Quaternion & getRotation();
};

std::string & Sonar::getId()
{
  return m_id;
}

std::string & Sonar::getFrameId()
{
  return m_frame_id;
}

ros::Publisher & Sonar::getPublisher()
{
  return m_publisher;
}

float Sonar::getMinRange(void)
{
  return m_min_range;
}

float Sonar::getMaxRange(void)
{
  return m_max_range;
}

float Sonar::getFieldOfView(void)
{
  return m_field_of_view;
}

template <typename T>
std::vector<T> as_vector(boost::property_tree::ptree const& pt, boost::property_tree::ptree::key_type const& key)
{
    std::vector<T> r;
    for (auto& item : pt.get_child(key))
        r.push_back(item.second.get_value<T>());
    return r;
}

geometry_msgs::Vector3 & Sonar::getTranslation()
{
    return m_translation;
}

geometry_msgs::Quaternion & Sonar::getRotation()
{
    return m_rotation;
}

Sonar::Sonar(boost::property_tree::ptree & component_descriptor, ros::NodeHandle &nh, std::string attachPoint)
{
  m_id = component_descriptor.get<std::string>("id");
  m_min_range = component_descriptor.get<float>("min_range");
  m_max_range = component_descriptor.get<float>("max_range");
  m_field_of_view = component_descriptor.get<float>("field_of_view");

  boost::optional<boost::property_tree::ptree&> location = component_descriptor.get_child_optional("location");

  if (location) {
      std::vector<float> location_pos = as_vector<float>(location.get(), "xyz");
      if (location_pos.size() == 3) {
          m_translation.x = location_pos.at(0);
          m_translation.y = location_pos.at(1);
          m_translation.z = location_pos.at(2);
      }
      // Euler angles, degrees.
      // We need to (ultimately) convert these to
      // Quaternions.
      std::vector<float> location_orient = as_vector<float>(location.get(), "rpy");
      if (location_orient.size() == 3) {
          float r = location_orient.at(0) * TF2SIMD_RADS_PER_DEG;
          float p = location_orient.at(1) * TF2SIMD_RADS_PER_DEG;
          float y = location_orient.at(2) * TF2SIMD_RADS_PER_DEG;

          tf2::Quaternion q;
          q.setRPY(r, p, y);
          m_rotation.x = q.x();
          m_rotation.y = q.y();
          m_rotation.z = q.z();
          m_rotation.w = q.w();
      }
  }

  m_frame_id = str(boost::format{"%1%_sonar_%2%"} % attachPoint.c_str() % m_id.c_str());
  m_publisher = nh.advertise<sensor_msgs::Range>(m_frame_id, 1);
}

class SonarArray {
private:
  ros::NodeHandle & m_nh;
  std::string m_attach_point;
  serial::Serial peripheral;
  boost::property_tree::ptree msg_device_discovery;
  boost::property_tree::ptree msg_subscribe;
  boost::property_tree::ptree msg_unsubscribe;

  bool m_initialized;

  std::map<std::string, Sonar*> m_sonars;
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  tf2_ros::TransformBroadcaster transformBroadcaster;

public:

    SonarArray(std::string serialPort, int32_t baud_rate, ros::NodeHandle &nh, std::string attachPoint);

    boost::property_tree::ptree read(void);

    void write(boost::property_tree::ptree & tree);

    void unsubscribe(void);
    void subscribe(void);
    void deviceDiscovery(void);

    bool initialized(void);

    void broadcast(void);

    Sonar *get(std::string id);

};


Sonar *SonarArray::get(std::string id)
{
  return m_sonars[id];
}

SonarArray::SonarArray(std::string serialPort, int32_t baud_rate, ros::NodeHandle &nh, std::string attachPoint)
  : peripheral(serialPort, baud_rate, serial::Timeout::simpleTimeout(5000)),
    m_attach_point(attachPoint),
    m_nh(nh)
{
  msg_device_discovery.put("msg", "device-discovery");

  msg_subscribe.put("msg", "subscribe");

  msg_unsubscribe.put("msg", "unsubscribe");

  m_initialized = false;
}

boost::property_tree::ptree SonarArray::read() {
  boost::property_tree::ptree response;

  peripheral.waitReadable();
    
  std::string response_str = peripheral.readline();

  ROS_DEBUG("<< %s", response_str.c_str());

  std::stringstream ss;
  ss << response_str;
  try {
    boost::property_tree::read_json(ss, response);
  }
  catch (boost::property_tree::json_parser_error &ex) {
    ROS_INFO("<<parse error");
  }

  return response;
}

void SonarArray::write(boost::property_tree::ptree & tree) {

  std::stringstream msg;
  boost::property_tree::json_parser::write_json(msg, tree, false);

  const std::string & msgstr = msg.str();

  size_t wrotebytes = peripheral.write(msgstr);

  //peripheral.write("\n");
  ROS_DEBUG(">> %s", msgstr.c_str());
  ROS_INFO("wrotebytes = %ld ", wrotebytes);
}

void SonarArray::unsubscribe(void)
{
  write(msg_unsubscribe);
}

void SonarArray::subscribe(void)
{
  write(msg_subscribe);
}

void SonarArray::deviceDiscovery(void)
{
  unsubscribe();

  for (int i = 0; i < 5; i++) {
    sleep(5);
    write(msg_device_discovery);
    sleep(1);

    boost::property_tree::ptree deviceInfo = read();
    boost::optional<boost::property_tree::ptree&> msgfield = deviceInfo.get_child_optional("msg");
    if (!msgfield) {
        continue;
    }

    std::string msgtypestr = deviceInfo.get<std::string>("msg");
    ROS_INFO("msgtype: %s", msgtypestr.c_str());
    if (msgtypestr != std::string("device-information")) {
        continue;
    }
    ROS_INFO("Found device data");

    
    BOOST_FOREACH(boost::property_tree::ptree::value_type &descriptor, deviceInfo.get_child("components")) {
        Sonar *sonar = new Sonar(descriptor.second, m_nh, m_attach_point);

        geometry_msgs::TransformStamped tf_transform;
        tf_transform.header.frame_id = tf::resolve(std::string(""), m_attach_point);
        tf_transform.child_frame_id = tf::resolve(std::string(""), sonar->getFrameId());
        tf_transform.transform.translation = sonar->getTranslation();
        tf_transform.transform.rotation = sonar->getRotation();
        tf_transforms.push_back(tf_transform);

        m_sonars[sonar->getId()] = sonar;
    }
    m_initialized = true;
  }
}

void SonarArray::broadcast()
{
    transformBroadcaster.sendTransform(tf_transforms);
}

bool SonarArray::initialized()
{
  return m_initialized;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_peripherals_sonar");
  ros::NodeHandle nh("~");

  std::string serialPort = getParamOrDefault(
                               nh,
                               "/ros_peripherals_sonar/serial_port",
                               std::string("/dev/ttyUSB0"));

  std::string attachPoint = getParamOrDefault(
                               nh,
                               "/ros_peripherals_sonar/attach_point",
                               std::string("ros_peripheral_1"));

  int32_t baud_rate = (int32_t)38400;


  SonarArray sonarArray(serialPort, baud_rate, nh, attachPoint);

  ROS_INFO("Starting ROS Sonar Module");
  ROS_INFO("Serial port set %s", serialPort.c_str());


  sonarArray.deviceDiscovery();
  if (!sonarArray.initialized()) {
      ROS_INFO("Could not read device data from sonar array at %s", serialPort.c_str());
      return 0;
  }

  ROS_INFO("Device discovered");

  sonarArray.subscribe();

  double field_of_view;
  double min_range;
  double max_range;

  nh.param<double>("field_of_view", field_of_view, .43632347);
  nh.param<double>("min_range", min_range, .05);
  nh.param<double>("max_range", max_range, 10);

  ros::Publisher sonarsTopic = nh.advertise<sensor_msgs::Range>("/sonars", 5);

  ros::Rate rate(50);

  std::string sonar_frame;
  ros::Publisher sonar_pub;

  ROS_INFO("ROS Peripheral Sonar Ready");

  int i = 0;
 
  while (ros::ok()) {

      boost::property_tree::ptree sonar_status_data = sonarArray.read();

      boost::optional<boost::property_tree::ptree&> sensorData = sonar_status_data.get_child_optional("sensor_data");
      if (sensorData) {
        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, sonar_status_data.get_child("sensor_data")) {

          boost::property_tree::ptree & sonarData = v.second;
  
          Sonar *sonar = sonarArray.get(sonarData.get<std::string>("id"));
  
  
          sensor_msgs::Range msg;
          msg.field_of_view = sonar->getFieldOfView();
          msg.min_range = sonar->getMinRange();
          msg.max_range = sonar->getMaxRange();
          msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
          msg.range = sonarData.get<float>("range");
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = sonar->getFrameId();
      
          sonarsTopic.publish(msg);
          sonar->getPublisher().publish(msg);
  
  
        }
  
      }

      if (i == 0) {
          sonarArray.broadcast();
      }
      i++;
      i = i % 10;

      rate.sleep();

  }

  return 0;
}

