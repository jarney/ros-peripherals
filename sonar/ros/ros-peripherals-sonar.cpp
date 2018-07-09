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
  float m_loc_x;
  float m_loc_y;
  float m_loc_z;

  float m_orient_x;
  float m_orient_y;
  float m_orient_z;
  float m_orient_w;

  std::string m_frame_id;
  ros::Publisher m_publisher;

public:
  Sonar(boost::property_tree::ptree & component_descriptor, ros::NodeHandle & nh);

  std::string & getId(void);

  std::string & getFrameId(void);
  ros::Publisher & getPublisher(void);

  float getMinRange(void);
  float getMaxRange(void);
  float getFieldOfView(void);

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

Sonar::Sonar(boost::property_tree::ptree & component_descriptor, ros::NodeHandle &nh)
{
  m_id = component_descriptor.get<std::string>("id");
  m_min_range = component_descriptor.get<float>("min_range");
  m_max_range = component_descriptor.get<float>("max_range");
  m_field_of_view = component_descriptor.get<float>("field_of_view");

/*
  boost::property_tree::ptree loc = component_descriptor.get<boost::property_tree::ptree>("location");

  std::vector<float> locv = as_vector<float>(loc, "pos");
  std::vector<float> orientv = as_vector<float>(loc, "orient");

  m_loc_x = locv.at(0);
  m_loc_y = locv.at(1);
  m_loc_z = locv.at(2);

  m_orient_x = orientv.at(0);
  m_orient_y = orientv.at(1);
  m_orient_z = orientv.at(2);
  m_orient_w = orientv.at(3);
*/

  m_frame_id = str(boost::format{"sonar_%1%"} % m_id.c_str());
  m_publisher = nh.advertise<sensor_msgs::Range>(m_frame_id, 1);
}

class SonarArray {
private:
  ros::NodeHandle & m_nh;
  serial::Serial peripheral;
  boost::property_tree::ptree msg_device_discovery;
  boost::property_tree::ptree msg_subscribe;
  boost::property_tree::ptree msg_unsubscribe;

  bool m_initialized;

  std::map<std::string, Sonar*> m_sonars;

public:

    SonarArray(std::string serialPort, int32_t baud_rate, ros::NodeHandle &nh);

    boost::property_tree::ptree read(void);

    void write(boost::property_tree::ptree & tree);

    void unsubscribe(void);
    void subscribe(void);
    void deviceDiscovery(void);

    bool initialized(void);

    Sonar *get(std::string id);

};


Sonar *SonarArray::get(std::string id)
{
  return m_sonars[id];
}

SonarArray::SonarArray(std::string serialPort, int32_t baud_rate, ros::NodeHandle &nh) 
  : peripheral(serialPort, baud_rate, serial::Timeout::simpleTimeout(5000)),
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

  ROS_INFO("<< %s", response_str.c_str());

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
  ROS_INFO(">> %s", msgstr.c_str());
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
        Sonar *s = new Sonar(descriptor.second, m_nh);
        m_sonars[s->getId()] = s;
    }

    m_initialized = true;
  }
}

bool SonarArray::initialized()
{
  return m_initialized;
}

typedef struct sensor_st {
  std::string sonar_frame;
  ros::Publisher sonar_pub;
} sensor_t;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_peripherals_sonar");
  ros::NodeHandle nh("~");

  std::string defaultPort = std::string("/dev/ttyUSB0");

  std::string serialPort = getParamOrDefault(
                               nh,
                               "ros_peripherals_sonar/serial_port",
                               defaultPort);


  // Setting this inconsistently with
  // the firmware is always going to be wrong.
  // Should this even be a parameter?  Probably not.
  int32_t baud_rate = (int32_t)38400;
  //                  getParamOrDefault(nh,
  //                        "ros_peripherals_sonar/serial_baud",
  //                        (int32_t)38400);

  SonarArray sonarArray(serialPort, baud_rate, nh);

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

  ros::Publisher pub = nh.advertise<sensor_msgs::Range>("/sonars", 5);

  ros::Rate rate(50);

  std::string sonar_frame;
  ros::Publisher sonar_pub;

  std::list<sensor_t> sensor_list;

  ROS_INFO("ROS Peripheral Sonar Ready");

  int i = 0;
 
  while (ros::ok()) {

      boost::property_tree::ptree sonar_status_data = sonarArray.read();

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
    
        pub.publish(msg);
        sonar->getPublisher().publish(msg);

        ROS_INFO("Device information: %f", msg.range);
      }
      rate.sleep();
  }

  return 0;
}

