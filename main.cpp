#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <vector>

#include "livox_sdk.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include "sensor_msgs/NavSatFix.h"

#define ROS_RATE 20
#define UPDATE_JUMP 4 // ROS_RATE / UPDATE_JUMP is the pos update rate
#define DRONE_SPEED 3.0

#define SCAN_RADIUS 10.0
#define LAYER_DELTA 5.0

#define DELTA_METERS_V 1.0 // maximum dz

#define PACKET_GAP_MISS_TIME (1500000) // 1.5ms
#define BD_ARGC_NUM (4)
#define BD_ARGV_POS (1)
#define COMMANDLINE_BD_SIZE (15)

mavros_msgs::State current_state;
geometry_msgs::Pose pose_in;
geometry_msgs::PoseStamped pose_stamped;
sensor_msgs::NavSatFix::ConstPtr gps_msg;
mavros_msgs::Altitude::ConstPtr alt_msg;

double roll = 0.0, pitch = 0.0, yaw = 0.0;
double worldx = 0.0, worldy = 0.0, worldz = 0.0;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

double e1x, e1y, e2x, e2y;
double c1x, c1y, c3x, c3y;
double c2x, c2y, c4x, c4y;
int diameter_time;

double destx = 0.0;
double desty = 0.0;
double dz = 0.0;
int travel_time = 0;

std::string timedir;
std::string gps_filename;
int num_records = 0;

double from_degrees(double d)
{
  return d * M_PI / 180.0;
}

double to_degrees(double r)
{
  return r * 180.0 / M_PI;
}

void get_time()
{
  // current date/time based on current system
  time_t now = time(0);
  // char *dt = ctime(&now);
  // std::cout << "The local date and time is: " << dt << std::endl;

  tm *ltm = localtime(&now);
  year = 1900 + ltm->tm_year;
  month = 1 + ltm->tm_mon;
  day = ltm->tm_mday;
  hour = ltm->tm_hour;
  minute = ltm->tm_min;
  second = ltm->tm_sec;
}

std::string get_time_str()
{
  std::stringstream sstm;
  sstm << year << "-" << month << "-" << day << "_" << hour << "-" << minute << "-" << second;
  return sstm.str();
}

/* A class to create and write data in a csv file. */
class CSVWriter
{
  std::string fileName;
  std::string delimeter;
  int linesCount;

public:
  CSVWriter(std::string filename, std::string delm = ",") : fileName(filename), delimeter(delm), linesCount(0)
  {
  }
  /*
	 * Member function to store a range as comma seperated value
	 */
  template <typename T>
  void add_row(T first, T last);
  void add_section(std::streambuf *str_section);
  void append_section(std::streambuf *str_section);
};

/*
 * This Function accepts a range and appends all the elements in the range
 * to the last row, seperated by delimeter (Default is comma)
 */
template <typename T>
void CSVWriter::add_row(T first, T last)
{
  std::fstream file;
  // Open the file in truncate mode if first line else in Append Mode
  file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));

  // Iterate over the range and add each lement to file seperated by delimeter.
  for (; first != last;)
  {
    file << *first;
    if (++first != last)
      file << delimeter;
  }
  file << "\n";
  linesCount++;

  // Close the file
  file.close();
}

void CSVWriter::add_section(std::streambuf *str_section)
{
  std::fstream file;
  // Open the file in truncate mode if first line else in Append Mode
  file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));

  // Iterate over the range and add each lement to file seperated by delimeter.
  file << str_section;
  linesCount++;

  // Close the file
  file.close();
}

void CSVWriter::append_section(std::streambuf *str_section)
{
  std::fstream file;
  // Open the file in truncate mode if first line else in Append Mode
  file.open(fileName, std::ios::out | std::ios::app);

  // Iterate over the range and add each lement to file seperated by delimeter.
  file << str_section;

  // Close the file
  file.close();
}

void getRPY(geometry_msgs::Quaternion qtn_msg)
{
  // tf2::Quaternion qtn;
  // quaternionMsgToTF(qtn_msg, qtn);
  // tf2::fromMsg(qtn_msg, qtn);
  tf2::Quaternion qtn = tf2::Quaternion(qtn_msg.x, qtn_msg.y, qtn_msg.z, qtn_msg.w);

  qtn.normalize();
  tf2::Matrix3x3 m(qtn);
  m.getRPY(roll, pitch, yaw);
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  pose_in = msg->pose;
  // ROS_INFO("pose: x=[%f], y=[%f], z=[%f]", pose_in.position.x, pose_in.position.y, pose_in.position.z);
  // ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  gps_msg = msg;
}

void alt_callback(const mavros_msgs::Altitude::ConstPtr &msg)
{
  alt_msg = msg;
}

typedef struct
{
  uint32_t receive_packet_count;
  uint32_t loss_packet_count;
  uint64_t last_timestamp;
} LidarPacketStatistic;

/* for device connect use ----------------------------------------------------------------------- */
typedef enum
{
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct
{
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
  LidarPacketStatistic statistic_info;
} DeviceItem;

DeviceItem lidars[kMaxLidarCount];

/* user add broadcast code here */
const char *broadcast_code_list[] = {
    "000000000000001",
};

#define BROADCAST_CODE_LIST_SIZE (sizeof(broadcast_code_list) / sizeof(intptr_t))

/* total broadcast code, include broadcast_code_list and commandline input */
std::vector<std::string> total_broadcast_code;

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point)
{
  p_dpoint->x = p_raw_point->x / 1000.0f;
  p_dpoint->y = p_raw_point->y / 1000.0f;
  p_dpoint->z = p_raw_point->z / 1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

// void compute_world_xyz(double lidarx, double lidary, double lidarz)
// {
//   tf2::Quaternion qtn;
//   qtn.setRPY(roll, pitch, yaw);
//   qtn.normalize();
//   tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse().normalize();
//   worldx = qtn_world.getX();
//   worldy = qtn_world.getY();
//   worldz = qtn_world.getZ();
// }

void compute_world_xyz(double lidarx, double lidary, double lidarz)
{
  // tf2::Quaternion qtn;
  // qtn.setRPY(roll, pitch, yaw);

  tf2::Quaternion qtn = tf2::Quaternion(pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
  // qtn.normalize();
  // tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse().normalize();
  tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse();
  worldx = qtn_world.getX();
  worldy = qtn_world.getY();
  worldz = qtn_world.getZ();
}

void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num)
{
  // std::cout << "data_num = " << data_num << std::endl;
  LivoxEthPacket *lidar_pack = data;

  if (!data || !data_num)
  {
    return;
  }

  if (handle >= kMaxLidarCount)
  {
    return;
  }

  if ((lidar_pack->timestamp_type == kTimestampTypeNoSync) ||
      (lidar_pack->timestamp_type == kTimestampTypePtp) ||
      (lidar_pack->timestamp_type == kTimestampTypePps))
  {
    LidarPacketStatistic *packet_statistic = &lidars[handle].statistic_info;
    uint64_t cur_timestamp = *((uint64_t *)(lidar_pack->timestamp));
    int64_t packet_gap = cur_timestamp - packet_statistic->last_timestamp;

    packet_statistic->receive_packet_count++;
    if (packet_statistic->last_timestamp)
    {
      if (packet_gap > PACKET_GAP_MISS_TIME)
      {
        packet_statistic->loss_packet_count++;
        // ROS_INFO("%d miss count : %ld %lu %lu %d", handle, packet_gap, cur_timestamp, packet_statistic->last_timestamp, packet_statistic->loss_packet_count);
      }
    }

    packet_statistic->last_timestamp = cur_timestamp;
  }

  LivoxRawPoint *p_point_data = (LivoxRawPoint *)lidar_pack->data;
  LivoxPoint tmp_point;

  if (num_records % 1000 == 0)
  {
    get_time();
    std::string time_str = get_time_str();
    gps_filename = timedir + time_str + ".csv";
  }

  // CSVWriter gps_writer(gps_filename);

  std::stringstream stream;
  while (data_num)
  {
    PointCloudConvert(&tmp_point, p_point_data);

    if (fabs(tmp_point.x) > 1e-6 || fabs(tmp_point.y) > 1e-6 || fabs(tmp_point.z) > 1e-6)
    {
      // 3. GPS rod shift
      stream << std::setprecision(10) << gps_msg->latitude << ",";
      stream << std::setprecision(11) << gps_msg->longitude << ",";
      stream << std::setprecision(7) << gps_msg->altitude << ",";
      stream << std::setprecision(7) << alt_msg->amsl << ",";

      stream << std::setprecision(4) << pose_in.position.x << ",";
      stream << std::setprecision(4) << pose_in.position.y << ",";
      stream << std::setprecision(4) << pose_in.position.z << ",";

      // getRPY(pose_in.orientation);
      compute_world_xyz(tmp_point.z, -tmp_point.y, tmp_point.x);
      stream << std::setprecision(4) << worldx << ",";
      stream << std::setprecision(4) << worldy << ",";
      stream << std::setprecision(4) << worldz << ",";
      stream << std::setprecision(4) << tmp_point.x << ",";
      stream << std::setprecision(4) << tmp_point.y << ",";
      stream << std::setprecision(4) << tmp_point.z << ",";
      stream << (float)tmp_point.reflectivity << "\n";
    }

    --data_num;
    p_point_data++;
  }

  // gps_writer.append_section(stream.rdbuf());
  std::fstream file;
  file.open(gps_filename, std::ios::out | std::ios::app);
  file << stream.rdbuf();
  file.close();

  num_records++;

  // std::cout << num_records << ": " << gps_filename << std::endl;

  return;
}

/** add bd to total_broadcast_code */
void add_broadcast_code(const char *bd_str)
{
  total_broadcast_code.push_back(bd_str);
}

/** add bd in broadcast_code_list to total_broadcast_code */
void add_local_broadcast_code(void)
{
  for (int i = 0; i < BROADCAST_CODE_LIST_SIZE; ++i)
  {
    add_broadcast_code(broadcast_code_list[i]);
  }
}

/** add commandline bd to total_broadcast_code */
void add_commandline_broadcast_code(const char *cammandline_str)
{
  char *strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char *bd_str = strtok(strs, pattern.c_str());
  while (bd_str != NULL)
  {
    ROS_INFO("commandline input bd:%s", bd_str);
    if (COMMANDLINE_BD_SIZE == strlen(bd_str))
    {
      add_broadcast_code(bd_str);
    }
    else
    {
      ROS_INFO("Invalid bd:%s", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete[] strs;
}

/** Callback function of starting sampling. */
void OnSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data)
{
  ROS_INFO("OnSampleCallback statue %d handle %d response %d", status, handle, response);
  if (status == kStatusSuccess)
  {
    if (response != 0)
    {
      lidars[handle].device_state = kDeviceStateConnect;
    }
  }
  else if (status == kStatusTimeout)
  {
    lidars[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data)
{
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(uint8_t status, uint8_t handle, DeviceInformationResponse *ack, void *data)
{
  if (status != kStatusSuccess)
  {
    ROS_INFO("Device Query Informations Failed %d", status);
  }
  if (ack)
  {
    ROS_INFO("firm ver: %d.%d.%d.%d",
             ack->firmware_version[0],
             ack->firmware_version[1],
             ack->firmware_version[2],
             ack->firmware_version[3]);
  }
}

/** Callback function of changing of device state. */
void OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
{
  if (info == NULL)
  {
    return;
  }

  ROS_INFO("OnDeviceChange broadcast code %s update type %d", info->broadcast_code, type);

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount)
  {
    return;
  }
  if (type == kEventConnect)
  {
    QueryDeviceInformation(handle, OnDeviceInformation, NULL);
    if (lidars[handle].device_state == kDeviceStateDisconnect)
    {
      lidars[handle].device_state = kDeviceStateConnect;
      lidars[handle].info = *info;
    }
  }
  else if (type == kEventDisconnect)
  {
    lidars[handle].device_state = kDeviceStateDisconnect;
  }
  else if (type == kEventStateChange)
  {
    lidars[handle].info = *info;
  }

  if (lidars[handle].device_state == kDeviceStateConnect)
  {
    ROS_INFO("Device State error_code %d", lidars[handle].info.status.status_code);
    ROS_INFO("Device State working state %d", lidars[handle].info.state);
    ROS_INFO("Device feature %d", lidars[handle].info.feature);
    if (lidars[handle].info.state == kLidarStateNormal)
    {
      if (lidars[handle].info.type == kDeviceTypeHub)
      {
        HubStartSampling(OnSampleCallback, NULL);
      }
      else
      {
        LidarStartSampling(handle, OnSampleCallback, NULL);
      }
      lidars[handle].device_state = kDeviceStateSampling;
    }
  }
}

void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
{
  if (info == NULL)
  {
    return;
  }

  ROS_INFO("Receive Broadcast Code %s", info->broadcast_code);
  bool found = false;

  for (int i = 0; i < total_broadcast_code.size(); ++i)
  {
    if (strncmp(info->broadcast_code, total_broadcast_code[i].c_str(), kBroadcastCodeSize) == 0)
    {
      found = true;
      break;
    }
  }
  if (!found)
  {
    ROS_INFO("Not in the broacast_code_list, please add it to if want to connect!");
    return;
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount)
  {
    SetDataCallback(handle, GetLidarData);
    lidars[handle].handle = handle;
    lidars[handle].device_state = kDeviceStateDisconnect;
  }
}

void state_callback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

bool no_position_yet()
{
  return fabs(pose_in.orientation.x) + fabs(pose_in.orientation.y) + fabs(pose_in.orientation.z) + fabs(pose_in.orientation.w) < 1e-6;
}

void delta_position(double dx, double dy, double dz)
{
  pose_stamped.pose.position.x += dx;
  pose_stamped.pose.position.y += dy;
  pose_stamped.pose.position.z += dz;
}

int get_travel_params(int curr_loc)
{
  if (curr_loc == -1)
  {
    dz = DELTA_METERS_V / 2.0;
    travel_time = diameter_time / 2;
  }
  else
  {
    dz = DELTA_METERS_V;
    travel_time = diameter_time;
  }

  switch (curr_loc)
  {
  case -1:
    curr_loc = 0;
    destx = e1x;
    desty = e1y;
    break;

  case 0:
    curr_loc = 2;
    destx = c2x;
    desty = c2y;
    break;

  case 1:
    curr_loc = 2;
    destx = c2x;
    desty = c2y;
    break;

  case 2:
    curr_loc = 3;
    destx = c3x;
    desty = c3y;
    break;

  case 3:
    curr_loc = 4;
    destx = c4x;
    desty = c4y;
    break;

  case 4:
    curr_loc = 1;
    destx = c1x;
    desty = c1y;
    break;
  }
  return curr_loc;
}

int main(int argc, char **argv)
{
  // // std::string rootdir = "/home/jiangchuan/livox_data/";
  // std::string rootdir = "/home/pi/livox_data/";
  // int status = mkdir(rootdir.c_str(), 0777);
  // get_time();
  // std::string time_str = get_time_str();
  // timedir = rootdir + time_str + "/";
  // status = mkdir(timedir.c_str(), 0777);
  // gps_filename = timedir + time_str + ".csv";

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  /* ros related */
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle nh;

  // TODO: Wait some time for the drone.
  // Call lidar when in POSITION mode.
  // Make a new folder of timestamp for each flight

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gps_callback);
  ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, alt_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Time::init();
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate((double)ROS_RATE);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("connecting to FCU ...");
  }

  // // wait for position mode
  // while (ros::ok() && current_state.mode != "POSCTL")
  // {
  //     // std::cout << current_state.mode << std::endl;
  //     ros::spinOnce();
  //     rate.sleep();
  //     ROS_INFO("waiting for position mode ...");
  // }

  // wait for local position feed
  while (ros::ok() && no_position_yet())
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("getting local position ...");
  }

  // /* Start Livox */
  // if (!Init())
  // {
  //   ROS_FATAL("Livox-SDK init fail!");
  //   return -1;
  // }

  // add_local_broadcast_code();
  // if (argc >= BD_ARGC_NUM)
  // {
  //   ROS_INFO("Commandline input %s", argv[BD_ARGV_POS]);
  //   add_commandline_broadcast_code(argv[BD_ARGV_POS]);
  // }

  // memset(lidars, 0, sizeof(lidars));
  // SetBroadcastCallback(OnDeviceBroadcast);
  // SetDeviceStateUpdateCallback(OnDeviceChange);

  // if (!Start())
  // {
  //   Uninit();
  //   return -1;
  // }
  // /* Start Livox Ends*/

  pose_stamped.pose = pose_in;

  //send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(pose_stamped);
    ros::spinOnce();
    rate.sleep();
  }

  // change to offboard mode
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  ros::Time last_request = ros::Time::now();
  while (ros::ok() && current_state.mode != "OFFBOARD")
  {
    if (ros::Time::now() - last_request > ros::Duration(5.0))
    {
      ROS_INFO(current_state.mode.c_str());
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    local_pos_pub.publish(pose_stamped);
    ros::spinOnce();
    rate.sleep();
  }

  // arm
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  while (ros::ok() && !current_state.armed)
  {
    if (ros::Time::now() - last_request > ros::Duration(5.0))
    {
      if (arming_client.call(arm_cmd) &&
          arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed");
      }
      last_request = ros::Time::now();
    }
    local_pos_pub.publish(pose_stamped);
    ros::spinOnce();
    rate.sleep();
  }

  // stay at 0.5m for 5 seconds
  int n_wait_sec = 2;
  ROS_INFO("stay at origin for %d seconds", n_wait_sec);
  delta_position(0.0, 0.0, 1);
  for (int i = 0; ros::ok() && i < n_wait_sec * ROS_RATE; i++)
  {
    local_pos_pub.publish(pose_stamped);
    ros::spinOnce();
    rate.sleep();
  }

  // Mission starts here
  getRPY(pose_in.orientation); // Get yaw
  ROS_INFO("  >>> INITIAL yaw = %1.1f degrees", to_degrees(yaw));

  double x0 = pose_stamped.pose.position.x;
  double y0 = pose_stamped.pose.position.y;

  double radius_sin = SCAN_RADIUS * sin(yaw);
  double radius_cos = SCAN_RADIUS * cos(yaw);
  e1x = x0 - radius_sin;
  e1y = y0 + radius_cos;
  e2x = x0 + radius_sin;
  e2y = y0 - radius_cos;

  c1x = e1x + radius_cos;
  c1y = e1y + radius_sin;
  c3x = e1x - radius_cos;
  c3y = e1y - radius_sin;

  c2x = e2x + radius_cos;
  c2y = e2y + radius_sin;
  c4x = e2x - radius_cos;
  c4y = e2y - radius_sin;

  // diameter_time = (int)(std::max(1.0, SCAN_RADIUS / DRONE_SPEED)) * 2; // Constant
  diameter_time = (int)(M_PI * SCAN_RADIUS / DRONE_SPEED);

  /*  O -> E1 -> {C2 -> C3 -> C4 -> C1} -> {C2 -> C3 -> C4 -> C1} */
  /* -1 ->  0 -> { 2 ->  3 ->  4 ->  1} -> { 2 ->  3 ->  4 ->  1} */

  int curr_loc = -1;

  for (int i = 0; i < 20; i++)
  {
    std::cout << curr_loc << " -> ";
    curr_loc = get_travel_params(curr_loc);
    std::cout << curr_loc << std::endl;

    // travel2next() starts
    double dxr = (destx - pose_in.position.x) / 2.0;
    double dyr = (desty - pose_in.position.y) / 2.0;
    int num_updates = travel_time * ROS_RATE / UPDATE_JUMP;
    double dtheta = M_PI / (double)num_updates;
    double idz = dz / (double)num_updates;
    for (int i = 0; i < num_updates; i++)
    {
      double cos_coef = cos(dtheta * i) - cos(dtheta * (i + 1));
      double idx = dxr * cos_coef;
      double idy = dyr * cos_coef;
      delta_position(idx, idy, idz);
      for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
      {
        local_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
      }
    }
    // travel2next() ends
  }

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0.0f;
  land_cmd.request.latitude = 0.0f;
  land_cmd.request.longitude = 0.0f;
  land_cmd.request.altitude = 0.0f;

  ROS_INFO("tring to land");
  while (ros::ok())
  {
    if (ros::Time::now() - last_request > ros::Duration(5.0))
    {
      if (land_client.call(land_cmd) && land_cmd.response.success)
      {
        ROS_INFO("Landing");
        break;
      }
      last_request = ros::Time::now();
    }
    local_pos_pub.publish(pose_stamped);
    ros::spinOnce();
    rate.sleep();
  }

  //Manual: MANUAL
  //Return: AUTO.RTL
  //AUTO_MISSION
  //Hold: AUTO.LOITER
  //Stabilized: STABILIZED
  //Altitude: ALTCTL
  //Follow Me: AUTO.FOLLOW_TARGET

  // Uninit();
}
