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
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <mavros_msgs/Altitude.h>
#include "sensor_msgs/NavSatFix.h"

#define PACKET_GAP_MISS_TIME (1500000) // 1.5ms
#define BD_ARGC_NUM (4)
#define BD_ARGV_POS (1)
#define COMMANDLINE_BD_SIZE (15)

geometry_msgs::Pose pose_in;
sensor_msgs::NavSatFix::ConstPtr gps_msg;
mavros_msgs::Altitude::ConstPtr alt_msg;

double roll = 0.0, pitch = 0.0, yaw = 0.0;
double worldx = 0.0, worldy = 0.0, worldz = 0.0;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

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

  CSVWriter gps_writer(gps_filename);

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

      getRPY(pose_in.orientation);
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

  gps_writer.append_section(stream.rdbuf());
  num_records++;

  std::cout << num_records << ": " << gps_filename << std::endl;

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

int main(int argc, char **argv)
{

  // std::string rootdir = "/home/jiangchuan/livox_data/";
  std::string rootdir = "/home/pi/livox_data/";
  int status = mkdir(rootdir.c_str(), 0777);

  get_time();
  std::string time_str = get_time_str();

  timedir = rootdir + time_str + "/";
  status = mkdir(timedir.c_str(), 0777);

  gps_filename = timedir + time_str + ".csv";

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO("Livox-SDK ros demo");

  if (!Init())
  {
    ROS_FATAL("Livox-SDK init fail!");
    return -1;
  }

  add_local_broadcast_code();
  if (argc >= BD_ARGC_NUM)
  {
    ROS_INFO("Commandline input %s", argv[BD_ARGV_POS]);
    add_commandline_broadcast_code(argv[BD_ARGV_POS]);
  }

  memset(lidars, 0, sizeof(lidars));
  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  if (!Start())
  {
    Uninit();
    return -1;
  }

  /* ros related */
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle nh;

  // TODO: Wait some time for the drone.
  // Call lidar when in POSITION mode.
  // Make a new folder of timestamp for each flight

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gps_callback);
  ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, alt_callback);

  ros::Time::init();
  ros::Rate rate(500); // 500 hz
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  Uninit();
}
