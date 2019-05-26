
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <string.h>
// #include <stdint.h>
// #include <math.h>
// #include <sstream>
// #include <fstream>
// #include <sys/stat.h>
// #include <vector>

// #include "livox_sdk.h"
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/CommandTOL.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>

// #include <tf2/convert.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// // #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// #include <mavros_msgs/State.h>
// #include <mavros_msgs/Altitude.h>
// #include "sensor_msgs/NavSatFix.h"

// #define ROS_RATE 20
// #define MIN_ARR_LEN 10
// #define DIST_ARR_LEN 10000
// #define UPDATE_JUMP 4 // ROS_RATE / UPDATE_JUMP is the pos update rate
// // #define UPDATE_JUMP 1 // ROS_RATE / UPDATE_JUMP is the pos update rate

// // #define MAX_SCAN_SEG 12
// // #define MAX_MID_SCAN_SEG 8
// #define MAX_SCAN_SEG 20
// #define MAX_MID_SCAN_SEG 16

// #define DETECT_THRESH 300 // 300 in 10000 for one seconds

// #define DRONE_SPEED_H 1.0
// #define DRONE_SPEED_V 1.0

// // #define SCAN_RADIUS 10.0
// // #define TURN_RADIUS 4.0
// #define SCAN_RADIUS 5.0
// #define TURN_RADIUS 2.0
// #define PHI M_PI / 4.0

// // #define INITIAL_RISE 10.0
// #define INITIAL_RISE 2.0
// #define DELTA_METERS_V 1.0 // maximum dz

// // #define NEAR_DIST 12.0
// // #define NEAR_DIST 1.0
// #define NEAR_DIST 5.0

// #define PACKET_GAP_MISS_TIME (1500000) // 1.5ms
// #define BD_ARGC_NUM (4)
// #define BD_ARGV_POS (1)
// #define COMMANDLINE_BD_SIZE (15)

// mavros_msgs::State current_state;
// geometry_msgs::Pose pose_in;
// geometry_msgs::PoseStamped pose_stamped;
// sensor_msgs::NavSatFix::ConstPtr gps_msg;
// mavros_msgs::Altitude::ConstPtr alt_msg;

// double roll = 0.0, pitch = 0.0, yaw = 0.0;
// double worldx = 0.0, worldy = 0.0, worldz = 0.0;
// int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

// float FAR_DIST = (float)(NEAR_DIST + 2.0 * DELTA_METERS_V);
// float mid_dist = (NEAR_DIST + FAR_DIST) / 2.0f;
// int in_mid_count = 0;

// std::string timedir;
// std::string gps_filename;
// int num_records = 0;

// bool initiating = true;
// float min_dist_arr[MIN_ARR_LEN];
// float dist_arr[DIST_ARR_LEN];
// int curr_dist_index = 0;
// int last_dist_index = DIST_ARR_LEN - 1;
// int update_rate = ROS_RATE / UPDATE_JUMP; // Position update rate

// double from_degrees(double d)
// {
//   return d * M_PI / 180.0;
// }

// double to_degrees(double r)
// {
//   return r * 180.0 / M_PI;
// }

// void get_time()
// {
//   // current date/time based on current system
//   time_t now = time(0);
//   // char *dt = ctime(&now);
//   // std::cout << "The local date and time is: " << dt << std::endl;

//   tm *ltm = localtime(&now);
//   year = 1900 + ltm->tm_year;
//   month = 1 + ltm->tm_mon;
//   day = ltm->tm_mday;
//   hour = ltm->tm_hour;
//   minute = ltm->tm_min;
//   second = ltm->tm_sec;
// }

// std::string get_time_str()
// {
//   std::stringstream sstm;
//   sstm << year << "-";
//   if (month < 10)
//   {
//     sstm << "0";
//   }
//   sstm << month << "-";
//   if (day < 10)
//   {
//     sstm << "0";
//   }
//   sstm << day << "_";
//   if (hour < 10)
//   {
//     sstm << "0";
//   }
//   sstm << hour << "-";
//   if (minute < 10)
//   {
//     sstm << "0";
//   }
//   sstm << minute << "-";
//   if (second < 10)
//   {
//     sstm << "0";
//   }
//   sstm << second;
//   return sstm.str();
// }

// void getRPY(double x, double y, double z, double w)
// {
//   // tf2::Quaternion qtn;
//   // quaternionMsgToTF(qtn_msg, qtn);
//   // tf2::fromMsg(qtn_msg, qtn);
//   tf2::Quaternion qtn = tf2::Quaternion(x, y, z, w);

//   qtn.normalize();
//   tf2::Matrix3x3 m(qtn);
//   m.getRPY(roll, pitch, yaw);
// }

// void getRPY(geometry_msgs::Quaternion qtn_msg)
// {
//   getRPY(qtn_msg.x, qtn_msg.y, qtn_msg.z, qtn_msg.w);
// }

// // void getRPY(geometry_msgs::Quaternion qtn_msg)
// // {
// //   // tf2::Quaternion qtn;
// //   // quaternionMsgToTF(qtn_msg, qtn);
// //   // tf2::fromMsg(qtn_msg, qtn);
// //   tf2::Quaternion qtn = tf2::Quaternion(qtn_msg.x, qtn_msg.y, qtn_msg.z, qtn_msg.w);

// //   qtn.normalize();
// //   tf2::Matrix3x3 m(qtn);
// //   m.getRPY(roll, pitch, yaw);
// // }

// typedef struct
// {
//   uint32_t receive_packet_count;
//   uint32_t loss_packet_count;
//   uint64_t last_timestamp;
// } LidarPacketStatistic;

// /* for device connect use ----------------------------------------------------------------------- */
// typedef enum
// {
//   kDeviceStateDisconnect = 0,
//   kDeviceStateConnect = 1,
//   kDeviceStateSampling = 2,
// } DeviceState;

// typedef struct
// {
//   uint8_t handle;
//   DeviceState device_state;
//   DeviceInfo info;
//   LidarPacketStatistic statistic_info;
// } DeviceItem;

// DeviceItem lidars[kMaxLidarCount];

// /* user add broadcast code here */
// const char *broadcast_code_list[] = {
//     "000000000000001",
// };

// #define BROADCAST_CODE_LIST_SIZE (sizeof(broadcast_code_list) / sizeof(intptr_t))

// /* total broadcast code, include broadcast_code_list and commandline input */
// std::vector<std::string> total_broadcast_code;

// static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point)
// {
//   p_dpoint->x = p_raw_point->x / 1000.0f;
//   p_dpoint->y = p_raw_point->y / 1000.0f;
//   p_dpoint->z = p_raw_point->z / 1000.0f;
//   p_dpoint->reflectivity = p_raw_point->reflectivity;
// }

// void init_min_array()
// {
//   initiating = true;
//   for (int i = 0; i < MIN_ARR_LEN; i++)
//   {
//     min_dist_arr[i] = 1e6f + i;
//   }
//   initiating = false;
//   return;
// }

// float get_avg_min()
// {
//   float s = 0.0f;
//   for (int i = 0; i < MIN_ARR_LEN; i++)
//   {
//     s += min_dist_arr[i];
//   }
//   return s / MIN_ARR_LEN;
// }

// float get_avg_dist(float nsecs)
// {
//   int total_len = curr_dist_index;

//   if (total_len < DETECT_THRESH * nsecs) // No points detected
//   {
//     curr_dist_index = 0;
//     return 1e6;
//   }

//   float s = 0.0f;
//   for (int i = 0; i < total_len; i++)
//   {
//     s += dist_arr[i];
//   }
//   curr_dist_index = 0;
//   return s / total_len;
// }

// int binarySearch(float a[], float item, int low, int high) // high = n-1
// {
//   if (high <= low)
//   {
//     return (item > a[low]) ? (low + 1) : low;
//   }
//   int mid = (low + high) / 2;

//   if (item == a[mid])
//   {
//     return mid + 1;
//   }

//   if (item > a[mid])
//   {
//     return binarySearch(a, item, mid + 1, high);
//   }
//   return binarySearch(a, item, low, mid - 1);
// }

// void insertSorted(float a[], int n, int index, float item)
// {
//   if (index >= n)
//   {
//     return;
//   }
//   for (int i = n - 1; i > index; i--)
//   {
//     a[i] = a[i - 1];
//   }
//   a[index] = item;
//   return;
// }

// // void compute_world_xyz(double lidarx, double lidary, double lidarz)
// // {
// //   tf2::Quaternion qtn;
// //   qtn.setRPY(roll, pitch, yaw);
// //   qtn.normalize();
// //   tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse().normalize();
// //   worldx = qtn_world.getX();
// //   worldy = qtn_world.getY();
// //   worldz = qtn_world.getZ();
// // }

// void compute_world_xyz(double lidarx, double lidary, double lidarz)
// {
//   // tf2::Quaternion qtn;
//   // qtn.setRPY(roll, pitch, yaw);

//   tf2::Quaternion qtn = tf2::Quaternion(pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
//   // qtn.normalize();
//   // tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse().normalize();
//   tf2::Quaternion qtn_world = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse();
//   worldx = qtn_world.getX();
//   worldy = qtn_world.getY();
//   worldz = qtn_world.getZ();
// }

// void string_to_file(std::string filename, std::string str)
// {
//   std::stringstream stream;
//   stream << str;
//   std::fstream file;
//   file.open(filename, std::ios::out | std::ios::app);
//   file << stream.rdbuf();
//   file.close();
// }

// void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num)
// {
//   // std::cout << "data_num = " << data_num << std::endl;
//   LivoxEthPacket *lidar_pack = data;

//   if (!data || !data_num)
//   {
//     return;
//   }

//   if (handle >= kMaxLidarCount)
//   {
//     return;
//   }

//   if ((lidar_pack->timestamp_type == kTimestampTypeNoSync) ||
//       (lidar_pack->timestamp_type == kTimestampTypePtp) ||
//       (lidar_pack->timestamp_type == kTimestampTypePps))
//   {
//     LidarPacketStatistic *packet_statistic = &lidars[handle].statistic_info;
//     uint64_t cur_timestamp = *((uint64_t *)(lidar_pack->timestamp));
//     int64_t packet_gap = cur_timestamp - packet_statistic->last_timestamp;

//     packet_statistic->receive_packet_count++;
//     if (packet_statistic->last_timestamp)
//     {
//       if (packet_gap > PACKET_GAP_MISS_TIME)
//       {
//         packet_statistic->loss_packet_count++;
//         // ROS_INFO("%d miss count : %ld %lu %lu %d", handle, packet_gap, cur_timestamp, packet_statistic->last_timestamp, packet_statistic->loss_packet_count);
//       }
//     }

//     packet_statistic->last_timestamp = cur_timestamp;
//   }

//   LivoxRawPoint *p_point_data = (LivoxRawPoint *)lidar_pack->data;
//   LivoxPoint tmp_point;

//   if (num_records % 1000 == 0)
//   {
//     get_time();
//     std::string time_str = get_time_str();
//     gps_filename = timedir + time_str + ".csv";
//   }

//   std::stringstream stream;
//   while (data_num)
//   {
//     PointCloudConvert(&tmp_point, p_point_data);

//     // if (tmp_point.x > 1e-6 && !initiating)
//     // {
//     //   int index = binarySearch(min_dist_arr, tmp_point.x, 0, MIN_ARR_LEN - 1);
//     //   insertSorted(min_dist_arr, MIN_ARR_LEN, index, tmp_point.x);
//     // }

//     // Collect distances
//     dist_arr[curr_dist_index] = tmp_point.x;
//     if (curr_dist_index < last_dist_index)
//     {
//       curr_dist_index++;
//     }

//     // Save to csv
//     if (tmp_point.x > 1e-6 || fabs(tmp_point.y) > 1e-6 || fabs(tmp_point.z) > 1e-6)
//     {
//       // 3. GPS rod shift
//       stream << std::setprecision(10) << gps_msg->latitude << ",";
//       stream << std::setprecision(11) << gps_msg->longitude << ",";
//       stream << std::setprecision(7) << gps_msg->altitude << ",";
//       stream << std::setprecision(7) << alt_msg->amsl << ",";

//       stream << std::setprecision(4) << pose_in.position.x << ",";
//       stream << std::setprecision(4) << pose_in.position.y << ",";
//       stream << std::setprecision(4) << pose_in.position.z << ",";

//       // getRPY(pose_in.orientation);
//       compute_world_xyz(tmp_point.z, -tmp_point.y, tmp_point.x);
//       stream << std::setprecision(4) << worldx << ",";
//       stream << std::setprecision(4) << worldy << ",";
//       stream << std::setprecision(4) << worldz << ",";
//       stream << std::setprecision(4) << tmp_point.x << ",";
//       stream << std::setprecision(4) << tmp_point.y << ",";
//       stream << std::setprecision(4) << tmp_point.z << ",";
//       stream << (float)tmp_point.reflectivity << "\n";
//     }

//     --data_num;
//     p_point_data++;
//   }

//   // gps_writer.append_section(stream.rdbuf());
//   std::fstream file;
//   file.open(gps_filename, std::ios::out | std::ios::app);
//   file << stream.rdbuf();
//   file.close();

//   num_records++;

//   // std::cout << num_records << ": " << gps_filename << std::endl;

//   return;
// }

// /** add bd to total_broadcast_code */
// void add_broadcast_code(const char *bd_str)
// {
//   total_broadcast_code.push_back(bd_str);
// }

// /** add bd in broadcast_code_list to total_broadcast_code */
// void add_local_broadcast_code(void)
// {
//   for (int i = 0; i < BROADCAST_CODE_LIST_SIZE; ++i)
//   {
//     add_broadcast_code(broadcast_code_list[i]);
//   }
// }

// /** add commandline bd to total_broadcast_code */
// void add_commandline_broadcast_code(const char *cammandline_str)
// {
//   char *strs = new char[strlen(cammandline_str) + 1];
//   strcpy(strs, cammandline_str);

//   std::string pattern = "&";
//   char *bd_str = strtok(strs, pattern.c_str());
//   while (bd_str != NULL)
//   {
//     ROS_INFO("commandline input bd:%s", bd_str);
//     if (COMMANDLINE_BD_SIZE == strlen(bd_str))
//     {
//       add_broadcast_code(bd_str);
//     }
//     else
//     {
//       ROS_INFO("Invalid bd:%s", bd_str);
//     }
//     bd_str = strtok(NULL, pattern.c_str());
//   }

//   delete[] strs;
// }

// /** Callback function of starting sampling. */
// void OnSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data)
// {
//   ROS_INFO("OnSampleCallback statue %d handle %d response %d", status, handle, response);
//   if (status == kStatusSuccess)
//   {
//     if (response != 0)
//     {
//       lidars[handle].device_state = kDeviceStateConnect;
//     }
//   }
//   else if (status == kStatusTimeout)
//   {
//     lidars[handle].device_state = kDeviceStateConnect;
//   }
// }

// /** Callback function of stopping sampling. */
// void OnStopSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data)
// {
// }

// /** Query the firmware version of Livox LiDAR. */
// void OnDeviceInformation(uint8_t status, uint8_t handle, DeviceInformationResponse *ack, void *data)
// {
//   if (status != kStatusSuccess)
//   {
//     ROS_INFO("Device Query Informations Failed %d", status);
//   }
//   if (ack)
//   {
//     ROS_INFO("firm ver: %d.%d.%d.%d",
//              ack->firmware_version[0],
//              ack->firmware_version[1],
//              ack->firmware_version[2],
//              ack->firmware_version[3]);
//   }
// }

// /** Callback function of changing of device state. */
// void OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
// {
//   if (info == NULL)
//   {
//     return;
//   }

//   ROS_INFO("OnDeviceChange broadcast code %s update type %d", info->broadcast_code, type);

//   uint8_t handle = info->handle;
//   if (handle >= kMaxLidarCount)
//   {
//     return;
//   }
//   if (type == kEventConnect)
//   {
//     QueryDeviceInformation(handle, OnDeviceInformation, NULL);
//     if (lidars[handle].device_state == kDeviceStateDisconnect)
//     {
//       lidars[handle].device_state = kDeviceStateConnect;
//       lidars[handle].info = *info;
//     }
//   }
//   else if (type == kEventDisconnect)
//   {
//     lidars[handle].device_state = kDeviceStateDisconnect;
//   }
//   else if (type == kEventStateChange)
//   {
//     lidars[handle].info = *info;
//   }

//   if (lidars[handle].device_state == kDeviceStateConnect)
//   {
//     ROS_INFO("Device State error_code %d", lidars[handle].info.status.status_code);
//     ROS_INFO("Device State working state %d", lidars[handle].info.state);
//     ROS_INFO("Device feature %d", lidars[handle].info.feature);
//     if (lidars[handle].info.state == kLidarStateNormal)
//     {
//       if (lidars[handle].info.type == kDeviceTypeHub)
//       {
//         HubStartSampling(OnSampleCallback, NULL);
//       }
//       else
//       {
//         LidarStartSampling(handle, OnSampleCallback, NULL);
//       }
//       lidars[handle].device_state = kDeviceStateSampling;
//     }
//   }
// }

// void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
// {
//   if (info == NULL)
//   {
//     return;
//   }

//   ROS_INFO("Receive Broadcast Code %s", info->broadcast_code);
//   bool found = false;

//   for (int i = 0; i < total_broadcast_code.size(); ++i)
//   {
//     if (strncmp(info->broadcast_code, total_broadcast_code[i].c_str(), kBroadcastCodeSize) == 0)
//     {
//       found = true;
//       break;
//     }
//   }
//   if (!found)
//   {
//     ROS_INFO("Not in the broacast_code_list, please add it to if want to connect!");
//     return;
//   }

//   bool result = false;
//   uint8_t handle = 0;
//   result = AddLidarToConnect(info->broadcast_code, &handle);
//   if (result == kStatusSuccess && handle < kMaxLidarCount)
//   {
//     SetDataCallback(handle, GetLidarData);
//     lidars[handle].handle = handle;
//     lidars[handle].device_state = kDeviceStateDisconnect;
//   }
// }

// void state_callback(const mavros_msgs::State::ConstPtr &msg)
// {
//   current_state = *msg;
// }

// void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//   if (msg)
//   {
//     pose_in = msg->pose;
//     // ROS_INFO("pose: x=[%f], y=[%f], z=[%f]", pose_in.position.x, pose_in.position.y, pose_in.position.z);
//     // ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
//   }
// }

// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
//   if (msg)
//   {
//     gps_msg = msg;
//   }
// }

// void alt_callback(const mavros_msgs::Altitude::ConstPtr &msg)
// {
//   if (msg)
//   {
//     alt_msg = msg;
//   }
// }

// bool no_position_yet()
// {
//   return fabs(pose_in.orientation.x) + fabs(pose_in.orientation.y) + fabs(pose_in.orientation.z) + fabs(pose_in.orientation.w) < 1e-6;
// }

// void set_position(double x, double y, double z)
// {
//   pose_stamped.pose.position.x = x;
//   pose_stamped.pose.position.y = y;
//   pose_stamped.pose.position.z = z;
// }

// void printArr()
// {
//   printf("min_array = ");
//   for (int i = 0; i < MIN_ARR_LEN; i++)
//   {
//     printf("%g  ", min_dist_arr[i]);
//   }
//   printf("\n");
// }

// double get_travel_dz_min(double increment_z)
// {
//   float wire_dist = get_avg_min();
//   if (wire_dist < NEAR_DIST) // Too close
//   {
//     return -increment_z;
//   }
//   if (wire_dist < FAR_DIST) // Middle
//   {
//     in_mid_count++;
//     return (double)(wire_dist - mid_dist);
//   }
//   return increment_z; // Too far
// }

// double get_travel_dz(float wire_dist, double increment_z)
// {
//   if (wire_dist < NEAR_DIST) // Too close
//   {
//     return -increment_z;
//   }
//   if (wire_dist < FAR_DIST) // Middle
//   {
//     in_mid_count++;
//     return (double)(wire_dist - mid_dist);
//   }
//   return increment_z; // Too far
// }

// double compute_dist_latlon(double lat1, double long1, double lat2, double long2)
// {
//   double dlat1 = from_degrees(lat1);
//   double dlong1 = from_degrees(long1);
//   double dlat2 = from_degrees(lat2);
//   double dlong2 = from_degrees(long2);
//   double dLong = dlong1 - dlong2;
//   double dLat = dlat1 - dlat2;
//   double aHarv = pow(sin(dLat / 2.0), 2.0) + cos(lat1) * cos(lat2) * pow(sin(dLong / 2), 2);
//   double cHarv = 2 * atan2(sqrt(aHarv), sqrt(1.0 - aHarv));
//   // Earth's radius from wikipedia varies between 6,356.750 km — 6,378.135 km (˜3,949.901 — 3,963.189 miles)
//   // The IUGG value for the equatorial radius of the Earth is 6378.137 km (3963.19 mile)
//   const double earth = 6378137; // meters
//   return earth * cHarv;
// }

// void delta_orientation(double droll, double dpitch, double dyaw)
// {
//   tf2::Quaternion delta_qtn;
//   delta_qtn.setRPY(droll, dpitch, dyaw);

//   // tf2::Quaternion qtn;
//   // quaternionMsgToTF(pose_stamped.pose.orientation, qtn);
//   // tf2::fromMsg(pose_stamped.pose.orientation, qtn);

//   geometry_msgs::Quaternion orn = pose_stamped.pose.orientation;
//   tf2::Quaternion qtn = tf2::Quaternion(orn.x, orn.y, orn.z, orn.w);

//   qtn = delta_qtn * qtn;
//   qtn.normalize();
//   // quaternionTFToMsg(qtn, pose_stamped.pose.orientation);
//   // pose_stamped.pose.orientation = tf2::toMsg(qtn);
//   pose_stamped.pose.orientation.w = qtn.getW();
//   pose_stamped.pose.orientation.x = qtn.getX();
//   pose_stamped.pose.orientation.y = qtn.getY();
//   pose_stamped.pose.orientation.z = qtn.getZ();
// }

// double rotate_x(double x, double y, double sinyaw, double cosyaw)
// {
//   return x * cosyaw - y * sinyaw;
// }

// double rotate_y(double x, double y, double sinyaw, double cosyaw)
// {
//   return x * sinyaw + y * cosyaw;
// }

// double dist(double x1, double y1, double x2, double y2)
// {
//   double deltax = x1 - x2;
//   double deltay = y1 - y2;
//   return sqrt(deltax * deltax + deltay * deltay);
// }

// // Manual: MANUAL
// // Return: AUTO.RTL
// // AUTO_MISSION
// // Hold: AUTO.LOITER
// // Stabilized: STABILIZED
// // Altitude: ALTCTL
// // Follow Me: AUTO.FOLLOW_TARGET
// int main(int argc, char **argv)
// {
//   /* Prepare GPS csv file and log txt file starts */
//   // init_min_array();
//   // std::string rootdir = "/home/jiangchuan/livox_data/";
//   std::string rootdir = "/home/pi/livox_data/";
//   int status = mkdir(rootdir.c_str(), 0777);
//   get_time();
//   std::string time_str = get_time_str();
//   timedir = rootdir + time_str + "/";
//   status = mkdir(timedir.c_str(), 0777);
//   gps_filename = timedir + time_str + ".csv";
//   std::string log_filename = timedir + time_str + ".txt";
//   /* Prepare GPS csv file and log txt file ends */

//   /* Start Livox */
//   if (!Init())
//   {
//     ROS_FATAL("Livox-SDK init fail, trying to land");
//     return -1;
//   }
//   add_local_broadcast_code();
//   if (argc >= BD_ARGC_NUM)
//   {
//     ROS_INFO("Commandline input %s", argv[BD_ARGV_POS]);
//     add_commandline_broadcast_code(argv[BD_ARGV_POS]);
//   }
//   memset(lidars, 0, sizeof(lidars));
//   SetBroadcastCallback(OnDeviceBroadcast);
//   SetDeviceStateUpdateCallback(OnDeviceChange);
//   if (!Start())
//   {
//     ROS_INFO("Livox not started, trying to land");
//     Uninit();
//     return -1;
//   }
//   /* Start Livox Ends*/

//   std::cout << "Finished starting Livox lidar" << std::endl;
//   string_to_file(log_filename, "Finished starting Livox lidar\n");

//   /* ros related */
//   ros::init(argc, argv, "livox_lidar_publisher");
//   ros::NodeHandle nh;

//   std::cout << "Initiated ROS" << std::endl;
//   string_to_file(log_filename, "Initiated ROS\n");

//   // TODO: Compensate GPS rod

//   ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
//   ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
//   ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gps_callback);
//   ros::Subscriber alt_sub = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, alt_callback);
//   ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//   ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//   ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
//   ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

//   std::cout << "Declared ROS Subscribers and ServiceClients" << std::endl;
//   string_to_file(log_filename, "Declared ROS Subscribers and ServiceClients\n");

//   // ros::Time::init();
//   //the setpoint publishing rate MUST be faster than 2Hz
//   ros::Rate rate((double)ROS_RATE);

//   int n_try_rate = 20 * ROS_RATE;
//   // wait for FCU connection
//   for (int i = 0; ros::ok() && !current_state.connected && i < n_try_rate; i++)
//   {
//     ros::spinOnce();
//     rate.sleep();
//     // ROS_INFO("connecting to FCU ...");
//     // string_to_file(log_filename, "connecting to FCU ...\n");
//   }
//   std::cout << "Connected to flight control unit" << std::endl;
//   string_to_file(log_filename, "Connected to flight control unit\n");

//   // // wait for local position feed
//   // for (int i = 0; ros::ok() && no_position_yet() && i < n_try_rate; i++)
//   // {
//   //   ros::spinOnce();
//   //   rate.sleep();
//   //   ROS_INFO("getting local position ...");
//   //   string_to_file(log_filename, "getting local position ...\n");
//   // }
//   // std::cout << "Got position feed" << std::endl;
//   // string_to_file(log_filename, "Got position feed\n");

//   set_position(pose_in.position.x, pose_in.position.y, pose_in.position.z);
//   pose_stamped.pose.position.z = pose_stamped.pose.position.z > 0 ? pose_stamped.pose.position.z + 2.0 : 2.0;
//   std::cout << "Initial z = " << pose_stamped.pose.position.z << std::endl;

//   double sum_x = 0.0;
//   double sum_y = 0.0;

//   double sum_qtn_x = 0.0;
//   double sum_qtn_y = 0.0;
//   double sum_qtn_z = 0.0;
//   double sum_qtn_w = 0.0;

//   // int n_setpts = 100;
//   int n_setpts = 40;
//   //send a few setpoints before starting
//   for (int i = 0; ros::ok() && i < n_setpts; i++)
//   {
//     local_pos_pub.publish(pose_stamped);
//     sum_x += pose_in.position.x;
//     sum_y += pose_in.position.y;
//     sum_qtn_x += pose_in.orientation.x;
//     sum_qtn_y += pose_in.orientation.y;
//     sum_qtn_z += pose_in.orientation.z;
//     sum_qtn_w += pose_in.orientation.w;
//     ros::spinOnce();
//     rate.sleep();
//   }
//   // std::cout << "Sent a few setpoints bofore starting" << std::endl;
//   // string_to_file(log_filename, "Sent a few setpoints bofore starting\n");

//   // change to offboard mode
//   mavros_msgs::SetMode offb_set_mode;
//   offb_set_mode.request.custom_mode = "OFFBOARD";
//   ros::Time last_request = ros::Time::now();
//   int n_try = 5;
//   int i_try = 0;
//   while (ros::ok() && current_state.mode != "OFFBOARD" && i_try < n_try)
//   {
//     if (ros::Time::now() - last_request > ros::Duration(5.0))
//     {
//       ROS_INFO(current_state.mode.c_str());
//       if (set_mode_client.call(offb_set_mode) &&
//           offb_set_mode.response.mode_sent)
//       {
//         ROS_INFO("Offboard enabled");
//       }
//       last_request = ros::Time::now();
//       i_try++;
//     }
//     local_pos_pub.publish(pose_stamped);
//     ros::spinOnce();
//     rate.sleep();
//     // ROS_INFO("setting offboard mode ...");
//   }

//   std::cout << "Engaged offboard mode" << std::endl;
//   string_to_file(log_filename, "Engaged offboard mode\n");

//   // arm
//   mavros_msgs::CommandBool arm_cmd;
//   arm_cmd.request.value = true;
//   i_try = 0;
//   while (ros::ok() && !current_state.armed && i_try < n_try)
//   {
//     if (ros::Time::now() - last_request > ros::Duration(5.0))
//     {
//       if (arming_client.call(arm_cmd) &&
//           arm_cmd.response.success)
//       {
//         ROS_INFO("Vehicle armed");
//       }
//       last_request = ros::Time::now();
//       i_try++;
//     }
//     local_pos_pub.publish(pose_stamped);
//     ros::spinOnce();
//     rate.sleep();
//     // ROS_INFO("arming ...");
//   }
//   std::cout << "Armed" << std::endl;
//   string_to_file(log_filename, "Armed\n");

//   //////////////////////////////////
//   double x0 = sum_x / n_setpts;
//   double y0 = sum_y / n_setpts;

//   double qtn_x0 = sum_qtn_x / n_setpts;
//   double qtn_y0 = sum_qtn_y / n_setpts;
//   double qtn_z0 = sum_qtn_z / n_setpts;
//   double qtn_w0 = sum_qtn_w / n_setpts;

//   std::cout << "Avg time loc: x = " << x0 << ", y = " << y0 << std::endl;
//   std::cout << "Initial pose_stamped.pose.position.z = " << pose_stamped.pose.position.z << std::endl;
//   std::cout << "Initial pose_in.position.z = " << pose_in.position.z << std::endl;

//   pose_stamped.pose.position.x = x0;
//   pose_stamped.pose.position.y = y0;
//   pose_stamped.pose.orientation.x = qtn_x0;
//   pose_stamped.pose.orientation.y = qtn_y0;
//   pose_stamped.pose.orientation.z = qtn_z0;
//   pose_stamped.pose.orientation.w = qtn_w0;

//   /////////////////////////////
//   // Initial rise 10 m
//   ROS_INFO("Initial rise for %g meters", INITIAL_RISE);
//   double rise_time = INITIAL_RISE / DRONE_SPEED_V;
//   int num_updates = (int)(round(rise_time * ROS_RATE / UPDATE_JUMP));
//   double idz = DRONE_SPEED_V * UPDATE_JUMP / ROS_RATE;
//   double startz = pose_stamped.pose.position.z;

//   curr_dist_index = 0;
//   for (int i = 0; i < num_updates; i++)
//   {
//     if (i % update_rate == 0 && i != 0)
//     {
//       float wire_dist = get_avg_dist(1.0f);
//       if (wire_dist < mid_dist)
//       {
//         break;
//       }
//     }
//     set_position(x0, y0, startz + (i + 1) * idz);
//     for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
//     {
//       local_pos_pub.publish(pose_stamped);
//       ros::spinOnce();
//       rate.sleep();
//     }
//   }
//   std::cout << "Completed initial rise" << std::endl;
//   string_to_file(log_filename, "Completed initial rise\n");

//   // Mission starts here
//   getRPY(qtn_x0, qtn_y0, qtn_z0, qtn_w0); // Get yaw
//   // getRPY(pose_in.orientation);            // Get yaw
//   ROS_INFO("  >>> INITIAL yaw = %1.1f degrees", to_degrees(yaw));

//   //////////////////////////////////////
//   double sinyaw = sin(yaw);
//   double cosyaw = cos(yaw);
//   double sinphi = sin(PHI);
//   double cosphi = cos(PHI);
//   double piphi = M_PI / 4.0 - PHI / 2.0;
//   double sinpiphi = sin(piphi);
//   double cospiphi = cos(piphi);

//   double h = SCAN_RADIUS + TURN_RADIUS * (cospiphi / sinpiphi - 1);
//   std::cout << "h = " << h << std::endl;

//   double xc1 = h * cosphi / sinphi - TURN_RADIUS;
//   double yc1 = h - TURN_RADIUS * cosphi / sinpiphi;
//   double xc2 = xc1;
//   double yc2 = -yc1;
//   double xc3 = -xc1;
//   double yc3 = yc1;
//   double xc4 = -xc1;
//   double yc4 = -yc1;

//   double rho = h / sinphi - TURN_RADIUS * cospiphi / sinpiphi;
//   double x1 = rho * cosphi;
//   double y1 = rho * sinphi;
//   double x2 = h * cosphi / sinphi;
//   double y2 = yc1;
//   double x3 = x2;
//   double y3 = -y2;
//   double x4 = x1;
//   double y4 = -y1;
//   double x5 = -x1;
//   double y5 = y1;
//   double x6 = -x2;
//   double y6 = y2;
//   double x7 = -x2;
//   double y7 = -y2;
//   double x8 = -x1;
//   double y8 = -y1;
//   double x1r = x0 + rotate_x(x1, y1, sinyaw, cosyaw);
//   double y1r = y0 + rotate_y(x1, y1, sinyaw, cosyaw);
//   double x2r = x0 + rotate_x(x2, y2, sinyaw, cosyaw);
//   double y2r = y0 + rotate_y(x2, y2, sinyaw, cosyaw);
//   double x3r = x0 + rotate_x(x3, y3, sinyaw, cosyaw);
//   double y3r = y0 + rotate_y(x3, y3, sinyaw, cosyaw);
//   double x4r = x0 + rotate_x(x4, y4, sinyaw, cosyaw);
//   double y4r = y0 + rotate_y(x4, y4, sinyaw, cosyaw);
//   double x5r = x0 + rotate_x(x5, y5, sinyaw, cosyaw);
//   double y5r = y0 + rotate_y(x5, y5, sinyaw, cosyaw);
//   double x6r = x0 + rotate_x(x6, y6, sinyaw, cosyaw);
//   double y6r = y0 + rotate_y(x6, y6, sinyaw, cosyaw);
//   double x7r = x0 + rotate_x(x7, y7, sinyaw, cosyaw);
//   double y7r = y0 + rotate_y(x7, y7, sinyaw, cosyaw);
//   double x8r = x0 + rotate_x(x8, y8, sinyaw, cosyaw);
//   double y8r = y0 + rotate_y(x8, y8, sinyaw, cosyaw);

//   // Lines
//   double xstart = x0;
//   double ystart = y0;
//   double xend = x1r;
//   double yend = y1r;
//   double travel_time = dist(xstart, ystart, xend, yend) / DRONE_SPEED_H;
//   int num_updates0 = (int)(round(travel_time * ROS_RATE / UPDATE_JUMP));
//   double idx = (xend - xstart) / (double)num_updates0;
//   double idy = (yend - ystart) / (double)num_updates0;
//   double x_arr0[num_updates0];
//   double y_arr0[num_updates0];
//   for (int i = 0; i < num_updates0; i++)
//   {
//     x_arr0[i] = xstart + (i + 1) * idx;
//     y_arr0[i] = ystart + (i + 1) * idy;
//   }

//   xstart = x8r, ystart = y8r, xend = x1r, yend = y1r;
//   travel_time = dist(xstart, ystart, xend, yend) / DRONE_SPEED_H;
//   int num_updates1 = (int)(round(travel_time * ROS_RATE / UPDATE_JUMP));
//   idx = (xend - xstart) / (double)num_updates1;
//   idy = (yend - ystart) / (double)num_updates1;
//   double x_arr1[num_updates1];
//   double y_arr1[num_updates1];
//   for (int i = 0; i < num_updates1; i++)
//   {
//     x_arr1[i] = xstart + (i + 1) * idx;
//     y_arr1[i] = ystart + (i + 1) * idy;
//   }

//   xstart = x2r, ystart = y2r, xend = x3r, yend = y3r;
//   travel_time = dist(xstart, ystart, xend, yend) / DRONE_SPEED_H;
//   int num_updates2 = (int)(round(travel_time * ROS_RATE / UPDATE_JUMP));
//   idx = (xend - xstart) / (double)num_updates2;
//   idy = (yend - ystart) / (double)num_updates2;
//   double x_arr2[num_updates2];
//   double y_arr2[num_updates2];
//   for (int i = 0; i < num_updates2; i++)
//   {
//     x_arr2[i] = xstart + (i + 1) * idx;
//     y_arr2[i] = ystart + (i + 1) * idy;
//   }

//   xstart = x4r, ystart = y4r, xend = x5r, yend = y5r;
//   travel_time = dist(xstart, ystart, xend, yend) / DRONE_SPEED_H;
//   int num_updates3 = (int)(round(travel_time * ROS_RATE / UPDATE_JUMP));
//   idx = (xend - xstart) / (double)num_updates3;
//   idy = (yend - ystart) / (double)num_updates3;
//   double x_arr3[num_updates3];
//   double y_arr3[num_updates3];
//   for (int i = 0; i < num_updates3; i++)
//   {
//     x_arr3[i] = xstart + (i + 1) * idx;
//     y_arr3[i] = ystart + (i + 1) * idy;
//   }

//   xstart = x6r, ystart = y6r, xend = x7r, yend = y7r;
//   travel_time = dist(xstart, ystart, xend, yend) / DRONE_SPEED_H;
//   int num_updates4 = (int)(round(travel_time * ROS_RATE / UPDATE_JUMP));
//   idx = (xend - xstart) / (double)num_updates4;
//   idy = (yend - ystart) / (double)num_updates4;
//   double x_arr4[num_updates4];
//   double y_arr4[num_updates4];
//   for (int i = 0; i < num_updates4; i++)
//   {
//     x_arr4[i] = xstart + (i + 1) * idx;
//     y_arr4[i] = ystart + (i + 1) * idy;
//   }

//   // Circles
//   double phirange = M_PI / 2.0 + PHI;
//   double cdist = phirange * TURN_RADIUS;

//   double ctravel_time = cdist / DRONE_SPEED_H;
//   int cnum_updates = (int)(round(ctravel_time * ROS_RATE / UPDATE_JUMP));
//   double cx_arr1[cnum_updates];
//   double cy_arr1[cnum_updates];
//   double cx_arr2[cnum_updates];
//   double cy_arr2[cnum_updates];
//   double cx_arr3[cnum_updates];
//   double cy_arr3[cnum_updates];
//   double cx_arr4[cnum_updates];
//   double cy_arr4[cnum_updates];

//   double theta1 = M_PI / 2.0 + PHI;
//   double theta2 = 0.0;
//   double theta3 = M_PI / 2.0 - PHI;
//   double theta4 = M_PI;
//   double dtheta = phirange / (double)cnum_updates;
//   for (int i = 0; i < cnum_updates; i++)
//   {
//     double itheta = theta1 - (i + 1) * dtheta;
//     double cx = xc1 + TURN_RADIUS * cos(itheta);
//     double cy = yc1 + TURN_RADIUS * sin(itheta);
//     cx_arr1[i] = x0 + rotate_x(cx, cy, sinyaw, cosyaw);
//     cy_arr1[i] = y0 + rotate_y(cx, cy, sinyaw, cosyaw);

//     itheta = theta2 - (i + 1) * dtheta;
//     cx = xc2 + TURN_RADIUS * cos(itheta);
//     cy = yc2 + TURN_RADIUS * sin(itheta);
//     cx_arr2[i] = x0 + rotate_x(cx, cy, sinyaw, cosyaw);
//     cy_arr2[i] = y0 + rotate_y(cx, cy, sinyaw, cosyaw);

//     itheta = theta3 + (i + 1) * dtheta;
//     cx = xc3 + TURN_RADIUS * cos(itheta);
//     cy = yc3 + TURN_RADIUS * sin(itheta);
//     cx_arr3[i] = x0 + rotate_x(cx, cy, sinyaw, cosyaw);
//     cy_arr3[i] = y0 + rotate_y(cx, cy, sinyaw, cosyaw);

//     itheta = theta4 + (i + 1) * dtheta;
//     cx = xc4 + TURN_RADIUS * cos(itheta);
//     cy = yc4 + TURN_RADIUS * sin(itheta);
//     cx_arr4[i] = x0 + rotate_x(cx, cy, sinyaw, cosyaw);
//     cy_arr4[i] = y0 + rotate_y(cx, cy, sinyaw, cosyaw);
//   }

//   ROS_INFO("Begin 8 style scan >>>");
//   int curr_loc = 0;
//   int iseg = 0;
//   bool completed = false;
//   float last_wire_dist = mid_dist;
//   // while (iseg < MAX_SCAN_SEG && in_mid_count < MAX_MID_SCAN_SEG)
//   while (true)
//   {
//     std::cout << "iseg = " << iseg << ", curr_loc = " << curr_loc << " -> ";
//     double *cx_arr;
//     double *cy_arr;
//     double dz = 0;
//     int curr_num_update = 0;

//     switch (curr_loc)
//     {
//     case 0:
//       curr_num_update = num_updates0;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 2.0);
//       cx_arr = x_arr0;
//       cy_arr = y_arr0;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 1;
//       break;
//     case 1:
//       curr_num_update = cnum_updates;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = cx_arr1;
//       cy_arr = cy_arr1;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 2;
//       break;
//     case 2:
//       curr_num_update = num_updates2;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = x_arr2;
//       cy_arr = y_arr2;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 3;
//       break;
//     case 3:
//       curr_num_update = cnum_updates;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = cx_arr2;
//       cy_arr = cy_arr2;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 4;
//       break;
//     case 4:
//       curr_num_update = num_updates3;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V);
//       cx_arr = x_arr3;
//       cy_arr = y_arr3;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 5;
//       break;
//     case 5:
//       curr_num_update = cnum_updates;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = cx_arr3;
//       cy_arr = cy_arr3;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 6;
//       break;
//     case 6:
//       curr_num_update = num_updates4;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = x_arr4;
//       cy_arr = y_arr4;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 7;
//       break;
//     case 7:
//       curr_num_update = cnum_updates;
//       dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 3.0);
//       cx_arr = cx_arr4;
//       cy_arr = cy_arr4;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 8;
//       break;
//     case 8:
//       if (iseg < MAX_SCAN_SEG && in_mid_count < MAX_MID_SCAN_SEG)
//       {
//         curr_num_update = num_updates1;
//         dz = get_travel_dz(last_wire_dist, DELTA_METERS_V);
//       }
//       else
//       {
//         completed = true;
//         curr_num_update = num_updates1 / 2;
//         dz = get_travel_dz(last_wire_dist, DELTA_METERS_V / 2.0);
//       }
//       cx_arr = x_arr1;
//       cy_arr = y_arr1;
//       std::cout << "Delta Z = " << dz << std::endl;
//       curr_loc = 1;
//       break;
//     }

//     // TODO: 1. Print the trajectory to see why there is a kink from first line to circle
//     //

//     double idz = dz / (double)curr_num_update;
//     double startz = pose_stamped.pose.position.z;
//     for (int i = 0; i < curr_num_update; i++)
//     {
//       set_position(cx_arr[i], cy_arr[i], startz + (i + 1) * idz);
//       for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
//       {
//         local_pos_pub.publish(pose_stamped);
//         ros::spinOnce();
//         rate.sleep();
//       }
//     }
//     if (completed)
//     {
//       break;
//     }
//     last_wire_dist = get_avg_dist((float)(curr_num_update * UPDATE_JUMP) / (float)ROS_RATE);
//     iseg++;
//   }
//   string_to_file(log_filename, "Completed periodic flight\n");

//   // Final drop 1 m to make sure it returns to x0, y0
//   ROS_INFO("Final drop 1 m to make sure it returns to x0, y0");
//   double drop_time = DELTA_METERS_V / DRONE_SPEED_V;
//   num_updates = (int)(round(drop_time * ROS_RATE / UPDATE_JUMP));
//   idz = -DRONE_SPEED_V * UPDATE_JUMP / ROS_RATE;
//   startz = pose_stamped.pose.position.z;
//   for (int i = 0; i < num_updates; i++)
//   {
//     set_position(x0, y0, startz + (i + 1) * idz);
//     for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
//     {
//       local_pos_pub.publish(pose_stamped);
//       ros::spinOnce();
//       rate.sleep();
//     }
//   }
//   string_to_file(log_filename, "Dropped 1 meter to return to x0, y0\n");

//   ROS_INFO("tring to land");
//   mavros_msgs::CommandTOL land_cmd;
//   land_cmd.request.yaw = 0.0f;
//   land_cmd.request.latitude = 0.0f;
//   land_cmd.request.longitude = 0.0f;
//   land_cmd.request.altitude = 0.0f;
//   while (ros::ok())
//   {
//     if (ros::Time::now() - last_request > ros::Duration(5.0))
//     {
//       if (land_client.call(land_cmd) && land_cmd.response.success)
//       {
//         ROS_INFO("Landing");
//         break;
//       }
//       last_request = ros::Time::now();
//     }
//     local_pos_pub.publish(pose_stamped);
//     ros::spinOnce();
//     rate.sleep();
//   }
//   string_to_file(log_filename, "Loaded\n");

//   Uninit();
//   string_to_file(log_filename, "Uninit Livox lidar done\n");

//   return 0;
// }




/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "tf/tf.h"

#define FLIGHT_ALTITUDE 2.0f

mavros_msgs::State current_state;
geometry_msgs::Quaternion current_qtn_msg;
tf::Quaternion current_qtn;
geometry_msgs::PoseStamped pose;

void state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_qtn_msg = msg->pose.orientation;
    // ROS_INFO("Local Pos Seq: [%d]", msg->header.seq);
    // ROS_INFO("Local Pos Position x: [%f], y: [%f], z: [%f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // ROS_INFO("Local Pos Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_lidar_publisher");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    // Wait for 5 seconds to get local position
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // ros::Duration(5).sleep();

    while (ros::ok() && fabs(current_qtn_msg.x) + fabs(current_qtn_msg.y) + fabs(current_qtn_msg.z) + fabs(current_qtn_msg.w) < 1e-6)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("getting local position...");
    }

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("Before trans x: [%f], y: [%f], z: [%f], w: [%f]", current_qtn_msg.x, current_qtn_msg.y, current_qtn_msg.z, current_qtn_msg.w);
    quaternionMsgToTF(current_qtn_msg, current_qtn);
    current_qtn.normalize();
    ROS_INFO("After trans x: [%f], y: [%f], z: [%f], w: [%f]", current_qtn.x(), current_qtn.y(), current_qtn.z(), current_qtn.w());

    quaternionTFToMsg(current_qtn, pose.pose.orientation);

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // change to offboard mode and arm
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
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

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
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    for (int iseg = 0; iseg < 4; iseg++)
    {
        // go to the first waypoint
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = FLIGHT_ALTITUDE;

        ROS_INFO("going to the first way point");
        for (int i = 0; ros::ok() && i < 10 * 20; ++i)
        {
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("first way point finished!");

        // go to the second waypoint
        pose.pose.position.x = 0;
        pose.pose.position.y = 1;
        pose.pose.position.z = FLIGHT_ALTITUDE;

        //send setpoints for 10 seconds
        ROS_INFO("going to second way point");
        for (int i = 0; ros::ok() && i < 10 * 20; ++i)
        {

            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("second way point finished!");

        // go to the third waypoint
        pose.pose.position.x = 1;
        pose.pose.position.y = 1;
        pose.pose.position.z = FLIGHT_ALTITUDE;
        //send setpoints for 10 seconds
        ROS_INFO("going to third way point");
        for (int i = 0; ros::ok() && i < 10 * 20; ++i)
        {

            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("third way point finished!");

        // go to the forth waypoint
        pose.pose.position.x = 1;
        pose.pose.position.y = 0;
        pose.pose.position.z = FLIGHT_ALTITUDE;
        //send setpoints for 10 seconds
        ROS_INFO("going to forth way point");
        for (int i = 0; ros::ok() && i < 10 * 20; ++i)
        {

            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("forth way point finished!");
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    for (int i = 0; ros::ok() && i < 10 * 20; ++i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
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
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

