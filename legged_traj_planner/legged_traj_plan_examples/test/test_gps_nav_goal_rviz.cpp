/**
 * @file test_gps_nav_goal_rviz.cpp
 * @brief RViz test for gps_nav_goal_topic_cb mapping logic: visualize RTK origin and mapped goal.
 *
 * This node re-implements the minimal math inside Hexapod201StateSequencePlanner::gps_nav_goal_topic_cb
 * so you can test/observe the visualization in RViz without running the full planner.
 *
 * Subscriptions:
 *   /gps/fix   (sensor_msgs/NavSatFix)          : RTK lat/lon/alt (reference)
 *   /heading   (geometry_msgs/QuaternionStamped): RTK heading quaternion (from driver.py)
 *   /gps_nav_goal (sensor_msgs/NavSatFix)      : target lat/lon/alt
 *
 * Publications:
 *   visualizer_markers (visualization_msgs/MarkerArray)
 *
 * Parameters:
 *   ~frame_id (string, default: "rtk_map")
 *   ~viz_frame_id (string, default: "") : 如果非空，则 Marker 发布在该 frame 下（推荐: base_link），显示相对机器人坐标
 *   ~world_origin_x (double, default: 0.0)  : RTK world position x (for visualization)
 *   ~world_origin_y (double, default: 0.0)  : RTK world position y
 *   ~world_origin_z (double, default: 0.0)  : RTK world position z
 *   ~rtk_heading_yaw_offset_deg (double, default: 0.0) : RTK heading yaw 常量偏置（度）。修正方式为 corrected = raw + offset。
 *       例如：RTK 比实际大 90°（162.678 vs 72.678），则设置为 -90。
 *   ~publish_nav_goal (bool, default: false) : 是否额外发布一个 PoseStamped 导航目标
 *   ~nav_goal_topic (string, default: "/move_base_simple/goal") : 导航目标发布话题
 *   ~nav_goal_frame_id (string, default: "world") : 导航目标 pose 的 frame_id
 *
 * Notes:
 *   - This node assumes ENU axes align with the configured world frame (yaw_offset=0).
 *     If you want to test yaw alignment, set ~robot_yaw_world and/or override yaw_offset.
 */

#include <cmath>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
static double wrapToPi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

struct UtmCoord
{
  int zone = 0;          // 1..60
  bool northp = true;    // true: northern hemisphere
  double easting = 0.0;  // meters
  double northing = 0.0; // meters
};

static int lonToUtmZone(double lon_deg)
{
  // UTM zones are 6-deg wide, zone 1 starts at lon=-180.
  // Clamp to [1,60].
  int zone = static_cast<int>(std::floor((lon_deg + 180.0) / 6.0)) + 1;
  if (zone < 1)
    zone = 1;
  if (zone > 60)
    zone = 60;
  return zone;
}

// WGS84 lat/lon (deg) -> UTM (easting/northing in meters).
// Formula based on standard Transverse Mercator projection used by UTM.
// Note: This implementation does not handle the Norway/Svalbard special zone rules.
static UtmCoord latLonToUtm(double lat_deg, double lon_deg, int zone_override = 0)
{
  // WGS84 ellipsoid
  constexpr double a = 6378137.0;                  // semi-major axis
  constexpr double f = 1.0 / 298.257223563;        // flattening
  constexpr double e2 = f * (2.0 - f);             // eccentricity^2
  constexpr double ep2 = e2 / (1.0 - e2);          // e'^2
  constexpr double k0 = 0.9996;

  UtmCoord out;
  out.northp = (lat_deg >= 0.0);
  out.zone = (zone_override != 0) ? zone_override : lonToUtmZone(lon_deg);

  const double lat = lat_deg * M_PI / 180.0;
  const double lon = lon_deg * M_PI / 180.0;
  const double lon0_deg = (out.zone - 1) * 6.0 - 180.0 + 3.0; // central meridian
  const double lon0 = lon0_deg * M_PI / 180.0;

  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double tan_lat = std::tan(lat);

  const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  const double T = tan_lat * tan_lat;
  const double C = ep2 * cos_lat * cos_lat;
  const double A = cos_lat * (lon - lon0);

  // Meridional arc
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;
  const double M = a * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat
                        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * std::sin(2.0 * lat)
                        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat)
                        - (35.0 * e6 / 3072.0) * std::sin(6.0 * lat));

  const double A2 = A * A;
  const double A3 = A2 * A;
  const double A4 = A3 * A;
  const double A5 = A4 * A;
  const double A6 = A5 * A;

  // Easting
  double x = k0 * N * (A + (1.0 - T + C) * A3 / 6.0
                       + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0);
  x += 500000.0;

  // Northing
  double y = k0 * (M + N * tan_lat * (A2 / 2.0
                                      + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0
                                      + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0));
  if (!out.northp)
  {
    // false northing for southern hemisphere
    y += 10000000.0;
  }

  out.easting = x;
  out.northing = y;
  return out;
}

// UTM-based delta in local plane (meters).
// We force target to be computed in the same zone as reference, to keep deltas continuous.
static void wgs84DeltaToUtmMeters(double ref_lat_deg, double ref_lon_deg,
                                  double tgt_lat_deg, double tgt_lon_deg,
                                  double &out_east_m, double &out_north_m)
{
  const UtmCoord ref = latLonToUtm(ref_lat_deg, ref_lon_deg, 0);
  const UtmCoord tgt = latLonToUtm(tgt_lat_deg, tgt_lon_deg, ref.zone);

  // If hemisphere differs from ref, delta across equator will be wrong due to false northing;
  // for typical use (same local area) this shouldn't happen.
  out_east_m = tgt.easting - ref.easting;
  out_north_m = tgt.northing - ref.northing;
}

static visualization_msgs::Marker makeSphere(const std::string &frame_id, int id,
                                            double x, double y, double z,
                                            double radius,
                                            float r, float g, float b, float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "gps_goal_test";
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.scale.x = radius * 2.0;
  m.scale.y = radius * 2.0;
  m.scale.z = radius * 2.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = ros::Duration(0.0);
  return m;
}

static visualization_msgs::Marker makeLine(const std::string &frame_id, int id,
                                          double x0, double y0, double z0,
                                          double x1, double y1, double z1,
                                          double width,
                                          float r, float g, float b, float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "gps_goal_test";
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  geometry_msgs::Point p0;
  p0.x = x0;
  p0.y = y0;
  p0.z = z0;
  geometry_msgs::Point p1;
  p1.x = x1;
  p1.y = y1;
  p1.z = z1;
  m.points.push_back(p0);
  m.points.push_back(p1);
  m.lifetime = ros::Duration(0.0);
  return m;
}

static visualization_msgs::Marker makeDashedLine(const std::string &frame_id, int id,
                                                 double x0, double y0, double z0,
                                                 double yaw,
                                                 double total_length,
                                                 double dash_len,
                                                 double gap_len,
                                                 double width,
                                                 float r, float g, float b, float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "gps_goal_test";
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;

  const double dx = std::cos(yaw);
  const double dy = std::sin(yaw);

  double s = 0.0;
  while (s < total_length)
  {
    const double seg_start = s;
    const double seg_end = std::min(s + dash_len, total_length);

    geometry_msgs::Point p0;
    p0.x = x0 + dx * seg_start;
    p0.y = y0 + dy * seg_start;
    p0.z = z0;
    geometry_msgs::Point p1;
    p1.x = x0 + dx * seg_end;
    p1.y = y0 + dy * seg_end;
    p1.z = z0;

    m.points.push_back(p0);
    m.points.push_back(p1);

    s += dash_len + gap_len;
  }

  m.lifetime = ros::Duration(0.0);
  return m;
}
} // namespace

class GpsNavGoalRvizTest
{
public:
  explicit GpsNavGoalRvizTest(ros::NodeHandle nh)
      : nh_(std::move(nh))
      , tf_buffer_()
      , tf_listener_(tf_buffer_)
  {
  nh_.param<std::string>("frame_id", frame_id_, std::string("rtk_map"));
  nh_.param<std::string>("viz_frame_id", viz_frame_id_, std::string(""));
  nh_.param<std::string>("anchor_frame_id", anchor_frame_id_, std::string("rtk_antenna"));
  nh_.param<bool>("use_anchor_as_origin", use_anchor_as_origin_, true);
  nh_.param<double>("anchor_origin_offset_forward_m", anchor_origin_offset_forward_m_, 0.01);
  nh_.param<double>("anchor_origin_offset_left_m", anchor_origin_offset_left_m_, 0.01);
  nh_.param<double>("anchor_origin_offset_up_m", anchor_origin_offset_up_m_, 0.0);
  nh_.param<double>("world_origin_x", world_origin_x_, 0.0);
  nh_.param<double>("world_origin_y", world_origin_y_, 0.0);
  nh_.param<double>("world_origin_z", world_origin_z_, 0.0);
  nh_.param<double>("robot_yaw_world", robot_yaw_world_fallback_, 0.0);

  // If true, latch the first received /gps/fix as start reference (blue point).
  // This is useful when you want the start to be "the first RTK fix after a walk command".
  nh_.param<bool>("auto_start_from_first_fix", auto_start_from_first_fix_, false);

  nh_.param<double>("rtk_heading_length", rtk_heading_length_, 3.0);
  nh_.param<double>("rtk_heading_dash_len", rtk_heading_dash_len_, 0.3);
  nh_.param<double>("rtk_heading_gap_len", rtk_heading_gap_len_, 0.2);
  nh_.param<double>("rtk_heading_width", rtk_heading_width_, 0.05);

  // Heading conversion / calibration
  // rtk_heading_yaw_offset_deg: applied after convention conversion.
  // convention:
  //  - "ros_yaw"   : quaternion already follows ROS ENU yaw (0=+X/east, CCW positive).
  //  - "north_cw"  : heading is compass angle (0=north/+Y, clockwise positive).
  //                 Convert via yaw_ros = (pi/2) - heading.
  //  - "north_ccw" : heading is 0=north/+Y, counter-clockwise positive.
  //                 Convert via yaw_ros = (pi/2) + heading.
  nh_.param<std::string>("rtk_heading_convention", rtk_heading_convention_, std::string("ros_yaw"));
  nh_.param<double>("rtk_heading_yaw_offset_deg", rtk_heading_yaw_offset_deg_, 0.0);

  // If true, the mapping reference yaw (used to rotate goal/fix) will use the converted+offset heading.
  // If false, mapping ignores heading (ref_yaw=0), but RViz still draws the magenta heading line.
  // This is handy when you only want to debug display without rotating the mapped goal.
  nh_.param<bool>("use_heading_for_mapping", use_heading_for_mapping_, true);

  // If true, latch mapping reference (RTK lat/lon/alt + RTK yaw) at the moment we receive
  // the first /gps_nav_goal. After latching, goal mapping will NOT change with /gps/fix updates.
  nh_.param<bool>("latch_reference_on_goal", latch_reference_on_goal_, true);

  nh_.param<bool>("publish_nav_goal", publish_nav_goal_, false);
  nh_.param<std::string>("nav_goal_topic", nav_goal_topic_, std::string("/move_base_simple/goal"));
  nh_.param<std::string>("nav_goal_frame_id", nav_goal_frame_id_, std::string("world"));

  nh_.param<bool>("use_tf_yaw", use_tf_yaw_, true);
  nh_.param<std::string>("tf_world_frame", tf_world_frame_, frame_id_);
  nh_.param<std::string>("tf_base_frame", tf_base_frame_, std::string("base_link"));

    if (tf_world_frame_.empty())
    {
      tf_world_frame_ = frame_id_;
    }

  // Publish to a global topic so RViz can subscribe with the default name.
  ros::NodeHandle nh_global;
  marker_pub_ = nh_global.advertise<visualization_msgs::MarkerArray>("/visualizer_markers", 1, true);
  nav_goal_pub_ = nh_global.advertise<geometry_msgs::PoseStamped>(nav_goal_topic_, 1, false);

  rtk_fix_sub_ = nh_.subscribe("/gps/fix", 5, &GpsNavGoalRvizTest::rtkFixCb, this);
  heading_sub_ = nh_.subscribe("/heading", 5, &GpsNavGoalRvizTest::headingCb, this);
  start_sub_ = nh_.subscribe("/gps_start_position", 1, &GpsNavGoalRvizTest::startCb, this);
  goal_sub_ = nh_.subscribe("/gps_nav_goal", 1, &GpsNavGoalRvizTest::goalCb, this);

    // Heartbeat: publish at low rate so RViz users can confirm the node is alive.
  heartbeat_timer_ = nh_.createTimer(ros::Duration(0.5), &GpsNavGoalRvizTest::heartbeatTimerCb, this);

  ROS_INFO_STREAM("[gps_goal_test] node started. Publishing MarkerArray on /visualizer_markers in frame_id='" << (viz_frame_id_.empty() ? frame_id_ : viz_frame_id_) << "'.");
  ROS_INFO("[gps_goal_test] topics: /gps/fix (NavSatFix, RTK realtime), /gps_start_position (NavSatFix, start/ref), /gps_nav_goal (NavSatFix, goal), /heading (QuaternionStamped)");
  ROS_INFO_STREAM("[gps_goal_test] start mode: " << (auto_start_from_first_fix_ ? "auto_start_from_first_fix=TRUE" : "manual /gps_start_position"));
  ROS_INFO_STREAM("[gps_goal_test] latch_reference_on_goal=" << (latch_reference_on_goal_ ? "true" : "false"));
  ROS_INFO_STREAM("[gps_goal_test] publish_nav_goal=" << (publish_nav_goal_ ? "true" : "false") << ", nav_goal_topic='" << nav_goal_topic_ << "', nav_goal_frame_id='" << nav_goal_frame_id_ << "'");
  ROS_INFO_STREAM("[gps_goal_test] rtk_heading_yaw_offset_deg=" << rtk_heading_yaw_offset_deg_ << " (corrected = raw + offset)");
  ROS_INFO_STREAM("[gps_goal_test] rtk_heading_convention='" << rtk_heading_convention_ << "'"
                                                              << ", use_heading_for_mapping=" << (use_heading_for_mapping_ ? "true" : "false"));
  ROS_INFO_STREAM("[gps_goal_test] anchor: use_anchor_as_origin=" << (use_anchor_as_origin_ ? "true" : "false")
                                                                  << ", anchor_frame_id='" << anchor_frame_id_ << "'"
                                                                  << ", origin_offset(fwd,left,up)= (" << anchor_origin_offset_forward_m_ << ", "
                                                                  << anchor_origin_offset_left_m_ << ", " << anchor_origin_offset_up_m_ << ") m");
  ROS_INFO_STREAM("[gps_goal_test] yaw source: "
                    << (use_tf_yaw_ ? "TF" : "param")
                    << ", tf_world_frame='" << tf_world_frame_ << "', tf_base_frame='" << tf_base_frame_ << "', fallback robot_yaw_world=" << robot_yaw_world_fallback_);
  }

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool getAnchorOriginInFrame(double &ox, double &oy, double &oz)
  {
    if (!use_anchor_as_origin_)
      return false;
    if (anchor_frame_id_.empty())
      return false;

    try
    {
      // anchor pose expressed in frame_id_
      const geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          frame_id_, anchor_frame_id_, ros::Time(0), ros::Duration(0.05));

      ox = tf_msg.transform.translation.x;
      oy = tf_msg.transform.translation.y;
      oz = tf_msg.transform.translation.z;

      // Offset is expressed in anchor local axes, then rotated into frame_id_.
      const double fwd = anchor_origin_offset_forward_m_;
      const double left = anchor_origin_offset_left_m_;
      const double up = anchor_origin_offset_up_m_;
      if (std::fabs(fwd) > 1e-9 || std::fabs(left) > 1e-9 || std::fabs(up) > 1e-9)
      {
        tf2::Quaternion q;
        tf2::fromMsg(tf_msg.transform.rotation, q);
        const tf2::Vector3 off_local(fwd, left, up);
        const tf2::Vector3 off_world = tf2::quatRotate(q, off_local);
        ox += off_world.x();
        oy += off_world.y();
        oz += off_world.z();
      }
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "[gps_goal_test] TF lookup failed (" << frame_id_ << "->" << anchor_frame_id_
                               << ") for anchor origin: " << ex.what() << ". Falling back to world_origin_* params.");
      return false;
    }
  }

  bool transformPointFromWorldToFrame(const std::string &target_frame,
                                      double xw, double yw, double zw,
                                      double &xo, double &yo, double &zo)
  {
    try
    {
      const geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          target_frame, frame_id_, ros::Time(0), ros::Duration(0.05));

      // Apply transform: p_target = R * p_source + t
      // NOTE: xw/yw/zw are expressed in frame_id_. They must NOT include the world origin offset;
      // callers should pass coordinates already in frame_id_.
      tf2::Quaternion q;
      tf2::fromMsg(tf_msg.transform.rotation, q);
      const tf2::Vector3 p_src(xw, yw, zw);
      const tf2::Vector3 t(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
      const tf2::Vector3 p_dst = tf2::quatRotate(q, p_src) + t;

      xo = p_dst.x();
      yo = p_dst.y();
      zo = p_dst.z();
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[gps_goal_test] TF transform failed (" << frame_id_ << "->" << target_frame << "): " << ex.what());
      return false;
    }
  }
  void rtkFixCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    rtk_lat_ = msg->latitude;
    rtk_lon_ = msg->longitude;
    rtk_alt_ = msg->altitude;
    //goal_alt_ = msg->altitude;
    has_fix_ = true;

    if (auto_start_from_first_fix_ && !has_start_)
    {
      start_lat_ = rtk_lat_;
      start_lon_ = rtk_lon_;
      start_alt_ = rtk_alt_;
      has_start_ = true;
      ROS_WARN_STREAM("[gps_goal_test] auto latched start from first /gps/fix: (lat,lon,alt)= (" << start_lat_ << ", " << start_lon_ << ", " << start_alt_ << ")");
    }
    publishMarkers();
  }

  void headingCb(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
  {
    // Driver publishes a quaternion, but "yaw" convention can still differ:
    // - some drivers encode compass heading (0=north, clockwise),
    // - while ROS yaw is ENU (0=east, CCW).
    const double raw_yaw = tf2::getYaw(msg->quaternion);

    // Step1: convert convention -> ROS ENU yaw.
    double yaw_ros = raw_yaw;
    if (rtk_heading_convention_ == "north_cw")
    {
      // heading=0 north, clockwise positive. Convert to ROS yaw (0 east, CCW): yaw = 90deg - heading
      yaw_ros = (M_PI * 0.5) - raw_yaw;
    }
    else if (rtk_heading_convention_ == "north_ccw")
    {
      // heading=0 north, CCW positive. Convert: yaw = 90deg + heading
      yaw_ros = (M_PI * 0.5) + raw_yaw;
    }

    // Step2: apply constant offset calibration.
    const double offset_rad = rtk_heading_yaw_offset_deg_ * M_PI / 180.0;
    rtk_yaw_ = wrapToPi(yaw_ros + offset_rad);

  ROS_WARN_STREAM("[gps_goal_test] heading parsed: raw=" << (raw_yaw * 180.0 / M_PI)
          << " deg, conv='" << rtk_heading_convention_
          << "' => yaw_ros=" << (yaw_ros * 180.0 / M_PI)
          << " deg, offset=" << rtk_heading_yaw_offset_deg_
          << " deg, final rtk_yaw=" << (rtk_yaw_ * 180.0 / M_PI) << " deg");
    has_yaw_ = true;
    publishMarkers();
  }

  void goalCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    goal_lat_ = msg->latitude;
    goal_lon_ = msg->longitude;
    goal_alt_ = msg->altitude;
    has_goal_ = true;

    // Latch reference once, at the moment the first goal arrives.
    if (latch_reference_on_goal_ && !reference_latched_)
    {
      if (!has_fix_)
      {
        ROS_WARN_STREAM("[gps_goal_test] Can't latch reference on goal: /gps/fix not received yet.");
      }
      else
      {
        ref_lat_ = rtk_lat_;
        ref_lon_ = rtk_lon_;
        ref_alt_ = rtk_alt_;
        has_ref_pos_ = true;

        if (has_yaw_)
        {
          ref_rtk_yaw_ = rtk_yaw_;
          has_ref_yaw_ = true;
        }
        else
        {
          ref_rtk_yaw_ = 0.0;
          has_ref_yaw_ = false;
          ROS_WARN_STREAM("[gps_goal_test] Reference pos latched, but /heading not available. ref_yaw=0 will be used.");
        }

        reference_latched_ = true;
        ROS_WARN_STREAM("[gps_goal_test] REFERENCE LATCHED ON GOAL: ref(lat,lon,alt)= (" << ref_lat_ << ", " << ref_lon_ << ", " << ref_alt_
                                                                                         << "), ref_yaw=" << ref_rtk_yaw_ * 180.0 / M_PI << " deg");
      }
    }
    publishMarkers();
  }

  void startCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    start_lat_ = msg->latitude;
    start_lon_ = msg->longitude;
    start_alt_ = msg->altitude;
    has_start_ = true;
    publishMarkers();
  }

  void publishMarkers()
  {
    // Update robot yaw from TF if enabled.
    updateRobotYawFromTf();

    // Visualization origin in frame_id_. Prefer TF anchor if available; otherwise use params.
    double origin_x = world_origin_x_;
    double origin_y = world_origin_y_;
    double origin_z = world_origin_z_;
    {
      double ax = 0.0, ay = 0.0, az = 0.0;
      if (getAnchorOriginInFrame(ax, ay, az))
      {
        origin_x = ax;
        origin_y = ay;
        origin_z = az;
      }
    }

  const std::string out_frame = viz_frame_id_.empty() ? frame_id_ : viz_frame_id_;

    visualization_msgs::MarkerArray arr;

    // Always publish origin marker so RViz won't be empty.
  const double alive_x = viz_frame_id_.empty() ? origin_x : 0.0;
  const double alive_y = viz_frame_id_.empty() ? origin_y : 0.0;
  const double alive_z = viz_frame_id_.empty() ? origin_z : 0.0;
  arr.markers.push_back(makeSphere(out_frame, 1,
                  alive_x, alive_y, alive_z,
                                    0.05,
                                    0.0f, 1.0f, 0.0f, 1.0f));

    // Shared mapping reference (used by both green RTK fix marker and dashed heading anchor).
    const bool has_mapping_ref = (reference_latched_ && has_ref_pos_) || has_start_;
    const double map_ref_lat = (reference_latched_ && has_ref_pos_) ? ref_lat_ : start_lat_;
    const double map_ref_lon = (reference_latched_ && has_ref_pos_) ? ref_lon_ : start_lon_;
    const double map_ref_alt = (reference_latched_ && has_ref_pos_) ? ref_alt_ : start_alt_;
    const double map_ref_yaw = use_heading_for_mapping_
                                   ? ((reference_latched_ && has_ref_yaw_) ? ref_rtk_yaw_ : (has_yaw_ ? rtk_yaw_ : 0.0))
                                   : 0.0;
    // Yaw offset: align ENU/UTM heading to world using (robot_yaw_world - map_ref_yaw).
    const double yaw_offset = wrapToPi(robot_yaw_world_ - map_ref_yaw);
    // --- 在这行下面，加入以下代码，把这个角度变成真正的坐标系旋转 ---
    if (has_mapping_ref) 
    {
      geometry_msgs::TransformStamped transformStamped;
      ROS_WARN_STREAM_THROTTLE(2.0, "DEBUG INFO -> " 
    << " robot_yaw_world_: " << robot_yaw_world_ 
    << " map_ref_yaw: " << map_ref_yaw 
    << " yaw_offset: " << yaw_offset);
      
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "world";      // 父坐标系：机器人的建图世界
      transformStamped.child_frame_id = frame_id_;     // 子坐标系：rtk_map (默认值)

      // 1. 位置对齐：假设开机时两个世界的中心高度差 1.0 米（保持和你之前参数一致）
      transformStamped.transform.translation.x = origin_x;
      transformStamped.transform.translation.y = origin_y;
      transformStamped.transform.translation.z = origin_z + 1.0; 

      // 2. 朝向对齐：落实你的核心思路！把算出的 heading 偏差转换成四元数
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw_offset); // 绕 Z 轴旋转
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      // 3. 向整个 ROS 系统广播：“我现在用真实的 heading 修正了 RTK 的 X 轴方向！”
      tf_broadcaster_.sendTransform(transformStamped);
    }

    // =========================================================================
    // 1. 实时 RTK 定位 (绿球) 的解耦计算
    // =========================================================================
    bool has_fix_world = false;
    double fix_x_world = origin_x, fix_y_world = origin_y, fix_z_world = origin_z;
    double fix_x_rtk = origin_x, fix_y_rtk = origin_y, fix_z_rtk = origin_z;
    double fix_x_marker = origin_x, fix_y_marker = origin_y, fix_z_marker = origin_z;

    if (has_fix_ && has_mapping_ref)
    {
      double fix_dx_east = 0.0;
      double fix_dy_north = 0.0;
      wgs84DeltaToUtmMeters(map_ref_lat, map_ref_lon, rtk_lat_, rtk_lon_, fix_dx_east, fix_dy_north);

      const double fix_dist = std::hypot(fix_dx_east, fix_dy_north);
      const double fix_angle_enu = std::atan2(fix_dy_north, fix_dx_east);

      // (A) 真实的 World 坐标 (必须加 yaw_offset)
      const double fix_angle_world = wrapToPi(fix_angle_enu + yaw_offset);
      fix_x_world = origin_x + fix_dist * std::cos(fix_angle_world);
      fix_y_world = origin_y + fix_dist * std::sin(fix_angle_world);
      fix_z_world = origin_z + (rtk_alt_ - map_ref_alt);

      // (B) 纯粹的 rtk_map 坐标 (纯 ENU 偏差)
      fix_x_rtk = origin_x + fix_dx_east;
      fix_y_rtk = origin_y + fix_dy_north;
      fix_z_rtk = origin_z + (rtk_alt_ - map_ref_alt);

      has_fix_world = true;

      // (C) 决定给 RViz 绿球画哪个坐标
      if (out_frame == frame_id_) // 如果发布在 rtk_map
      {
          fix_x_marker = fix_x_rtk;
          fix_y_marker = fix_y_rtk;
          fix_z_marker = fix_z_rtk;
      }
      else if (out_frame == "world") // 如果发布在 world
      {
          fix_x_marker = fix_x_world;
          fix_y_marker = fix_y_world;
          fix_z_marker = fix_z_world;
      }
      else 
      {
          fix_x_marker = fix_x_world;
          fix_y_marker = fix_y_world;
          fix_z_marker = fix_z_world;
          if (!viz_frame_id_.empty())
          {
              transformPointFromWorldToFrame(viz_frame_id_, fix_x_world, fix_y_world, fix_z_world, fix_x_marker, fix_y_marker, fix_z_marker);
          }
      }
    }

    // =========================================================================
    // 2. RTK 航向虚线 (洋红色线)
    // =========================================================================
    if (has_fix_ && has_yaw_ && (rtk_heading_length_ > 1e-3) && (rtk_heading_dash_len_ > 1e-3))
    {
      // 这里的原点使用刚才确定好的正确 marker 坐标
      double dash_x_o = has_fix_world ? fix_x_marker : origin_x;
      double dash_y_o = has_fix_world ? fix_y_marker : origin_y;
      double dash_z_o = has_fix_world ? fix_z_marker : origin_z;

      arr.markers.push_back(makeDashedLine(out_frame, 5,
                                           dash_x_o, dash_y_o, dash_z_o,
                                           rtk_yaw_, // 因为是在 out_frame 里，不需要加 yaw_offset
                                           rtk_heading_length_,
                                           rtk_heading_dash_len_,
                                           std::max(0.0, rtk_heading_gap_len_),
                                           rtk_heading_width_,
                                           1.0f, 0.0f, 1.0f, 1.0f));
    }

    // =========================================================================
    // 3. 基础点绘制 (蓝球 & 绿球)
    // =========================================================================
    if (!has_mapping_ref)
    {
      marker_pub_.publish(arr);
      return;
    }

    // Start (blue)
    const double start_x = viz_frame_id_.empty() ? origin_x : 0.0;
    const double start_y = viz_frame_id_.empty() ? origin_y : 0.0;
    const double start_z = viz_frame_id_.empty() ? origin_z : 0.0;
    arr.markers.push_back(makeSphere(out_frame, 4, start_x, start_y, start_z, 0.05, 0.0f, 0.4f, 1.0f, 1.0f));

    // RTK fix (green)
    if (has_fix_world)
    {
      arr.markers.push_back(makeSphere(out_frame, 6, fix_x_marker, fix_y_marker, fix_z_marker, 0.05, 0.0f, 1.0f, 0.0f, 1.0f));
    }

    if (!has_goal_)
    {
      marker_pub_.publish(arr);
      return;
    }

    // =========================================================================
    // 4. 目标点 (黄球) 的解耦计算
    // =========================================================================
    double dx_east = 0.0;
    double dy_north = 0.0;
    wgs84DeltaToUtmMeters(map_ref_lat, map_ref_lon, goal_lat_, goal_lon_, dx_east, dy_north);

    const double dist = std::hypot(dx_east, dy_north);
    const double angle_enu = std::atan2(dy_north, dx_east);

    // (A) 发给机器人底盘的 World 坐标 (必须加 yaw_offset！)
    const double angle_world = wrapToPi(angle_enu + yaw_offset);
    const double goal_x_world = origin_x + dist * std::cos(angle_world);
    const double goal_y_world = origin_y + dist * std::sin(angle_world);
    const double goal_z_world = origin_z + (goal_alt_ - map_ref_alt);

    ROS_INFO_THROTTLE(1.0, 
      "[GOAL_CALC_DEBUG] "
      "东向偏移dx=%.4f m, 北向偏移dy=%.4f m, 直线距离dist=%.4f m, "    
      "原始ENU角度=%.4f rad (%.2f°), 最终世界角度=%.4f rad (%.2f°)",   
      dx_east, dy_north, dist,
      angle_enu, angle_enu * 180.0 / M_PI,
      angle_world, angle_world * 180.0 / M_PI
    );

    // (B) 发给 RViz 渲染的 rtk_map 坐标 (绝对不能加 yaw_offset！)
    const double goal_x_rtk = origin_x + dx_east;
    const double goal_y_rtk = origin_y + dy_north;
    const double goal_z_rtk = origin_z + (goal_alt_ - map_ref_alt);

    // (C) 决定给 RViz 的黄球画哪个点
    double goal_x = 0.0, goal_y = 0.0, goal_z = 0.0;

    if (out_frame == frame_id_) // 如果发给 rtk_map
    {
        goal_x = goal_x_rtk;
        goal_y = goal_y_rtk;
        goal_z = goal_z_rtk;
    }
    else if (out_frame == "world") // 如果发给 world
    {
        goal_x = goal_x_world;
        goal_y = goal_y_world;
        goal_z = goal_z_world;
    }
    else 
    {
        goal_x = goal_x_world;
        goal_y = goal_y_world;
        goal_z = goal_z_world;
        if (!viz_frame_id_.empty())
        {
          transformPointFromWorldToFrame(viz_frame_id_, goal_x_world, goal_y_world, goal_z_world, goal_x, goal_y, goal_z);
        }
    }

    // Goal (yellow)
    arr.markers.push_back(makeSphere(out_frame, 2, goal_x, goal_y, goal_z, 0.05, 1.0f, 1.0f, 0.0f, 1.0f));

    // Line (cyan)
    arr.markers.push_back(makeLine(out_frame, 3, start_x, start_y, start_z, goal_x, goal_y, goal_z, 0.06, 0.0f, 1.0f, 1.0f, 1.0f));

    // =========================================================================
    // 5. 将真正的底盘坐标发给导航系统
    // =========================================================================
    if (publish_nav_goal_)
    {
      geometry_msgs::PoseStamped nav_goal;
      nav_goal.header.stamp = ros::Time::now();
      nav_goal.header.frame_id = nav_goal_frame_id_; 
      
      // 强制使用带旋转偏移的 world 坐标给机器人！
      nav_goal.pose.position.x = goal_x_world;
      nav_goal.pose.position.y = goal_y_world;
      nav_goal.pose.position.z = goal_z_world;
      nav_goal.pose.orientation.w = 1.0;
      nav_goal_pub_.publish(nav_goal);
    }

    marker_pub_.publish(arr);
  ROS_INFO_STREAM_THROTTLE(1.0,
                             "[gps_goal_test] origin(param) (x,y,z)= (" << origin_x << ", " << origin_y << ", " << origin_z << ")"
                             << ", rtk(lat,lon,alt)= (" << rtk_lat_ << ", " << rtk_lon_ << ", " << rtk_alt_
                             << "), start(lat,lon,alt)= (" << start_lat_ << ", " << start_lon_ << ", " << start_alt_
                             << "), goal(lat,lon,alt)= (" << goal_lat_ << ", " << goal_lon_ << ", " << goal_alt_
                             << ") -> world(x,y,z)= (" << goal_x_world << ", " << goal_y_world << ", " << goal_z_world
                             << "), robot_yaw_world=" << robot_yaw_world_
                             << ", yaw_offset=" << yaw_offset);
  }

  void updateRobotYawFromTf()
  {
    if (!use_tf_yaw_)
    {
      robot_yaw_world_ = robot_yaw_world_fallback_;
      has_robot_yaw_ = true;
      return;
    }

    try
    {
      // Use time 0 for the latest available transform.
      const geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          tf_world_frame_, tf_base_frame_, ros::Time(0), ros::Duration(0.05));
      const double yaw = tf2::getYaw(tf_msg.transform.rotation);
      robot_yaw_world_ = yaw;
      has_robot_yaw_ = true;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "[gps_goal_test] TF yaw updated: yaw=" << yaw);
    }
    catch (const tf2::TransformException &ex)
    {
      // Fall back to parameter.
      robot_yaw_world_ = robot_yaw_world_fallback_;
      has_robot_yaw_ = false;
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "[gps_goal_test] TF lookup failed (" << tf_world_frame_ << "->" << tf_base_frame_
                               << "): " << ex.what() << ". Using fallback robot_yaw_world=" << robot_yaw_world_fallback_);
    }
  }

  void heartbeatTimerCb(const ros::TimerEvent &)
  {
    // Periodically print input readiness and publish origin marker.
    ROS_INFO_STREAM_THROTTLE(2.0,
                             "[gps_goal_test] ready: fix=" << (has_fix_ ? "Y" : "N")
                             << ", start=" << (has_start_ ? "Y" : "N")
                             << ", goal=" << (has_goal_ ? "Y" : "N")
                             << ", yaw=" << (has_yaw_ ? "Y" : "N")
                             << ". (Publishing origin marker regardless.)");
    publishMarkers();
  }

private:
  ros::NodeHandle nh_;
  std::string frame_id_;
  std::string viz_frame_id_;

  // Anchor TF frame (typically rtk_antenna)
  std::string anchor_frame_id_;
  bool use_anchor_as_origin_ = true;
  // Offset applied on top of anchor origin, expressed in anchor local axes.
  // forward=+X, left=+Y, up=+Z.
  double anchor_origin_offset_forward_m_ = 0.01;
  double anchor_origin_offset_left_m_ = 0.01;
  double anchor_origin_offset_up_m_ = 0.0;

  ros::Publisher marker_pub_;
  ros::Publisher nav_goal_pub_;
  ros::Subscriber rtk_fix_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Timer heartbeat_timer_;

  bool publish_nav_goal_ = false;
  std::string nav_goal_topic_;
  std::string nav_goal_frame_id_;

  double world_origin_x_ = 0.0;
  double world_origin_y_ = 0.0;
  double world_origin_z_ = 0.0;
  double robot_yaw_world_fallback_ = 0.0;
  double robot_yaw_world_ = 0.0;
  bool has_robot_yaw_ = false;

  double rtk_heading_length_ = 3.0;
  double rtk_heading_dash_len_ = 0.3;
  double rtk_heading_gap_len_ = 0.2;
  double rtk_heading_width_ = 0.05;

  std::string rtk_heading_convention_;
  double rtk_heading_yaw_offset_deg_ = 0.0;
  bool use_heading_for_mapping_ = true;

  bool use_tf_yaw_ = true;
  std::string tf_world_frame_;
  std::string tf_base_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double rtk_lat_ = 0.0;
  double rtk_lon_ = 0.0;
  double rtk_alt_ = 0.0;
  double rtk_yaw_ = 0.0;


  double goal_lat_ = 0.0;
  double goal_lon_ = 0.0;
  double goal_alt_ = 0.0;

  double start_lat_ = 0.0;
  double start_lon_ = 0.0;
  double start_alt_ = 0.0;

  bool has_fix_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool has_yaw_ = false;

  bool auto_start_from_first_fix_ = false;

  // Latch reference on first goal.
  bool latch_reference_on_goal_ = true;
  bool reference_latched_ = false;
  double ref_lat_ = 0.0;
  double ref_lon_ = 0.0;
  double ref_alt_ = 0.0;
  double ref_rtk_yaw_ = 0.0;
  bool has_ref_pos_ = false;
  bool has_ref_yaw_ = false;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_gps_nav_goal_rviz");
  ros::NodeHandle nh("~");
  GpsNavGoalRvizTest node(nh);
  ros::spin();
  return 0;
}
