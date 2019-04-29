#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <gazebo_rad_msgs/Cone.h>
#include <gazebo_rad_msgs/RadiationSource.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/ParamLoader.h>

namespace compton_camera_python
{

/* class Cone //{ */

class Cone {

public:
  Cone(Eigen::Isometry3d pose, double angle);
  Eigen::Isometry3d pose;
  double            angle;
};

Cone::Cone(Eigen::Isometry3d pose, double angle) : pose(pose), angle(angle){};

//}

/* class RadiationSource //{ */

class RadiationSource {

public:
  ros::Time                        last_update;
  gazebo_rad_msgs::RadiationSource source_msg;

  RadiationSource(gazebo_rad_msgs::RadiationSource source);
};
RadiationSource::RadiationSource(gazebo_rad_msgs::RadiationSource source) : source_msg(source) {

  last_update = ros::Time::now();
};

//}

/* ComptonPlotter //{ */
class ComptonPlotter : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

  /* ros::Publisher  publisher_odometry; */

private:
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

private:
  ros::Subscriber subscriber_cone;
  void            callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg);
  int             max_cones_;
  double          cone_length_;
  double          source_size_;

  std::list<Cone> cones;
  std::mutex      mutex_cones;

private:
  ros::Subscriber subscriber_source;
  void            callbackSource(const gazebo_rad_msgs::RadiationSourceConstPtr &msg);

  std::map<int, RadiationSource> sources;
  std::mutex                     mutex_sources;

private:
  ros::Timer main_timer;
  double     main_timer_rate_;
  void       mainTimer(const ros::TimerEvent &event);
};
//}

/* inInit() //{ */

void ComptonPlotter::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonPlotter]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonPlotter");

  param_loader.load_param("max_cones", max_cones_);
  param_loader.load_param("cone_length", cone_length_);
  param_loader.load_param("main_timer_rate", main_timer_rate_);
  param_loader.load_param("source_size", source_size_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_cone   = nh_.subscribe("cone_in", 1, &ComptonPlotter::callbackCone, this, ros::TransportHints().tcpNoDelay());
  subscriber_source = nh_.subscribe("source_in", 1, &ComptonPlotter::callbackSource, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  /* publisher_odometry = nh_.advertise<nav_msgs::Odometry>("odometry_out", 1); */

  // --------------------------------------------------------------
  // |                     visualization tools                    |
  // --------------------------------------------------------------

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("local_origin", "/rviz_visual_markers"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting

  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &ComptonPlotter::mainTimer, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[ComptonPlotter]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[ComptonPlotter]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackCone() //{ */

void ComptonPlotter::callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg) {

  if (!is_initialized)
    return;

  std::scoped_lock lock(mutex_cones);

  ROS_INFO("[ComptonPlotter]: got a cone!");

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation().x() = msg->pose.position.x;
  pose1.translation().y() = msg->pose.position.y;
  pose1.translation().z() = msg->pose.position.z;

  Eigen::Quaterniond q = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  pose1 = pose1 * q;

  Cone new_cone(pose1, msg->angle);
  cones.push_back(new_cone);

  if (cones.size() > uint(max_cones_)) {
    cones.pop_front();
  }
}

//}

/* callbackSource() //{ */

void ComptonPlotter::callbackSource(const gazebo_rad_msgs::RadiationSourceConstPtr &msg) {

  if (!is_initialized)
    return;

  std::scoped_lock lock(mutex_sources);

  std::map<int, RadiationSource>::iterator it;
  it = sources.find(msg->id);

  if (it == sources.end()) {

    RadiationSource new_source(*msg);

    sources.insert(std::pair<int, RadiationSource>(msg->id, new_source));

    ROS_INFO("[ComptonPlotter]: registering new source [%d]", msg->id);

  } else {

    it->second.last_update = ros::Time::now();
    it->second.source_msg  = *msg;
  }
}

//}

/* mainTimer() //{ */

void ComptonPlotter::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  visual_tools_->deleteAllMarkers();

  // plot the cones
  {
    std::scoped_lock lock(mutex_cones);

    std::list<Cone>::iterator it;
    for (it = cones.begin(); it != cones.end(); it++) {
      visual_tools_->publishCone(it->pose, it->angle, rviz_visual_tools::colors::TRANSLUCENT_LIGHT, cone_length_);
    }
  }

  // plot the radiation sources
  {
    std::scoped_lock lock(mutex_sources);

    std::map<int, RadiationSource>::iterator it;
    for (it = sources.begin(); it != sources.end(); it++) {

      if ((ros::Time::now() - it->second.last_update).toSec() > 1.0) {

        ROS_INFO("[ComptonPlotter]: removing source %d", it->first);
        sources.erase(it);
        continue;
      }

      double size = source_size_ / 2.0;

      Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
      pose1.translation().x() = it->second.source_msg.x - size;
      pose1.translation().y() = it->second.source_msg.y - size;
      pose1.translation().z() = it->second.source_msg.z - size;

      Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
      pose2.translation().x() = it->second.source_msg.x + size;
      pose2.translation().y() = it->second.source_msg.y + size;
      pose2.translation().z() = it->second.source_msg.z + size;

      visual_tools_->publishCuboid(pose1.translation(), pose2.translation(), rviz_visual_tools::colors::BLUE);
    }
  }

  visual_tools_->trigger();
}

//}

}  // namespace compton_camera_python

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_python::ComptonPlotter, nodelet::Nodelet)
