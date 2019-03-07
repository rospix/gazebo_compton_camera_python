#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <gazebo_rad_msgs/Cone.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Eigen>

namespace compton_camera_python
{

  /* class Cone //{ */
  
  class Cone {
  
    public:
      Cone(Eigen::Isometry3d pose, double angle);
      Eigen::Isometry3d pose;
      double angle;
  };
  
  Cone::Cone(Eigen::Isometry3d pose, double angle) : pose(pose), angle(angle) {};
  
  //}

  /* ConePlotter //{ */
  class ConePlotter : public nodelet::Nodelet {

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
    void callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg);
    std::list<Cone> cones;
    int max_cones_;
    double cone_length_;
    
  };
  //}

  /* inInit() //{ */

  void ConePlotter::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ROS_INFO("[ConePlotter]: initializing");

    ros::Time::waitForValid();

    nh_.getParam("max_cones", max_cones_);
    nh_.getParam("cone_length", cone_length_);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_cone = nh_.subscribe("cone_in", 1, &ConePlotter::callbackCone, this, ros::TransportHints().tcpNoDelay());

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

    // | ----------------------- finish init ---------------------- |

    is_initialized = true;

    ROS_INFO("[ConePlotter]: initialized");
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  /* callbackCone() //{ */

  void ConePlotter::callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg) {

    if (!is_initialized)
      return;

    ROS_INFO("[ConePlotter]: got a cone!");

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

    visual_tools_->deleteAllMarkers();

    std::list<Cone>::iterator it;
    for (it = cones.begin(); it != cones.end(); it++) {
      visual_tools_->publishCone(it->pose, it->angle, rviz_visual_tools::colors::TRANSLUCENT_LIGHT, cone_length_);
    }

    visual_tools_->trigger();
  }

  //}

}  // namespace compton_camera_python

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_python::ConePlotter, nodelet::Nodelet)
