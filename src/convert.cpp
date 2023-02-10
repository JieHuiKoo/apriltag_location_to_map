#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#include "apriltag_location_to_map/GetTargetLocation.h"

ros::Subscriber sub;
ros::Publisher  pub_front;
ros::Publisher  pub_right;
ros::Publisher  pub_left;
ros::Publisher  pub_back;

class AprilTagLocation{
  public:
    ros::Time old_timestamp = ros::Time(0);
    ros::Time new_timestamp = ros::Time(0);

    tf::StampedTransform transform;
    geometry_msgs::PoseStamped lastKnownPoseStamped;
    geometry_msgs::PoseStamped offsetPoseStamped;
    std::string frame_name;
    bool has_been_observed = false;

    AprilTagLocation(std::string frame_name, geometry_msgs::PoseStamped offsetPoseStamped) 
    {
      this->offsetPoseStamped = offsetPoseStamped;
      this->frame_name = frame_name;
    }

    void update_pose()
    {
      this->old_timestamp = this->new_timestamp;

      this->lastKnownPoseStamped.pose.position.x = this->transform.getOrigin().x();
      this->lastKnownPoseStamped.pose.position.y = this->transform.getOrigin().y();
      this->lastKnownPoseStamped.pose.position.z = this->transform.getOrigin().z();
      this->lastKnownPoseStamped.pose.orientation.x = this->transform.getRotation().getX();
      this->lastKnownPoseStamped.pose.orientation.y = this->transform.getRotation().getY();
      this->lastKnownPoseStamped.pose.orientation.z = this->transform.getRotation().getZ();
      this->lastKnownPoseStamped.pose.orientation.w = this->transform.getRotation().getW();
      this->lastKnownPoseStamped.header.stamp = this->transform.stamp_;
      this->lastKnownPoseStamped.header.frame_id = this->transform.frame_id_;

      // Offset the pose to determine the target location, because the location of apriltag is not the location where we want the robot to move to
      tf2::Quaternion q_orig, q_rot, q_new;
      tf2::convert(this->lastKnownPoseStamped.pose.orientation, q_orig);
      tf2::convert(this->offsetPoseStamped.pose.orientation, q_rot);
      q_new = q_rot*q_orig;
      q_new.normalize();

      this->lastKnownPoseStamped.pose.position.x = this->lastKnownPoseStamped.pose.position.x + this->offsetPoseStamped.pose.position.x;
      this->lastKnownPoseStamped.pose.position.y = this->lastKnownPoseStamped.pose.position.y + this->offsetPoseStamped.pose.position.y;
      this->lastKnownPoseStamped.pose.position.z = this->lastKnownPoseStamped.pose.position.z + this->offsetPoseStamped.pose.position.z;

      tf2::convert(q_new, this->lastKnownPoseStamped.pose.orientation);
    }

    bool is_currently_observed()
    {
      return (this->new_timestamp != this->old_timestamp);
    }

    void update_timestamp()
    {
      this->new_timestamp = this->transform.stamp_;
      this->has_been_observed = true;
    }
};

geometry_msgs::PoseStamped create_poseStamped(double x, double y, double z, double rad_x, double rad_y, double rad_z)
{
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose.position.x = x;
  poseStamped.pose.position.y = y;
  poseStamped.pose.position.z = z;

  // This determines the orientation of the target location with respect to the apriltag
  tf2::Quaternion q_rot;
  q_rot.setRPY(rad_x, rad_y, rad_z);

  poseStamped.pose.orientation.x = q_rot[0];
  poseStamped.pose.orientation.y = q_rot[1];
  poseStamped.pose.orientation.z = q_rot[2];
  poseStamped.pose.orientation.w = q_rot[3];

  return poseStamped;
}

// Table Side
AprilTagLocation tableSide_apriltag("/tableSide", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507));

// Drink Drop Off Point
float drop_off_point_width = 0.05;
float drop_off_point_length = 0.10;
AprilTagLocation drinkFront_apriltag("/tag_20", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
AprilTagLocation drinkLeft_apriltag("/tag_17", create_poseStamped(drop_off_point_length/2, -drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 + 1.507)); // Left
AprilTagLocation drinkBack_apriltag("/tag_18", create_poseStamped(drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
AprilTagLocation drinkRight_apriltag("/tag_19", create_poseStamped(drop_off_point_length/2, drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 - 1.507)); // Right

// Trash Drop off point
float trash_drop_off_point_width = 0.05;
float trash_drop_off_point_length = 0.10;
AprilTagLocation trashFront_apriltag("/tag_16", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
AprilTagLocation trashLeft_apriltag("/tag_13", create_poseStamped(trash_drop_off_point_length/2, -trash_drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 + 1.507)); // Left
AprilTagLocation trashBack_apriltag("/tag_14", create_poseStamped(trash_drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
AprilTagLocation trashRight_apriltag("/tag_15", create_poseStamped(trash_drop_off_point_length/2, trash_drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 - 1.507)); // Right

// Pot Drop off point
float pot_drop_off_point_width = 0.05;
float pot_drop_off_point_length = 0.10;
AprilTagLocation potFront_apriltag("/tag_10", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
AprilTagLocation potLeft_apriltag("/tag_11", create_poseStamped(pot_drop_off_point_length/2, -pot_drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 + 1.507)); // Left
AprilTagLocation potBack_apriltag("/tag_9", create_poseStamped(pot_drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
AprilTagLocation potRight_apriltag("/tag_8", create_poseStamped(pot_drop_off_point_length/2, pot_drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 - 1.507)); // Right

// Declare transform listener pointer first, because transform listener needs to be initialised after node is initialised
tf::TransformListener* p_listener = NULL;

bool is_apriltag_found(AprilTagLocation* apriltag, tf::TransformListener* listener)
{
  if (listener->frameExists(apriltag->frame_name))
    {
      try{
        listener->waitForTransform("/map", apriltag->frame_name, ros::Time(0), ros::Duration(1));
        listener->lookupTransform("/map", apriltag->frame_name, ros::Time(0), apriltag->transform);
      }
      catch(const std::exception& e){
        ROS_INFO("%s seen but not currently observed", apriltag->frame_name.c_str());
      }
      apriltag->update_timestamp();
      
      if (apriltag->is_currently_observed())
      {
        apriltag->update_pose();
        // pub_target_pose_location.publish(apriltag->lastKnownPoseStamped);
        std::cout << "(Found " << apriltag->frame_name << " )" << std::endl;
        return true;
      }
      else
      {
        return false;
      }
    }  
}

bool return_target_location(apriltag_location_to_map::GetTargetLocation::Request  &req,
                            apriltag_location_to_map::GetTargetLocation::Response &res)
{
  if (req.requested_target == "pickup")
  {
    res.target_pose = tableSide_apriltag.lastKnownPoseStamped;

    if (tableSide_apriltag.has_been_observed)
    {
      res.has_been_observed = true;
    }
    else
    {
      res.has_been_observed = false;
    }
  }
  else if (req.requested_target == "drink_dropoff")
  {
    if (drinkFront_apriltag.has_been_observed)
    {
      res.target_pose = drinkFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (drinkLeft_apriltag.has_been_observed)
    {
      res.target_pose = drinkLeft_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (drinkRight_apriltag.has_been_observed)
    {
      res.target_pose = drinkRight_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (drinkBack_apriltag.has_been_observed)
    {
      res.target_pose = drinkBack_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;      
    }
    else
    {
      res.target_pose = drinkFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;    
    }
  }
  else if (req.requested_target == "pot_dropoff")
  {
    if (potFront_apriltag.has_been_observed)
    {
      res.target_pose = potFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (potLeft_apriltag.has_been_observed)
    {
      res.target_pose = potLeft_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (potRight_apriltag.has_been_observed)
    {
      res.target_pose = potRight_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (potBack_apriltag.has_been_observed)
    {
      res.target_pose = potBack_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;      
    }
    else
    {
      res.target_pose = potFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;    
    }
  }
  else if (req.requested_target == "trash_dropoff")
  {
    if (trashFront_apriltag.has_been_observed)
    {
      res.target_pose = trashFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (trashLeft_apriltag.has_been_observed)
    {
      res.target_pose = trashLeft_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (trashRight_apriltag.has_been_observed)
    {
      res.target_pose = trashRight_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
    }
    else if (trashBack_apriltag.has_been_observed)
    {
      res.target_pose = trashBack_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;      
    }
    else
    {
      res.target_pose = trashFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;    
    }
  }

  return true;
}

void find_apriltags(const nav_msgs::Odometry::ConstPtr& msg)
{
  is_apriltag_found(&tableSide_apriltag, p_listener);

  is_apriltag_found(&drinkFront_apriltag, p_listener);
  is_apriltag_found(&drinkLeft_apriltag, p_listener);
  is_apriltag_found(&drinkBack_apriltag, p_listener);
  is_apriltag_found(&drinkRight_apriltag, p_listener);

  is_apriltag_found(&trashFront_apriltag, p_listener);
  is_apriltag_found(&trashLeft_apriltag, p_listener);
  is_apriltag_found(&trashBack_apriltag, p_listener);
  is_apriltag_found(&trashRight_apriltag, p_listener);

  is_apriltag_found(&potFront_apriltag, p_listener);
  is_apriltag_found(&potLeft_apriltag, p_listener);
  is_apriltag_found(&potBack_apriltag, p_listener);
  is_apriltag_found(&potRight_apriltag, p_listener);

  // pub_front.publish(tableSide_apriltag.lastKnownPoseStamped.pose);
  pub_front.publish(drinkFront_apriltag.lastKnownPoseStamped);
  pub_right.publish(drinkRight_apriltag.lastKnownPoseStamped);
  pub_left.publish(drinkLeft_apriltag.lastKnownPoseStamped);
  pub_back.publish(drinkBack_apriltag.lastKnownPoseStamped);
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "convert");

  ros::NodeHandle node;

  p_listener = new(tf::TransformListener);

  ros::Rate rate(10.0);

  // Sleep to wait for the received transforms to be queued
  ros::Duration(1.0).sleep();
  
  ros::ServiceServer service = node.advertiseService("GetTargetLocation", return_target_location);
  pub_front = node.advertise<geometry_msgs::PoseStamped>("/Front", 1);
  pub_right = node.advertise<geometry_msgs::PoseStamped>("/Right", 1);
  pub_left = node.advertise<geometry_msgs::PoseStamped>("/Left", 1);
  pub_back = node.advertise<geometry_msgs::PoseStamped>("/Back", 1);
  
  // We use odometry purely as a subscriber to register callback and update the registration of apriltags
  ros::Subscriber sub = node.subscribe("/Odometry", 1, find_apriltags);

  ros::spin();

  return 0;
};