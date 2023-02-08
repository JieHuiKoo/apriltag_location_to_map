#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Subscriber sub;
ros::Publisher pub_target_pose_location;
ros::Publisher pub_target_pose_type;

class AprilTagLocation{
  public:
    ros::Time old_timestamp = ros::Time(0);
    ros::Time new_timestamp = ros::Time(0);

    tf::StampedTransform transform;
    geometry_msgs::PoseStamped lastKnownPoseStamped;
    geometry_msgs::PoseStamped offsetPoseStamped;
    std::string frame_name;

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

    bool is_valid()
    {
      return (this->new_timestamp != this->old_timestamp);
    }

    void update_timestamp()
    {
      this->new_timestamp = this->transform.stamp_;
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

bool is_apriltag_found(AprilTagLocation *apriltag, tf::TransformListener *listener)
{
  if (listener->frameExists(apriltag->frame_name))
    {
      listener->lookupTransform("/map", apriltag->frame_name, ros::Time(0), apriltag->transform);
      apriltag->update_timestamp();
      
      if (apriltag->is_valid())
      {
        apriltag->update_pose();
        pub_target_pose_location.publish(apriltag->lastKnownPoseStamped);
        std::cout << "(Found " << apriltag->frame_name << " )" << std::endl;
        return true;
      }
      else
      {
        std::cout << "(Not Found!)" << std::endl;
        return false;
      }
    }  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "convert");

  ros::NodeHandle node;

  tf::TransformListener listener;

  pub_target_pose_location = node.advertise<geometry_msgs::PoseStamped>("/aprilTag_locations", 1);

  ros::Rate rate(10.0);

  // Table Side
  AprilTagLocation tableSide_apriltag("/tableSide", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507));

  // Drink Drop Off Point
  float drop_off_point_width = 0.05;
  float drop_off_point_length = 0.10;
  AprilTagLocation drinkFront_apriltag("/tag_20", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
  AprilTagLocation drinkLeft_apriltag("/tag_17", create_poseStamped(drop_off_point_length/2, -drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 - 1.507)); // Left
  AprilTagLocation drinkBack_apriltag("/tag_18", create_poseStamped(drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
  AprilTagLocation drinkRight_apriltag("/tag_19", create_poseStamped(drop_off_point_length/2, drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 + 1.507)); // Right

  // Trash Drop off point
  AprilTagLocation trashFront_apriltag("/tag_16", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
  AprilTagLocation trashLeft_apriltag("/tag_13", create_poseStamped(drop_off_point_length/2, -drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 - 1.507)); // Left
  AprilTagLocation trashBack_apriltag("/tag_14", create_poseStamped(drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
  AprilTagLocation trashRight_apriltag("/tag_15", create_poseStamped(drop_off_point_length/2, drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 + 1.507)); // Right

  // Pot Drop off point
  drop_off_point_width = 0.05;
  drop_off_point_length = 0.10;
  AprilTagLocation potFront_apriltag("/tag_10", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); // Front
  AprilTagLocation potLeft_apriltag("/tag_11", create_poseStamped(drop_off_point_length/2, -drop_off_point_width/2 - 0.2, 0, 0, 0, 1.507 - 1.507)); // Left
  AprilTagLocation potBack_apriltag("/tag_9", create_poseStamped(drop_off_point_width + 0.2, 0, 0, 0, 0, 1.507 + 3.14159)); // Back
  AprilTagLocation potRight_apriltag("/tag_8", create_poseStamped(drop_off_point_length/2, drop_off_point_width/2 + 0.2, 0, 0, 0, 1.507 + 1.507)); // Right

  // Sleep to wait for the received transforms to be queued
  ros::Duration(1.0).sleep();



  while (node.ok()){
    
    
    is_apriltag_found(&tableSide_apriltag, &listener);
    is_apriltag_found(&drinkFront_apriltag, &listener);
    is_apriltag_found(&drinkLeft_apriltag, &listener);
    is_apriltag_found(&drinkBack_apriltag, &listener);
    is_apriltag_found(&drinkRight_apriltag, &listener);
    rate.sleep();
  }
  return 0;
};