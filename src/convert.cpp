#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

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

      this->lastKnownPoseStamped.pose = project_to_XY_plane(this->lastKnownPoseStamped.pose);
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

    geometry_msgs::Pose project_to_XY_plane(geometry_msgs::Pose input_pose)
    {
      std::cout<<"original q"<<input_pose.orientation<<std::endl;
      tf2::Quaternion input_q(input_pose.orientation.x, input_pose.orientation.y , input_pose.orientation.z, input_pose.orientation.w);
      tf2::Matrix3x3 m(input_q);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      std::cout<<"original rpy|"<<roll<<"|"<<pitch<<"|"<<yaw<<std::endl;
      tf2::Quaternion output_q;
      output_q.setRPY(0.00, 0.00, yaw);

      input_pose.orientation.x = output_q[0];
      input_pose.orientation.y = output_q[1];
      input_pose.orientation.z = output_q[2];
      input_pose.orientation.w = output_q[3];
      std::cout<<input_pose.orientation<<std::endl;

      return input_pose;
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

// Drink Drop Off Arm Point
AprilTagLocation drink_arm_apriltag("/tag_20", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); 

// Trash Drop Off Arm Point
AprilTagLocation trash_arm_apriltag("/tag_16", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); 

// Pot Drop off Arm Point
AprilTagLocation pot_arm_apriltag("/tag_10", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507));

// Declare transform listener pointer first, because transform listener needs to be initialised after node is initialised
tf::TransformListener* p_listener = NULL;

bool is_apriltag_found(AprilTagLocation* apriltag, tf::TransformListener* listener, std::string reference_frame)
{
  if (listener->frameExists(apriltag->frame_name))
    {
      try{
        listener->waitForTransform(reference_frame, apriltag->frame_name, ros::Time(0), ros::Duration(1));
        listener->lookupTransform(reference_frame, apriltag->frame_name, ros::Time(0), apriltag->transform);
      }
      catch(const std::exception& e){
        ROS_INFO("[GetTargetLocation]: %s seen but not currently observed", apriltag->frame_name.c_str());
      }
      apriltag->update_timestamp();
      
      if (apriltag->is_currently_observed())
      {
        apriltag->update_pose();
        // pub_target_pose_location.publish(apriltag->lastKnownPoseStamped);
        std::cout << "[GetTargetLocation]: (Found " << apriltag->frame_name << " )" << std::endl;
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
  ROS_INFO("[GetTargetLocation]: %s requested", req.requested_target.c_str());

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
      res.is_currently_observed = potFront_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (potLeft_apriltag.has_been_observed)
    {
      res.target_pose = potLeft_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potLeft_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (potRight_apriltag.has_been_observed)
    {
      res.target_pose = potRight_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potRight_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (potBack_apriltag.has_been_observed)
    {
      res.target_pose = potBack_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potBack_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else
    {
      res.target_pose = potFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;   
    }
  }
  else if (req.requested_target == "trash_dropoff")
  {
    if (trashFront_apriltag.has_been_observed)
    {
      res.target_pose = trashFront_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashFront_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (trashLeft_apriltag.has_been_observed)
    {
      res.target_pose = trashLeft_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashLeft_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (trashRight_apriltag.has_been_observed)
    {
      res.target_pose = trashRight_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashRight_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else if (trashBack_apriltag.has_been_observed)
    {
      res.target_pose = trashBack_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashBack_apriltag.is_currently_observed();   
      res.has_been_observed = true;      
    }
    else
    {
      res.target_pose = trashFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;
    }
  }
  else if (req.requested_target == "trash_arm_dropoff")
  {
    if (trash_arm_apriltag.has_been_observed)
    {
      res.target_pose = trash_arm_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trash_arm_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else
    {
      res.target_pose = trashFront_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;
    }
  }
    else if (req.requested_target == "drink_arm_dropoff")
  {
    if (drink_arm_apriltag.has_been_observed)
    {
      res.target_pose = drink_arm_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = drink_arm_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else
    {
      res.target_pose = drink_arm_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;
    }
  }
    else if (req.requested_target == "pot_arm_dropoff")
  {
    if (pot_arm_apriltag.has_been_observed)
    {
      res.target_pose = pot_arm_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = pot_arm_apriltag.is_currently_observed();   
      res.has_been_observed = true;
    }
    else
    {
      res.target_pose = pot_arm_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;
    }
  }

  ROS_INFO("[GetTargetLocation]: %s | has been observed: %d | is_currently_observed: %d", 
                                                          req.requested_target.c_str(), 
                                                          (int) res.has_been_observed, 
                                                          (int) res.is_currently_observed);

  return true;
}


void find_apriltags(const sensor_msgs::ImageConstPtr& msg)
{
  is_apriltag_found(&tableSide_apriltag, p_listener, "/map");

  is_apriltag_found(&drinkFront_apriltag, p_listener, "/map");
  is_apriltag_found(&drinkLeft_apriltag, p_listener, "/map");
  is_apriltag_found(&drinkBack_apriltag, p_listener, "/map");
  is_apriltag_found(&drinkRight_apriltag, p_listener, "/map");

  is_apriltag_found(&trashFront_apriltag, p_listener, "/map");
  is_apriltag_found(&trashLeft_apriltag, p_listener, "/map");
  is_apriltag_found(&trashBack_apriltag, p_listener, "/map");
  is_apriltag_found(&trashRight_apriltag, p_listener, "/map");

  is_apriltag_found(&potFront_apriltag, p_listener, "/map");
  is_apriltag_found(&potLeft_apriltag, p_listener, "/map");
  is_apriltag_found(&potBack_apriltag, p_listener, "/map");
  is_apriltag_found(&potRight_apriltag, p_listener, "/map");

  is_apriltag_found(&pot_arm_apriltag, p_listener, "/map");
  is_apriltag_found(&trash_arm_apriltag, p_listener, "/map");
  is_apriltag_found(&drink_arm_apriltag, p_listener, "/map");

  pub_front.publish(tableSide_apriltag.lastKnownPoseStamped);
  // For Debug
  // pub_front.publish(drinkFront_apriltag.lastKnownPoseStamped);
  // pub_right.publish(drinkRight_apriltag.lastKnownPoseStamped);
  // pub_left.publish(drinkLeft_apriltag.lastKnownPoseStamped);
  // pub_back.publish(drinkBack_apriltag.lastKnownPoseStamped);
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "convert");

  ros::NodeHandle node;
  ROS_INFO("|GetTargetLocation| Service Started");

  p_listener = new(tf::TransformListener);

  ros::Rate rate(10.0);

  // Sleep to wait for the received transforms to be queued
  ros::Duration(1.0).sleep();
  
  ros::ServiceServer service = node.advertiseService("GetTargetLocation", return_target_location);
  pub_front = node.advertise<geometry_msgs::PoseStamped>("/Front", 1);
  pub_right = node.advertise<geometry_msgs::PoseStamped>("/Right", 1);
  pub_left = node.advertise<geometry_msgs::PoseStamped>("/Left", 1);
  pub_back = node.advertise<geometry_msgs::PoseStamped>("/Back", 1);

  // We subscribe to camera purely as a subscriber to register callback and update the registration of apriltags
  // ros::Subscriber sub = node.subscribe("/frontCamera/color/image_rect_color/", 1, find_apriltags);

  ros::Subscriber sub = node.subscribe("/tag_detections_image", 1, find_apriltags);

  ros::spin();

  return 0;
};