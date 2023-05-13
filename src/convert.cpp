#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

#include "apriltag_location_to_map/GetTargetLocation.h"
#include <queue>

ros::Subscriber sub;
ros::Publisher  pub_target_pose;


class AprilTagLocation{
  public:
    ros::Time old_timestamp = ros::Time(0);
    ros::Time new_timestamp = ros::Time(0);

    tf::StampedTransform transform_apriltag_to_map;

    geometry_msgs::PoseStamped lastKnownPoseStamped;
    geometry_msgs::PoseStamped offsetPoseStamped;
    std::string frame_name;
    bool has_been_observed = false;
    bool is_currently_observed = false;

    std::queue<geometry_msgs::PoseStamped> poseBuffer;

    AprilTagLocation(std::string frame_name, geometry_msgs::PoseStamped offsetPoseStamped) 
    {
      this->offsetPoseStamped = offsetPoseStamped;
      this->frame_name = frame_name;
    }

    void update_offsetPoseStamped(geometry_msgs::PoseStamped offsetPoseStamped)
    {
      this->offsetPoseStamped = offsetPoseStamped;
    }

    geometry_msgs::PoseStamped convert_transform_to_poseStamped(tf::StampedTransform transform)
    {
      geometry_msgs::PoseStamped output_poseStamped;
      
      output_poseStamped.pose.position.x = transform.getOrigin().x();
      output_poseStamped.pose.position.y = transform.getOrigin().y();
      output_poseStamped.pose.position.z = transform.getOrigin().z();
      output_poseStamped.pose.orientation.x = transform.getRotation().getX();
      output_poseStamped.pose.orientation.y = transform.getRotation().getY();
      output_poseStamped.pose.orientation.z = transform.getRotation().getZ();
      output_poseStamped.pose.orientation.w = transform.getRotation().getW();
      output_poseStamped.header.stamp = transform.stamp_;
      output_poseStamped.header.frame_id = transform.frame_id_;

      return output_poseStamped;
    }
    
    geometry_msgs::PoseStamped offset_pose(geometry_msgs::PoseStamped offset, geometry_msgs::PoseStamped output_poseStamped)
    {
      // Offset the pose to determine the target location, because the location of apriltag is not the location where we want the robot to move to
      tf2::Quaternion q_orig, q_rot, q_new;
      tf2::convert(output_poseStamped.pose.orientation, q_orig);
      tf2::convert(offset.pose.orientation, q_rot);
      q_new = q_rot*q_orig;
      q_new.normalize();

      output_poseStamped.pose.position.x = output_poseStamped.pose.position.x + offset.pose.position.x;
      output_poseStamped.pose.position.y = output_poseStamped.pose.position.y + offset.pose.position.y;
      output_poseStamped.pose.position.z = output_poseStamped.pose.position.z + offset.pose.position.z;

      tf2::convert(q_new, output_poseStamped.pose.orientation);

      return output_poseStamped;
    }

    geometry_msgs::PoseStamped transform_to_PoseStampedinTargetFrame(geometry_msgs::PoseStamped poseStamped, tf::StampedTransform transform)
    {
      geometry_msgs::PoseStamped pose_in_target_frame;
      
      tf::Point point_in_source_frame(poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
      tf::Quaternion orientation_in_source_frame(poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w);

      tf::Point point_in_target_frame;
      tf::Quaternion orientation_in_target_frame;
      point_in_target_frame = transform * point_in_source_frame;
      orientation_in_target_frame = transform.getRotation() * orientation_in_source_frame;

      pose_in_target_frame.pose.position.x = point_in_target_frame.getX();
      pose_in_target_frame.pose.position.y = point_in_target_frame.getY();
      pose_in_target_frame.pose.position.z = point_in_target_frame.getZ();
      
      pose_in_target_frame.pose.orientation.x = orientation_in_target_frame.getX();
      pose_in_target_frame.pose.orientation.y = orientation_in_target_frame.getY();
      pose_in_target_frame.pose.orientation.z = orientation_in_target_frame.getZ();
      pose_in_target_frame.pose.orientation.w = orientation_in_target_frame.getW();

      pose_in_target_frame.header.stamp = transform.stamp_;
      pose_in_target_frame.header.frame_id = transform.frame_id_;

      return pose_in_target_frame;
    }

    geometry_msgs::PoseStamped get_poseStamped_from_buffer(geometry_msgs::PoseStamped currentPoseStamped)
    {
      if (this->poseBuffer.size() == 3)
      {
        this->poseBuffer.pop();
      }
      this->poseBuffer.push(currentPoseStamped);
      std::cout<< (this->poseBuffer.size()) << std::endl;
      // Find Average
      geometry_msgs::PoseStamped averagePoseStamped;
      std::queue<geometry_msgs::PoseStamped> poseBuffer_copy = this->poseBuffer;
      averagePoseStamped.header = currentPoseStamped.header;
      while(!poseBuffer_copy.empty())
      {
        geometry_msgs::PoseStamped poseStampedInBuffer = poseBuffer_copy.front();
        
        averagePoseStamped.pose.position.x = averagePoseStamped.pose.position.x + poseStampedInBuffer.pose.position.x;
        averagePoseStamped.pose.position.y = averagePoseStamped.pose.position.y + poseStampedInBuffer.pose.position.y;
        averagePoseStamped.pose.position.z = averagePoseStamped.pose.position.z + poseStampedInBuffer.pose.position.z;

        averagePoseStamped.pose.orientation.x = averagePoseStamped.pose.orientation.x + poseStampedInBuffer.pose.orientation.x;
        averagePoseStamped.pose.orientation.y = averagePoseStamped.pose.orientation.y + poseStampedInBuffer.pose.orientation.y;
        averagePoseStamped.pose.orientation.z = averagePoseStamped.pose.orientation.z + poseStampedInBuffer.pose.orientation.z;
        averagePoseStamped.pose.orientation.w = averagePoseStamped.pose.orientation.w + poseStampedInBuffer.pose.orientation.w;

        poseBuffer_copy.pop();
      }

      averagePoseStamped.pose.position.x = averagePoseStamped.pose.position.x/this->poseBuffer.size();
      averagePoseStamped.pose.position.y = averagePoseStamped.pose.position.y/this->poseBuffer.size();
      averagePoseStamped.pose.position.z = averagePoseStamped.pose.position.z/this->poseBuffer.size();

      averagePoseStamped.pose.orientation.x = averagePoseStamped.pose.orientation.x/this->poseBuffer.size();
      averagePoseStamped.pose.orientation.y = averagePoseStamped.pose.orientation.y/this->poseBuffer.size();
      averagePoseStamped.pose.orientation.z = averagePoseStamped.pose.orientation.z/this->poseBuffer.size();
      averagePoseStamped.pose.orientation.w = averagePoseStamped.pose.orientation.w/this->poseBuffer.size();
      
      return averagePoseStamped;
    }

    void update_pose()
    {      
      geometry_msgs::PoseStamped apriltag_location_mapOrigin;

      this->old_timestamp = this->new_timestamp;

      apriltag_location_mapOrigin = transform_to_PoseStampedinTargetFrame(this->offsetPoseStamped, this->transform_apriltag_to_map);
      
      apriltag_location_mapOrigin.pose = project_to_XY_plane(apriltag_location_mapOrigin.pose);
      
      this->lastKnownPoseStamped = apriltag_location_mapOrigin;//get_poseStamped_from_buffer(apriltag_location_mapOrigin);
    }

    bool newer_observed()
    {
      return (this->new_timestamp != this->old_timestamp);
    }

    void update_timestamp()
    {
      this->new_timestamp = this->transform_apriltag_to_map.stamp_;
      this->has_been_observed = true;
    }

    geometry_msgs::Pose project_to_XY_plane(geometry_msgs::Pose input_pose)
    {
      tf2::Quaternion input_q(input_pose.orientation.x, input_pose.orientation.y , input_pose.orientation.z, input_pose.orientation.w);
      tf2::Matrix3x3 m(input_q);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      tf2::Quaternion output_q;
      output_q.setRPY(0.00, 0.00, yaw);

      input_pose.orientation.x = output_q[0];
      input_pose.orientation.y = output_q[1];
      input_pose.orientation.z = output_q[2];
      input_pose.orientation.w = output_q[3];

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

//with respect to teh apriltag, what is the offset we want to go to.
//+x  -> rightward
//+z -> backward
//+y -> upward
// Table Side
AprilTagLocation tableSide_apriltag("/tableSide", create_poseStamped(0, 0, 0.85, 0, 1.5708, 0));

// Drink Drop Off Point
float drop_off_point_width = 0.05;
float drop_off_point_length = 0.10;
AprilTagLocation drinkFront_apriltag("/tag_20", create_poseStamped(0, 0, 1.1, 0, 1.5708, 0)); // Front
AprilTagLocation drinkLeft_apriltag("/tag_17", create_poseStamped(drop_off_point_length/2 + 1.1, 0, -drop_off_point_width/2, 0, 1.5708 + 1.5708, 0)); // Left
AprilTagLocation drinkBack_apriltag("/tag_18", create_poseStamped(0, 0, drop_off_point_length - 1.1, 0, -1.5708, 0)); // Back
AprilTagLocation drinkRight_apriltag("/tag_19", create_poseStamped(-drop_off_point_length/2 - 1.1, drop_off_point_width/2, 0, 0, 0, 1.507 - 1.507)); // Right

// Trash Drop off point
float trash_drop_off_point_width = 0.05;
float trash_drop_off_point_length = 0.10;
AprilTagLocation trashFront_apriltag("/tag_16", create_poseStamped(0, 0, 1.1, 0, 1.5708, 0)); // Front
AprilTagLocation trashLeft_apriltag("/tag_13", create_poseStamped(-trash_drop_off_point_length/2 - 1.1, 0, trash_drop_off_point_width/2, 0, 0, 1.507 + 1.507)); // Left
AprilTagLocation trashBack_apriltag("/tag_14", create_poseStamped(0, 0, trash_drop_off_point_length + 1.1, 0, 0, 1.507 + 3.14159)); // Back
AprilTagLocation trashRight_apriltag("/tag_15", create_poseStamped(trash_drop_off_point_length/2 + 1.1, 0, trash_drop_off_point_width/2, 0, 0, 1.507 - 1.507)); // Right

// Pot Drop off point
float pot_drop_off_point_width = 0.05;
float pot_drop_off_point_length = 0.10;
AprilTagLocation potFront_apriltag("/tag_16", create_poseStamped(0, 0, -1.1, 0, 0, 1.507)); // Front (supposed to be tag_10)
AprilTagLocation potLeft_apriltag("/tag_13", create_poseStamped(-pot_drop_off_point_width/2 - 1.1, 0, pot_drop_off_point_length/2, 0, 0, 1.507 + 1.507)); // Left (supposed to be tag_11)
AprilTagLocation potBack_apriltag("/tag_14", create_poseStamped(0, 0, pot_drop_off_point_width + 1.1, 0, 0, 1.507 + 3.14159)); // Back (supposed to be tag_9)
AprilTagLocation potRight_apriltag("/tag_15", create_poseStamped(pot_drop_off_point_width/2 + 1.1, 0, pot_drop_off_point_length/2, 0, 0, 1.507 - 1.507)); // Right (supposed to be tag_8)

// Coordinate for where the object should be released relative to the qr code

// Drink Drop Off Arm Point
AprilTagLocation drink_arm_apriltag("/tag_20", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); 

// Trash Drop Off Arm Point
AprilTagLocation trash_arm_apriltag("/tag_16", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); 

// Pot Drop off Arm Point
AprilTagLocation pot_arm_apriltag("/tag_16", create_poseStamped(-0.2, 0, 0, 0, 0, 1.507)); //(supposed to be tag_10)

// Declare transform listener pointer first, because transform listener needs to be initialised after node is initialised
tf::TransformListener* p_listener = NULL;

bool is_apriltag_found(AprilTagLocation* apriltag, tf::TransformListener* listener, std::string reference_frame)
{
  if (listener->frameExists(apriltag->frame_name))
  {
    try{
      listener->waitForTransform("/map", apriltag->frame_name, ros::Time(0), ros::Duration(1));
      listener->lookupTransform("/map", apriltag->frame_name, ros::Time(0), apriltag->transform_apriltag_to_map);
      apriltag->update_timestamp();
    }
    catch(const std::exception& e){
      ROS_INFO("[GetTargetLocation]: %s seen but not currently observed", apriltag->frame_name.c_str());
    }
    
    if (apriltag->newer_observed())
    {
      apriltag->is_currently_observed = true;
      apriltag->update_pose();
      // pub_target_pose_location.publish(apriltag->lastKnownPoseStamped);
      std::cout << "[GetTargetLocation]: Currently Observing " << apriltag->frame_name << " )" << std::endl;
      return true;
    }
    else
    {
      apriltag->is_currently_observed = false;
      return false;
    }
  }  
}

bool return_target_location(apriltag_location_to_map::GetTargetLocation::Request  &req,
                            apriltag_location_to_map::GetTargetLocation::Response &res)
{
  ROS_INFO("[GetTargetLocation]: %s requested", req.requested_target.c_str());

  if (req.clear_target)
  {
    if (req.requested_target == "pickup")
    {
      tableSide_apriltag.has_been_observed = false;
    }
    else if (req.requested_target == "drink_dropoff")
    {
      drinkFront_apriltag.has_been_observed = false;
      drinkBack_apriltag.has_been_observed = false;
      drinkLeft_apriltag.has_been_observed = false;
      drinkRight_apriltag.has_been_observed = false;
    }
    else if (req.requested_target == "pot_dropoff")
    {
      potFront_apriltag.has_been_observed = false;
      potBack_apriltag.has_been_observed = false;
      potLeft_apriltag.has_been_observed = false;
      potRight_apriltag.has_been_observed = false;
    }
    else if (req.requested_target == "trash_dropoff")
    {
      trashFront_apriltag.has_been_observed = false;
      trashBack_apriltag.has_been_observed = false;
      trashLeft_apriltag.has_been_observed = false;
      trashRight_apriltag.has_been_observed = false;
    }
  }

  if (req.requested_target == "pickup")
  {
    res.target_pose = tableSide_apriltag.lastKnownPoseStamped;

    if (tableSide_apriltag.has_been_observed)
    {
      res.has_been_observed = true;
      res.is_currently_observed = tableSide_apriltag.is_currently_observed;
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
      res.is_currently_observed = drinkFront_apriltag.is_currently_observed;
    }
    else if (drinkLeft_apriltag.has_been_observed)
    {
      res.target_pose = drinkLeft_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
      res.is_currently_observed = drinkLeft_apriltag.is_currently_observed;
    }
    else if (drinkRight_apriltag.has_been_observed)
    {
      res.target_pose = drinkRight_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true;
      res.is_currently_observed = drinkRight_apriltag.is_currently_observed;
    }
    else if (drinkBack_apriltag.has_been_observed)
    {
      res.target_pose = drinkBack_apriltag.lastKnownPoseStamped;
      res.has_been_observed = true; 
      res.is_currently_observed = drinkBack_apriltag.is_currently_observed;
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
      res.is_currently_observed = potFront_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (potLeft_apriltag.has_been_observed)
    {
      res.target_pose = potLeft_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potLeft_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (potRight_apriltag.has_been_observed)
    {
      res.target_pose = potRight_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potRight_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (potBack_apriltag.has_been_observed)
    {
      res.target_pose = potBack_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = potBack_apriltag.is_currently_observed;   
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
      res.is_currently_observed = trashFront_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (trashLeft_apriltag.has_been_observed)
    {
      res.target_pose = trashLeft_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashLeft_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (trashRight_apriltag.has_been_observed)
    {
      res.target_pose = trashRight_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashRight_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else if (trashBack_apriltag.has_been_observed)
    {
      res.target_pose = trashBack_apriltag.lastKnownPoseStamped;
      res.is_currently_observed = trashBack_apriltag.is_currently_observed;   
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
      res.is_currently_observed = trash_arm_apriltag.is_currently_observed;   
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
      res.is_currently_observed = drink_arm_apriltag.is_currently_observed;   
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
      res.is_currently_observed = pot_arm_apriltag.is_currently_observed;   
      res.has_been_observed = true;
    }
    else
    {
      res.target_pose = pot_arm_apriltag.lastKnownPoseStamped;
      res.has_been_observed = false;
      res.is_currently_observed = false;
    }
  }

  ROS_INFO("[GetTargetLocation]: %s | Has been observed: %d | Is currently observed: %d", 
                                                          req.requested_target.c_str(), 
                                                          (int) res.has_been_observed,
                                                          (int) res.is_currently_observed);

  // For debug
  pub_target_pose.publish(res.target_pose);

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

	// pub_target_pose.publish(final_pose);
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "convert");

  ros::NodeHandle node;
  ROS_INFO("[GetTargetLocation]: Service Started");

  p_listener = new(tf::TransformListener);

  ros::Rate rate(10.0);

  // Sleep to wait for the received transforms to be queued
  ros::Duration(1.0).sleep();
  
  ros::ServiceServer service = node.advertiseService("GetTargetLocation", return_target_location);
  pub_target_pose = node.advertise<geometry_msgs::PoseStamped>("/target_location_pose", 1);

  // We subscribe to camera purely as a subscriber to register callback and update the registration of apriltags
  // ros::Subscriber sub = node.subscribe("/frontCamera/color/image_rect_color/", 1, find_apriltags);

  ros::Subscriber sub = node.subscribe("/tag_detections_image", 1, find_apriltags);

  ros::spin();

  return 0;
};