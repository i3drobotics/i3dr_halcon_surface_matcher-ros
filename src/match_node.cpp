#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>

#include <std_msgs/String.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <asr_halcon_bridge/halcon_pointcloud.h>
#include <tf/transform_broadcaster.h>

#include <halconmatcher/halconmatcher.h>

std::unique_ptr<HalconMatcher> matcher;
ros::Publisher pub;
ros::Publisher poses;

tf::Vector3 poseToVector(const geometry_msgs::Pose& pose) {
	const float px = static_cast<float>(pose.position.x);
	const float py = static_cast<float>(pose.position.y);
	const float pz = static_cast<float>(pose.position.z);
	return tf::Vector3({px, py, pz});
}


// Convert ROS pose to Eigen::Quaternionf
tf::Quaternion poseToQuaternion(const geometry_msgs::Pose& pose) {
	const float w = static_cast<float>(pose.orientation.w);
	const float x = static_cast<float>(pose.orientation.x);
	const float y = static_cast<float>(pose.orientation.y);
	const float z = static_cast<float>(pose.orientation.z);
	return tf::Quaternion(w, x, y, z);
}

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  using namespace halcon_bridge;

  ROS_INFO("Converting point cloud to Halcon.");

  auto halcon_ptr = toHalconCopy(cloud_msg);
  auto model = halcon_ptr->model;

  matcher->load_scene(*model);
  matcher->match();

  for(auto res : matcher->objects){

    if(res.score < 0.05){
      continue;
    }

    ROS_INFO("XYZ: %g %g %g", res.pose[0], res.pose[1], res.pose[2]);
    ROS_INFO("YPR: %g %g %g", res.pose[3], res.pose[4], res.pose[5]);

    object_recognition_msgs::RecognizedObject object;
    object.confidence = res.score;
    
    geometry_msgs::PoseWithCovarianceStamped object_pose;
    object_pose.header.stamp = ros::Time::now();
    object_pose.pose.pose.position.x = res.pose[0];
    object_pose.pose.pose.position.y = res.pose[1];
    object_pose.pose.pose.position.z = res.pose[2];

    tf::Quaternion quat;
    quat.setEuler(angles::from_degrees(res.pose[3]),
                  angles::from_degrees(res.pose[4]),
                  angles::from_degrees(res.pose[5]));

    object_pose.pose.pose.orientation.x = quat.x();
    object_pose.pose.pose.orientation.y = quat.y();
    object_pose.pose.pose.orientation.z = quat.z();
    object_pose.pose.pose.orientation.w = quat.w();
    object_pose.header.frame_id = "cameraLeft_optical";
    object.pose = object_pose;
  
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(poseToVector(object_pose.pose.pose));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cameraLeft_optical", "found_object"));

    pub.publish(object);
    poses.publish(object.pose);
  }

}

int main(int argc, char* argv[]){

  ros::init(argc, argv, "halcon_matcher");
  ros::NodeHandle nh("~");

  std::string reference_model_path;
  if(nh.getParam("reference_model", reference_model_path))
  {

    if(reference_model_path == ""){
      ROS_ERROR("Empty reference model");
    }else{
      std::cout << reference_model_path << std::endl; 
    }

    ROS_INFO("Reference model: %s", reference_model_path.c_str());


    try{
      matcher = std::unique_ptr<HalconMatcher>(new HalconMatcher);
    }catch(HalconCpp::HOperatorException &e){
      ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      ROS_ERROR("Check your dongle is plugged in!");
      exit(1);
    }

    matcher->init_model(reference_model_path, 0.03, "mm");

    ros::Subscriber sub = nh.subscribe("points2", 1, pointcloud_callback);
    pub = nh.advertise<object_recognition_msgs::RecognizedObject> ("recognized_objects", 1);
    poses = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);

    ros::spin();
  }else{
    std::cout << reference_model_path << std::endl;
    ROS_ERROR("No path to reference model.");
  }

  return 0;
}
