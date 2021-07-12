#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>

#include <std_msgs/String.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <asr_halcon_bridge/halcon_pointcloud.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <halconmatcher/halconmatcher.h>

std::unique_ptr<HalconMatcher> matcher;
ros::Publisher pub;
ros::Publisher model_pub;
ros::Publisher poses;
visualization_msgs::Marker marker;
double confidence_threshold = 0.2;
std::string model_name = "serfow_cylinder";
std::string object_key = "";

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

    if(res.score < confidence_threshold){
      ROS_INFO("No objects with high enough confidence.");
      continue;
    }

    object_recognition_msgs::RecognizedObject object;
    object.confidence = res.score;
    object.type.key = object_key;
    
    geometry_msgs::PoseWithCovarianceStamped object_pose;
    
    try{
      object_pose.header.stamp = ros::Time::now();

      object_pose.pose.pose.position.x = res.pose[0];
      object_pose.pose.pose.position.y = res.pose[1];
      object_pose.pose.pose.position.z = res.pose[2];

      tf::Quaternion quat;
      quat.setRPY(angles::from_degrees(res.pose[4]),
                    angles::from_degrees(res.pose[5]),
                    angles::from_degrees(res.pose[6]));

      HalconCpp::HPose pose;
      pose.CreatePose(0,0,0,res.pose[3], res.pose[4], res.pose[5], "Rp+T", "gba", "point");
      HalconCpp::HQuaternion quaternion;
      quaternion.PoseToQuat(pose);
      quaternion = quaternion.QuatNormalize();

      object_pose.pose.pose.orientation.w = (double)quaternion.ConvertToTuple().ToDArr()[0];
      object_pose.pose.pose.orientation.x = (double)quaternion.ConvertToTuple().ToDArr()[1];
      object_pose.pose.pose.orientation.y = (double)quaternion.ConvertToTuple().ToDArr()[2];
      object_pose.pose.pose.orientation.z = (double)quaternion.ConvertToTuple().ToDArr()[3];

      object_pose.header.frame_id = "cameraLeft_optical";

      ROS_INFO("XYZ: %g %g %g", res.pose[0], res.pose[1], res.pose[2]);
      ROS_INFO("YPR: %g %g %g", res.pose[3], res.pose[4], res.pose[5]);

      object.pose = object_pose;
  
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(poseToVector(object_pose.pose.pose));
      transform.setRotation(quat);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cameraLeft_optical", "found_object"));
    }catch(...){
      ROS_WARN("Caught exception.");
      continue;
    }
    
    marker.header.frame_id = "cameraLeft_optical";
    marker.header.stamp = ros::Time::now();
    marker.ns = "recognised_objects";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set colour to yellow
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    // Seems to work for mm models
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    marker.pose.orientation = object_pose.pose.pose.orientation;
    marker.pose.position = object_pose.pose.pose.position;

    marker.text = "Tube";
    pub.publish(object);
    poses.publish(object.pose);
    model_pub.publish(marker);
  }

}

int main(int argc, char* argv[]){

  ros::init(argc, argv, "match_surface");
  ros::NodeHandle nh("~");

  nh.param<double>("confidence_threshold", confidence_threshold, 0.15);
  nh.param<std::string>("object_key", object_key, "");
  
  std::string reference_model_path;
  if(nh.getParam("reference_model", reference_model_path))
  {

    if(reference_model_path == ""){
      ROS_ERROR("Empty reference model");
      exit(1);
    }else{
      std::cout << reference_model_path << std::endl; 
    }

    ROS_INFO("Reference model: %s", reference_model_path.c_str());
    nh.param<std::string>("mesh_resource", marker.mesh_resource, "");


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
    model_pub = nh.advertise<visualization_msgs::Marker> ("model_with_pose", 1);
    poses = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);

    ros::spin();
  }else{
    std::cout << reference_model_path << std::endl;
    ROS_ERROR("No path to reference model.");
    exit(1);
  }

  ROS_INFO("node closed");

  return 0;
}
