#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <asr_halcon_bridge/halcon_pointcloud.h>

#include <halconmatcher/halconmatcher.h>

HalconMatcher matcher;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
      using namespace halcon_bridge;
      ROS_INFO("Got point cloud callback");

      ROS_DEBUG("Converting point cloud to Halcon.");
      auto halcon_ptr = toHalconCopy(cloud_msg);
      auto model = halcon_ptr->model;

      matcher.load_scene(*model);
      matcher.match();
}

int main(int argc, char* argv[]){

  ros::init(argc, argv, "halcon_pc_match_server");
  ros::NodeHandle nh;

  std::string reference_model_path;
  if (nh.getParam("reference_model", reference_model_path))
  {
    matcher.init_model(reference_model_path);

    ros::Subscriber sub = nh.subscribe("points", 1, pointcloud_callback);
    ros::Publisher pub = nh.advertise<object_recognition_msgs::RecognizedObject> ("recognized_objects", 1);

    ros::spin();
  }else{
    ROS_ERROR("No path to reference model.");
  }

  return 0;
}
