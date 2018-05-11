#include<halconmatcher/halconmatcher.h>
#include <ros/ros.h>
#include <ros/console.h>

using namespace HalconCpp;

HalconMatcher::HalconMatcher(void){
  try{
    HTuple test;
  }catch(HOperatorException &e){
    ROS_ERROR("Error: %s", e.ErrorMessage().Text());
  }
}

void HalconMatcher::load_scene(HObjectModel3D scene){
  this->scene = scene;
}

void HalconMatcher::load_scene(std::string filename, std::string units){
  try{
    scene.ReadObjectModel3d(filename.c_str(), units.c_str(), HTuple(), HTuple());
  }catch(HOperatorException &e){
      std::cout << "Error reading reference model" << std::endl;
      ROS_ERROR("Error: %s", e.ErrorMessage().Text());
  }
}

void HalconMatcher::init_model(std::string filename, float sampling_distance, std::string units){

     HTuple hv_Status;

     ROS_INFO("Opening %s", filename.c_str());

    try{
       reference_object.ReadObjectModel3d(filename.c_str(), units.c_str(), HTuple(), HTuple());
       reference_object.WriteObjectModel3d("ply", "reference.ply", HTuple(), HTuple());
    }catch(HOperatorException &e){
      std::cout << "Error reading reference model " << filename << std::endl;
       ROS_ERROR("Error: %s", e.ErrorMessage().Text());
    }

    try{
       reference_surface = reference_object.CreateSurfaceModel(sampling_distance, "pose_ref_rel_sampling_distance", 0.01);
    }catch(HOperatorException &e){
       std::cout << "Error making surface model" << std::endl;
       ROS_ERROR("Error: %s", e.ErrorMessage().Text());
    }

}

void HalconMatcher::match_vanilla(){
    HTuple  hv_TISMount, hv_Status, hv_TISMountSM;
    HTuple  hv_ObjectModel3D, hv_ObjectModel3DOut, hv_ObjectModel3DThresholded;
    HTuple  hv_Pose, hv_Score, hv_plane_distances, hv_ObjectModel3DConnected;
    HTuple  hv_ObjectModel3DSelected, hv_ObjectModel3D_, hv_TISMountTrans;

    hv_ObjectModel3D = scene;

    try{
      ROS_INFO("Calculate plane fit");
      FitPrimitivesObjectModel3d(hv_ObjectModel3D, "primitive_type", "plane", &hv_ObjectModel3DOut);
    }catch(HOperatorException &e){
      ROS_ERROR("Plane fit: %s", e.ErrorMessage().Text());
      return;
    }

     try{
      ROS_INFO("Calculate plane distances");
    DistanceObjectModel3d(hv_ObjectModel3D, hv_ObjectModel3DOut, HTuple(), 0.05, HTuple(), 
        HTuple());
    }catch(HOperatorException &e){
          ROS_ERROR("Plane distances: %s", e.ErrorMessage().Text());
          return;
        }

    try{
      ROS_INFO("Thresholding by distance");
      SelectPointsObjectModel3d(hv_ObjectModel3D, "&distance", 0.004, 0.010, &hv_ObjectModel3DThresholded);
    }catch(HOperatorException &e){
      ROS_ERROR("Distance threshold: %s", e.ErrorMessage().Text());
      ROS_ERROR("Possibly no plane was detected in the point cloud");
      return;
    }
    
    try{
      ROS_INFO("Finding connected components");
      ConnectionObjectModel3d(hv_ObjectModel3DThresholded, "distance_3d", 0.01, &hv_ObjectModel3DConnected);
    }catch(HOperatorException &e){
      ROS_ERROR("Connected components error: %s", e.ErrorMessage().Text());
      return;
    }

    ROS_INFO("Found %ld candidate objects", hv_ObjectModel3DConnected.Length());

    SelectObjectModel3d(hv_ObjectModel3DConnected, "num_points", "and", 1000, 10000, 
        &hv_ObjectModel3DSelected);

    FindSurfaceModel(reference_surface, hv_ObjectModel3DSelected, 0.1, 0.8, 0, "true", 
        HTuple(), HTuple(), &hv_Pose, &hv_Score, &hv_ObjectModel3D_);

}

void HalconMatcher::match(){

  objects.clear();

  // Really figure out why this doesn't work in memory...
  scene.WriteObjectModel3d("ply", "/home/htp/output.ply", HTuple(), HTuple());
  load_scene("/home/htp/output.ply", "m");

  try{
    scene = scene.SelectPointsObjectModel3d("point_coord_z", 0.2, 1.5);
  }catch(HOperatorException &e){
    ROS_ERROR("Initial Z threshold: %s", e.ErrorMessage().Text());
    return;
  }

  HObjectModel3D plane_fit;
  try{
    ROS_INFO("Calculate plane fit");
    plane_fit = scene.FitPrimitivesObjectModel3d("primitive_type", "plane");
  }catch(HOperatorException &e){
    ROS_ERROR("Plane fit: %s", e.ErrorMessage().Text());
    return;
  }

  try{
    ROS_INFO("Calculate plane distances");
    double max_plane_distance = 0.05;
    scene.DistanceObjectModel3d(plane_fit, HPose(), max_plane_distance, "distance_to", "primitive");
  }catch(HOperatorException &e){
    ROS_ERROR("Plane distances: %s", e.ErrorMessage().Text());
    return;
  }

  HObjectModel3D thresholded;
  try{
    ROS_INFO("Thresholding by distance");
    thresholded = scene.SelectPointsObjectModel3d("&distance", 0.004, 0.010);
  }catch(HOperatorException &e){
    ROS_ERROR("Distance threshold: %s", e.ErrorMessage().Text());
    ROS_ERROR("Possibly no plane was detected in the point cloud");
    return;
  }

  HObjectModel3DArray connected;
  try{
    ROS_INFO("Finding connected components");
    connected = thresholded.ConnectionObjectModel3d("distance_3d", 0.01);
  }catch(HOperatorException &e){
    ROS_ERROR("Connected components error: %s", e.ErrorMessage().Text());
    return;
  }

  ROS_INFO("Found %ld candidate objects", connected.Length());

  HObjectModel3DArray hv_ObjectModel3DSelected;
  try{
    ROS_INFO("Filtering  objects");
    hv_ObjectModel3DSelected = scene.SelectObjectModel3d(connected, "num_points", "and", 1000, 10000);
  }catch(HOperatorException &e){
    ROS_ERROR("Selecting objects: %s", e.ErrorMessage().Text());
    return;
  }

  ROS_DEBUG("Finding surface model");
  double rel_sample_dist = 0.1;
  double keypoint_fraction = 0.8;
  double min_score = 0;

  if(hv_ObjectModel3DSelected.Length() == 0){
    ROS_WARN("No objects segmented");
  }

  for(int i = 0; i < hv_ObjectModel3DSelected.Length(); i++){

      found_object new_object;

      HObjectModel3D segmented = hv_ObjectModel3DSelected.Tools()[i];

      new_object.points = segmented;
      
      HTuple match_score, hv_ObjectModel3D_, found_pose;
      
      try{
        FindSurfaceModel(reference_surface, segmented, rel_sample_dist, keypoint_fraction, min_score, "true", HTuple(), HTuple(), &found_pose, &match_score, &hv_ObjectModel3D_);
        ROS_INFO("Score: %f", (float) match_score);
        new_object.score = match_score;
      }catch(HOperatorException &e){
        std::cout << e.ErrorMessage().Text() << std::endl;
        ROS_ERROR("Fit: %s", e.ErrorMessage().Text());
        return;
      }catch(HTupleAccessException &e){
        std::cout << e.ErrorMessage().Text() << std::endl;
      }

      for(int i=0; i < 6; i++){
        new_object.pose.push_back((double) found_pose[i]);
      }

      objects.push_back(new_object);
  }

  return;
}