#include<halconmatcher/halconmatcher.h>

using namespace HalconCpp;

void HalconMatcher::load_scene(HObjectModel3D scene){
  this->scene = scene;
}

void HalconMatcher::load_scene(std::string filename, std::string units){
  try{
    scene.ReadObjectModel3d(filename.c_str(), units.c_str(), HTuple(), HTuple());
  }catch(HOperatorException &e){
      std::cout << "Error reading reference model" << std::endl;
       //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
  }
}

void HalconMatcher::init_model(std::string filename, float sampling_distance, std::string units){

     HTuple hv_Status;

     //ROS_INFO("Opening %s", filename.c_str());

    try{
       reference_object.ReadObjectModel3d(filename.c_str(), units.c_str(), HTuple(), HTuple());
    }catch(HOperatorException &e){
      std::cout << "Error reading reference model " << filename << std::endl;
       //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
    }

    try{
       reference_surface = reference_object.CreateSurfaceModel(sampling_distance, "pose_ref_rel_sampling_distance", 0.01);
    }catch(HOperatorException &e){
        std::cout << "Error making surface model" << std::endl;
       //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
    }

    reference_object.WriteObjectModel3d("ply", "reference.ply", HTuple(), HTuple());
    
  }

  void HalconMatcher::match(){

    HObjectModel3D plane_fit;
    try{
      //ROS_INFO("Calculate plane fit");
      plane_fit = scene.FitPrimitivesObjectModel3d("primitive_type", "plane");
    }catch(HOperatorException &e){
      //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      return;
    }

    try{
      //ROS_INFO("Calculate plane distances");
      double max_plane_distance = 0.05;
      scene.DistanceObjectModel3d(plane_fit, HPose(), max_plane_distance, "distance_to", "primitive");
    }catch(HOperatorException &e){
      //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      return;
    }

    HObjectModel3D thresholded;
    try{
      //ROS_INFO("Thresholding by distance");
      thresholded = scene.SelectPointsObjectModel3d("&distance", 0.004, 0.010);
    }catch(HOperatorException &e){
      //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      return;
    }

    HObjectModel3DArray connected;
    try{
      //ROS_INFO("Finding connected components");
      connected = thresholded.ConnectionObjectModel3d("distance_3d", 0.01);
    }catch(HOperatorException &e){
      //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      return;
    }

    //ROS_INFO("Found %ld candidate objects", connected.Length());

    HObjectModel3DArray hv_ObjectModel3DSelected;
    try{
      //ROS_INFO("Selecting largest  objects");
      hv_ObjectModel3DSelected = scene.SelectObjectModel3d(connected, "num_points", "and", 1000, 10000);
    }catch(HOperatorException &e){
      //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
      return;
    }

    //ROS_DEBUG("Finding surface model");
    double rel_sample_dist = 0.1;
    double keypoint_fraction = 0.8;
    double min_score = 0;

    if(hv_ObjectModel3DSelected.Length() == 0){
      //ROS_WARN("No objects segmented");
    }

    for(int i = 0; i < hv_ObjectModel3DSelected.Length(); i++){

        HObjectModel3D segmented = hv_ObjectModel3DSelected.Tools()[i];

        /*Useful for debugging segmented point clouds

        std::stringstream fname;
        fname << "test_" << i << ".ply";

        segmented.WriteObjectModel3d("ply", fname.str().c_str(), HTuple(), HTuple());
        */

        HTuple match_score;
        HSurfaceMatchingResult surface_matcher;
        HPose pose = HSurfaceMatchingResult().FindSurfaceModel(reference_surface, segmented, rel_sample_dist, keypoint_fraction, min_score, "true", HTuple(), HTuple(), &match_score);

        try{
          std::cout << match_score << std::endl;
        }catch(HOperatorException &e){
          //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
          return;
        }catch(HTupleAccessException &e){
          //ROS_ERROR("Error: %s", e.ErrorMessage().Text());
        }
        
        /*
        if(res.IsHandleValid()){
          try{
            ROS_INFO("Finding pose");
            HTuple pose = res.GetSurfaceMatchingResult("pose", 0);

            pose.WriteTuple("/home/htp/test.txt");

            for(int j=0; j < pose.Length(); j++){
              ROS_INFO("%d %f", j, pose.ToDArr()[j]);
            }

          }catch(HOperatorException &e){
            ROS_ERROR("Error: %s", e.ErrorMessage().Text());
            return;
          }
        }else{
          ROS_WARN("Invalid handle");
        }
        */
        
    }

    return;
}