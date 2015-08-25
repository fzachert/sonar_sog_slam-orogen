#ifndef sonar_sog_slam_TYPES_HPP
#define sonar_sog_slam_TYPES_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <base/Pose.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace sonar_sog_slam {
  
  struct GroundtruthFeature{
    
    int id;
    base::Vector3d pos;
    
  };
  
  struct GroundtruthMap{
    
    std::vector<GroundtruthFeature> features;
    
  };


  struct EvaluationData{
    
    double position_diff;
    double dead_reackoning_diff;
    double position_variance;
    double max_error;
    double dead_reackoning_variance;
    double map_error;

    int number_of_measurements;    
    
  };
  
}

#endif

