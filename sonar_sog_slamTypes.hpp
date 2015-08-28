#ifndef sonar_sog_slam_TYPES_HPP
#define sonar_sog_slam_TYPES_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>

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
    bool flag;
    
  };
  
  struct GroundtruthMap{
    
    double tolerance;
    std::vector<GroundtruthFeature> features;
    
  };

  
  struct MapEvaluation{
    double map_variance;
    int number_of_missing_features;
    int number_of_features_without_gt;
    
  };

  struct EvaluationData{
    
    base::Time time;
    
    double position_diff;
    double dead_reackoning_diff;
    double position_variance;
    double max_error;
    double dead_reackoning_variance;
    double map_error;

    int number_of_measurements;

    MapEvaluation map_eval;    
    
  };
  
}

#endif

