/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Evaluation.hpp"

using namespace sonar_sog_slam;

Evaluation::Evaluation(std::string const& name)
    : EvaluationBase(name)
{
}

Evaluation::Evaluation(std::string const& name, RTT::ExecutionEngine* engine)
    : EvaluationBase(name, engine)
{
}

Evaluation::~Evaluation()
{
}

void Evaluation::groundtruth_positionCallback(const base::Time &ts, const ::base::samples::RigidBodyState &groundtruth_position_sample)
{
  if(groundtruth_position_sample.cov_position(0,0) < _groundtruth_variance_threshold.get()
      && groundtruth_position_sample.cov_position(1,1) < _groundtruth_variance_threshold.get())
  {
   groundtruth = groundtruth_position_sample;
  }
  
  _groundtruth_samples.write(groundtruth_position_sample);
  
}

void Evaluation::slam_positionCallback(const base::Time &ts, const ::base::samples::RigidBodyState &slam_position_sample)
{
    
  if( std::fabs( ts.toSeconds() - groundtruth.time.toSeconds() ) < _timediff_tolerance.get()){
    
    eval_data.time = ts;
    
    eval_data.position_diff = (groundtruth.position.block<2,1>(0,0) - slam_position_sample.position.block<2,1>(0,0)).norm();
    sum_square_error += std::pow(eval_data.position_diff, 2.0); 
    
    if(eval_data.position_diff > eval_data.max_error)
      eval_data.max_error = eval_data.position_diff;
    
    eval_data.number_of_measurements++;
    eval_data.position_variance += ( 1.0 / (double)eval_data.number_of_measurements)
      * ( std::pow(eval_data.position_diff,2.0) - eval_data.position_variance );
      
    eval_data.dead_reackoning_variance += ( 1.0 / (double)eval_data.number_of_measurements)
      * (std::pow(eval_data.dead_reackoning_diff,2.0) - eval_data.dead_reackoning_variance );
    
    _eval_data.write(eval_data);
  }
  
  
}

void Evaluation::dead_reackoning_positionCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dead_reackoning_position_sample)
{

  if( std::fabs( ts.toSeconds() - groundtruth.time.toSeconds() ) < _timediff_tolerance.get()){
    
    eval_data.time = ts;
    
    eval_data.dead_reackoning_diff = (groundtruth.position.block<2,1>(0,0) - dead_reackoning_position_sample.position.block<2,1>(0,0)).norm();
    
    _eval_data.write(eval_data);
  }
  
  
}
  
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Evaluation.hpp for more detailed
// documentation about them.

bool Evaluation::configureHook()
{
    if (! EvaluationBase::configureHook())
        return false;
    return true;
}
bool Evaluation::startHook()
{
    if (! EvaluationBase::startHook())
        return false;
    
    sum_square_error = 0.0;
    sum_square_error_dead_reackoning = 0.0;
    
    eval_data.max_error = 0.0;
    eval_data.number_of_measurements = 0;
    
    eval_data.position_diff = 0.0;
    eval_data.dead_reackoning_diff = 0.0;
    
    eval_data.position_variance = 0.0;
    eval_data.dead_reackoning_variance = 0.0;
    
    real_features = _real_features.get();
    
    gt_map.features.clear();
    for(std::vector<GroundtruthFeature>::iterator it = real_features.features.begin(); it != real_features.features.end(); it++){
      
      Simple_Feature sf;
      sf.pos = it->pos;
      sf.descriptor = 3;
      
      gt_map.simple_features.push_back(sf);
      
    }
    
    
    return true;
}
void Evaluation::updateHook()
{
    EvaluationBase::updateHook();
    
    SOG_Map map;
    
    while( _map_samples.read(map) == RTT::NewData){
      
      for(std::vector<GroundtruthFeature>::iterator it = real_features.features.begin(); it != real_features.features.end(); it++){
	
	it->flag = false;
      }
      
      
      eval_data.map_eval.number_of_features_without_gt = 0;
      eval_data.map_eval.number_of_missing_features = real_features.features.size();
      eval_data.map_eval.map_variance = 0.0;
      
      for(std::vector<Simple_Feature>::iterator it = map.simple_features.begin(); it != map.simple_features.end(); it++){
	
	double min_dist = real_features.tolerance;
	std::vector<GroundtruthFeature>::iterator min_it;
	
	for(std::vector<GroundtruthFeature>::iterator it_gt = real_features.features.begin(); it_gt != real_features.features.end(); it_gt++){
	  
	  if(!it_gt->flag){
	    double dist = ( it_gt->pos - it->pos).norm();
	    
	    if( dist < min_dist){
	      
	      min_dist = dist;
	      min_it = it_gt;
	      
	    }	      
	    
	  }
	  
	}	
	
	if( min_dist < real_features.tolerance){
	  min_it->flag = true;
	  eval_data.map_eval.number_of_missing_features--;
	  eval_data.map_eval.map_variance += std::pow( min_dist, 2.0);
	  
	}else{
	  
	  eval_data.map_eval.number_of_features_without_gt++;
	  
	}
	
	
      }     
      
      if(real_features.features.size() > 0){
	eval_data.map_eval.map_variance /= (double)real_features.features.size();
      }
      
      gt_map.time = map.time;
      _groundtruth_map.write( gt_map);
      
    }
    
    
}
void Evaluation::errorHook()
{
    EvaluationBase::errorHook();
}
void Evaluation::stopHook()
{
    EvaluationBase::stopHook();
}
void Evaluation::cleanupHook()
{
    EvaluationBase::cleanupHook();
}
