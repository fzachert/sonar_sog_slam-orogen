/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include<base/logging.h>

using namespace sonar_sog_slam;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    model_config = _model_config.get();
    filter_config = _filter_config.get();
    got_initial_feature = false;
    got_initial_groundtruth = false;
    coordinate_transformation = base::Vector2d::Zero();
    rbs_out.position = base::Vector3d::Zero();
    
    sog_slam.init(filter_config, model_config);
    
    LOG_INFO_S << "Started SOG-Slam-Task";
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    
    _map_samples.write( sog_slam.get_map() );
     DebugOutput d_out = sog_slam.get_debug();
//     std::cout << "Write init feature like. : " << d_out.initial_feature_likelihood << std::endl;
    _debug_output.write( d_out);
    _particles.write(sog_slam.getParticleSet() );
    
    state_machine();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}


void Task::depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    if(depth_samples_sample.hasValidPosition(2)){
    
      last_depth_sample = ts;
      last_sample = ts;
      
      sog_slam.set_depth( depth_samples_sample.position.z() );
    }
}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{

  if( orientation_samples_sample.hasValidOrientation() ){
    
    last_orientation_sample = ts;
    last_sample = ts;
    
    sog_slam.set_orientation( orientation_samples_sample.orientation);
    sog_slam.set_time( ts);
  }
    
    if(filter_config.estimate_middle)
      rbs_out = sog_slam.estimate_middle();
    else
      rbs_out = sog_slam.estimate();
    
    
    rbs_out.position.block<2,1>(0,0) += coordinate_transformation;
    _pose_samples.write( rbs_out);
    
}

void Task::sonar_samplesCallback(const base::Time &ts, const ::sonar_image_feature_extractor::SonarFeatures &sonar_samples_sample)
{
  
  LOG_DEBUG_S << "Got sonar-sample with " << sonar_samples_sample.features.size() << " features, at " << sonar_samples_sample.time.toString();
  
  last_sonar_sample = ts;
  last_sample = ts;
  
  
  if( state_machine() ){
    
    if(sonar_samples_sample.features.size() > 0)
      got_initial_feature = true;
    
    
    sog_slam.observe_features( sonar_samples_sample, filter_config.sonar_weight );
    
  }
  
}

void Task::velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
  
    if( velocity_samples_sample.hasValidVelocity()){
    
      last_velocity_sample = ts;
      last_sample = ts;
      
      if( state_machine() && got_initial_feature){
	
	sog_slam.update( velocity_samples_sample, DummyMap());
	sog_slam.update_dead_reackoning( velocity_samples_sample);
	
	rbs_dead_reackoning = sog_slam.estimate_dead_reackoning();
	
	rbs_dead_reackoning.position.block<2,1>(0,0) += coordinate_transformation;
	_dead_reackoning_samples.write( rbs_dead_reackoning);	
	
	
      }
      
    }
    
}

void Task::initial_groundtruthCallback(const base::Time &ts, const ::base::samples::RigidBodyState &initial_groundtruth_sample)
{
  
  if( (!got_initial_groundtruth) && got_initial_feature && state_machine())
  {
    got_initial_groundtruth = true;
    coordinate_transformation = initial_groundtruth_sample.position.block<2,1>(0,0) - rbs_out.position.block<2,1>(0,0);
    
  }
  
}

bool Task::state_machine(){
  
  if(last_orientation_sample.isNull() || last_sample.toSeconds() - last_orientation_sample.toSeconds() > _timeout.get() )
  {
    change_state(NO_ORIENTATION);
    return false;
    
  }else if(last_depth_sample.isNull() || last_sample.toSeconds() - last_depth_sample.toSeconds() > _timeout.get() )
  {
    change_state(NO_DEPTH);
    return false;
    
  }else if(last_velocity_sample.isNull() || last_sample.toSeconds() - last_velocity_sample.toSeconds() > _timeout.get() )
  {
    change_state(NO_VELOCITY);
    
  }else if(last_sonar_sample.isNull() || last_sample.toSeconds() - last_sonar_sample.toSeconds() > _timeout.get() )
  {
    change_state(NO_SONAR);
    
  }else if(!got_initial_feature){
    change_state(NO_INITIAL_FEATURE);
    
  }else if(got_initial_groundtruth){
    change_state(LOCALIZING_WITH_REFERENCE);
    
  }else{
    change_state(LOCALIZING);
  }
  
  return true;
  
}

void Task::change_state( States s){
  
  if( state() != s)
    state(s);
  
}



