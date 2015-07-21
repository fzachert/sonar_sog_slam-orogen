/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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
    
    sog_slam.init(filter_config, model_config);
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    
    _map_samples.write( sog_slam.getMap() );
    _debug_output.write( sog_slam.getDebug() );
    
}

void Task::stopHook()
{
    TaskBase::stopHook();
}


void Task::depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    if(base::isNaN(depth_samples_sample.position.z()))
      return;
  
    last_depth_sample = ts;
    last_sample = ts;
    
    sog_slam.set_depth( depth_samples_sample.position.z() );
}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{

  if( base::samples::RigidBodyState::isValidValue( orientation_samples_sample.orientation ) ){
    
    last_orientation_sample = ts;
    last_sample = ts;
    
    sog_slam.set_orientation( orientation_samples_sample.orientation);
  }
    
}

void Task::sonar_samplesCallback(const base::Time &ts, const ::sonar_image_feature_extractor::SonarFeatures &sonar_samples_sample)
{
    
  last_sonar_sample = ts;
  last_sample = ts;
  
  if( state_machine() ){
    
    sog_slam.observeFeatures( sonar_samples_sample, filter_config.sonar_weight );
    
  }
  
}

void Task::velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    last_velocity_sample = ts;
    last_sample = ts;
    
    if( state_machine() ){
      
      sog_slam.update( velocity_samples_sample, DummyMap());
      
    }
    
}



bool Task::state_machine(){
  
  if(last_orientation_sample.isNull() || last_sample.toSeconds() - last_orientation_sample.toSeconds() < _timeout.get() )
  {
    change_state(NO_ORIENTATION);
    return false;
    
  }else if(last_depth_sample.isNull() || last_sample.toSeconds() - last_depth_sample.toSeconds() < _timeout.get() )
  {
    change_state(NO_DEPTH);
    return false;
    
  }else if(last_velocity_sample.isNull() || last_sample.toSeconds() - last_velocity_sample.toSeconds() < _timeout.get() )
  {
    change_state(NO_VELOCITY);
    
  }else if(last_sonar_sample.isNull() || last_sample.toSeconds() - last_sonar_sample.toSeconds() < _timeout.get() )
  {
    change_state(NO_SONAR);
    
  }else{
    change_state(LOCALIZING);
  }
  
  return true;
  
}

void Task::change_state( States s){
  
  if( state() != s)
    state(s);
  
}



