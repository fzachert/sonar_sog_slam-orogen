/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SONAR_SOG_SLAM_TASK_TASK_HPP
#define SONAR_SOG_SLAM_TASK_TASK_HPP

#include "sonar_sog_slam/TaskBase.hpp"
#include <sonar_sog_slam/slam/sog_slam.hpp>
#include <sonar_sog_slam/maps/sog_map.hpp>
#include <sonar_sog_slam/types/filter_config.hpp>
#include <sonar_sog_slam/types/model_config.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Time.hpp>

namespace sonar_sog_slam {

    class Task : public TaskBase
    {
	friend class TaskBase;
    
    private:
      SOG_Slam sog_slam;
      FilterConfig filter_config;
      ModelConfig model_config;
      base::samples::RigidBodyState rbs_out, rbs_dead_reackoning;
      
      base::Time last_orientation_sample;
      base::Time last_sonar_sample;
      base::Time last_depth_sample;
      base::Time last_velocity_sample;
      base::Time last_sample;
      bool got_initial_feature;
      bool got_initial_groundtruth;
      base::Vector2d coordinate_transformation;
    
      bool state_machine();
      void change_state( States s);
      
    protected:  
  

    public:

        Task(std::string const& name = "sonar_sog_slam::Task");

        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        bool configureHook();

        bool startHook();

        void updateHook();

        void stopHook();
	
        virtual void depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample);

        virtual void orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

        virtual void sonar_samplesCallback(const base::Time &ts, const ::sonar_image_feature_extractor::SonarFeatures &sonar_samples_sample);

        virtual void velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample);	
	
	virtual void initial_groundtruthCallback(const base::Time &ts, const ::base::samples::RigidBodyState &initial_groundtruth_sample);
	
    };
}

#endif

