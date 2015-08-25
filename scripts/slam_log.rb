require "orocos"
require "vizkit"
include Orocos


#log = Log::Replay.open('/home/fabio/gemini.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen/20150605-1143/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1552/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1613/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/traj1/extract/traj1.log')
log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/traj2/updated/traj2.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/traj3/traj3.log')

Orocos.run "sonar_image_feature_extractor::SonarBeamProcessing" =>"sife", "gemini_scan_converter::Task" =>"gem",
    "sonar_sog_slam::Task" => "slam" , "sonar_sog_slam::Evaluation" => "eval" do#,  :valgrind => ['slam'] do
  
  sife = Orocos.name_service.get('sife')
  gem = Orocos.name_service.get('gem')
  slam = Orocos.name_service.get('slam')
  evaluation = Orocos.name_service.get('eval')
  
  #####################################################
  #Sonar-stuff
  
  
  #auto connect
#  log.gemini.frame.connect_to gem.frame
  log.sonar_gemini.connect_to gem.frame
  gem.sonar_scan.connect_to sife.sonar_image
  
  Orocos.apply_conf_file(sife, 'config/svm.yml', ['default'])  
  
  gem.column = false
  gem.toogle = true
  
  sife.debug_mode = :SMOOTHING
  sife.smooth_mode = :AVG
  sife.threshold_mode = :ADAPTIVE_MEAN
  sife.distance_mode = :EUKLIDIAN
  
  sife.blur = 3
  sife.threshold = 0.15
  sife.adaptive_threshold_neighborhood = 15  
  
  sife.cluster_min_size = 10
  sife.cluster_max_size = 300
  sife.cluster_noise = 0.8
  
  sife.ignore_labels = [3] #[3]  
  
  sife.max_sonar_range = 16
  
  sife.configure
  sife.start
  
  gem.configure
  gem.start
  
  
  ##############################################
  # Slam-stuff
  Orocos.apply_conf_file(slam, 'config/sonar_sog_slam::Task.yml', ['default'])
  Orocos.apply_conf_file(evaluation, 'config/evaluation.yml',['default'])
  
  log.dvl_seapilot.velocity_samples.connect_to slam.velocity_samples
  log.orientation_in_map.orientation_in_map.connect_to slam.orientation_samples, :type => :buffer, :size => 100
  log.pressure_sensor.depth_samples.connect_to slam.depth_samples
  log.uw_particle_localization.pose_samples.connect_to slam.initial_groundtruth, :type => :buffer, :size => 100
  sife.detected_buoy.connect_to slam.sonar_samples
  
  slam.pose_samples.connect_to evaluation.slam_position, :type => :buffer, :size => 100
  slam.dead_reackoning_samples.connect_to evaluation.dead_reackoning_position, :type => :buffer, :size => 10
  log.uw_particle_localization.pose_samples.connect_to evaluation.groundtruth_position, :type => :buffer, :size => 100
  
  evaluation.configure
  evaluation.start
  
  slam.configure
  slam.start
  
  Vizkit.display sife
  Vizkit.display slam
  Vizkit.display evaluation
#  Vizkit.display gem
  Vizkit.control log
  Vizkit.exec
end