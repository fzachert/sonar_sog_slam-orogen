require "orocos"
require "vizkit"
include Orocos


#log = Log::Replay.open('/home/fabio/gemini.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen/20150605-1143/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1552/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1613/flatfish_sonars.0.log')
log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/traj1/extract/traj1.log')

Orocos.run "sonar_image_feature_extractor::SonarBeamProcessing" =>"sife", "gemini_scan_converter::Task" =>"gem",
    "sonar_sog_slam::Task" => "slam" do
  
  sife = Orocos.name_service.get('sife')
  gem = Orocos.name_service.get('gem')
  slam = Orocos.name_service.get('slam')
  
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
  
  #log.dvl_seapilot.velocity_samples.connect_to slam.velocity_samples
  #log.orientation_estimator.orientation_samples_out.connect_to slam.orientation_samples
  #log.pressure_sensor.depth_samples.connect_to slam.depth_samples
  #sife.detected_buoy.connect_to slam.sonar_samples
  
  slam.configure
  slam.start
  
  Vizkit.display sife
  Vizkit.display slam
#  Vizkit.display gem
  Vizkit.control log
  Vizkit.exec
end