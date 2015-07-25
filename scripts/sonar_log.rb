require "orocos"
require "vizkit"
include Orocos


#log = Log::Replay.open('/home/fabio/gemini.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen/20150605-1143/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1552/flatfish_sonars.0.log')
#log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/bojen2/20150610-1613/flatfish_sonars.0.log')
log = Log::Replay.open('/media/fabio/WINDOWS/LOGS/traj1/extract/traj1.log')

Orocos.run "sonar_image_feature_extractor::SonarBeamProcessing" =>"sife", "gemini_scan_converter::Task" =>"gem"   do
  sife = Orocos.name_service.get('sife')
  gem = Orocos.name_service.get('gem')

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
  
  Vizkit.display sife
#  Vizkit.display gem
  Vizkit.control log
  Vizkit.exec
end