require 'orocos'
require 'vizkit'

include Orocos
#Orocos.CORBA.name_service = cfg["nameserver"].to_s

#Orocos::CORBA.name_service.ip = "192.168.128.51"  #20"
Orocos.initialize

#view3d = Vizkit.default_loader.create_plugin 'vizkit3d::Vizkit3DWidget'
view3d = Vizkit.vizkit3d_widget
view3d.show

rbs = view3d.createPlugin("base", "RigidBodyStateVisualization")
rbs2 = view3d.createPlugin("base", "RigidBodyStateVisualization")
traj = view3d.createPlugin("base", "TrajectoryVisualization")
traj2 = view3d.createPlugin("base", "TrajectoryVisualization")
traj3 = view3d.createPlugin("base", "TrajectoryVisualization")
ps = view3d.createPlugin("uw_localization", "ParticleSetVisualization")
mv = view3d.createPlugin("sonar_sog_slam", "SOGMapVisualization")
mv2 = view3d.createPlugin("sonar_sog_slam", "SOGMapVisualization")
env = view3d.createPlugin("uw_localization", "MapVisualization")

view3d.grid.setPluginEnabled(false)
view3d.grid.setGridCols( 40)
view3d.grid.setGridRows(36)
view3d.grid.setShowCoordinates(true)
#view3d.grid.setPose( [9,19,0], [1.0, 0.0, 0.0, 0.0] )
#view3d.grid.setGridTransformation( [9, 10,0] )
view3d.grid.setPluginEnabled(true)
#view3d.grid.Rows( 18)
#view3d.grid.setCols(20)
#puts (view3d.grid.methods)

#traj.setColor(Qt::Color.new(255,0,0))
#traj2.setColor(Qt::Color.new(0,0,255))
traj.setColor([[1.0,0,0]])
traj.setMaxNumberOfPoints( 999999)
traj2.setColor([[0,0,1.0]])
traj2.setMaxNumberOfPoints( 999999)
traj3.setColor([[1.0,1.0,1.0]])
traj3.setMaxNumberOfPoints( 999999)

Orocos.run do
    slam = TaskContext.get 'slam'
    eval = TaskContext.get 'eval'

    slam.pose_samples.connect_to do |sample,_|
        rbs.updateData(sample)
        sample
    end
    
    slam.pose_samples.connect_to do |sample, _|
	traj.updateTrajectory(sample.position)
	sample
    end

    slam.dead_reackoning_samples.connect_to do |sample,_|
        rbs2.updateData(sample)
        sample
    end

   slam.dead_reackoning_samples.connect_to do |sample,_|
	traj2.updateTrajectory(sample.position)
	sample
   end
    
   eval.groundtruth_samples.connect_to do |sample,_|
	traj3.updateTrajectory(sample.position)
	sample
   end

    slam.map_samples.connect_to do |sample,_|
        mv.updateData(sample)
        sample
    end

    eval.groundtruth_map.connect_to do |sample,_|
	mv2.updateData(sample)
	sample
    end    

    #Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
    slam.particles.connect_to do |sample, _|
        ps.updateParticleSet(sample)
        sample
    end



    Vizkit.exec
end
