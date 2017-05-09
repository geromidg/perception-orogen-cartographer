#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'transformer/runtime'
require 'vizkit'


include Orocos

Bundles.initialize

Bundles::transformer::load_conf(Bundles::find_file('config', 'transforms_scripts.rb'))

Orocos.run 'stereo::Task' => 'stereo', 
		'cartographer::Task' => 'cartographer' do 

    #Orocos.log_all_ports 
        
    # new components to run on top of the log
    stereo = Orocos.name_service.get 'stereo'
    cartographer = Orocos.name_service.get 'cartographer'
    Orocos.conf.apply(stereo, ['panCam'], :override => true)
    Orocos.conf.apply(cartographer, ['hdpr'], :override => true)

	# open log file to be postprocessed
    if ARGV.size == 0 then
		log_replay = Orocos::Log::Replay.open( "pancam.log",
												"waypoint_navigation.log")
    else
		log_replay = Orocos::Log::Replay.open(ARGV[0]+"pancam.log",
													ARGV[0]+"waypoint_navigation.log")
    end

    # uses timestamp when data was acquired
    log_replay.use_sample_time = true
    
    # new connection (either to logfed ports or new components)
    log_replay.pancam_panorama.left_frame_out.connect_to( stereo.left_frame)
    log_replay.pancam_panorama.right_frame_out.connect_to( stereo.right_frame)
    
    stereo.distance_frame.connect_to(cartographer.distance_image)
	log_replay.gps_heading.pose_samples_out.connect_to(cartographer.pose_in)
	log_replay.pancam_panorama.pan_angle_out_degrees.connect_to(cartographer.ptu_pan)
    log_replay.pancam_panorama.tilt_angle_out_degrees.connect_to(cartographer.ptu_tilt)
    
    Bundles.transformer.setup(cartographer)

    # start the components
    stereo.configure
    stereo.start
        
    cartographer.configure
    cartographer.start
    
    
	#show the control widget for the log file
	Vizkit.control log_replay
	Vizkit.display stereo.disparity_frame
	
	
	#start gui
	Vizkit.exec

    Readline::readline("Press enter to exit \n") do
    end
end
