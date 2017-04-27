/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <stdlib.h>
#include <time.h>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

using namespace cartographer;

Task::Task(std::string const& name)
    :   TaskBase(name),
        mpEnv(NULL), 
        mpFrameNode(NULL), 
		mpTravGrid(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    :   TaskBase(name, engine),
        mpEnv(NULL), 
        mpFrameNode(NULL), 
        mpTravGrid(NULL)
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

	frame_helper::CameraCalibration calib = _cameraCalibration.get();    
    
    // set algorithm parameters
    local_map.setCameraParameters(calib.width,calib.height,calib.cx,calib.cy, calib.fx, calib.fy);
	local_map.setMapParameters(_local_map_size.get(),_local_map_resolution.get(),_slope_map_scale.get());
	local_map.setPcFiltersParameters(_leaf_size.get(),_k_points.get());
	Eigen::Vector4d a = _pointcloud_cut_min.get();
	Eigen::Vector4d b = _pointcloud_cut_max.get();
	local_map.setPcLimitsParameters(a.cast<float>(),b.cast<float>());

	
	// set camera geometric parameters
	camera_to_ptu = _camera_to_ptu.get();	
	ptu_to_center = _ptu_to_center.get();
	ptu_rotation_offset = _ptu_rotation_offset.get();
	body_rotation_offset = _body_rotation_offset.get(); // todo as of now it is unused

	// set robot obstacles and slope parameters
	rover_normal_clearance = _rover_normal_clearance.get();
	laplacian_kernel_size = _laplacian_kernel_size.get();
	rover_normal_gradeability = _rover_normal_gradeability.get();
	local_map.setObstacleLaplacian(_laplacian_kernel_size.get(), _laplacian_threshold.get());
	local_map.setObstacleDetection(_obstacle_kernel_size.get(), _obstacle_iterations.get(),
					_obstacle_vicinity_kernel_size.get(),_obstacle_vicinity_iterations.get());
						
	// set global map
	global_height_map.setMapParameters(_global_map_size.get(),_global_map_resolution.get(),_global_safety_offset.get());
	global_obstacle_map.setMapParameters(_global_map_size.get(),_global_map_resolution.get(),_global_safety_offset.get());
	global_slope_map.setMapParameters(_global_map_size.get(),_global_map_resolution.get(),_global_safety_offset.get());
	global_slope_thresh_map.setMapParameters(_global_map_size.get(),_global_map_resolution.get(),_global_safety_offset.get());
	
	// set cost map
	cost_map.setObstacleDilation(_obstacle_cost_dilation.get(), _obstacle_cost_dilation_niter.get());
	cost_map.setMapBlurring(_obstacle_blur.get(), _obstacle_blur_max_cost.get());
	cost_map.setAdditionalSlopePenalty(_max_cost_slope.get(), _slope_max_cost.get());
	cost_map.setCostConstants(_cost_base.get(), _cost_offset.get(), _cost_max.get());

	
	// set up envire (todo check if every time)
	
	Nrow = _envire_size.get();
	Ncol = _envire_size.get(); // todo from config
	if(mpEnv != NULL) {
		delete mpEnv;
	}
	mpEnv = new envire::Environment(); 
	envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(
		(size_t) Nrow, (size_t) Ncol,
		_global_map_resolution.get(),
		_global_map_resolution.get()); // todo from config
	mpEnv->attachItem(trav);
	mpTravGrid = trav;
	// Creates a shared-pointer from the passed reference.
	envire::TraversabilityGrid::ArrayType& travData = mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
	boost::shared_ptr<envire::TraversabilityGrid::ArrayType> mpTravData;
	mpTravData = boost::shared_ptr<envire::TraversabilityGrid::ArrayType>(&travData, NullDeleter());

	trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5));
    std::cout << "Setting class #0 to 0.5 traversability."<< std::endl;
    
    
    
    for (int i = 0; i < SBPL_MAX_COST; ++i)
    {
        sbplCostToClassID[i] = i+2;
        //std::cout << "Mapping " << i+1 << " to " << i+2 << std::endl;   
    }
    sbplCostToClassID[SBPL_MAX_COST] = 1;
    //std::cout << "Mapping " << SBPL_MAX_COST+1 << " to " << 1 << std::endl;   

    // ASSOCIATING CLASS ID WITH TRAVERSABILITY
    

    double travVal;
    // class 1 reserved for obstacle (will be used instead of 21)
    trav->setTraversabilityClass(1, envire::TraversabilityClass(0.0));
    std::cout << "Setting class #" << 1
            <<  " to " << 0.0 << " traversability." << std::endl;

    int cost;
    // TODO: MAY SKIP CLASSES BELOW MIN_COST IN ORDER NOT TO BIAS UNKNOWN AREAS
    // those are set to 2nd highest traversability
    // but 2nd highest registered class, or 2nd highest value in the grid??
    for(cost = 1; cost < SBPL_MAX_COST+1; ++cost) {
        travVal = ((double)(SBPL_MAX_COST+1 - cost));
        travVal /= SBPL_MAX_COST;
        trav->setTraversabilityClass(sbplCostToClassID[cost-1], envire::TraversabilityClass(travVal));
        std::cout << "Setting class #" << sbplCostToClassID[cost-1]
            <<  " to " << travVal << " traversability." << std::endl;
    }
 
    for(int x=0; x < Nrow;  x++)
    {
        //row = groundTruth.at(x);
        for(int y=0; y < Ncol; y++)
        {
            // Set all to unknown
            trav->setTraversability(0, x, y);   
            trav->setProbability(1.0, x, y);
        }
	}
	
	sync_count = 0;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    //_pointcloud.read(try_cloud);
    //static int ii = 0;
    
    if(_distance_image.read(distance_image) == RTT::NewData)
    {
			_pose_ptu.read(pose_ptu);
			_pose_imu.read(pose_imu);
			
			// Start processing
			base::Time t1[20];
			t1[0] = base::Time::now();
			
			local_map.distance2pointCloud(distance_image.data);
			t1[1] = base::Time::now();

			local_map.pointColudFiltering();
			t1[2] = base::Time::now();

			//************************************************************************
			
			/********************
			OLD ATTITUDE
			*********************/
			// Set up rotation
			//double roll, pitch, yaw;
			//pitch = -pose_imu.getPitch();
			//roll = -pose_imu.getRoll();
			//yaw = pose_imu.getYaw();
			
			if(_pose_vicon.connected()) // todo x,y,z are at the moment assumed to be there, will be changed
			{
				//_pose_vicon.read(pose_vicon);
				//yaw = pose_vicon.getYaw();
			}
			else
				std::cout << "huge issue" << std::endl;
			
			//float tmp_yaw = yaw;
			//yaw = 0; // yaw to 0 for the moment TODO
			
			/********************
			OLD ATTITUDE
			*********************/

			/********************
			NEW ATTITUDE
			*********************/
			Eigen::Affine3d tf_imu, tf_vicon;
			
			
			if(!_imu2world_osg.get(distance_image.time, tf_imu, false))
			{
				//throw std::runtime_error("[Cartographer] [FATAL ERROR]: transformation for transformer imu2world not found.");
				//return;
			}

			if(!_body2world_osg.get(distance_image.time, tf_vicon, false))
			{
				//throw std::runtime_error("[Cartographer] [FATAL ERROR]: transformation for transformer body2world not found.");
				//return;
			}

			// roll, pitch, yaw
			double roll, pitch, yaw;
			pose_imu.orientation = Eigen::Quaterniond(tf_imu.linear());
			pose_vicon.orientation = Eigen::Quaterniond(tf_vicon.linear());

			yaw = pose_vicon.getYaw();
			pitch = -pose_imu.getPitch();
			roll = -pose_imu.getRoll();
			float tmp_yaw = yaw;
			yaw = 0; // yaw to 0 for the moment TODO
			

			/********************
			NEW ATTITUDE
			*********************/
			
			Eigen::Quaterniond attitude;
			attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(yaw+body_rotation_offset[0], Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(pitch+body_rotation_offset[1], Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(roll+body_rotation_offset[2], Eigen::Vector3d::UnitX()));
										
			// dirty ptu fix
			pitch = pose_ptu.getPitch();
			roll = pose_ptu.getRoll();
			yaw = pose_ptu.getYaw();
			Eigen::Quaterniond ptu_attitude;
			ptu_attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(yaw+ptu_rotation_offset[0], Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(pitch+ptu_rotation_offset[1], Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(roll+ptu_rotation_offset[2], Eigen::Vector3d::UnitX()));
										
			local_map.pointCloud2flatRobotReference(attitude, 
													ptu_to_center,
													ptu_attitude,
													camera_to_ptu);
			t1[3] = base::Time::now();

			//****************************local****************************************

			local_map.pointCloud2heightMap();
			t1[4] = base::Time::now();

			local_map.heightMapInterpolate();
			t1[5] = base::Time::now();

			local_map.heightMap2SlopeMap();
			t1[6] = base::Time::now();

			local_map.detectObstacles(rover_normal_clearance);
			t1[7] = base::Time::now();

            local_map.thresholdSlopeMap(rover_normal_gradeability);
			t1[8] = base::Time::now();

            local_map.computeTraversability();
			t1[9] = base::Time::now();

            
            //*****************************global**height******************************
            
            
            if(tf_vicon.translation().x() == tf_vicon.translation().x()) // not nan
            {
            global_height_map.realWorldOrientation(local_map.getHeightMap(), local_map.getMask(), tmp_yaw);
			t1[10] = base::Time::now();

            
            global_height_map.addToWorld(tf_vicon.translation().x(), tf_vicon.translation().y(), tf_vicon.translation().z());
			t1[11] = base::Time::now();

            //*****************************global**obstacle****************************
            
            global_obstacle_map.realWorldOrientation(local_map.getObstacles(), local_map.getMask(), tmp_yaw);
            
            global_obstacle_map.addToWorld(tf_vicon.translation().x(), tf_vicon.translation().y(), 0.0);
            
			//*****************************global**slope*******************************
            
            global_slope_map.realWorldOrientation(local_map.getSlopeMap(), local_map.getMask(), tmp_yaw);
            
            global_slope_map.addToWorld(tf_vicon.translation().x(), tf_vicon.translation().y(), 0.0);
            
			//*****************************global**slope_thres*************************
            
            global_slope_thresh_map.realWorldOrientation(local_map.getSlopeMapThresholded(), local_map.getMask(), tmp_yaw);
            
            global_slope_thresh_map.addToWorld(tf_vicon.translation().x(), tf_vicon.translation().y(), 0.0);
			t1[12] = base::Time::now();
        
            
			//**********************************cost***********************************
			
			// todo find more direct approach
			cv::Mat tmp1,tmp2,tmp3;
			
			global_slope_map.getGlobalMap().copyTo(tmp1);
			global_obstacle_map.getGlobalMap().copyTo(tmp2); 
			global_slope_thresh_map.getGlobalMap().copyTo(tmp3);
			
			tmp2.setTo(1.0,tmp2>0.4);
			tmp3.setTo(1.0,tmp3>0.4);
			
			tmp2.setTo(0.0,tmp2<0.4);
			tmp3.setTo(0.0,tmp3<0.4);
			
			cost_map.calculateCostMap(tmp1, 
									tmp2, 
									tmp3);
									
			t1[13] = base::Time::now();

									
			//**********************************cost*to*envire*************************
			cv::Mat temp_cost1, temp_cost2, no_cost1, no_cost2;
			temp_cost1 = cost_map.getCostMap();
			temp_cost1(cv::Rect(_envire_origin.get(), _envire_origin.get(),_envire_size.get()+1,_envire_size.get()+1)).copyTo(temp_cost2); // only terrain region
			temp_cost2.convertTo(temp_cost2, CV_8UC1);
			
			no_cost1 = global_height_map.getGlobalMap();
			no_cost1(cv::Rect(_envire_origin.get(), _envire_origin.get(),_envire_size.get()+1,_envire_size.get()+1)).copyTo(no_cost2);
			
			temp_cost2.setTo(0,no_cost2!=no_cost2);

			envire::TraversabilityGrid::ArrayType& travData = 
				mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
			
			int cost;
			for(int x=0; x < Nrow;  x++)
			{
				//row = groundTruth.at(x);
				for(int y=0; y < Ncol; y++)
				{
					cost = temp_cost2.at<char>(x,y);
					if(cost == 0)
						mpTravGrid->setTraversability( 0, x, y);
					else
						mpTravGrid->setTraversability( sbplCostToClassID[cost-1], x, y);
				}
			}
			
			envire::OrocosEmitter emitter_tmp(mpEnv, _traversability_map);
			emitter_tmp.setTime(base::Time::now());
			emitter_tmp.flush(); 
			
			t1[14] = base::Time::now();


			}

			/* ########################## just display ########################## */
			
			// heightMap frame
			base::samples::frame::Frame customImg = customCVconversion(local_map.getHeightMap());
			_heightMap_frame.write(customImg);
			
			// heightMap interpolated frame
		    customImg = customCVconversion(local_map.getHeightMapInterp());
		    _heightMapInterp_frame.write(customImg);
			
			// slopeMap frame
			customImg = customCVconversion(local_map.getSlopeMap());
			_slopeMap_frame.write(customImg);
			
			// slope map thresholded
			customImg = customCVconversion(local_map.getSlopeMapThresholded());
			_slopeMapThresholded_frame.write(customImg);
			
			// laplacian
			customImg = customCVconversion(local_map.getLaplacian());
			_laplacian_frame.write(customImg);
			
			// laplacian thresholded
			customImg = customCVconversion(local_map.getLaplacianThresholded());
			_laplacianThresholded_frame.write(customImg);
			
			// obstacles exceeding threshold
			customImg = customCVconversion(local_map.getObstacles());
			_obstacles_frame.write(customImg);
			
			// traversability map
			/*customImg = customCVconversion(local_map.getTraversability());
			_traversability_frame.write(customImg);
			
			// mask
			customImg = customCVconversion(local_map.getMask());
			_mask_frame.write(customImg);
			
			// global map
			customImg = customCVconversion(global_height_map.getGlobalMap());
			_global_frame.write(customImg);
			
			// debug map
			customImg = customCVconversion(global_obstacle_map.getGlobalMap());
			_debug_frame.write(customImg);

			// debug map2
			customImg = customCVconversion(global_slope_map.getGlobalMap());
			_debug_frame2.write(customImg);
			
			// debug map3
			customImg = customCVconversion(global_slope_thresh_map.getGlobalMap());
			_debug_frame3.write(customImg);
			
			// cost
			customImg = customCVconversion(temp_cost2);
			_cost_frame.write(customImg);*/
			
			/* ########################## just display end ########################## */
			
			/* ########################## Display pointcloud ########################## */
			/*pcl::PointCloud<pcl::PointXYZ>::Ptr p1 = local_map.getPointCloud();
			pcl::PointCloud<pcl::PointXYZ>::Ptr p2 = local_map.getPointCloudFiltered();
			base::samples::Pointcloud pp1;
			base::samples::Pointcloud pp2;
						
			fromPCLPointCloud(pp1, *p1, 1);
			fromPCLPointCloud(pp2, *p2, 1),
			
			_pointcloud_in.write(pp1);
			_pointcloud_filter.write(pp2);*/
			/* ########################## Display pointcloud end ########################## */
			t1[15] = base::Time::now();
			
			//for(int iii = 1; iii<16; iii++)
			//	std::cout << "time" << iii << ": " << (t1[iii]-t1[iii-1]) << std::endl;

			_sync_out.write(sync_count);
			sync_count++;
	}
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    
    /*cv::FileStorage file("/home/fjalar/Desktop/height.xml", cv::FileStorage::WRITE);
    cv::Mat a = global_height_map.getGlobalMap();
    a.setTo(0.0,a!=a);
    file << "a" << a;
    
    cv::FileStorage file1("/home/fjalar/Desktop/obstacle.xml", cv::FileStorage::WRITE);
    cv::Mat b = global_obstacle_map.getGlobalMap();
    b.setTo(0.0,b!=b);

    file1 << "b" << b;
    
    cv::FileStorage file2("/home/fjalar/Desktop/slope.xml", cv::FileStorage::WRITE);
    cv::Mat c = global_slope_map.getGlobalMap();
    c.setTo(0.0,c!=c);

    file2 << "c" << c;
    
    cv::FileStorage file3("/home/fjalar/Desktop/slope_thresh.xml", cv::FileStorage::WRITE);
    cv::Mat d = global_slope_thresh_map.getGlobalMap();
	d.setTo(0.0,d!=d);
    file3 << "d" << d;
    
    cv::FileStorage file4("/home/fjalar/Desktop/cost.xml", cv::FileStorage::WRITE);
    cv::Mat e = cost_map.getCostMap();
    file4 << "e" << e;*/
    
    std::cout << "stopped" << std::endl;
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
        std::cout << "cleaned" << std::endl;

}

// borrowed from https://github.com/exoter-rover/slam-orogen-icp/blob/master/tasks/GIcp.cpp
void Task::fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pcl_pc.size());

    if(density <= 0.0 || pcl_pc.size() == 0)
    {
        return;
    }
    else if(sample_count >= pcl_pc.size())
    {
        mask.resize(pcl_pc.size(), true);
    }
    else
    {
        mask.resize(pcl_pc.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pcl_pc.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pcl_pc.size(); ++i)
    {
        if(mask[i])
        {
            pc.points.push_back(::base::Point(pcl_pc.points[i].x, pcl_pc.points[i].y, pcl_pc.points[i].z));
        }
    }
}

// returns an opencv image
base::samples::frame::Frame Task::customCVconversion(cv::Mat CVimg)
{
	cv::normalize(CVimg, CVimg, 0, 255, cv::NORM_MINMAX, CV_32F);
	CVimg.convertTo(CVimg, CV_8U);
	
	base::samples::frame::Frame BASEimg( 
		CVimg.rows, CVimg.cols, 8, 
		base::samples::frame::MODE_GRAYSCALE );								
	frame_helper::FrameHelper::copyMatToFrame(CVimg, BASEimg);
	
	return BASEimg;
}