/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <stdlib.h>
#include <time.h>

using namespace cartographer;

Task::Task(std::string const& name)
    :   TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    :   TaskBase(name, engine)
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
    local_map.setPcFiltersParameters(_leaf_size.get(),_k_points.get(), _use_statistical_filter.get());
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

    sync_count = 0;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // start pancam_panorama
    bool tmpbool = true;
    _sync_out.write(tmpbool);

    // set goal
    Eigen::Vector3d goal = _reach_goal.get();
    goal_rbs.position = goal;
    _goal.write(goal_rbs);

    return true;

}
void Task::updateHook()
{
    TaskBase::updateHook();

    //_pointcloud.read(try_cloud);
    //static int ii = 0;

    if(_distance_image.read(distance_image) == RTT::NewData)
    {
        // stop pancam panorama movement
        bool tmpbool2 = false;
        _sync_out.write(tmpbool2);

        //_pose_ptu.read(pose_ptu); // not relevan (yet?) for hdpr
        _pose_imu.read(pose_imu);

        // Start processing
        base::Time t1[20];
        t1[0] = base::Time::now();

        local_map.distance2pointCloud(distance_image.data);
        t1[1] = base::Time::now();

        local_map.pointColudFiltering();
        t1[2] = base::Time::now();

        //********************Transforms******************************************

        // Body attitude and psoition source choice

        // a component is connected giving already a pose referenced to the rover geometrical center
        double x_pos, y_pos, z_pos;
        double yaw, pitch, roll, tmp_yaw;
        Eigen::Quaterniond attitude;

        if(_pose_in.connected()) 
        {

            Eigen::Affine3d tf_pose;
            // damn transformer, if a total pose is given, the target MUST be gnns_utm both here, in the .orogen and in the damn component providing the pose
            if(!_body2viso_world.get(distance_image.time, tf_pose, false))
            {
                //throw std::runtime_error("[Cartographer] [FATAL ERROR]: transformation for transformer imu2gnss_utm not found.");
                //return;

            } 
            // roll, pitch, yaw
            pose_in.orientation = Eigen::Quaterniond(tf_pose.linear());

            yaw = pose_in.getYaw();
            pitch = pose_in.getPitch();
            roll = pose_in.getRoll();
            tmp_yaw = yaw;
            yaw = 0; // yaw to 0 for the moment TODO

            x_pos = tf_pose.translation().x();
            y_pos = tf_pose.translation().y();
            z_pos = tf_pose.translation().z();

            attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(yaw+body_rotation_offset[0], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(pitch+body_rotation_offset[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll+body_rotation_offset[2], Eigen::Vector3d::UnitX()));
        }
        // Otherwise, IMU and another component (typically vicon) are connected
        else
        {

            Eigen::Affine3d tf_imu, tf_vicon;


            if(!_imu2world_osg.get(distance_image.time, tf_imu, false))
            {
                //throw std::runtime_error("[Cartographer] [FATAL ERROR]: transformation for transformer imu2world not found.");
                //return;
            }

            //if(!_body2world_osg.get(distance_image.time, tf_vicon, false))
            //{
            //throw std::runtime_error("[Cartographer] [FATAL ERROR]: transformation for transformer body2world not found.");
            //return;
            //}

            // roll, pitch, yaw
            double roll, pitch, yaw;
            pose_imu.orientation = Eigen::Quaterniond(tf_imu.linear());
            pose_vicon.orientation = Eigen::Quaterniond(tf_vicon.linear());

            yaw = pose_vicon.getYaw();
            pitch = -pose_imu.getPitch();
            roll = -pose_imu.getRoll();
            tmp_yaw = yaw;
            yaw = 0; // yaw to 0 for the moment TODO

            //x_pos = tf_vicon.translation().x();
            //y_pos = tf_vicon.translation().y();
            //z_pos = tf_vicon.translation().z();

            attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(yaw+body_rotation_offset[0], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(pitch+body_rotation_offset[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll+body_rotation_offset[2], Eigen::Vector3d::UnitX()));
        }

        // Managing PTU

        // dirty ptu fix with ad hoc HDPR ptu 2 panorama output convention
        double ptu_pitch, ptu_yaw;
        _ptu_pan.read(ptu_yaw);
        _ptu_tilt.read(ptu_pitch);
        //ptu_pitch += 45.0;
        //			ptu_pitch -= 90.0;
        //ptu_pitch -= 90.0;
        //			ptu_pitch *=-2;
        //ptu_pitch += 45.0;
        ptu_pitch = (180 - ptu_pitch)/2;
        ptu_pitch -= 12.5;
        ptu_pitch = ptu_pitch/180.0*M_PI;
        ptu_yaw = ptu_yaw/180.0*M_PI;
        Eigen::Quaterniond ptu_attitude;
        ptu_attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(ptu_yaw+ptu_rotation_offset[0], Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(ptu_pitch+ptu_rotation_offset[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0+ptu_rotation_offset[2], Eigen::Vector3d::UnitX()));

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
        //customImg = customCVconversion(local_map.getTraversability());
        //_traversability_frame.write(customImg);

        // mask
        //customImg = customCVconversion(local_map.getMask());
        //_mask_frame.write(customImg);

        /* ########################## Display pointcloud ########################## */
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr p1 = local_map.getPointCloud();
          pcl::PointCloud<pcl::PointXYZ>::Ptr p2 = local_map.getPointCloudFiltered();
          base::samples::Pointcloud pp1;
          base::samples::Pointcloud pp2;

          fromPCLPointCloud(pp1, *p1, 1);
          fromPCLPointCloud(pp2, *p2, 1),

          _pointcloud_in.write(pp1);
          _pointcloud_filter.write(pp2);*/

        bool tmpbool = true;
        _sync_out.write(tmpbool);
        sync_count++;

        _goal.write(goal_rbs);

    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

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
