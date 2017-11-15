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

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // set algorithm parameters
    local_map.setCameraParameters(1024, 768,
        509.74846, 376.99711, 837.98636, 838.56569);
    local_map.setMapParameters(5.0, 0.03);
    local_map.setPcFiltersParameters(0.015, 20, true);
    local_map.setPcLimitsParameters(
        Eigen::Vector4f(0.0, -2.0, -1.5, 0.0),
        Eigen::Vector4f(4.0, 2.0, 1.5, 0.0));

    // set camera geometric parameters
    camera_to_ptu = Eigen::Vector3d(0.0, 0.06, 0.02);
    body_rotation_offset =  Eigen::Vector3d(0.0, 0.0, 0.0);
    ptu_to_center = Eigen::Vector3d(0.09, 0.0, 0.9);
    ptu_rotation_offset = Eigen::Vector3d(0.0, -0.07, 0.0);

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

    if(_distance_image.read(distance_image) == RTT::NewData)
    {
        _pose_imu.read(pose_imu);
        double roll = -pose_imu.getRoll();
        double pitch = -pose_imu.getPitch();

        Eigen::Quaterniond attitude = Eigen::Quaternion <double> (
                Eigen::AngleAxisd(body_rotation_offset[0],
                Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(body_rotation_offset[1] + pitch,
                Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(body_rotation_offset[2] + roll,
                Eigen::Vector3d::UnitX()));

        Eigen::Quaterniond ptu_attitude = Eigen::Quaternion <double> (
                Eigen::AngleAxisd(ptu_rotation_offset[0],
                Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(ptu_rotation_offset[1] + (M_PI / 2),
                Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(ptu_rotation_offset[2],
                Eigen::Vector3d::UnitX()));

        local_map.distance2pointCloud(distance_image.data);
        local_map.pointColudFiltering();
        local_map.pointCloud2flatRobotReference(attitude, ptu_to_center,
            ptu_attitude, camera_to_ptu);
        local_map.pointCloud2heightMap();
        local_map.heightMapInterpolate();

        base::samples::frame::Frame customImg;
        customImg = customCVconversion(local_map.getHeightMap());
        _heightMap_frame.write(customImg);
        customImg = customCVconversion(local_map.getHeightMapInterp());
        _heightMapInterp_frame.write(customImg);

        /* pcl::PointCloud<pcl::PointXYZ>::Ptr p1 = local_map.getPointCloud(); */
        /* pcl::PointCloud<pcl::PointXYZ>::Ptr p2 = local_map.getPointCloudFiltered(); */
        /* base::samples::Pointcloud pp1; */
        /* base::samples::Pointcloud pp2; */
        /* fromPCLPointCloud(pp1, *p1, 1); */
        /* fromPCLPointCloud(pp2, *p2, 1), */
        /* _pointcloud_in.write(pp1); */
        /* _pointcloud_filter.write(pp2); */
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
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

