/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CARTOGRAPHER_TASK_TASK_HPP
#define CARTOGRAPHER_TASK_TASK_HPP

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <base/samples/Frame.hpp>
#include "cartographer/TaskBase.hpp"
#include <frame_helper/FrameHelper.h>

#include <pcl/point_cloud.h>

#include <cartographer/localMap.hpp>

namespace cartographer {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

     The corresponding C++ class can be edited in tasks/Task.hpp and
     tasks/Task.cpp, and will be put in the cartographer namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
     task('custom_task_name','cartographer::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */

    class Task : public TaskBase
    {
        friend class TaskBase;

    protected:
        LocalMap local_map;
        base::samples::DistanceImage distance_image;
        base::samples::RigidBodyState pose_imu;

        // Translations/Rotations
        Eigen::Vector3d camera_to_ptu;
        Eigen::Vector3d ptu_to_center;
        Eigen::Vector3d ptu_rotation_offset;
        Eigen::Vector3d body_rotation_offset;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "cartographer::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);


        /** Default deconstructor of Task
        */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
         needs_configuration
         ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        // borrowed from https://github.com/exoter-rover/slam-orogen-icp/blob/master/tasks/GIcp.hpp
        void fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density = 1.0);

        base::samples::frame::Frame customCVconversion(cv::Mat CVimg);
    };
}

#endif

