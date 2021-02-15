#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <omorobot_r1_mini_hw/omorobot_hw_interface.h>

class ControlNode
{
    public:
        ControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            double control_frequency = 0.0;
            pnh.param<double>("rate", control_frequency, 10.0);

            robot_hw_ = boost::make_shared<OMORobotHWInterface>();
            assert(robot_hw_->init(nh, pnh));

            ros::Duration(0.5).sleep();
            ROS_INFO("[%s] ready. start controller...", ros::this_node::getName().c_str());

            cm_ = boost::make_shared<controller_manager::ControllerManager>(&(*robot_hw_), nh);

            period_ = ros::Duration(1.0 / control_frequency);
            loop_timer_ = nh.createTimer(period_, &ControlNode::callback, this);
            loop_timer_.start();
        }

        ~ControlNode()
        {
            loop_timer_.stop();
        }

    private:
        void callback(const ros::TimerEvent& event)
        {
            ros::Time start_time = ros::Time::now();

            robot_hw_->read(ros::Time::now(), period_);
            cm_->update(ros::Time::now(), period_);
            robot_hw_->write(ros::Time::now(), period_);

            ros::Duration elapse_time = ros::Time::now() - start_time;
            ROS_INFO("control_period: %f", elapse_time.toSec());
        }

    private:
        boost::shared_ptr<hardware_interface::RobotHW> robot_hw_;
        boost::shared_ptr<controller_manager::ControllerManager> cm_;
        ros::Timer loop_timer_;
        ros::Duration period_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omorobot_control_node");
    ros::AsyncSpinner spinner(4);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ControlNode m(nh, pnh);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}