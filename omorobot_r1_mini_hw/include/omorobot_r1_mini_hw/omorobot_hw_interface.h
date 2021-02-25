#ifndef OMOROBOT_HW_INTERFACE_H_
#define OMOROBOT_HW_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <libserial/SerialPort.h>

class OMORobotHWInterface: public hardware_interface::RobotHW
{
    public:
        OMORobotHWInterface();
        ~OMORobotHWInterface();

    public:
        virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        virtual void read(const ros::Time& time, const ros::Duration& period);
        virtual void write(const ros::Time& time, const ros::Duration& period);
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:
        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        hardware_interface::ImuSensorInterface imu_sensor_interface_;

        std::vector<double> joint_cmd_;
        std::vector<double> joint_pos_;
        std::vector<double> joint_vel_;
        std::vector<double> joint_eff_;
        std::vector<long int> last_encoder_value_;

        double orientation_[4];
        double angular_vel_[3];
        double linear_accel_[3];
        double orientation_cov_[9];
        double angular_vel_cov_[9];
        double linear_accel__cov_[9];

        bool is_initialized_;
        double motor_gear_ratio_;

        boost::shared_ptr<LibSerial::SerialPort> serial_port_;
        boost::mutex lock;
};

#endif //OMOROBOT_HW_INTERFACE_H_