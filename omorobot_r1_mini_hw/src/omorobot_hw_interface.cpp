#include "omorobot_r1_mini_hw/omorobot_hw_interface.h"
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>


OMORobotHWInterface::OMORobotHWInterface()
{
    last_encoder_value_.resize(2, 0);
}

OMORobotHWInterface::~OMORobotHWInterface()
{

}

bool OMORobotHWInterface::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    // get serial port name: ex) /dev/ttyUSB0
    std::string portName;
    if(!pnh.getParam("port_name", portName))
    {
        ROS_ERROR("[%s] Failed to get port name. Please set the parameter ~port_name", ros::this_node::getName().c_str());
        return false;
    }

    // get baudrate
    int baudrate;
    if(!pnh.getParam("baudrate", baudrate)) {
        ROS_ERROR("[%s] Failed to get baudrate. Please set the parameter ~baudrate", ros::this_node::getName().c_str());
        return false;
    }

    serial_port_ = boost::make_shared<LibSerial::SerialPort>();
    serial_port_->Open(portName);

    switch(baudrate)
    {
        case 115200:
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            break;
        case 57600:
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_57600);
            break;
        case 19200:
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_19200);
            break;
        default:
            ROS_ERROR("Failed to set baudrate. This baudrate is not support yet...");
            return false;
    }

    serial_port_->FlushIOBuffers();
    ros::Duration(0.5).sleep();

    // Clear Encoder Value
    serial_port_->Write("$cENCOD,0\r\n");
    ros::Duration(0.5).sleep();
    serial_port_->FlushIOBuffers();
    ros::Duration(0.5).sleep();

    // Read Current Encoder Value
    serial_port_->Write("$qENCOD\r\n");
    std::string recv_msg;
    serial_port_->ReadLine(recv_msg);

    std::vector<std::string> recv_msg_arr;
    boost::algorithm::split(recv_msg_arr, recv_msg, boost::is_any_of(","));

    last_encoder_value_[0] = atoi(recv_msg_arr[1].c_str());
    last_encoder_value_[1] = atoi(recv_msg_arr[2].c_str());

    ROS_INFO("[%s] initial encoder value %ld %ld...", ros::this_node::getName().c_str(), last_encoder_value_[0], last_encoder_value_[1]);


    // hardware_interface
    joint_cmd_.resize(2);
    for(int i = 0; i < joint_cmd_.size(); i++) {
        joint_cmd_[i] = 0.0;
    }
    joint_pos_.resize(2);
    joint_vel_.resize(2);
    joint_eff_.resize(2);

    hardware_interface::JointStateHandle state_handle_l_wheel("l_wheel_joint", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_l_wheel);
    hardware_interface::JointStateHandle state_handle_r_wheel("r_wheel_joint", &joint_pos_[1], &joint_vel_[1], &joint_eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_r_wheel);

    registerInterface(&jnt_state_interface_);

    // joint_velocity_interface
    hardware_interface::JointHandle vel_handle_l_wheel(jnt_state_interface_.getHandle("l_wheel_joint"), &joint_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_l_wheel);
    hardware_interface::JointHandle vel_handle_r_wheel(jnt_state_interface_.getHandle("r_wheel_joint"), &joint_cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_r_wheel);

    registerInterface(&jnt_vel_interface_);

    ROS_INFO("[%s] Initialized successfully.", ros::this_node::getName().c_str());
    return true;
}

void OMORobotHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    ROS_INFO("read start...");

    serial_port_->Write("$qENCOD\r\n");

    std::string recv_msg;
    serial_port_->ReadLine(recv_msg);
    std::vector<std::string> recv_msg_arr;
    boost::algorithm::split(recv_msg_arr, recv_msg, boost::is_any_of(","));

    ROS_INFO("read mid [%s]...", recv_msg.c_str());

    int32_t current_enc[2] = {0, 0};
    current_enc[0] = atoi(recv_msg_arr[1].c_str());
    current_enc[1] = atoi(recv_msg_arr[2].c_str());

    joint_eff_[0] = 0.0;
    joint_eff_[1] = 0.0;

    joint_vel_[0] = (current_enc[0] - last_encoder_value_[0]) / 44.0 * (1.0 / period.toSec()) * (2.0 * M_PI) / 21.3 * -1.0;
    joint_vel_[1] = (current_enc[1] - last_encoder_value_[1]) / 44.0 * (1.0 / period.toSec()) * (2.0 * M_PI) / 21.3 * -1.0;

    joint_pos_[0] += (current_enc[0] - last_encoder_value_[0]) / 44.0 / 21.3 * (2.0 * M_PI) * -1.0;
    joint_pos_[1] += (current_enc[1] - last_encoder_value_[1]) / 44.0 / 21.3 * (2.0 * M_PI) * -1.0;

    last_encoder_value_[0] = current_enc[0];
    last_encoder_value_[1] = current_enc[1];

    ROS_INFO("read done...");
}

void OMORobotHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
    // input joint_cmd_[0], joint_cmd_[1] -> rad/s
    // input data to motor driver -> rpm

    int16_t l_vel = (int16_t)((joint_cmd_[0] * 60.0 / (2.0 * M_PI) * 1.0) * -1.0);
    int16_t r_vel = (int16_t)((joint_cmd_[1] * 60.0 / (2.0 * M_PI) * 1.0) * -1.0);

    boost::format msg =  boost::format("$cRPM,%1%,%2%\r\n") % l_vel % r_vel;
    std::string send_str = msg.str();
    ROS_INFO("%s", send_str.c_str());

    serial_port_->Write(send_str);
}

bool OMORobotHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                ROS_ERROR_STREAM("Bad interface: " << res_it->hardware_interface);
                std::cout << res_it->hardware_interface;
                return false;
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    ROS_ERROR_STREAM("Bad resource: " << (*ctrl_res));
                    std::cout << (*ctrl_res);
                    return false;
                }
            }
        }
    }
    return true;
}

void OMORobotHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                throw hardware_interface::HardwareInterfaceException("Hardware_interface " + res_it->hardware_interface + " is not registered");
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    throw hardware_interface::HardwareInterfaceException("Resource " + *ctrl_res + " is not registered");
                }
            }
        }
    }
}
