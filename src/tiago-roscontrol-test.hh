/*
 * Author: Olivier STASSE
 */

#ifndef RC_SOT_CONTROLLER_H
#define RC_SOT_CONTROLLER_H

#include <string>
#include <map>

#pragma GCC diagnostic push
#pragma GCC system_header
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pal_hardware_interfaces/actuator_temperature_interface.h>
#include <pluginlib/class_list_macros.h>
#pragma GCC diagnostic pop

#include <ros/ros.h>
#include <control_toolbox/pid.h>

namespace tiago_roscontrol_test
{
  enum ControlMode { POSITION, VELOCITY };
  namespace lhi = hardware_interface;
  namespace lci = controller_interface;

  struct ControlPDMotorControlData
  {
    control_toolbox::Pid pid_controller;

    ControlPDMotorControlData();
    //    void read_from_xmlrpc_value(XmlRpc::XmlRpcValue &aXRV);
    void read_from_xmlrpc_value(const std::string &prefix);
  };

  struct JointSotHandle
  {
    lhi::JointHandle joint;
    double desired_init_pose;
    // This should not be handled in roscontrol_sot package. The control type
    // should be handled in SoT directly, by externalizing the integration from
    // the Device.
    //ControlMode sot_control_mode;
    ControlMode ros_control_mode;
  };

  typedef std::map<std::string,JointSotHandle>::iterator it_joint_sot_h;
#ifndef CONTROLLER_INTERFACE_KINETIC
  typedef std::set<std::string> ClaimedResources;
#endif
  /**
     This class encapsulates the Stack of Tasks inside the ros-control infra-structure.

   */
  class TiagoRosControlTest : public lci::ControllerBase
  {

  private:

    /// @{ \name Ros-control related fields

    /// \brief Vector of joint handles.
    std::map<std::string,JointSotHandle> joints_;
    std::vector<std::string> joints_name_;

    /// \brief Interface to the joints controlled in position.
    lhi::PositionJointInterface * pos_iface_;

    /// \brief Interface to the joints controlled in position.
    lhi::VelocityJointInterface * vel_iface_;

    /// @}

    const std::string type_name_;

    /// \brief Adapt the interface to Gazebo simulation
    bool simulation_mode_;

    /// \brief Implement a PD controller for the robot when the dynamic graph
    /// is not on.
    std::map<std::string, ControlPDMotorControlData> velocity_mode_pd_motors_;

    /// \brief Verbosity level for ROS messages during initRequest/initialization phase.
    /// 0: no messages or error 1: info 2: debug
    int verbosity_level_;

  public :

    TiagoRosControlTest ();

    /// \brief Read the configuration files,
    /// claims the request to the robot and initialize the Stack-Of-Tasks.
    bool initRequest (lhi::RobotHW * robot_hw,
		      ros::NodeHandle &robot_nh,
		      ros::NodeHandle &controller_nh,
		      ClaimedResources & claimed_resources);

    /// \brief Display claimed resources
    void displayClaimedResources(ClaimedResources & claimed_resources);

    /// \brief Claims
    bool init();

    /// \brief Read the sensor values, calls the control graph, and apply the control.
    ///
    void update(const ros::Time&, const ros::Duration& );
    /// \brief Starting by filling the sensors.
    void starting(const ros::Time&);
    /// \brief Stopping the control
    void stopping(const ros::Time&);

  protected:
    /// Initialize the roscontrol interfaces
    bool initInterfaces(lhi::RobotHW * robot_hw,
			ros::NodeHandle &,
			ros::NodeHandle &,
			ClaimedResources & claimed_resources);

    /// Initialize the hardware interface using the joints.
    bool initJoints();

    ///@{ \name Read the parameter server
    /// \brief Entry point
    bool readParams(ros::NodeHandle &robot_nh);

    /// \brief Creates the list of joint names.
    bool readParamsJointNames(ros::NodeHandle &robot_nh);

    /// \brief Read the control mode.
    bool readParamsControlMode(ros::NodeHandle & robot_nh);

    /// \brief Read the PID information of the robot in velocity mode.
    bool readParamsVelocityControlPDMotorControlData(ros::NodeHandle &robot_nh);

    /// \brief Read the desired initial pose of the robot in position mode.
    bool readParamsPositionControlData(ros::NodeHandle &robot_nh);

    /// \brief Read verbosity level to display messages mostly during initialization
    void readParamsVerbosityLevel(ros::NodeHandle &robot_nh);
    ///@}

    ///@{ Control the robot while waiting for the SoT
    /// Default control in velocity.
    void localStandbyVelocityControlMode(const ros::Duration& period);
    /// Default control in position.
    void localStandbyPositionControlMode(const ros::Time&, const ros::Duration& );

    ///@}

    /// Returns control mode by reading rosparam.
    /// It reads /tiago_roscontrol_test/control_mode/joint_name
    /// and check 
    bool
    getJointControlMode(std::string &joint_name,
			JointSotHandle &aJointSotHandle);

    ros::Time start_;
  };
}

#endif /* RC_SOT_CONTROLLER_H */
