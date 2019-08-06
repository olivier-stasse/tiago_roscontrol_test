#include <fstream>
#include <iomanip>
#include <dlfcn.h>
#include <sstream>


#include "tiago-roscontrol-test.hh"

#include<ros/console.h>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/rcoh2sot.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}

#define RESETDEBUG4()
#define ODEBUG4FULL(x)
#define ODEBUG4(x)

/// lhi: nickname for local_hardware_interface
/// Depends if we are on the real robot or not.

namespace lhi=hardware_interface;
using namespace lhi;

namespace tiago_roscontrol_test
{
  typedef std::map<std::string,std::string>::iterator it_map_rt_to_sot;
  typedef std::map<std::string,std::string>::iterator it_control_mode;

  ControlPDMotorControlData::ControlPDMotorControlData()
  {
  }

  void ControlPDMotorControlData::read_from_xmlrpc_value
  (const std::string &prefix)
  {
    pid_controller.initParam(prefix);
  }

  TiagoRosControlTest::
  TiagoRosControlTest():
    // Store 32 DoFs for 5 minutes (1 Khz: 5*60*1000)
    // -> 124 Mo of data.
    type_name_("TiagoRosControlTest"),
    simulation_mode_(false),
    verbosity_level_(0)
  {
    RESETDEBUG4();
  }

  void TiagoRosControlTest::
  displayClaimedResources(ClaimedResources & claimed_resources)
  {
#ifdef CONTROLLER_INTERFACE_KINETIC
    ClaimedResources::iterator it_claim;
    ROS_INFO_STREAM("Size of claimed resources: "<< claimed_resources.size());
    for (it_claim = claimed_resources.begin();
	 it_claim != claimed_resources.end();
	 ++it_claim)
      {
	hardware_interface::InterfaceResources & aclaim = *it_claim;
	ROS_INFO_STREAM("Claimed by TiagoRosControlTest: " << aclaim.hardware_interface);

	for(std::set<std::string>::iterator
	      it_set_res=aclaim.resources.begin();
	    it_set_res!=aclaim.resources.end();
	    it_set_res++)
	  {
	    ROS_INFO_STREAM(" Resources belonging to the interface:" <<
			    *it_set_res);
	  }

      }
#else
    std::set<std::string >::iterator it_claim;
    ROS_INFO_STREAM("Size of claimed resources: "<< claimed_resources.size());
    for (it_claim = claimed_resources.begin();
	 it_claim != claimed_resources.end();
	 ++it_claim)
      {
	std::string aclaim = *it_claim;
	ROS_INFO_STREAM("Claimed by TiagoRosControlTest: " << aclaim);
      }
#endif
  }

  bool TiagoRosControlTest::
  initRequest (lhi::RobotHW * robot_hw,
	       ros::NodeHandle &robot_nh,
	       ros::NodeHandle &controller_nh,
	       ClaimedResources & claimed_resources)
  {
    /// Read the parameter server
    if (!readParams(robot_nh))
      return false;

    /// Create ros control interfaces to hardware
    /// Recalls: init() is called by initInterfaces()
    if (!initInterfaces(robot_hw,robot_nh,controller_nh,claimed_resources))
      return false;

    return true;
  }

  bool TiagoRosControlTest::
  initInterfaces(lhi::RobotHW * robot_hw,
		 ros::NodeHandle &,
		 ros::NodeHandle &,
		 ClaimedResources & claimed_resources)
  {
    std::string lns;
    lns="hardware_interface";

    // Check if construction finished cleanly
    if (state_!=CONSTRUCTED)
      {
	ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      }

    // Get a pointer to the joint position control interface
    pos_iface_ = robot_hw->get<PositionJointInterface>();
    if (!pos_iface_)
      {
	ROS_WARN("This controller did not find  a hardware interface of type PositionJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required."
		 , lns.c_str());
      }

    // Get a pointer to the joint velocity control interface
    vel_iface_ = robot_hw->get<VelocityJointInterface>();
    if (!vel_iface_)
      {
	ROS_WARN("This controller did not find  a hardware interface of type VelocityJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required."
		 , lns.c_str());
      }

    // Return which resources are claimed by this controller
    pos_iface_->clearClaims();
    vel_iface_->clearClaims();

    if (! init())
      {
	ROS_ERROR("Failed to initialize sot-controller" );
	std :: cerr << "FAILED LOADING SOT CONTROLLER" << std::endl;
	return false ;
      }
    if (verbosity_level_>0)
      ROS_INFO_STREAM("Initialization of interfaces for sot-controller Ok !");

#ifdef CONTROLLER_INTERFACE_KINETIC
    hardware_interface::InterfaceResources iface_res;
    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<PositionJointInterface>();
    iface_res.resources = pos_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<VelocityJointInterface>();
    iface_res.resources = vel_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);

    pos_iface_->clearClaims();
    vel_iface_->clearClaims();
#else
    claimed_resources = pos_iface_->getClaims();
    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    pos_iface_->clearClaims();

    claimed_resources = vel_iface_->getClaims();
    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    vel_iface_->clearClaims();
#endif
    if (verbosity_level_>0)
      ROS_INFO_STREAM("Initialization of sot-controller Ok !");
    // success
    state_ = INITIALIZED;

    return true;
  }

  bool TiagoRosControlTest::
  init()
  {
    if (!initJoints())
      return false;
    return true;
  }

  void TiagoRosControlTest::
  readParamsVerbosityLevel(ros::NodeHandle &robot_nh)
  {
    if (robot_nh.hasParam("/sot_controller/verbosity_level"))
      {
	robot_nh.getParam("/sot_controller/verbosity_level",verbosity_level_);
	ROS_INFO_STREAM("Verbosity_level " << verbosity_level_);
      }
  }

  bool TiagoRosControlTest::
  readParamsPositionControlData(ros::NodeHandle &)
  {
    return false;
  }

  bool TiagoRosControlTest::
  readParamsVelocityControlPDMotorControlData(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/sot_controller/velocity_control_pd_motor_init/gains"))
      {
       XmlRpc::XmlRpcValue xml_rpc_ecpd_init;
       robot_nh.getParamCached("/sot_controller/velocity_control_pd_motor_init/gains",
                               xml_rpc_ecpd_init);

       /// Display gain during transition control.
       if (verbosity_level_>0)
    	 ROS_INFO("/sot_controller/velocity_control_pd_motor_init/gains: %d %d %d\n",
    		  xml_rpc_ecpd_init.getType(),
		  XmlRpc::XmlRpcValue::TypeArray,
		  XmlRpc::XmlRpcValue::TypeStruct);

       velocity_mode_pd_motors_.clear();

       for (size_t i=0;i<joints_name_.size();i++)
         {
	   // Check if the joint should be in ROS VELOCITY mode
	   std::map<std::string,JointSotHandle>::iterator
	     search_joint_sot_handle = joints_.find(joints_name_[i]);
	   if (search_joint_sot_handle!=joints_.end())
	     {
	       JointSotHandle aJointSotHandle = search_joint_sot_handle->second;
	       if (aJointSotHandle.ros_control_mode==VELOCITY)
		 {
		   // Test if PID data is present
		   if (xml_rpc_ecpd_init.hasMember(joints_name_[i]))
		     {
		       std::string prefix=
			 "/sot_controller/velocity_control_pd_motor_init/gains/"
			 + joints_name_[i];
		       velocity_mode_pd_motors_[joints_name_[i]].
			 read_from_xmlrpc_value(prefix);
		     }
		   else
		     ROS_ERROR("No PID data for velocity controlled joint %s in /sot_controller/velocity_control_pd_motor_init/gains/",
			       joints_name_[i].c_str());
		 }
	     }

         }
       return true;
      }

    ROS_ERROR("No parameter /sot_controller/velocity_controler_pd_motor_init");
    return false;
  }

  bool TiagoRosControlTest::
  readParamsJointNames(ros::NodeHandle &robot_nh)
  {
    /// Check if the /sot_controller/joint_names parameter exists.
    if (robot_nh.hasParam("/sot_controller/joint_names"))
      {
    	/// Read the joint_names list from this parameter
    	robot_nh.getParam("/sot_controller/joint_names",
    			  joints_name_);
    	for(std::vector<std::string>::size_type i=0;i<joints_name_.size();i++)
    	  {
    	    if (verbosity_level_>0)
    	      ROS_INFO_STREAM("joints_name_[" << i << "]=" << joints_name_[i]);
	  }
      }
    else
      return false;

    return true;
  }

  bool TiagoRosControlTest::
  getJointControlMode(std::string &joint_name,
		      JointSotHandle &aJointSotHandle)
  {
    std::string scontrol_mode;
    static const std::string svelocity("VELOCITY"),sposition("POSITION");
    static const std::string ros_control_mode = "ros_control_mode";

    /// Read the list of control_mode
    ros::NodeHandle rnh_ns("/sot_controller/control_mode/"+joint_name);

    ControlMode joint_control_mode;
    if (!rnh_ns.getParam(ros_control_mode,scontrol_mode))
      {
        ROS_ERROR("No %s for %s - We found %s",
    	      ros_control_mode.c_str(),
    	      joint_name.c_str(),
    	      scontrol_mode.c_str());
        return false;
      }

    if      (scontrol_mode==sposition)
      joint_control_mode=POSITION;
    else if (scontrol_mode==svelocity)
      joint_control_mode=VELOCITY;
    else {
      ROS_ERROR("%s for %s not understood. Expected %s or %s. Got %s",
    	    ros_control_mode.c_str(),
    	    joint_name.c_str(),
                sposition.c_str(),
                svelocity.c_str(),
    	    scontrol_mode.c_str());
      return false;
    }

    aJointSotHandle.ros_control_mode = joint_control_mode;
    //aJointSotHandle.sot_control_mode = joint_control_mode;

    return true;
  }

  bool TiagoRosControlTest::
  readParamsControlMode(ros::NodeHandle &robot_nh)
  {
    std::map<std::string,std::string> mapControlMode;

    // Read param from control_mode.
    if (robot_nh.hasParam("/sot_controller/control_mode"))
      {
	/// For each listed joint
	for(unsigned int idJoint=0;idJoint<joints_name_.size();idJoint++)
	  {
	    std::string joint_name = joints_name_[idJoint];
	    JointSotHandle &aJoint = joints_[joint_name];
	    if (!getJointControlMode(joint_name,aJoint))
	      return false;
	    ROS_INFO("joint_name[%d]=%s, control_mode=%d",
                idJoint,joint_name.c_str(),aJoint.ros_control_mode);
	  }
      }
    else
      {
	ROS_INFO_STREAM("Default control mode : position");
      }
    /// Always return true;
    return true;
  }

  bool TiagoRosControlTest::
  readParams(ros::NodeHandle &robot_nh)
  {
    /// Read the level of verbosity for the controller (0: quiet, 1: info, 2: debug).
    /// Default to quiet
    readParamsVerbosityLevel(robot_nh);

    /// Read /sot_controller/simulation_mode to know if we are in simulation mode
    // Defines if we are in simulation node.
    if (robot_nh.hasParam("/sot_controller/simulation_mode"))
      simulation_mode_ = true;

    /// Calls readParamsJointNames
    // Reads the list of joints to be controlled.
    if (!readParamsJointNames(robot_nh))
      return false;

    /// Calls readParamsControlMode.
    // Defines if the control mode is position or velocity
    if (!readParamsControlMode(robot_nh))
      return false;

    readParamsVelocityControlPDMotorControlData(robot_nh);
    readParamsPositionControlData(robot_nh);
    return true;
  }

  bool TiagoRosControlTest::
  initJoints()
  {
    // Init Joint Names.
    //    joints_.resize(joints_name_.size());


    for (unsigned int i=0;i<joints_name_.size();i++)
      {
	bool notok=true;

	while (notok)
	  {
	    std::string &joint_name = joints_name_[i];
	    try
	      {
		JointSotHandle &aJointSotHandle = joints_[joint_name];
                switch (aJointSotHandle.ros_control_mode)
                {
                  case POSITION:
		    aJointSotHandle.joint = pos_iface_->getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name << " in position "
				      << i << " " << aJointSotHandle.joint.getName());
                    break;
                  case VELOCITY:
		    aJointSotHandle.joint = vel_iface_->getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name << " in velocity "
				      << i << " " << aJointSotHandle.joint.getName());
                    break;
                }

		// throws on failure
		notok=false;
		aJointSotHandle.desired_init_pose =
		  aJointSotHandle.joint.getPosition();

	      }
	    catch (...)
	      {
		ROS_ERROR_STREAM("Could not find joint "
				 << joint_name);
		return false;
	      }
	  }
      }

    return true ;

  }

  void TiagoRosControlTest::
  localStandbyVelocityControlMode(const ros::Duration& period)
  {
    static bool first_time=true;

    /// Iterate over all the joints
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
        /// Find the joint
        std::string joint_name = joints_name_[idJoint];
	std::map<std::string,ControlPDMotorControlData>::iterator
	  search_ecpd = velocity_mode_pd_motors_.find(joint_name);

	if (search_ecpd!=velocity_mode_pd_motors_.end())
	  {
	    ControlPDMotorControlData & ecpdcdata = search_ecpd->second;
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;

	    double vel_err = 0 - aJoint.getVelocity();
	    double err = aJointSotHandle.desired_init_pose - aJoint.getPosition();

	    double local_command = ecpdcdata.pid_controller.computeCommand(err,vel_err,period);

	    aJoint.setCommand(local_command);

	    assert(aJoint.getName() == joint_name);
	    if (first_time)
	      if (verbosity_level_>1) {
		ROS_INFO("Control joint %s (id %d) to %f\n",
			 joint_name.c_str(),idJoint,
			 aJoint.getPosition());
	      }
	  }
      }
    first_time=false;
  }

  void TiagoRosControlTest::
  localStandbyPositionControlMode()
  {
    static bool first_time=true;

    /// Iterate over all the joints
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
        /// Find the joint
        std::string joint_name = joints_name_[idJoint];

	// If it is position mode control.
	if (joints_[joint_name].ros_control_mode==POSITION)
	  {
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;

	    aJoint.setCommand(aJointSotHandle.desired_init_pose);

	    assert(aJoint.getName() == joint_name);
	    if (first_time)
	      if (verbosity_level_>1) {
		ROS_INFO("Control joint %s (id %d) to %f\n",
			 joint_name.c_str(),idJoint,
			 aJoint.getPosition());
	      }
	  }
      }
    first_time=false;
  }

  void TiagoRosControlTest::
  update(const ros::Time&, const ros::Duration& period)
   {
     // But in velocity mode it means that we are sending 0
     // Therefore implements a default PD controller on the system.
     // Applying both to handle mixed system.
     localStandbyVelocityControlMode(period);
     localStandbyPositionControlMode();
   }

  void TiagoRosControlTest::
  starting(const ros::Time &)
  {
  }

  void TiagoRosControlTest::
  stopping(const ros::Time &)
  {
  }


  PLUGINLIB_EXPORT_CLASS(tiago_roscontrol_test::TiagoRosControlTest,
			 lci::ControllerBase)
}
