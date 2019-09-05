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

using namespace rc_sot_system;

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
    verbosity_level_(0),
    ltime_(0)
  {
    RESETDEBUG4();
    profileLog_.length=300000;
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

  void TiagoRosControlTest::initLogs()
  {
    ROS_INFO_STREAM("Initialize log data structure" << profileLog_.nbDofs);
    ROS_INFO_STREAM("Initialize log data structure" << profileLog_.nbForceSensors);


    /// Initialize the size of the data to store.
    ///
    /// Set temporary profileLog to one
    /// because DataOneIter is just for one iteration.
    size_t tmp_length = profileLog_.length;
    profileLog_.length = 1;
    DataOneIter_.init(profileLog_);

    /// Set profile Log to real good value for the stored data.
    profileLog_.length= tmp_length;
    /// Initialize the data logger for 300s.
//    RcSotLog_.init(profileLog_);

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

    /// Create all the internal data structures for logging.
    initLogs();

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
	ROS_ERROR("Cannot initialize this controller because "
                  "it failed to be constructed");
      }

    // Get a pointer to the joint position control interface
    posvel_iface_ = robot_hw->get<PosVelJointInterface>();
    if (!pos_iface_)
      {
	ROS_WARN("This controller did not find  a hardware "
                 "interface of type PositionJointInterface."
		 " Make sure this is registered in the %s::"
                 "RobotHW class if it is required."
		 , lns.c_str());
      }

    
    
    // Get a pointer to the joint velocity control interface
    vel_iface_ = robot_hw->get<VelocityJointInterface>();
    if (!vel_iface_)
      {
	ROS_WARN("This controller did not find  a hardware "
                 " interface of type VelocityJointInterface."
		 " Make sure this is registered in the %s::RobotHW "
                 "class if it is required."
		 , lns.c_str());
      }

    // Get a pointer to the joint position and velocity control interface
    posvel_iface_ = robot_hw->get<PosVelJointInterface>();
    if (!posvel_iface_)
    {
      ROS_WARN("This controller did not find  a hardware interface of type PosVelJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required."
		 , lns.c_str());
    }

    // Get a pointer to the joint effort control interface
    effort_iface_ = robot_hw->get<EffortJointInterface>();
    if (! effort_iface_)
      {
	ROS_WARN("This controller did not find a hardware interface of type EffortJointInterface."
		 " Make sure this is registered in the %s::RobotHW class if it is required.",
		 lns.c_str());
      }

    // Get a pointer to the force-torque sensor interface
    ft_iface_ = robot_hw->get<ForceTorqueSensorInterface>();
    if (! ft_iface_ )
      {
	ROS_WARN("This controller did not find a hardware interface of type '%s '. "
		 " Make sure this is registered inthe %s::RobotHW class if it is required.",
		  internal :: demangledTypeName<ForceTorqueSensorInterface>().c_str(),lns.c_str());
      }

    // Get a pointer to the IMU sensor interface
    imu_iface_ = robot_hw->get<ImuSensorInterface>();
    if (! imu_iface_)
      {
	ROS_WARN("This controller did not find a hardware interface of type '%s'."
		 " Make sure this is registered in the %s::RobotHW class if it is required.",
		    internal :: demangledTypeName<ImuSensorInterface>().c_str(),lns.c_str());
      }

    // Temperature sensor not available in simulation mode
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
	// Get a pointer to the actuator temperature sensor interface
	act_temp_iface_ = robot_hw->get<ActuatorTemperatureSensorInterface>();
	if (!act_temp_iface_)
	  {
	    ROS_WARN("This controller did not find a hardware interface of type '%s'."
		     " Make sure this is registered in the %s::RobotHW class if it is required.",
		      internal :: demangledTypeName<ActuatorTemperatureSensorInterface>().c_str(),lns.c_str());
	  }
#endif
      }


    // Return which resources are claimed by this controller
    pos_iface_->clearClaims();
    vel_iface_->clearClaims();
    effort_iface_->clearClaims();

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

    iface_res.hardware_interface =
      hardware_interface::internal::
      demangledTypeName<EffortJointInterface>();
    iface_res.resources = effort_iface_->getClaims();
    claimed_resources.push_back(iface_res);

    /// Display claimed ressources
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);

    pos_iface_->clearClaims();
    vel_iface_->clearClaims();
    effort_iface_->clearClaims();
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

    claimed_resources = effort_iface_->getClaims();
    if (verbosity_level_>0)
      displayClaimedResources(claimed_resources);
    effort_iface_->clearClaims();
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
    if (!initIMU()) {
      ROS_WARN("could not initialize IMU sensor(s).");
    }
    if (!initForceSensors()) {
      ROS_WARN("could not initialize force sensor(s).");
    }
    if (!initTemperatureSensors()) {
      ROS_WARN("could not initialize temperature sensor(s).");
    }

    return true;
  }

  void TiagoRosControlTest::
  readParamsVerbosityLevel(ros::NodeHandle &robot_nh)
  {
    if (robot_nh.hasParam("/tiago_roscontrol_test/verbosity_level"))
      {
	robot_nh.getParam("/tiago_roscontrol_test/verbosity_level",verbosity_level_);
	ROS_INFO_STREAM("Verbosity_level " << verbosity_level_);
      }
    if (robot_nh.hasParam("/tiago_roscontrol_test/log/size"))
      {
	int llength;
	robot_nh.getParam("/tiago_roscontrol_test/log/size",llength);
	profileLog_.length=(unsigned int)llength;
	ROS_INFO_STREAM("Size of the log " << profileLog_.length);
      }

  }

  bool TiagoRosControlTest::
  readParamsPositionControlData(ros::NodeHandle &)
  {
    return false;
  }

  bool TiagoRosControlTest::
  readParamsEffortControlPDMotorControlData(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/tiago_roscontrol_test/effort_control_pd_motor_init/gains"))
      {
       XmlRpc::XmlRpcValue xml_rpc_ecpd_init;
       robot_nh.getParamCached("/tiago_roscontrol_test/effort_control_pd_motor_init/gains",
                               xml_rpc_ecpd_init);

       /// Display gain during transition control.
       if (verbosity_level_>0)
    	 ROS_INFO("/tiago_roscontrol_test/effort_control_pd_motor_init/gains: %d %d %d\n",
    		  xml_rpc_ecpd_init.getType(),
		  XmlRpc::XmlRpcValue::TypeArray,
		  XmlRpc::XmlRpcValue::TypeStruct);

       effort_mode_pd_motors_.clear();

       for (size_t i=0;i<joints_name_.size();i++)
         {
	   // Check if the joint should be in ROS EFFORT mode
	   std::map<std::string,JointSotHandle>::iterator
	     search_joint_sot_handle = joints_.find(joints_name_[i]);
	   if (search_joint_sot_handle!=joints_.end())
	     {
	       JointSotHandle aJointSotHandle = search_joint_sot_handle->second;
	       if (aJointSotHandle.ros_control_mode==EFFORT)
		 {
		   // Test if PID data is present
		   if (xml_rpc_ecpd_init.hasMember(joints_name_[i]))
		     {
		       std::string prefix=
			 "/tiago_roscontrol_test/effort_control_pd_motor_init/gains/"
			 + joints_name_[i];
		       effort_mode_pd_motors_[joints_name_[i]].
			 read_from_xmlrpc_value(prefix);
		     }
		   else
		     ROS_ERROR("No PID data for effort controlled joint %s in /tiago_roscontrol_test/effort_control_pd_motor_init/gains/",
			       joints_name_[i].c_str());
		 }
	     }

         }
       return true;
      }

    ROS_WARN("No parameter /tiago_roscontrol_test/effort_controler_pd_motor_init");
    return false;
  }

  bool TiagoRosControlTest::
  readParamsVelocityControlPDMotorControlData(ros::NodeHandle &robot_nh)
  {
    // Read libname
    if (robot_nh.hasParam("/tiago_roscontrol_test/velocity_control_pd_motor_init/gains"))
      {
       XmlRpc::XmlRpcValue xml_rpc_ecpd_init;
       robot_nh.getParamCached("/tiago_roscontrol_test/velocity_control_pd_motor_init/gains",
                               xml_rpc_ecpd_init);

       /// Display gain during transition control.
       if (verbosity_level_>0)
    	 ROS_INFO("/tiago_roscontrol_test/velocity_control_pd_motor_init/gains: %d %d %d\n",
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
			 "/tiago_roscontrol_test/velocity_control_pd_motor_init/gains/"
			 + joints_name_[i];
		       velocity_mode_pd_motors_[joints_name_[i]].
			 read_from_xmlrpc_value(prefix);
		     }
		   else
		     ROS_ERROR("No PID data for velocity controlled joint %s in /tiago_roscontrol_test/velocity_control_pd_motor_init/gains/",
			       joints_name_[i].c_str());
		 }
	     }

         }
       return true;
      }

    ROS_WARN("No parameter /tiago_roscontrol_test/velocity_controler_pd_motor_init");
    return false;
  }

  bool TiagoRosControlTest::
  readParamsJointNames(ros::NodeHandle &robot_nh)
  {
    /// Check if the /tiago_roscontrol_test/joint_names parameter exists.
    if (robot_nh.hasParam("/tiago_roscontrol_test/joint_names"))
      {
    	/// Read the joint_names list from this parameter
    	robot_nh.getParam("/tiago_roscontrol_test/joint_names",
    			  joints_name_);
    	for(std::vector<std::string>::size_type i=0;i<joints_name_.size();i++)
    	  {
    	    if (verbosity_level_>0)
    	      ROS_INFO_STREAM("joints_name_[" << i << "]=" << joints_name_[i]);

    	    if (modelURDF_.use_count())
    	      {
    		urdf::JointConstSharedPtr aJCSP = modelURDF_->getJoint(joints_name_[i]);
    		if (aJCSP.use_count()!=0)
    		  {
    		    if (verbosity_level_>0)
    		      ROS_INFO_STREAM( joints_name_[i] + " found in the robot model" );
    		  }
    		else
    		  {
    		    ROS_ERROR(" %s not found in the robot model",joints_name_[i].c_str());
    		    return false;
    		  }
    	      }
    	    else
    	      {
    		ROS_ERROR("No robot model loaded in /robot_description");
    		return false;
	      }
	  }
      }
    else
      return false;

    /// Deduce from this the degree of freedom number.
    nbDofs_ = joints_name_.size();
    profileLog_.nbDofs = nbDofs_;

    return true;
  }

  bool TiagoRosControlTest::
  getJointControlMode(std::string &joint_name,
		      JointSotHandle &aJointSotHandle)
  {
    std::string scontrol_mode;
    static const std::string seffort("EFFORT"),svelocity("VELOCITY"),sposition("POSITION");
    static const std::string spositionwvelocity("POSITIONWVELOCITY");
    static const std::string ros_control_mode = "ros_control_mode";

    /// Read the list of control_mode
    ros::NodeHandle rnh_ns("/tiago_roscontrol_test/control_mode/"+joint_name);

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
    else if (scontrol_mode==spositionwvelocity)
      joint_control_mode=POSITIONWVELOCITY;
    else if (scontrol_mode==svelocity)
      joint_control_mode=VELOCITY;
    else if (scontrol_mode==seffort)
      joint_control_mode=EFFORT;
    else {
      ROS_ERROR("%s for %s not understood. Expected %s, %s or %s. Got %s",
    	    ros_control_mode.c_str(),
    	    joint_name.c_str(),
                sposition.c_str(),
                svelocity.c_str(),
                seffort.c_str(),
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
    if (robot_nh.hasParam("/tiago_roscontrol_test/control_mode"))
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
  readUrdf(ros::NodeHandle &robot_nh)
  {
    /// Reading the parameter /robot_description which contains the robot
    /// description
    if (!robot_nh.hasParam("/robot_description"))
      {
	ROS_ERROR("ROS application does not have robot_description");
	return false;
      }
    std::string robot_description_str;

    robot_nh.getParam("/robot_description",robot_description_str);

    modelURDF_ = urdf::parseURDF(robot_description_str);
    if (verbosity_level_>0)
      ROS_INFO("Loaded /robot_description %ld",modelURDF_.use_count());
    return true;
  }

  bool TiagoRosControlTest::
  readParams(ros::NodeHandle &robot_nh)
  {
    /// Read the level of verbosity for the controller (0: quiet, 1: info, 2: debug).
    /// Default to quiet
    readParamsVerbosityLevel(robot_nh);

    /// Read /tiago_roscontrol_test/simulation_mode to know if we are in simulation mode
    // Defines if we are in simulation node.
    if (robot_nh.hasParam("/tiago_roscontrol_test/simulation_mode"))
      simulation_mode_ = true;

    /// Read URDF file.
    readUrdf(robot_nh);

    /// Calls readParamsJointNames
    // Reads the list of joints to be controlled.
    if (!readParamsJointNames(robot_nh))
      return false;

    /// Calls readParamsControlMode.
    // Defines if the control mode is position or velocity
    if (!readParamsControlMode(robot_nh))
      return false;

    readParamsEffortControlPDMotorControlData(robot_nh);
    readParamsVelocityControlPDMotorControlData(robot_nh);
    readParamsPositionControlData(robot_nh);
    return true;
  }

  bool TiagoRosControlTest::
  initJoints()
  {
    // Init Joint Names.
    //    joints_.resize(joints_name_.size());


    for (unsigned int i=0;i<nbDofs_;i++)
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
		      ROS_INFO_STREAM("Found joint " << joint_name <<
                                      " in position "
				      << i << " " <<
                                      aJointSotHandle.joint.getName());
                    break;
                  case POSITIONWVELOCITY:
		    aJointSotHandle.posvelH = posvel_iface_->
                        getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name <<
                                      " in velocity "
				      << i << " " <<
                                      aJointSotHandle.joint.getName());
                    
                  case VELOCITY:
		    aJointSotHandle.joint = vel_iface_->getHandle(joint_name);
		    if (verbosity_level_>0)
		      ROS_INFO_STREAM("Found joint " << joint_name <<
                                      " in velocity " << i << " " <<
                                      aJointSotHandle.joint.getName());
                    break;
                  case EFFORT:
                    aJointSotHandle.joint = effort_iface_->
                        getHandle(joint_name);
                    if (verbosity_level_>0)
                      ROS_INFO_STREAM("Found joint " << joint_name <<
                                      " in effort "
                                      << i << " "
                                      << aJointSotHandle.joint.getName());
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

  bool TiagoRosControlTest::
  initIMU()
  {
    if (!imu_iface_) return false;

    // get all imu sensor names
    const std :: vector<std :: string >& imu_iface_names = imu_iface_->getNames();
    if (verbosity_level_>0)
      {
	for (unsigned i=0; i <imu_iface_names.size(); i++)
	  ROS_INFO("Got sensor %s", imu_iface_names[i].c_str());
      }
    for (unsigned i=0; i <imu_iface_names.size(); i++){
      // sensor handle on imu
      imu_sensor_.push_back(imu_iface_->getHandle(imu_iface_names[i]));
    }

    return true ;
  }

  bool TiagoRosControlTest::
  initForceSensors()
  {
    profileLog_.nbForceSensors = 0.0;
    if (!ft_iface_) return false;

    // get force torque sensors names package.
    const std::vector<std::string>& ft_iface_names = ft_iface_->getNames();
    if (verbosity_level_>0)
      {
	for (unsigned i=0; i <ft_iface_names.size(); i++)
	  ROS_INFO("Got sensor %s", ft_iface_names[i].c_str());
      }
    for (unsigned i=0; i <ft_iface_names.size(); i++){
      // sensor handle on torque forces
      ft_sensors_.push_back(ft_iface_->getHandle(ft_iface_names[i]));
    }
    profileLog_.nbForceSensors = ft_iface_names.size();
    return true;
  }

  bool TiagoRosControlTest::
  initTemperatureSensors()
  {
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
        if (!act_temp_iface_) return false;

	// get temperature sensors names
	const std::vector<std::string>& act_temp_iface_names = act_temp_iface_->getNames();

	if (verbosity_level_>0)
	  {
	    ROS_INFO("Actuator temperature sensors: %ld",act_temp_iface_names.size() );

	    for (unsigned i=0; i <act_temp_iface_names.size(); i++)
	      ROS_INFO("Got sensor %s", act_temp_iface_names[i].c_str());
	  }

	for (unsigned i=0; i <act_temp_iface_names.size(); i++){
	  // sensor handle on actuator temperature
	  act_temp_sensors_.push_back(act_temp_iface_->getHandle(act_temp_iface_names[i]));
	}
#endif
      }

    return true;
  }

  void TiagoRosControlTest::
  fillSensorsIn(std::string &title, std::vector<double> & data)
  {
    (void) title;
    (void) data;
    /*
    /// Tries to find the mapping from the local validation
    /// to the SoT device.
    it_map_rt_to_sot it_mapRC2Sot= mapFromRCToSotDevice_.find(title);
    /// If the mapping is found
    if (it_mapRC2Sot!=mapFromRCToSotDevice_.end())
      {
	/// Expose the data to the SoT device.
	std::string lmapRC2Sot = it_mapRC2Sot->second;
	sensorsIn_[lmapRC2Sot].setName(lmapRC2Sot);
	sensorsIn_[lmapRC2Sot].setValues(data);
      }
    */
  }

  void TiagoRosControlTest::
  fillJoints()
  {

    /// Fill positions, velocities and torques.
    for(unsigned int idJoint=0;
	idJoint<joints_name_.size();
	idJoint++)
      {
	it_joint_sot_h anItJoint = joints_.find(joints_name_[idJoint]);
	if (anItJoint!=joints_.end())
	  {
	    JointSotHandle & aJoint = anItJoint->second;
	    DataOneIter_.motor_angle[idJoint] = aJoint.joint.getPosition();

#ifdef TEMPERATURE_SENSOR_CONTROLLER
	    DataOneIter_.joint_angle[idJoint] = aJoint.joint.getAbsolutePosition();
#endif
	    DataOneIter_.velocities[idJoint] = aJoint.joint.getVelocity();

#ifdef TEMPERATURE_SENSOR_CONTROLLER
	    DataOneIter_.torques[idJoint] = aJoint.joint.getTorqueSensor();
#endif
	    DataOneIter_.motor_currents[idJoint] = aJoint.joint.getEffort();
	  }
      }

    /// Update SoT internal values
    std::string ltitle("motor-angles");
    fillSensorsIn(ltitle,DataOneIter_.motor_angle);
    ltitle = "joint-angles";
    fillSensorsIn(ltitle,DataOneIter_.joint_angle);
    ltitle = "velocities";
    fillSensorsIn(ltitle,DataOneIter_.velocities);
    ltitle = "torques";
    fillSensorsIn(ltitle,DataOneIter_.torques);
    ltitle = "currents";
    fillSensorsIn(ltitle,DataOneIter_.motor_currents);

  }

  void TiagoRosControlTest::setSensorsImu(std::string &name,
					   int IMUnb,
					   std::vector<double> & data)
  {
    std::ostringstream labelOss;
    labelOss << name << IMUnb;
    std::string label_s = labelOss.str();
    //fillSensorsIn(label_s,data);
    (void) data;
  }

  void TiagoRosControlTest::
  fillImu()
  {
    for(unsigned int idIMU=0;idIMU<imu_sensor_.size();idIMU++)
      {
	/// Fill orientations, gyrometer and acceleration from IMU.
	if (imu_sensor_[idIMU].getOrientation())
	  {
	    for(unsigned int idquat = 0;idquat<4;idquat++)
	      {
		DataOneIter_.orientation[idquat] = imu_sensor_[idIMU].getOrientation ()[idquat];
	      }
	  }
	if (imu_sensor_[idIMU].getAngularVelocity())
	  {
	    for(unsigned int idgyrometer = 0;idgyrometer<3;
		idgyrometer++)
	      {
		DataOneIter_.gyrometer[idgyrometer] =
		  imu_sensor_[idIMU].getAngularVelocity()[idgyrometer];
	      }
	  }
	if (imu_sensor_[idIMU].getLinearAcceleration())
	  {
	    for(unsigned int idlinacc = 0;idlinacc<3;
		idlinacc++)
	      {
		DataOneIter_.accelerometer[idlinacc] =
		  imu_sensor_[idIMU].getLinearAcceleration()[idlinacc];
	      }
	  }

	std::string orientation_s("orientation_");
	setSensorsImu(orientation_s, idIMU, DataOneIter_.orientation);

	std::string gyrometer_s("gyrometer_");
	setSensorsImu(gyrometer_s, idIMU, DataOneIter_.gyrometer);

	std::string accelerometer_s("accelerometer_");
	setSensorsImu(accelerometer_s, idIMU, DataOneIter_.accelerometer);
      }
  }

  void TiagoRosControlTest::
  fillForceSensors()
  {
    for(unsigned int idFS=0;idFS<ft_sensors_.size();
	idFS++)
      {
	for(unsigned int idForce=0;idForce<3;idForce++)
	  DataOneIter_.force_sensors[idFS*6+idForce]=
	    ft_sensors_[idFS].getForce()[idForce];
	for(unsigned int idTorque=0;idTorque<3;idTorque++)
	  DataOneIter_.force_sensors[idFS*6+3+idTorque]=
	    ft_sensors_[idFS].getTorque()[idTorque];
      }


    std::string alabel("forces");
    fillSensorsIn(alabel,DataOneIter_.force_sensors);
  }

  void TiagoRosControlTest::
  fillTempSensors()
  {
    if (!simulation_mode_)
      {
#ifdef TEMPERATURE_SENSOR_CONTROLLER
	for(unsigned int idFS=0;idFS<act_temp_sensors_.size();idFS++)
	  {
	    DataOneIter_.temperatures[idFS]=  act_temp_sensors_[idFS].getValue();
	  }
#endif
      }
    else
      {
	for(unsigned int idFS=0;idFS<nbDofs_;idFS++)
	  DataOneIter_.temperatures[idFS]=  0.0;
      }

    std::string alabel("act-temp");
    fillSensorsIn(alabel,DataOneIter_.temperatures);
  }

  void TiagoRosControlTest::
  fillSensors()
  {
    fillJoints();
    fillImu();
    fillForceSensors();
    fillTempSensors();
  }

  void TiagoRosControlTest::
  localStandbyEffortControlMode(const ros::Duration& period)
  {
    // ROS_INFO("Compute command for effort mode: %d %d",joints_.size(),effort_mode_pd_motors_.size());
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
	std::string joint_name = joints_name_[idJoint];
	std::map<std::string,ControlPDMotorControlData>::iterator
	  search_ecpd = effort_mode_pd_motors_.find(joint_name);

	if (search_ecpd!=effort_mode_pd_motors_.end())
	  {
	    ControlPDMotorControlData & ecpdcdata = search_ecpd->second;
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::JointHandle &aJoint = aJointSotHandle.joint;

	    double vel_err = 0 - aJoint.getVelocity();
            double err = aJointSotHandle.desired_init_pose - aJoint.getPosition();

	    double local_command = ecpdcdata.pid_controller.computeCommand(err,vel_err,period);
	    // Apply command
	    aJoint.setCommand(local_command);
	  }
      }
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

            aJoint.setCommand(0.2);

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
  localStandbyPosVelControlMode(const ros::Time& time,const ros::Duration& period)
  {
    static bool first_time=true;

    /// Iterate over all the joints
    for(unsigned int idJoint=0;idJoint<joints_.size();idJoint++)
      {
        /// Find the joint
        std::string joint_name = joints_name_[idJoint];

        if (joints_[joint_name].ros_control_mode==POSITIONWVELOCITY)
	  {
	    JointSotHandle &aJointSotHandle = joints_[joint_name];
	    lhi::PosVelJointHandle & aJoint = aJointSotHandle.posvelH;
            
	    double err = aJointSotHandle.desired_init_pose -
                aJoint.getPosition();
            double t = (time-start_).toSec();
            double T = 5.; // Period in seconds.
            double A =0.025;
            double cmd_pos = aJointSotHandle.desired_init_pose +
                A*std::sin(2*M_PI*t/T);
            double cmd_vel = A*std::cos(2*M_PI*t/T)*2*M_PI/T;
            
            aJoint.setCommand(cmd_pos,cmd_vel);

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
  localStandbyPositionControlMode(const ros::Time& time, const ros::Duration& )
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

            if (joint_name == "torso_lift_joint") {
              double t = (time-start_).toSec();
              double T = 5.; // Period in seconds.
              double A = 0.025;
              double l = aJointSotHandle.desired_init_pose;
              double lM = std::max (l, 0.3);
              double lm = std::min (l, 0.002);

              if      (l + A > lM) {
                A = std::max (lM - l, 0.);
                aJointSotHandle.desired_init_pose -= 0.001;
              } else if (l - A < lm) {
                A = std::max (l - lm, 0.);
                aJointSotHandle.desired_init_pose += 0.001;
              }
              if (A < 0.001) {
                ROS_WARN ("Amplitude is lower than 1 millimeter.");
              }
              aJoint.setCommand(aJointSotHandle.desired_init_pose + A*std::sin(2 * M_PI * t / T));
            }
            else if (joint_name == "head_1_joint")
            {
              double t = (time-start_).toSec();
              double T = 5.; // Period in seconds.
              double A = 0.25;
              double l = aJointSotHandle.desired_init_pose;
              double lM = std::max (l, 1.0);
              double lm = std::min (l, -1.0);

              if      (l + A > lM) {
                A = std::max (lM - l, 0.);
                aJointSotHandle.desired_init_pose -= 0.001;
              } else if (l - A < lm) {
                A = std::max (l - lm, 0.);
                aJointSotHandle.desired_init_pose += 0.001;
              }
              if (A < 0.001) {
                ROS_WARN ("Amplitude is lower than 1 millimeter.");
              }
              aJoint.setCommand(aJointSotHandle.desired_init_pose +
                                A*std::sin(2 * M_PI * t / T));
            }
            else if (joint_name == "arm_6_joint")
            {
              double t = (time-start_).toSec();
              double T = 5.; // Period in seconds.
              double A = 0.25;
              double l = aJointSotHandle.desired_init_pose;
              double lM = std::max (l, 1.0);
              double lm = std::min (l, -1.0);

              if      (l + A > lM) {
                A = std::max (lM - l, 0.);
                aJointSotHandle.desired_init_pose -= 0.001;
              } else if (l - A < lm) {
                A = std::max (l - lm, 0.);
                aJointSotHandle.desired_init_pose += 0.001;
              }
              if (A < 0.001) {
                ROS_WARN ("Amplitude is lower than 1 millimeter.");
              }
              aJoint.setCommand(aJointSotHandle.desired_init_pose +
                                A*std::sin(2 * M_PI * t / T));
            }
            else
            {
              aJoint.setCommand(aJointSotHandle.desired_init_pose);
            }

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
  update(const ros::Time& time, const ros::Duration& period)
   {
     // But in velocity mode it means that we are sending 0
     /// Update the sensors.
     fillSensors();

     // Therefore implements a default PD controller on the system.
     // Applying both to handle mixed system.
     localStandbyEffortControlMode(period);
     localStandbyVelocityControlMode(period);
     localStandbyPositionControlMode(time, period);
     ltime_++;
   }

  void TiagoRosControlTest::
  starting(const ros::Time & time)
  {
    start_ = time;

    fillSensors();
  }

  void TiagoRosControlTest::
  stopping(const ros::Time &)
  {
    std::string afilename("/tmp/sot.log");
    //RcSotLog_.save(afilename);
  }


  PLUGINLIB_EXPORT_CLASS(tiago_roscontrol_test::TiagoRosControlTest,
			 lci::ControllerBase)
}
