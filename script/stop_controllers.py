import rospy

from controller_manager_msgs.srv import SwitchController,UnloadController

stop_controllers=['arm_controller','head_controller','torso_controller','gripper_controller','mobile_base_controller']
rospy.init_node('StoppingControllers')
swCtl=rospy.ServiceProxy('/controller_manager/switch_controller',SwitchController)
resp = swCtl(stop_controllers=stop_controllers,strictness=2)
print(resp)
unloadCtl=rospy.ServiceProxy('/controller_manager/unload_controller',UnloadController)
for unload_controller in stop_controllers:
  resp = unloadCtl(unload_controller)
  print(resp)
