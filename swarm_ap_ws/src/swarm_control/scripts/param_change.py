#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import ParamSet, ParamSetRequest, ParamSetResponse
from mavros_msgs.msg import ParamValue

class ParameterSetter:
    def __init__(self):
        rospy.init_node('parameter_setter', anonymous=True)

        rospy.wait_for_service('/d1/mavros/param/set')

        self.param_set_service = rospy.ServiceProxy('/d1/mavros/param/set', ParamSet)

    def set_parameter(self, param_name, value):
        if isinstance(value, float):
            param_value = ParamValue(real=value)
        elif isinstance(value, int):
            param_value = ParamValue(integer=value)
        else:
            rospy.logerr("Unsupported parameter type.")
            return False

        req = ParamSetRequest()
        req.param_id = param_name
        req.value = param_value

        try:
            resp = self.param_set_service(req)
            if resp.success:
                rospy.loginfo(f"Parameter '{param_name}' set to {value} successfully.")
                return True
            else:
                rospy.logerr(f"Failed to set parameter '{param_name}'.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

if __name__ == '__main__':
    try:
        param_setter = ParameterSetter()

        param_setter.set_parameter("SIM_GPS_NOISE", 0.5)  # Set SIM_GPS_NOISE to 0.5
        param_setter.set_parameter("SIM_GPS_GLITCH", 0.1) # Set SIM_GPS_GLITCH to 0.1

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
