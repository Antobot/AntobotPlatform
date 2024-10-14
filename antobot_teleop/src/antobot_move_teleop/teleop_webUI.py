#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

################################################################################################################
### Functional Description:     Teleoperation class for webUI
################################################################################################################

import rospy
from geometry_msgs.msg import Twist
import time # Use system time as ROS time can lag, and this is a human interface

from antobot_manager_msgs.srv import jobManagerUserInput, jobManagerUserInputResponse

class teleop_webUI:
    def __init__(self):

        # All teleop classes require a cmd_vel
        self.cmd_vel = Twist()

        # The cmd_vel data is taken directly from the webUI topic
        self.webUI_cmd_vel_sub = rospy.Subscriber("/webUI/cmd_vel", Twist, self.cmd_vel_callback)  # subscriber for Joy message

        # The default last update to prevent the data being used
        self.lastUpdate=0

        # Create a service to allow webUI to send trigger commands. These do not go directly to allow webUI to be cutoff by local user.
        self.serviceUserInput = rospy.Service('/antobot/teleop/webUIinput', jobManagerUserInput, self.__serviceCallbackUserInput)

        # Map variables to each button that should trigger a function
        self.trigger_active=False   # Triggers the teleop function to use this mode
        self.trigger_pause=False
        self.trigger_resume=False
        self.trigger_abort=False
        self.trigger_goHome=False
        self.trigger_uvSwitch=False
        self.trigger_camSwitch = False
        self.trigger_recSwitch = False
        self.trigger_recoverJob=False
        self.trigger_rowFollowForwards=False
        self.trigger_rowFollowBackwards=False
        self.trigger_shutdown=False
        self.trigger_releaseForceStop=False # A way of triggering this is needed

        self.debug=False # Set to true to log webUI inputs

    def cmd_vel_callback(self, data):
        # Simply updates the cmd_vel variable to match incoming data from the webUI
        self.cmd_vel = data

        # Track when this was last updated
        self.lastUpdate = time.time()

        # All new webUI data triggers the mode to be active
        self.trigger_active=True

############################################################################################################

    def __serviceCallbackUserInput(self,request):
        """Captures direct user input from service"""


        # This should reactivate the webUI mode
        self.lastUpdate = time.time()


        # User inputs are int8, with 5 current supported values:
        # 0 - no input (default) - This is a test request only
        # 1 - Resume job (used for expected user confirmation states and to exit pause)
        # 2 - Pause job (stops robot task execution, allows for teleop, does not interupt logic execution, does not release robot)
        # 3 - Abort (cancels job, frees robot for pool. robot should be taken home by traffic manager)
        # 4 - Start default job - if a default job has been assigned this is a direct method to trigger this

        # Update the class variable
        self.__directUserInput=request.userInput
        self.__directUserInputSourceID=request.source







        # Each request has different and sometimes complex logic, handled by individual methods
        # Current returns only specify if a parameter was changed or not. This might be better if it includes more logic
        if self.__directUserInput == 0: # Test
            rospy.loginfo('Teleop WebUI: User requested test')
            returnString="Teleop WebUI: User requested test"
            returnBool=True
        elif self.__directUserInput == 1: # Resume
            rospy.loginfo('Teleop WebUI: User requested resume')
            self.trigger_resume=True
            returnString="Teleop WebUI: User requested resume"
            returnBool=True
        elif self.__directUserInput == 2: # Pause
            rospy.loginfo('Teleop WebUI: User requested pause')
            self.trigger_pause=True
            returnString="Teleop WebUI: User requested pause"
            returnBool=True
        elif self.__directUserInput == 3: # Abort
            rospy.loginfo('Teleop WebUI: User requested abort')
            self.trigger_abort=True
            returnString="Teleop WebUI: User requested abort"
            returnBool=True
        elif self.__directUserInput == 4: # Home
            rospy.loginfo('Teleop WebUI: User requested go home')
            self.trigger_goHome=True
            returnString="Teleop WebUI: User requested go home"
            returnBool=True

        ## 5 is reserved?

        elif self.__directUserInput == 6: # Job recovery
            rospy.loginfo('Teleop WebUI: User requested previous job recovery')
            self.trigger_recoverJob=True
            returnString="Teleop WebUI: User requested previous job recovery"
            returnBool=True


        ## WebUI only extensions - these don't get forwarded to jobManager

        elif self.__directUserInput == 7: # UV toggle
            rospy.loginfo('Teleop WebUI: User requested UV light toggle')
            self.trigger_uvSwitch=True
            returnString="Teleop WebUI: User requested UV light toggle"
            returnBool=True

        elif self.__directUserInput == 8: # Release safety stop
            rospy.loginfo('Teleop WebUI: User requested to release safety stop')
            self.trigger_releaseForceStop=True
            returnString="Teleop WebUI: User requested to release safety stop"
            returnBool=True

        elif self.__directUserInput == 9: # Row follow forward
            rospy.loginfo('Teleop WebUI: User requested standalone row following - forwards')
            self.trigger_rowFollowForwards=True
            returnString="Teleop WebUI: User requested standalone row following - forwards"
            returnBool=True

        elif self.__directUserInput == 10: # Row follow backwards
            rospy.loginfo('Teleop WebUI: User requested standalone row following - backwards')
            self.trigger_rowFollowBackwards=True
            returnString="Teleop WebUI: User requested standalone row following - backwards"
            returnBool=True

        elif self.__directUserInput == 11: # Shutdown
            rospy.loginfo('Teleop WebUI: User requested shutdown')
            self.trigger_shutdown=True
            returnString="Teleop WebUI: User requested shutdown"
            returnBool=True

        else: # Unknown
            rospy.loginfo('Teleop WebUI: User request unknown')
            returnString="Teleop WebUI: User request unknown"
            returnBool=False

        # Unset the user input by reverting back to default
        self.__directUserInput == 0


        # Print and send a response to the client
        returnMessage = jobManagerUserInputResponse()
        returnMessage.responseBool=returnBool
        returnMessage.responseString=returnString
        #rospy.loginfo('SW2002: JobManager: User return string - '+ returnString)
        return returnMessage
        
############################################################################################################