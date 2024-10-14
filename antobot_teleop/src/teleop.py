#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description:     Creates a master program to allow for switching between our main control methods:
# # #                       1) Keyboard control (inherited from keyboard_teleop.Keyboard_Teleop) and 2) Joystick control (receives messages from teleop_joy)


# Owner: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
import yaml
import json
import sys
import os
import time

from std_msgs.msg import Int8, UInt8, Empty, Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from pathlib import Path


# Import the different methods of input
# Each method should provide a /cmd_vel output by default
from antobot_move_teleop.teleop_joystick import teleop_joystick
from antobot_move_teleop.teleop_keyboard import teleop_keyboard
from antobot_move_teleop.teleop_webUI import teleop_webUI


from antobot_move_cartograph.navigatorLoadXMLClient import *
from antobot_manager_jobs.jobManagerAssignJobClient import *
from antobot_manager_jobs.jobManagerUserInputClient import *

# from antobot_manager_jobs.uvManagerClient import uvManagerClient
from antobot_devices_camera.camManagerClient import camManagerClient
from antobot_devices_msgs.srv import recManagerStartRec, recManagerStartRecResponse


####################################################################################

# Needs sorting

parent_folder = Path(__file__).resolve().parent.parent

path = str(parent_folder) + "/config/robot_job.yaml"

with open(path, 'r') as yamlfile:
    # data = yaml.load(yamlfile, Loader=yaml.FullLoader)
    data = yaml.safe_load(yamlfile)

    saved_routes_dir = data['saved_routes_dir']
    xml_file = data['xml_file']
    job_file = data['job_file']


####################################################################################


class MasterTeleop:
    def __init__(self):

        # Create classes for each input method
        self.teleop_method=dict()
        self.teleop_method[0] = teleop_joystick()
        self.teleop_method[1] = teleop_keyboard()
        self.teleop_method[2] = teleop_webUI()
        self.teleop_mode=None # No method is the default

        # Set a timeout, after which a teleop input is no longer considered active
        self.inputTimeout=10 # seconds

        # Set whether the standalone row following functions can be called - should not be true for customer robots!
        self.allowStandaloneRowFollowing=True

        # Set default variable states
        self.pdu_uvState = 0 # PDU data on UV lights state (on or off)
        self.record_status = False
        
        # Set default output states
        self.force_stop_release = False
        self.soft_shutdown = False
        self.manual_intervention = False

        # Set states for master teleop
        self.trigger_active=False # Tracks if the teleop type is active
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
        self.trigger_releaseForceStop=False

        # Create an empty cmd_vel twist
        self.teleop_cmd_vel=Twist()

        # Create service clients
        self.job_client = assignJobClient(sourceID='teleop') # Labelling the request as coming from cartographer
        self.user_client = directUserInputClient(userInput=0, sourceID='teleop') 
        self.uvClient = uvManagerClient(uv_num=3, command=0)
        self.camClient = camManagerClient()
        self.start_recording_client = rospy.ServiceProxy('/antobot/manager/recManagerStart',recManagerStartRec)
        self.standAlone_client=directUserInputClient(userInput=None,serviceName = '/antobot/pathfollower_row/startStandalone', sourceID='teleop') 




        self.service_started = False
    
        self.row_follow_pub = rospy.Publisher('/row_following', Empty, queue_size=1) # Start and stop row following
        self.force_stop_release_pub = rospy.Publisher('/antobridge/force_stop_release', Bool, queue_size=1)  # publisher to release the force stop on Aurix
        self.manual_intervention_pub = rospy.Publisher('/joystick/manual_intervention',Bool,queue_size =1)
        self.soft_shutdown_pub = rospy.Publisher('/antobridge/soft_shutdown_button',Bool,queue_size=1) # publisher to send the soft shutdown signal to antobridge

        # Publisher for physical robot movement
        self.teleop_cmd_vel_pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=5)

        self.row_follow_activated = False
        self.row_follow_direction = True # True:forward False:backward
        self.row_activated_time = time.time()

####################################################################################

    def update(self):


        # The keyboard mode is currently the only mode that doesnt self update. Request the currently held keys each turn
        # teleop_keyboard can only handle a single key press at once
        self.teleop_method[1].update()

        # Get the current time and compare to when each of the teleop methods were last updated
        now=time.time()

        # Reset the teleop_mode
        self.teleop_mode=None

        # The priority order is established here
        # Priority is given to joystick > terminal(keyboard) > webUI > mqtt (not implemented)
        # If a mode has timed out, deactivate it. DO NOT ACTIVATE MODES HERE - that is manually triggered by each teleop method

        if (now-self.teleop_method[0].lastUpdate)<self.inputTimeout and (self.teleop_method[0].lastUpdate - self.row_activated_time > 1.0) :
            self.teleop_mode=0 # Set the index for the active method
            self.teleop_method[1].trigger_active=False # De-activate the other methods
            self.teleop_method[2].trigger_active=False

        elif (now-self.teleop_method[1].lastUpdate)<self.inputTimeout:
            self.teleop_mode=1 # Set the index for this method
            self.teleop_method[0].trigger_active=False # De-activate the other methods
            self.teleop_method[2].trigger_active=False

        elif (now-self.teleop_method[2].lastUpdate)<self.inputTimeout:
            self.teleop_mode=2 # Set the index for this method
            self.teleop_method[0].trigger_active=False # De-activate the other methods
            self.teleop_method[1].trigger_active=False
        else: # De-activate all methods
            self.teleop_method[0].trigger_active=False
            self.teleop_method[1].trigger_active=False
            self.teleop_method[2].trigger_active=False



        # Update the triggers and cmd_vel
        if self.teleop_mode is not None:

            # Update the master triggers based on the individual method being used
            self.trigger_active=self.teleop_method[self.teleop_mode].trigger_active
            self.trigger_pause=self.teleop_method[self.teleop_mode].trigger_pause
            self.trigger_resume=self.teleop_method[self.teleop_mode].trigger_resume
            self.trigger_abort=self.teleop_method[self.teleop_mode].trigger_abort
            self.trigger_goHome=self.teleop_method[self.teleop_mode].trigger_goHome
            self.trigger_uvSwitch=self.teleop_method[self.teleop_mode].trigger_uvSwitch
            self.trigger_camSwitch = self.teleop_method[self.teleop_mode].trigger_camSwitch
            self.trigger_recSwitch = self.teleop_method[self.teleop_mode].trigger_recSwitch
            self.trigger_recoverJob=self.teleop_method[self.teleop_mode].trigger_recoverJob
            self.trigger_rowFollowForwards=self.teleop_method[self.teleop_mode].trigger_rowFollowForwards
            self.trigger_rowFollowBackwards=self.teleop_method[self.teleop_mode].trigger_rowFollowBackwards
            self.trigger_shutdown=self.teleop_method[self.teleop_mode].trigger_shutdown
            self.trigger_releaseForceStop=self.teleop_method[self.teleop_mode].trigger_releaseForceStop

            # Reset the internal triggers onthe selected method
            # Do not reset trigger_active state
            self.teleop_method[self.teleop_mode].trigger_pause=False
            self.teleop_method[self.teleop_mode].trigger_resume=False
            self.teleop_method[self.teleop_mode].trigger_abort=False
            self.teleop_method[self.teleop_mode].trigger_goHome=False
            self.teleop_method[self.teleop_mode].trigger_uvSwitch=False
            self.teleop_method[self.teleop_mode].trigger_camSwitch = False
            self.teleop_method[self.teleop_mode].trigger_recSwitch = False
            self.teleop_method[self.teleop_mode].trigger_recoverJob=False
            self.teleop_method[self.teleop_mode].trigger_rowFollowForwards=False
            self.teleop_method[self.teleop_mode].trigger_rowFollowBackwards=False
            self.teleop_method[self.teleop_mode].trigger_shutdown=False
            self.teleop_method[self.teleop_mode].trigger_releaseForceStop=False

            # Update cmd_vel
            self.teleop_cmd_vel=self.teleop_method[self.teleop_mode].cmd_vel

        else: # No active teleop
            # Set states for master teleop
            self.trigger_active=False # Triggers when any teleop type is active
            self.trigger_pause=False
            self.trigger_resume=False
            self.trigger_abort=False
            self.trigger_goHome=False
            self.trigger_uvSwitch=False
            self.trigger_camSwitch = False
            self.trigger_recSwitch = False
            self.trigger_recoverJob= False
            self.trigger_rowFollowForwards= False
            self.trigger_rowFollowBackwards= False
            self.trigger_shutdown=False
            self.trigger_releaseForceStop=False

            # Create an empty cmd_vel twist
            self.teleop_cmd_vel=Twist()



        ######################################
        # Job manager direct inputs
        ######################################

        # Reset user input for manager client
        self.user_client.userInput = None

        # As only one input is expected at a time, the order here is has been selected to prioritise safety:
        # Abort > Pause > Resume > Home > Reload
        if self.trigger_abort:
            self.user_client.userInput = 3
            self.trigger_abort=False # Stop from double sending

        elif self.trigger_pause:
            self.user_client.userInput = 2
            self.trigger_pause=False # Stop from double sending

        elif self.trigger_resume:
            self.user_client.userInput = 1
            self.trigger_resume=False # Stop from double sending

        elif self.trigger_goHome:
            self.user_client.userInput = 4
            self.trigger_goHome=False # Stop from double sending

        #elif self.trigger_skipPhase: ## This has not been fully implemented, but was partially-developed for assist
        #    self.user_client.userInput = 5
        #    self.trigger_skipPhase=False

        elif self.trigger_recoverJob:
            self.user_client.userInput = 6
            self.trigger_recoverJob=False # Stop from double sending

        # Send request to manager server if input is made by user
        if self.user_client.userInput is not None:
            if self.user_client.checkForService():
                managerResponse = self.user_client.sendDirectUserInput()     
            else:
                print("Unable to make request")
                print("ROS service " + self.user_client.serviceName + " is not available")


        ######################################
        # Standalone row-following service
        ######################################

        # A method is needed to stop these from triggering, if the robot is already doing a job
        # For now, we can use a flag to lock them out

        if self.allowStandaloneRowFollowing:
            updated = False
            if self.trigger_active and self.row_follow_activated:
                print("Stop row following - other teleop method activated")
                self.standAlone_client.userInput = -1
                self.trigger_rowFollowForwards=False # Stop from double sending
                self.trigger_rowFollowBackwards = False
                updated = True

            elif (not self.row_follow_activated and self.trigger_rowFollowForwards) or (self.row_follow_activated and self.trigger_rowFollowForwards and not self.row_follow_direction):
                rospy.loginfo("Drive Forward autonomously through rows")
                self.standAlone_client.userInput = 1
                self.trigger_rowFollowForwards=False # Stop from double sending
                updated = True
            
            elif not self.row_follow_activated and self.trigger_rowFollowBackwards or (self.row_follow_activated and self.trigger_rowFollowBackwards and self.row_follow_direction):
                rospy.loginfo("Drive Backward autonomously through rows")
                self.standAlone_client.userInput = 2
                self.trigger_rowFollowBackwards=False # Stop from double sending
                updated = True


            if updated:
                if self.standAlone_client.checkForService():
                    managerResponse = self.standAlone_client.sendDirectUserInput()     
                    if self.standAlone_client.userInput == -1:
                        self.row_follow_activated = False
                    elif self.standAlone_client.userInput == 1:
                        self.row_follow_activated = True
                        self.row_follow_direction = True # forward
                        self.row_activated_time = time.time()
                    elif self.standAlone_client.userInput == 2:
                        self.row_follow_activated = True
                        self.row_follow_direction = False # backward
                        self.row_activated_time = time.time()
                    else:
                        self.row_follow_activated = False
                else:
                    print("Unable to make request")
                    print("ROS service " + self.standAlone_client.serviceName + " is not available")


        ######################################
        # Shutdown calls
        ######################################

        if self.trigger_shutdown:
            rospy.loginfo("Xavier will be powered off now")
            self.soft_shutdown_pub.publish(True)
            self.trigger_shutdown = False # Stop from double sending
            # As the robot will immediately begin powering down, there is no need to reset the request#
            # But it does prevent the system constantly resending the same request


        ######################################
        # UV calls
        ######################################

        if self.trigger_uvSwitch:

            self.trigger_uvSwitch=False # Stop from double sending

            if self.uvClient.checkForService():
                self.uvClient.uv_num = 3  # Both lights
                self.uvClient.command = 3 # Toggle
                print("Teleop command: UV ON")
                uv_response = self.uvClient.sendUvCommand()
            else:
                print("Unable to make request")
                print("ROS service " + self.uvClient.serviceName + " is not available")


        ######################################
        # cam calls
        ######################################

        if self.trigger_camSwitch:

            self.trigger_camSwitch=False # Stop from double sending

            if self.camClient.checkForService():
                self.camClient.camera_num = 3  # left camera
                self.camClient.command = 3 # Toggle
                recording_response = self.camClient.sendCameraCommand()
            else:
                print("Unable to make request")
                print("ROS service " + self.camClient.serviceName + " is not available")

        if self.trigger_recSwitch:
            self.trigger_recSwitch=False # Stop from double sending
            rospy.wait_for_service("/antobot/manager/recManagerStart")
            if (self.record_status == False):
                recordResponse =self.start_recording_client("all",True)
                if recordResponse.success:
                    self.record_status = True
            else:
                recordResponse =self.start_recording_client("all",False)
                if recordResponse.success:
                    self.record_status = False




        ######################################
        # Release safety locks
        ######################################

        # Only perform actions if the teleop method is active
        if self.trigger_active:

            if self.trigger_releaseForceStop:

                self.trigger_releaseForceStop=False # Stop from double sending

                self.force_stop_release = True
                self.manual_intervention = True
                self.force_stop_release_pub.publish(self.force_stop_release)
                self.manual_intervention_pub.publish(self.manual_intervention)
            

            ######################################
            # Drive robot
            ######################################

            self.teleop_cmd_vel_pub.publish(self.teleop_cmd_vel)



   #################################################################################### 
        
    def navSendXMLpath(self): # UNUSED?

        packagePath = "~/catkin_ws/src/AntoMove/antobot_move_cartograph/"

        # Create the class to handle client-side interaction
        loadXMLClient = navigatorLoadXMLClient(sourceID='cartographer') 

        # Check that the service is available before trying to send requests
        serviceState=loadXMLClient.checkForService()

        if serviceState: # If the service is available

            # loadXMLClient.filepath=packagePath+'/src/XML data/testCorutyardOSM8.antonav.xml'
            # loadXMLClient.filepath = '/src/XML data/brBrooks03.antonav.xml'
            loadXMLClient.filepath = xml_file
            loadXMLResponse = loadXMLClient.requestLoad()


            print('Requested to load XML file at ' + loadXMLClient.filepath)
            print('Server response code:' + str(loadXMLResponse.responseCode))
            print(loadXMLResponse.responseString)

        else:
            print('Unable to make request')
            print('ROS service ' + loadXMLClient.serviceName + ' is not available')

####################################################################################

    def requestJob_client(self): # UNUSED?
        
        # Check that the service is availble before trying to send requests
        serviceState=self.job_client.checkForService()

        # Load JSON job
        self.savedJobPath = saved_routes_dir + job_file

        with open(self.savedJobPath, 'r') as f:
            self.taskString = json.load(f)


        if serviceState: # If the service is available

            self.job_client.jobID = 1 # This needs to be a function call to a global jobID generator. Strings might be better with robotID prefix
            self.job_client.jobString=self.taskString
            newJobResponse = self.job_client.sendNewJob()

            print('Requested jobID: ' + str(self.job_client.jobID))
            print('Requested jobString:' + str(self.job_client.jobString))
            print('Server response code:' + str(newJobResponse.responseCode))
            print(newJobResponse.responseString)

        else:
            print('Unable to make request')
            print('ROS service ' + self.job_client.serviceName + ' is not available')

####################################################################################

# Main
if __name__ == '__main__':
    rospy.init_node("master_teleop")
    mt = MasterTeleop()

    rosRate=rospy.Rate(30) # Aim for a 30Hz output

    while not rospy.is_shutdown():
        mt.update()
        rosRate.sleep()
