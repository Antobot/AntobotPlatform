#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     sysMonitor looks after different system data such as CPU load, temperature, storage, and the
#                           battery level of the robot. It reports any changes in status to antoSupervisor via rosout.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

from jtop import jtop
import shutil
import rospy
import json
import yaml
import rospkg

from std_msgs.msg import Bool, UInt8
from antobot_platform_msgs.msg import UInt8_Array,Float32_Array,UInt16_Array
from antobot_platform_msgs.srv import softShutdown

disk="./"
min_gb=2
min_percent=10

class urcuMonitor():

    def __init__(self):

        self.jtop_ext = True    # If using this script inside of a docker ontainer, self.jtop_ext should be True

        #Xavier monitor
        if not self.jtop_ext:
            self.jetson = jtop()
            self.jetson.start()

        self.As_uBat = 55
        self.soft_shutdown_req = False
        self.voltage_pre = 56.0

        self.cpu_load_level_str = "low"
        self.cpu_temp_level_str = "low"
        self.cpuLoad_list = []
        self.cpuLoad_win_len = 10
        self.storage_level = 0  # Assume there is plenty of storage remaining
        self.As_sBatlvl = "none"

        self.sub_As_uBat = rospy.Subscriber("/antobridge/Ab_uBat",UInt8_Array,self.battery_callback)
        self.sub_soft_shutdown_button = rospy.Subscriber('/antobridge/soft_shutdown_button',Bool,self.soft_shutdown_callback)

        self.pub_soft_shutdown_req = rospy.Publisher("/antobridge/soft_shutdown_req", Bool,queue_size = 1)
        self.pub_soc = rospy.Publisher("/as/soc", UInt8, queue_size = 1)

        self.soft_shutdown_client = rospy.ServiceProxy('soft_shutdown_req',softShutdown)

        return

    def xavier_monitor(self):
        
        if not self.jtop_ext:
            self.xavier_monitor_int()
        else:
            self.xavier_monitor_ext()

        self.cpu_temp_level()
        self.cpu_load_level()

    def xavier_monitor_int(self):
        #access to Jtop and read the xavier info
        jtopstats = self.jetson.stats      
        self.As_cputemp = jtopstats["Temp CPU"]
        
        CPU1 = jtopstats["CPU1"]
        CPU2 = jtopstats["CPU2"]
        CPU3 = jtopstats["CPU3"]
        CPU4 = jtopstats["CPU4"]
        CPU5 = jtopstats["CPU5"]
        CPU6 = jtopstats["CPU6"]
        self.As_cpuLoad = (CPU1+CPU2+CPU3+CPU4+CPU5+CPU6)//6 #calculate the average CPU load
    
    def xavier_monitor_ext(self):

        xavier_datafile = "/root/xavier_data/xavier_data.json"
        try:
            with open(xavier_datafile, "r") as f:
                data = json.load(f)
                self.As_cputemp = data['cpuTemp']
                self.As_cpuLoad = data['cpuLoad']
        except Exception as e:
            rospy.loginfo("SW2122: Error while reading from file outside docker: {e}")

    def cpu_temp_level(self):
        cpu_temp_level_i = "low"
        if self.As_cputemp > 50.0:
            cpu_temp_level_i = "medium"
        if self.As_cputemp > 60.0:
            cpu_temp_level_i = "high"

        if cpu_temp_level_i is not self.cpu_temp_level_str:
            if cpu_temp_level_i == "low":
                rospy.loginfo("SF1000: CPU temp low (<50)")
            if cpu_temp_level_i == "medium":
                rospy.logwarn("SF1000: CPU temp medium (>50)")
            if cpu_temp_level_i == "high":
                rospy.logerr("SF1000: CPU temp high (>60)")
            self.cpu_temp_level_str = cpu_temp_level_i


    def cpu_load_level(self):
        self.cpuLoad_list.append(self.As_cpuLoad)
        if len(self.cpuLoad_list) > self.cpuLoad_win_len:
            self.cpuLoad_list.pop(0)

        self.cpuLoad_avg = sum(self.cpuLoad_list)/len(self.cpuLoad_list)

        cpu_load_i = "low"
        if self.cpuLoad_avg > 80.0:
            cpu_load_i = "medium"
        if self.cpuLoad_avg > 90.0:
            cpu_load_i = "high"

        if cpu_load_i is not self.cpu_load_level_str:
            if cpu_load_i == "low":
                rospy.loginfo("SF1100: CPU load low (<80%)")
            if cpu_load_i == "medium":
                rospy.logwarn("SF1100: CPU load medium (>80%)")
            if cpu_load_i == "high":
                rospy.logerr("SF1100: CPU load high (>90%)")
            self.cpu_load_level_str = cpu_load_i

    
    def storage_management(self): 
        #Returns True if there isn't enough disk space, False otherwise.# 
        du = shutil.disk_usage(disk)
        self.As_bStorage = False

        # Calculate the percentage of free space
        percent_free = 100 * du.free / du.total
        
        # Calculate how many free gigabytes
        gigabytes_free = du.free / (2**30)

        storage_level_i = 0
        if percent_free < 10:
            storage_level_i = 1
        if gigabytes_free < min_gb:
            storage_level_i = 2
        if percent_free < 1:
            storage_level_i = 3

        if storage_level_i is not self.storage_level:
            if storage_level_i == 1:
                rospy.loginfo("SF1200: Storage < 10 percent remaining")
            if storage_level_i == 2:
                rospy.logwarn("SF1200: Storage < 2 GB remaining")
            if storage_level_i == 3: 
                rospy.logerr("SF1200: Storage < 1 percent remaining")
            self.storage_level = storage_level_i

        return self.As_bStorage

    def battery_callback(self, Ab_uBat):
        # The callback function for battery data (/antobridge/Ab_uBat), get the voltage,filter it and pass it to battery_indication function to get the percentage voltage and level
        voltage_decimal = float(Ab_uBat.data[1])
        self.voltage = float(Ab_uBat.data[0])+ voltage_decimal/100
        if self.voltage == self.voltage_pre:
            self.voltage_cnt = self.voltage_cnt+1
        else:
            self.voltage_cnt = 0
        self.voltage_pre = self.voltage
        
        if self.voltage_cnt > 5:
            if self.voltage <self.As_uBat:
                self.As_uBat = self.voltage #low pass after continously appear 5 times
    

    def battery_lvl(self):
        #calculate the current soc and convert it to level battery
        counter = 100

        #The look up table for SOC 
        while counter >0:
            if self.As_uBat >= 44+counter*0.106 : 
                self.As_uSoC = counter
                break
            counter = counter - 1

        #The level convertion 
        if self.As_uSoC >= 80:
            battery_level_i = "high"
        elif self.As_uSoC >= 55:
            battery_level_i = "medium"
        elif self.As_uSoC >= 30:
            battery_level_i = "low"
        else:
            battery_level_i = "alert"

        if battery_level_i is not self.As_sBatlvl:
            if battery_level_i == "high":
                rospy.logdebug("SF1300: Battery >80 percent (high)")
            if battery_level_i == "medium":
                rospy.loginfo("SF1300: Battery >55 percent (medium)")
            if battery_level_i == 2:
                rospy.logwarn("SF1300: Battery >30 percent (low)")
            if battery_level_i == 3: 
                rospy.logerr("SF1300: Battery <30 percent (critical)")
            self.As_sBatlvl = battery_level_i
        
        self.pub_soc.publish(self.As_uSoC)


    def soft_shutdown_callback(self,soft_shutdown):  #soft shutdown button on joystick pressed
            if soft_shutdown.data == True:
                self.soft_shutdown_req = True


    def soft_shutdown_process(self): #send req to antobridge, call the power off service
        if self.soft_shutdown_req == True:
            print("in the soft shutdown process")
            self.pub_soft_shutdown_req.publish(self.soft_shutdown_req)
            try:
                soft_shutdown_reponse = self.soft_shutdown_client(1)
            except rospy.ServiceException as e:
                print("service call failed: %s" % e)

def main():
    rospy.init_node ('urcuMonitor') 
    rate = rospy.Rate(1)
    sysMonitor= urcuMonitor()
    try:
        while not rospy.is_shutdown():
            sysMonitor.storage_management()
            sysMonitor.battery_lvl()
            sysMonitor.xavier_monitor()
            sysMonitor.soft_shutdown_process()
            rate.sleep()
    # except:
    #     print("Exception occured!!")
    finally:
        if not sysMonitor.jtop_ext:
            sysMonitor.jetson.close

if __name__ == '__main__':
    main()
