# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "UAV2")
client.armDisarm(True, "UAV2")

client.takeoffAsync(vehicle_name="UAV2").join()  # takeoff

# square flight
client.moveToZAsync(-3, 1, vehicle_name="UAV2").join()  # 上升到3m高度
client.moveToPositionAsync(5, 0, -3, 1, vehicle_name="UAV2").join()  # 飞到（5,0）点坐标
client.moveToPositionAsync(5, 5, -3, 1, vehicle_name="UAV2").join()  # 飞到（5,5）点坐标
client.moveToPositionAsync(0, 5, -3, 1, vehicle_name="UAV2").join()  # 飞到（0,5）点坐标
client.moveToPositionAsync(0, 0, -3, 1, vehicle_name="UAV2").join()  # 回到（0,0）点坐标

client.landAsync(vehicle_name="UAV2").join()  # land
client.armDisarm(False)  # lock
client.enableApiControl(False)  # release control





# for i in range(9):
#     name = "UAV"+str(i+1)