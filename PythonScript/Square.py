# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

for i in range(4):
    client.enableApiControl(True, "UAV"+str(i+1))
    client.armDisarm(True, "UAV"+str(i+1))

    client.takeoffAsync(vehicle_name="UAV"+str(i+1)).join()  # takeoff

    # square flight
    client.moveToZAsync(-3, 1, vehicle_name="UAV"+str(i+1)).join()  # 上升到3m高度
    client.moveToPositionAsync(5, 0, -3, 1, vehicle_name="UAV"+str(i+1)).join()  # 飞到（5,0）点坐标
    client.moveToPositionAsync(5, 5, -3, 1, vehicle_name="UAV"+str(i+1)).join()  # 飞到（5,5）点坐标
    client.moveToPositionAsync(0, 5, -3, 1, vehicle_name="UAV"+str(i+1)).join()  # 飞到（0,5）点坐标
    client.moveToPositionAsync(0, 0, -3, 1, vehicle_name="UAV"+str(i+1)).join()  # 回到（0,0）点坐标

    client.landAsync(vehicle_name="UAV"+str(i+1)).join()  # land
    client.armDisarm(False)  # lock
    client.enableApiControl(False)  # release control





# setting.jason
# {
#   "SettingsVersion": 1.2,
#   "SimMode": "Multirotor",
#   "Vehicles": {
#     "UAV1": {
#       "VehicleType": "SimpleFlight",
#       "X": 0, "Y": 0, "Z": 0,
#       "Yaw": 0
#     },
#     "UAV2": {
#       "VehicleType": "SimpleFlight",
#       "X": 4, "Y": -3, "Z": 0,
#       "Yaw": 0
#     },
#     "UAV3": {
#       "VehicleType": "SimpleFlight",
#       "X": 4, "Y": 0, "Z": 0,
#       "Yaw": 0
#     },
#     "UAV4": {
#       "VehicleType": "SimpleFlight",
#       "X": 0, "Y": -3, "Z": 0,
#       "Yaw": 0
#     }
#   },
#   "ViewMode": "Manual",
#   "ClockSpeed": 4
# }