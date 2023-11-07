import time
import signal
import asyncio
from multiprocessing import Process
from path_planning.grid_maps import *
from path_planning.dijkstra_path_planner import *

from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.services.slam.client import SLAMClient
from viam.rpc.dial import Credentials, DialOptions

async def connect():
    creds = Credentials(
        type='robot-location-secret',
        payload='xhp2wcnjiej2m5kstyh2kmvrkx4q5yan60cncyamc61uvsed')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('kes-rover-main.0ltqp6fjer.viam.cloud', opts)

async def move_forward_1_foot(base):
    # Moves the Viam Rover forward 625mm at 625mm/s
    print("moving straight")
    await base.move_straight(velocity=625, distance=625)

async def move_backward_1_foot(base):
    # Moves the Viam Rover backward 625mm at 625mm/s
    print("moving backward")
    await base.move_straight(velocity=-625, distance=515)

async def spin_left_90_degrees(base):
    # Spins the Viam Rover 90 degrees at 100 degrees per second
    # Experimentally, I had to reduce by 10 degrees since 90 deg was overshot
    print("spinning left 90 degrees")
    await base.spin(velocity=100, angle=95)

async def spin_right_90_degrees(base):
    # Spins the Viam Rover 90 degrees at 100 degrees per second
    # Experimentally, I had to reduce by 10 degrees since 90 deg was overshot
    print("spinning right 90 degrees")
    await base.spin(velocity=100, angle=-95)

async def drive_right_1_foot(base):
    await spin_right_90_degrees(base)
    time.sleep(1)
    await move_forward_1_foot(base)
    time.sleep(1)
    # re-centers rover forward
    await spin_left_90_degrees(base)

async def drive_left_1_foot(base):
    await spin_left_90_degrees(base)
    time.sleep(1)
    await move_forward_1_foot(base)
    time.sleep(1)
    # re-centers rover forward
    await spin_right_90_degrees(base)

async def drive_to_next_tile(base, current_point: tuple, new_coordinate_pt: tuple):
    # drive forward
    if new_coordinate_pt[0] == current_point[0] - 1  and new_coordinate_pt[1] == current_point[1]:
        await move_forward_1_foot(base)

    # drive to left tile
    elif new_coordinate_pt[1] == current_point[1] - 1:
        await drive_left_1_foot(base)

    # drive to right tile
    elif new_coordinate_pt[1] == current_point[1] + 1:
        await drive_right_1_foot(base)
   
    # drive backward
    elif new_coordinate_pt[0] == current_point[0] + 1  and new_coordinate_pt[1] == current_point[1]:
        await move_backward_1_foot(base)

async def drive_perimeter_wall(base):
    # Moves the Viam Rover forward 2500mm at 625mm/s
    print("moving straight 5 feet")
    await base.move_straight(velocity=625, distance=2500)

async def walk_enclosure(base):
    # walk perimeter generate 2D Map with LiDAR Sensor
    await drive_perimeter_wall(base)
    time.sleep(3)
    await spin_right_90_degrees(base)
    time.sleep(2)
    await drive_perimeter_wall(base)
    time.sleep(3)
    await spin_right_90_degrees(base)
    time.sleep(2)
    await drive_perimeter_wall(base)
    time.sleep(3)
    await spin_right_90_degrees(base)
    time.sleep(2)
    await drive_perimeter_wall(base)
    time.sleep(3)
    await spin_right_90_degrees(base)
    time.sleep(2)

async def generate_point_cloud_map(robot_client):
    slam_service = SLAMClient.from_robot(robot=robot_client, name="SLAM-Service-2-0")
    pcd_map = await slam_service.get_point_cloud_map() # since we will be sending this a SIGTERM, we need to use a signal-handler so that object can either be returned or saved to file
    return pcd_map

# function wrapper trick to call async function using multiporcessing
# https://stackoverflow.com/questions/71678575/how-do-i-call-an-async-function-in-a-new-process-using-multiprocessing
def get_pcd_of_enclosure(robot_client):
    asyncio.run(generate_point_cloud_map(robot_client=robot_client))

async def get_2D_map_of_enclosure(robot_client, roverBase):
    # https://blog.pollithy.com/python/numpy/pointcloud/tutorial-pypcd

    # This function dispatches get_pcd_of_enclosure() as another process. 
    # Then when walk_enclosure() main process doesn't block anymore (i.e. rover returns back to base)
    # we terminate the collection of point cloud values
    process = Process(target=get_pcd_of_enclosure, args=(robot_client,))
    process.start()
    await walk_enclosure(base = roverBase)
    process.terminate()
    await asyncio.sleep(3) # blocking main process temporarily so assertions can pass!
    assert process.is_alive() is False
    assert process.exitcode == -signal.SIGTERM


# async def main():

#     #################################
#     #### INITIALIZATION PHASE #######
#     #################################

#     # TODO: see if this can dynamically be mapped to grid cell after SLAM localization
#     start_point = (4,0)

#     goal_point = input("Enter the goal point as x y: ")
#     goal_point = goal_point.split()
#     while(not (is_within_grid_bounds(int(goal_point[0]), GRID_WIDTH) and is_within_grid_bounds(int(goal_point[1]), GRID_HEIGHT))): #ensure bounds check
#         print("X must be >=0 and less than " +  str(GRID_WIDTH) + " and Y must be >= 0 and less than " + str(GRID_HEIGHT))
#         goal_point = input("Enter the goal point as x,y: ")
#         goal_point = goal_point.split()
    
#     goal_point = (int(goal_point[0]) , int(goal_point[1]))

#     print("connecting rover to Viam server...")
#     robot_client = await connect()
    
#     # Get the base component from the Viam Rover
#     roverBase = Base.from_robot(robot_client, 'viam_base')

#     #################################
#     #### PERCEPTION PHASE ###########
#     #################################

#     # await get_2D_map_of_enclosure(robot_client=robot_client, roverBase=roverBase)

#     #################################
#     #### MOTION PLANNING PHASE ######
#     #################################

#     print("calculating shortest path...")
#     shortest_path = return_shortest_path(start_point = start_point, goal_point = goal_point, width = GRID_WIDTH, height = GRID_HEIGHT, gridmap= DIAGONAL_OCCUPIED_GRID, resolution = STEP_COST)

#     if shortest_path is not None:
#         # Driving to destination
#         for node in shortest_path[1:]:
#             next_point = node.get_coordinate_pt()
#             print("driving to :" + str(next_point))
#             await drive_to_next_tile(base = roverBase, current_point = start_point, new_coordinate_pt = next_point)
#             time.sleep(1)
#             # need to update starting point since robot moved to a new position
#             start_point = next_point

#         # Returning back from destination
#         print("returning back to starting point...")
#         shortest_path.reverse()
#         for node in shortest_path[1:]:
#             next_point = node.get_coordinate_pt()
#             print("driving to :" + str(next_point))
#             await drive_to_next_tile(base = roverBase, current_point = start_point, new_coordinate_pt = next_point)
#             time.sleep(1)
#             # need to update starting point since robot moved to a new position
#             start_point = next_point
#     else:
#         print("Rover unable to find shortest path... ")

#     # close server connection
#     print("closing client connection to Viam server...")
#     await robot_client.close()

async def main():
# PCD REFERENCE: https://python.viam.dev/autoapi/viam/gen/service/slam/v1/slam_pb2/index.html#viam.gen.service.slam.v1.slam_pb2.GetPointCloudMapResponse
# NEXT STEPS:
# (3) - Create a module graphically plot PCD Points.
# (4) - Generate 2D binary occupancy grid map (either graphically by looking at the pixels or numerically by looking at raw dataset). The latter is be preferred choice for performance reasons

# TEST / CONFIRM that Point Cloud Can be Generated Sequentially After Drive (would make logic much easier)
# (Since SLAM service is configured to run whenever rover connects to VIAM server, it should automatically be building the map as a background process. So, I believe there is no need to have run a concurrent process while driving)
# (1) clear any internal SLAM state (by deleting directories) and have rover drive around enclosure. After rover drives around, call generate_point_cloud_map(), plot the points and affirm it matches it's environment  

    robot_client = await connect()
    pcd_map = await generate_point_cloud_map(robot_client)
    with open("map.pcd", 'wb') as outputFile: # w = write, b = raw bytes mode
        outputFile.write(pcd_map[0].encode())

    # confirm printout of pcd_map[0]
    # 1) clean data format
    # 1) save to pcd file
    # 1) load in pcd file to parser
    # 2) plot pcd map
    # 2) convert point cloud to occupancy grid!
    await robot_client.close()

if __name__ == '__main__':
    asyncio.run(main())
