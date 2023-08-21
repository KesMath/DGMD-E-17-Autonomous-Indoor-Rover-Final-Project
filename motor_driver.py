import time
import asyncio
import subprocess
from path_planning.grid_maps import *
from path_planning.dijkstra_path_planner import *
from gyroscope.mpu6050_driver import GyroscopeDriver

from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

# https://pypi.org/project/nest-asyncio/
# By design asyncio does not allow its event loop to be nested. 
# This module patches asyncio to allow nested use of asyncio.run and loop.run_until_complete.

gyroscope_driver = GyroscopeDriver()

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
    print("spinning left 90 degrees")
    await base.spin(velocity=100, angle=100)
    
async def spin_right_90_degrees(base):
    # Spins the Viam Rover 90 degrees at 100 degrees per second
    print("spinning right 90 degrees")
    await base.spin(velocity=100, angle=-90)

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


# async def get_2D_Map_of_enclosure():
#     driver = LidarDriver(port_name= "/dev/ttyUSB0")
#     driver.scan_enclosure()
#     driver.sampling_df.to_csv('slam/enclosure_sampling.csv', header = False, index = False)
    # SUBPROCESS ME: "python lidar/scan1.py > slam/sampling.csv"
    # SUBPROCESS ME: "python slam/map.py"

# async def main():
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

###### Sanity Check ProcessPool ##########
async def test_fn1():
    # mocks gyroscope polling - since it stops eventually
    i = 0
    while i < 10:
        #print("process1 triggered! " + str(i))
        i+=1
        time.sleep(1)
    print("i = " + str(i))
    return i

async def test_fn2():
    # mocks motor spinning - since it goes on indefinitely
    print("test_fn2() firing...")
    while True:
        continue

async def main():
    robot_client = await connect()
    roverBase = Base.from_robot(robot_client, 'viam_base')
    ########################## TESTING WITH ProcessPool() ##########################

    ## TECHNIQUE 0
    # call subprocess on polling sensor and monitor it's return code = rc
    # result = subprocess.run(args = ["python", "gyroscope/mpu6050_driver.py"], 
    #                         capture_output=True,
    #                         text=True,
    #                         shell=False,
    #                         timeout=False)
    # then in this main thread:
    # await spin_left_90_degrees()
    #   while True:
            #if rc = 0:
                # stop_moving()
            


    ### TECHNIQUE 1
    task = await gyroscope_driver.poll_sensor_until_orthogonally_left(roverBase=roverBase)
    await spin_left_90_degrees(roverBase)

    ### TECHNIQUE 2
    # Dispatch 2 processes - Process A for Sensor Polling, Process B for motor spinning
    # with ProcessPool() as pool:
    #     print("executing processes...")
    #     #p1 = pool.schedule(test_fn1)
    #     #p2 = pool.schedule(test_fn2)
    #     p1 = pool.schedule(gyroscope_driver.poll_sensor_until_orthogonally_left)
    #     p2 = pool.schedule(spin_left_90_degrees, roverBase)

    #     p1.result() # blocks until process completes!
    #     p2.cancel()
    #     assert p1.done()
    #     assert p2.done()

    #     pool.stop()
    #     pool.join()

        ### TECHNIQUE 3
        # when process A finishes (i.e. when rover turns 90deg,) terminate process B (i.e. stop motors from spinning)
        #executor will automatically shutdown when control flow exits context manager
        # while pool.active:
        #     # terminate process
        #     print("Process1 running: " + str(p1.running()))
        #     print("Process2 running: " + str(p2.running()))   
        #     if p1.done():
        #         print("terminating \"spin_left_90_degrees()\" process...")
        #         # https://pebble.readthedocs.io/en/latest/#pebble-processfuture
        #         # This class inherits from concurrent.futures.Future.
        #         # The sole difference with the parent class is the possibility to cancel running calls.
        #         p2.cancel()

        #         # confirm processes successfully killed
        #         assert p2.done()
        #         assert p1.done()

        #         # shutdown pool to break out of looping conditional
        #         pool.stop()
        
        # # confirming pool closed
        # assert not pool.active

    print("closing connection...")
    await robot_client.close()
if __name__ == '__main__':
    asyncio.run(main())
