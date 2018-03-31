import argparse
import time
import msgpack
from enum import Enum, auto
from skimage.morphology import medial_axis
from skimage.util import invert

import numpy as np

from planning_utils import heuristic, create_grid, prune_path, find_start_goal
from planning import a_star
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            print('target, local', type(self.target_position), type(self.local_position))
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 3

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', 'r') as f:
            latlon = f.readline()
        ll = latlon.strip().replace(',', '').split(' ')
        lat0, lon0 = float(ll[1]), float(ll[3])
        
        # TODO: set home position to (lat0, lon0, 0)
        # self.set_home_position(lat0, lon0, 0) # the comments is wrong oder
        self.set_home_position(lon0, lat0, 0)


        # TODO: retrieve current global position
        global_position = self.global_position
        print('retrieve current global position:')
        print(self.global_position)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

 
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(global_position, self.global_home)
        print('new local position:{}'.format(current_local_position))
        self._north, self._east, self._down = current_local_position
        print('self.local_position:{}'.format(self.local_position))
        
        # print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
        #                                                                  self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        # Determine offsets between grid and map
        north_offset = int(np.abs(np.min(data[:, 0])))
        east_offset = int(np.abs(np.min(data[:, 1])))

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # 310, 439

        # Define a grid for a particular altitude and safety margin around obstacles
        grid = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # medial_axis
        skeleton = medial_axis(invert(grid))
        # Define starting point on the grid (this is just grid center)
        grid_start = (north_offset, east_offset)
        # TODO: convert start position to current position rather than map center
        # new home will appear when to run this py file.
        grid_start = (int(current_local_position[0]+north_offset), int(current_local_position[1]+east_offset))
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (north_offset + 200, east_offset + 100)
        print('grid_goal:{}'.format(grid_goal))
        print('Local Start and Goal: ', grid_start, grid_goal)

        # find start and goal in medial axis
        skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
        print('skel start and goal: {}, {}'.format(skel_start, skel_goal))
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        # really??
        # global_goal = [-122, 37, 0]
        # grid_goal = tuple(global_to_local(global_goal, self.global_home)[:2])
        


        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # path, cost = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))
        print('len(path):{}'.format(len(path)))
        # TODO: prune path to minimize number of waypoints
        path = prune_path(path)
        print('new len(path):{}'.format(len(path)))
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] - north_offset, p[1] - east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        print(waypoints)
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
