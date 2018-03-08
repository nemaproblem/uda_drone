import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = [[10.0, 0.0, 3],[10.0, 10.0, 3],[0.0, 10.0, 3],[0.0, 0.0, 3]]
        self.in_mission = True
        self.check_state = {}
        self.current_waypoint = -1


        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        self.mission_altitude = 3

    def local_position_callback(self):
        """Trasition Take-Off -> Waypoint"""
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]
            if altitude > 0.95 * self.mission_altitude:
                self.waypoint_transition()
        """Set Next Waypoint As Target If Reached"""
        if self.flight_state == States.WAYPOINT:
            self.calculate_box()
        pass

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        pass

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            print("here")
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            print("state_callback_arming")
            if self.armed:
                print("Armed")
                self.takeoff_transition()
        elif self.flight_state == States.LANDING:
            if self.local_position[2] < 0.01:
                self.disarming_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()
        pass

    """navigate through the waypoints"""
    def calculate_box(self):
        print(self.current_waypoint)
        if abs(self.target_position[0]-self.local_position[0])<0.1 or self.current_waypoint == -1:
            if abs(self.target_position[1]-self.local_position[1])<0.1 or self.current_waypoint == -1:
                if self.current_waypoint == 3:
                    self.landing_transition()
                else:
                    self.current_waypoint = self.current_waypoint + 1
                    self.target_position = self.all_waypoints[self.current_waypoint]
                    self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0)
                    print(self.target_position)
        pass

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()

         # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state = States.ARMING
        pass



    def takeoff_transition(self):
        print("takeoff transition")
        self.target_position = np.array([0.0, 0.0, self.mission_altitude])
        self.takeoff(self.mission_altitude)
        self.flight_state = States.TAKEOFF


    def waypoint_transition(self):
        print("waypoint transition")
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.flight_state = States.DISARMING
        self.disarm()


    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
