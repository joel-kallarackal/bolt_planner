#!/usr/bin/python3
import math

import rospy
import queue

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from bolt_planner.msg import LatLongWayPoint
from bolt_planner.srv import BoolBoolService
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler


class Transformer:
    """
    Class functions:
    * __init__ :
        Establishes publishers, subscribers, services, and variables.
    * flush_queue:
        returns True if the queue was successfully flushed. Stops the vehicle, and clears move-base's goal queue.
    * pause_journey:
        returns True if the vehicle was successfully paused. Stops the vehicle, but does not clear move-base's goal queue. bypass=True doesn't wait for the the service to be available.
        Sending False to the service resumes the journey.
    * set_default_nsf:
        sets the default altitude to the current altitude.
    * update_proximity:
        updates the proximity to the current goal. Edits the almosst_there variable.
    * ll_to_nav:
        converts a LatLongWayPoint message to a NavSatFix message and publishes it.
    * enqueue goal:
        Enqueues a goal to the goal queue. rviz_path is also updated for visualization.
    * check_status:
        Checks if there is any error in the move_base status. If yes, it logs the error and and changes almost_there to True.
    * run:
        Runs the node.

    """
    def __init__(self, odom_topic="/odom", gps_nsf_topic="/vikram_sim/gps/fix"):
        """
        There are 3 ways to publish gps waypoints to move base:
        This node listens for waypoints in terms of latitude and longitude on gps_waypoints/latlong.
        Upon receiving such a message, it transforms that into a NavSatFix message and publishes it for transformation.

        Alternatively, messages can directly be sent to the navsat waypoints topic,
        if you have the waypoint preformatted that way.

        All transformed messages arrive as odometry messages. These are put in a queue as they arrive.
        Once we get pretty close to the current goal we switch to the next waypoint.

        The third way to set a waypoint is to simply send an odometry message to the topic conventionally used by the
        navsat transform as its output. This way, any XY waypoints will also get the added functionality of this code.

        Call the flush_queue service with value True to clear out all the queued waypoints.
        Call the pause_journey service with value True to temporarily stop the vehicle.
        Send value False to resume the journey.
        """

        self.goal_queue = queue.Queue()
        self.rviz_path = Path()
        self.rviz_path.header.frame_id = 'map'

        self.curr_goal = None
        self.curr_goal_pos = None
        self.almost_there = True
        self.halted = False
        self.proximity_thresh = 3  # in metres. Vehicle switches to next goal if it's this close to the current goal.

        # Publishers
        self.move_base_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.nav_sat_pub = rospy.Publisher("/gps_waypoints/navsat", NavSatFix, queue_size=10)
        self.rviz_goals_pub = rospy.Publisher("/gps_waypoints/path", Path, queue_size=1, latch=True)
        self.stop_mb_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.curr_pose_pub = rospy.Publisher("/curr_waypoint_pose_testing", PoseStamped, queue_size=1, latch=True)
        self.next_pose_pub = rospy.Publisher("/next_waypoint_pose_testing", PoseStamped, queue_size=1, latch=True)

        # Subscribers
        self.mb_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.check_status)
        self.gps_sub = rospy.Subscriber(gps_nsf_topic, NavSatFix, self.set_default_nsf)
        self.proximity_sub = rospy.Subscriber(odom_topic, Odometry, self.update_proximity)
        self.ll_sub = rospy.Subscriber("/gps_waypoints/latlong", LatLongWayPoint, self.ll_to_nav)
        self.odom_wpts_sub = rospy.Subscriber("/gps_waypoints/odom", Odometry, self.enqueue_goal)

        # Services
        self.flush_queue_service = rospy.Service("/gps_waypoints/flush_queue", BoolBoolService, self.flush_queue)
        self.pause_journey_service = rospy.Service("/gps_waypoints/pause_journey", BoolBoolService, self.pause_journey)
        self.seq = 0
        self.default_altitude = 0
        self.curr_waypoint = None

    def flush_queue(self, req: bool):
        rospy.wait_for_service('/gps_waypoints/flush_queue')
        try:
            if req:
                self.goal_queue = queue.Queue()
                # Setting almost there to True so that the run function publishes the next goal it gets.
                self.almost_there = True
                # Setting the present goal to None, and making the vehicle stop.
                self.curr_goal = None
                self.stop_mb_pub.publish(GoalID())

                rospy.loginfo("The GPS Waypoint Queue has been flushed.")
                return True

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed; Unable to flush queue. Cause:", e)
            return False

    def pause_journey(self, req: bool, bypass=False):
        if not bypass:
            rospy.wait_for_service('/gps_waypoints/pause_journey')
        try:
            if req:
                self.stop_mb_pub.publish(GoalID())
                self.halted = True
                rospy.loginfo("Vehicle will now stop. Send a False request to resume journey.")
                return True
            else:
                self.move_base_pub.publish(self.curr_goal)
                self.halted = False
                rospy.loginfo("The Vehicle will now resume the journey.")
                return True

        except rospy.ServiceException as e:
            if req:
                rospy.logerr("SERVICE CALL FAILED! UNABLE TO STOP VEHICLE! RETRYING. CAUSE OF FAILURE:", e)
                self.pause_journey(True, True)
            else:
                rospy.logerr("Service call failed; Unable to start the vehicle. Retrying. Cause:", e)
                self.pause_journey(False, True)
            return False

    def set_default_nsf(self, data: NavSatFix):

        """
        Some generic data is used to fill the NavSat message when latlong data is received.
        This function updates the default params using the first actual gps datapoint.
        Once this is done, the subscriber is shutdown.
        """

        self.default_altitude = data.altitude
        self.gps_sub.unregister()

    def update_proximity(self, data: Odometry):

        """Updates almost_there parameter based on proximity of the vehicle to the goal."""

        if self.curr_goal_pos is not None:
            # print("Proximity Checked") # DEBUG
            if math.sqrt((self.curr_goal_pos[0] - data.pose.pose.position.x) ** 2 + (
                    self.curr_goal_pos[1] - data.pose.pose.position.y) ** 2) < self.proximity_thresh:
                # print("Almost there") # DEBUG
                self.almost_there = True

    def ll_to_nav(self, data: LatLongWayPoint):

        """When a lat long waypoint is received, this converts it into a NavSat waypoint and publishes it."""

        rospy.loginfo("Received a GPS Waypoint.")
        navsat_waypoint = NavSatFix()
        navsat_waypoint.header.seq = self.seq
        navsat_waypoint.header.stamp = rospy.Time.now()

        # Default params
        navsat_waypoint.header.frame_id = "gps_link"
        navsat_waypoint.status.status = 2
        navsat_waypoint.status.service = 0
        navsat_waypoint.altitude = self.default_altitude
        navsat_waypoint.position_covariance = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        navsat_waypoint.position_covariance_type = 3

        # The actual stuff
        navsat_waypoint.latitude = data.latitude
        navsat_waypoint.longitude = data.longitude

        self.nav_sat_pub.publish(navsat_waypoint)

        self.seq += 1

    def enqueue_goal(self, data: Odometry):

        """
        Upon receiving an odometry goal, packages and enqueues a Move base goal.
        Keeps a copy of the position in an array for visualisation purposes.
        """

        rospy.loginfo("XY Waypoint Obtained.")
        mb_goal = MoveBaseActionGoal()

        mb_goal.header.seq = data.header.seq
        mb_goal.header.stamp = data.header.stamp
        mb_goal.header.frame_id = 'map'

        mb_goal.goal_id.stamp.secs = 0
        mb_goal.goal_id.stamp.nsecs = 0
        mb_goal.goal_id.id = ''

        mb_goal.goal.target_pose.header.seq = data.header.seq
        mb_goal.goal.target_pose.header.stamp = data.header.stamp
        mb_goal.goal.target_pose.header.frame_id = data.header.frame_id

        mb_goal.goal.target_pose.pose.position = data.pose.pose.position
        mb_goal.goal.target_pose.pose.orientation = data.pose.pose.orientation

        # Just for visualisation purposes
        rviz_pose = PoseStamped()
        rviz_pose.header = data.header

        rviz_pose.pose = data.pose.pose
        self.rviz_path.poses.append(rviz_pose)

        self.goal_queue.put(mb_goal)
        # print("Goal Enqueued") # DEBUG

    def check_status(self, data: GoalStatusArray):

        """Checks Goal Status updates to know if things are going well."""

        if len(data.status_list):
            latest_status = data.status_list.pop()
            if latest_status.status not in (0, 1, 2, 3):
                rospy.logerr(
                    f"Something is wrong. Move_Base has rejected the waypoint. \
                    Moving on to the next goal. Here is the last move_base status:{latest_status}")
                self.almost_there = True

        # STATUS LIST:
        # PENDING = 0 The goal has yet to be processed by the action server
        # ACTIVE = 1 The goal is currently being processed by the action server
        # PREEMPTED = 2 The goal received a cancel request after it started executing
        #    and has since completed its execution (Terminal State)
        # SUCCEEDED = 3 The goal was achieved successfully by the action server (Terminal State)
        # ABORTED = 4  The goal was aborted during execution by the action server due
        #    to some failure (Terminal State)
        # REJECTED = 5  The goal was rejected by the action server without being processed,
        #    because the goal was unattainable or invalid (Terminal State)
        # PREEMPTING = 6 The goal received a cancel request after it started executing
        #    and has not yet completed execution
        # RECALLING = 7 The goal received a cancel request before it started executing,
        #    but the action server has not yet confirmed that the goal is canceled
        # RECALLED = 8 The goal received a cancel request before it started executing
        #    and was successfully cancelled (Terminal State)
        # LOST = 9  An action client can determine that a goal is LOST. This should not be
        #    sent over the wire by an action server

    def run(self):

        """Core loop. Runs at 50 Hz.
        If the vehicle has almost reached the present goal, and we have a waypoint enqueued, and the vehicle is not in
        a halted state, we switch to the next waypoint.
        All the waypoints enqueued are published as a pose array for visualisation.
        """
        if self.curr_waypoint is None:
            if not self.goal_queue.empty():
                self.curr_waypoint: MoveBaseActionGoal = self.goal_queue.get()

        self.rviz_path.header.stamp = rospy.Time.now()
        self.rviz_goals_pub.publish(self.rviz_path)
        if not self.goal_queue.empty() and self.almost_there and not self.halted:
            next_waypoint: MoveBaseActionGoal = self.goal_queue.get()
            y_yaw = next_waypoint.goal.target_pose.pose.position.y - self.curr_waypoint.goal.target_pose.pose.position.y
            x_yaw = next_waypoint.goal.target_pose.pose.position.x - self.curr_waypoint.goal.target_pose.pose.position.x
            yaw = math.atan2(y_yaw, x_yaw)

            goal = self.curr_waypoint

            orientation_quaternion = quaternion_from_euler(0, 0, yaw)
            goal.goal.target_pose.pose.orientation.x = orientation_quaternion[0]
            goal.goal.target_pose.pose.orientation.y = orientation_quaternion[1]
            goal.goal.target_pose.pose.orientation.z = orientation_quaternion[2]
            goal.goal.target_pose.pose.orientation.w = orientation_quaternion[3]

            self.curr_goal_pos = (goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y)
            self.curr_goal = goal
            curr_pose = PoseStamped()
            curr_pose.pose = self.curr_waypoint.goal.target_pose.pose
            curr_pose.header = self.curr_waypoint.goal.target_pose.header
            self.curr_pose_pub.publish(curr_pose)
            # print(curr_pose)

            next_pose = PoseStamped()
            next_pose.pose = next_waypoint.goal.target_pose.pose
            next_pose.header = next_waypoint.goal.target_pose.header
            self.next_pose_pub.publish(next_pose)
            # print(next_pose)

            self.curr_waypoint = next_waypoint
            self.move_base_pub.publish(goal)
            self.almost_there = False

            rospy.loginfo(
                f"Moving autonomously to goal at x,y = {goal.goal.target_pose.pose.position.x}, "
                f"{goal.goal.target_pose.pose.position.y}, orientation = {goal.goal.target_pose.pose.orientation}")


instructions = """

GPS waypoint transformer is now live.
Publish waypoints to gps_waypoints/latlong as vikram_2d_nav/LatLongWayPoint
Alternatively, publish them to /gps_waypoints/navsat as sensor_msgs/NavSatFix
or to /gps_waypoints/odom as  nav_msgs/Odometry

Call the flush_queue service with value True to clear out all the queued waypoints.
Call the pause_journey service with value True to temporarily stop the vehicle.
Send value False to resume the journey.

Visualise /gps_waypoints/goal_array on rviz to see all the waypoints enqueued."""


def main():
    global instructions

    rospy.init_node('gps_waypoints')
    # TODO:
    #   Add ESTOP to the pause journey service as an extreme case.

    tf_object = Transformer()
    rospy.loginfo(instructions)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        tf_object.run()
        r.sleep()

if __name__ == "__main__":
    main()