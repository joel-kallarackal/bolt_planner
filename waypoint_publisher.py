#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import osmnx as ox
from bolt_planner.srv import WaypointGen,WaypointGenResponse
from bolt_planner.msg import LatLongWayPoint, AlmostThere
import queue

class Waypoint:  
    """
    Class functions:
    *__init__:
        Generates map, initializes publishers
    * generate_waypoints:
        Generates waypoints between start and destination, returns GeoPandasDataFrame
        of Waypoints and adds the waypoints to a queue
    * publish_first_waypoint:
        Publishes waypoints to the topic /gps_waypoints/latlong
    * publish_waypoint:
        Listens to /gps_waypoints/almost_there/ and publishes waypoints
    """
    def __init__(self,north=13.008803,south=12.983204,east=80.222879,west=80.246548):
        """
            Parameters 
            ----------
            north : 
                latitude
            south :
                latitude
            east :
                longitude
            west :
                longitude
        """
        """
        TODO:
            find correct latitudes and longitudes of north, south, east and west 
            of IITM
        """
        self.north, self.south, self.east, self.west = north, south, east, west
        self.G = ox.graph_from_bbox(north, south, east, west)
        self.waypoint_queue = queue.Queue()
        
        #Publishers
        self.waypoint_pub = rospy.Publisher('/gps_waypoints/latlong', LatLongWayPoint, queue_size=10)
        
        #Subscribers
        self.pub_next_waypoint = rospy.Subscriber("/gps_waypoints/almost_there/", AlmostThere, self.publish_waypoint)
        self.start_pub = True
           
    def generate_waypoints(self, lat_orig, long_orig, lat_des, long_des):
        """
            Parameters
            ----------
            lat_orig:
                start location latitude
            long_orig:
                start location longitude
            lat_des:
                destination location latitude
            long_des:
                destination location latitude
        """
        orig = ox.distance.nearest_nodes(self.G, long_orig, lat_orig)
        des = ox.distance.nearest_nodes(self.G, long_des, lat_des)
        route = ox.shortest_path(self.G, orig, des)
        gdf_nodes, gdf_edges = ox.graph_to_gdfs(self.G)
        data = gdf_nodes.loc[route]
        
        for lat,long in zip(data['geometry'].y,data['geometry'].x):
            self.waypoint_queue.put([lat,long])
        
        return data['geometry']
    
    def publish_first_waypoint(self,req):
        """
        Args:
            req (WaypointGenRequest): contains latitude and longitude of current location
            and destination

        Returns:
            res (WaypointGenResponse): contains list of latitudes and longitudes of waypoints 
            along the path
        """
        waypoints = self.generate_waypoints(req.orig_lat,req.orig_long,req.dest_lat,req.dest_long)
        
        res = WaypointGenResponse()
        msg = LatLongWayPoint()
    
        if not self.waypoint_queue.empty():
            latlong = self.waypoint_queue.get()
            msg.latitude = latlong[0]
            msg.longitude = latlong[1]
            self.waypoint_pub.publish(msg)
            res.status = "Successfully published first waypoint"
        else:
            res.status = "Failed to publish first waypoint"
        
        return res
            
    def publish_waypoint(self, data: AlmostThere):
        if data.almost_there and not self.waypoint_queue.empty():
            msg = LatLongWayPoint()
            latlong = self.waypoint_queue.get()
            msg.latitude = latlong[0]
            msg.longitude = latlong[1]
            self.waypoint_pub.publish(msg)
            
        

def main():
    waypoint_obj = Waypoint()
    rospy.init_node('waypoint_generator', anonymous=True)

    # Service
    # Input - latitude and longitude of start and destination
    # Call back function is publish_first_waypoint
    # Outputs success or failure 
    rospy.Service("generate_waypoints",WaypointGen,waypoint_obj.publish_first_waypoint) 
    
    rospy.spin()
                
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass