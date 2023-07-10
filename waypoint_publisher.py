#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geopandas as gpd
import pandas as pd
import osmnx as ox
from shapely import wkt
import matplotlib
from bolt_planner.srv import WaypointGen,WaypointGenResponse
from bolt_planner.msg import LatLongWayPoint

class Waypoint:   
    """
    Class functions:
    *__init__:
        Generates map, initializes publishers
    * generate_waypoints:
        Generates waypoints between start and destination, returns GeoPandasDataFrame
        of Waypoints
    * publish_waypoints:
        Publishes waypoints to the topic /gps_waypoints/latlong 
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
        self.north, self.south, self.east, self.west = north, south, east, west
        self.G = ox.graph_from_bbox(north, south, east, west)
        
        # Publishers
        self.waypoint_pub = rospy.Publisher('/gps_waypoints/latlong', LatLongWayPoint, queue_size=10)
           
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
        
        return data['geometry']
    
    def publish_waypoints(self,req):
        waypoints = self.generate_waypoints(req.orig_lat,req.orig_long,req.dest_lat,req.dest_long)
        res = WaypointGenResponse()
        res.latitudes = list(waypoints.y)
        res.longitudes = list(waypoints.x)
        
        for latitude,longitude in zip(res.latitudes,res.longitudes):
            msg = LatLongWayPoint()
            msg.latitude = latitude
            msg.longitude = longitude
            self.waypoint_pub.publish(msg)
            rospy.sleep(1)
            
        return res
        

def main():
    waypoint_obj = Waypoint()
    
    rospy.init_node('waypoint_generator', anonymous=True)
    rospy.Service("generate_waypoints",WaypointGen,waypoint_obj.publish_waypoints)
    rospy.spin()
                
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass