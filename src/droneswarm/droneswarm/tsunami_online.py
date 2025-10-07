



debug_takeoff_height = -5.0  # meters

def tsunami_online_loop(self):
    # TODO

    #self.get_logger().info(f"SUSUHIIIII")

    # testy:
    test_latitude = 37.412833
    test_longitude = -121.998055

    # Wait until drone have reached takeoff altitude
    if self.vehicle_local_z > debug_takeoff_height + 0.5:  # z is negative when
        self.publish_position_setpoint(0.0, 0.0, debug_takeoff_height)
        return

 

    lat, long = self.traversal_order_gps[30]  # first waypoint in traversal order
    #x, y = self.map_projection.global_to_local(lat, long)
    x, y = self.map_projection.global_to_local(test_latitude, test_longitude)

    self.publish_position_setpoint(x, y, -5.0)