from geometry_msgs.msg import Point
import math

class ALEXObject():
    
    def __init__(self, position: Point):
        self.position = position
    
    @staticmethod
    def point_offset_origin(x:float, y:float, offset:float):
        """
        Offset the point along the axis of the origin of the robot.
        """
        # Calculate distance from origin to original point
        distance = math.sqrt(x**2 + y**2)
        
        # Calculate scaling factor
        scaling_factor = 1 - (offset / distance)
        
        # Calculate new coordinates
        x1 = x * scaling_factor
        y1 = y * scaling_factor
        
        return x1, y1
    
    def get_pre_pick_up_pose(self, position: Point, offset: float=50):
        value = self.point_offset_origin(x = position.x, y = position.y, offset = offset)       
        return Point(*value, position.z)
    
    def get_pick_up_pose(self, position: Point, offset: float=30):
        value = self.point_offset_origin(x = position.x, y = position.y, offset = -offset)
        return Point(*value, position.z)
    
    def prep(self):
        raise NotImplementedError('Child class needs to implement prep/pick locations')     
        
    def pick(self):
        raise NotImplementedError('Child class needs to implement prep/pick locations')
    
    def place(self, target_position: Point):
        raise NotImplementedError('Child class needs to implement prep/pick locations')
    
    def move(self, target_position: Point):
        raise NotImplementedError('Child class needs to implement move to locations')
    
