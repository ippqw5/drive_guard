import math


class Vector3D:
    """
    3D vector class compatible with CARLA Vector3D.
    """
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def __str__(self):
        return f"Vector3D(x={self.x}, y={self.y}, z={self.z})"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if not isinstance(other, Vector3D):
            return False
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6 and abs(self.z - other.z) < 1e-6
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def __add__(self, other):
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Vector3D(self.x * other, self.y * other, self.z * other)
        elif isinstance(other, Vector3D):
            return Vector3D(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            raise TypeError(f"Cannot multiply Vector3D with {type(other)}")
    
    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Vector3D(self.x / other, self.y / other, self.z / other)
        elif isinstance(other, Vector3D):
            return Vector3D(self.x / other.x, self.y / other.y, self.z / other.z)
        else:
            raise TypeError(f"Cannot divide Vector3D by {type(other)}")
    
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


class Location(Vector3D):
    """
    Location class compatible with CARLA Location.
    Inherits from Vector3D as per CARLA API.
    """
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x, y, z)
    
    def __str__(self):
        return f"Location(x={self.x}, y={self.y}, z={self.z})"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if not isinstance(other, Location):
            return False
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6 and abs(self.z - other.z) < 1e-6
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def distance(self, location):
        """
        Returns Euclidean distance from this location to another one.
        
        Args:
            location (Location): The other point to compute the distance with.
            
        Returns:
            float: Distance in meters.
        """
        return math.sqrt((self.x - location.x)**2 + (self.y - location.y)**2 + (self.z - location.z)**2)


class Rotation:
    """
    Rotation class compatible with CARLA Rotation.
    """
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.roll = float(roll)
    
    def __str__(self):
        return f"Rotation(pitch={self.pitch}, yaw={self.yaw}, roll={self.roll})"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if not isinstance(other, Rotation):
            return False
        return (abs(self.pitch - other.pitch) < 1e-6 and 
                abs(self.yaw - other.yaw) < 1e-6 and 
                abs(self.roll - other.roll) < 1e-6)
    
    def __ne__(self, other):
        return not self.__eq__(other)


class Transform:
    """
    Transform class compatible with CARLA Transform.
    """
    def __init__(self, location=None, rotation=None):
        self.location = location if location is not None else Location()
        self.rotation = rotation if rotation is not None else Rotation()
    
    def __str__(self):
        return f"Transform({self.location}, {self.rotation})"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if not isinstance(other, Transform):
            return False
        return self.location == other.location and self.rotation == other.rotation
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def transform(self, in_point):
        """
        Translates a 3D point from local to global coordinates using the current transformation as frame of reference.
        
        Args:
            in_point (Location): Location in the space to which the transformation will be applied.
            
        Returns:
            Location: The transformed point.
        """
        # Simplified transformation - in a real implementation this would involve
        # proper rotation matrix multiplication
        # For now, just add the location offset
        return Location(
            in_point.x + self.location.x,
            in_point.y + self.location.y,
            in_point.z + self.location.z
        )
    
    def get_forward_vector(self):
        """
        Computes a forward vector using the rotation of the object.
        
        Returns:
            Vector3D: Forward vector.
        """
        # Convert yaw to radians and compute forward vector
        yaw_rad = math.radians(self.rotation.yaw)
        return Vector3D(
            math.cos(yaw_rad),
            math.sin(yaw_rad),
            0.0
        )
    
    def get_right_vector(self):
        """
        Computes a right vector using the rotation of the object.
        
        Returns:
            Vector3D: Right vector.
        """
        # Convert yaw to radians and compute right vector
        yaw_rad = math.radians(self.rotation.yaw)
        return Vector3D(
            math.sin(yaw_rad),
            -math.cos(yaw_rad),
            0.0
        )
    
    def get_up_vector(self):
        """
        Computes an up vector using the rotation of the object.
        
        Returns:
            Vector3D: Up vector.
        """
        # Simplified - in a real implementation this would consider pitch and roll
        return Vector3D(0.0, 0.0, 1.0)
    
    def get_matrix(self):
        """
        Computes the 4-matrix representation of the transformation.
        
        Returns:
            list: 4x4 transformation matrix as list of lists.
        """
        # Simplified 4x4 identity matrix with translation
        # In a real implementation, this would include proper rotation
        return [
            [1.0, 0.0, 0.0, self.location.x],
            [0.0, 1.0, 0.0, self.location.y],
            [0.0, 0.0, 1.0, self.location.z],
            [0.0, 0.0, 0.0, 1.0]
        ]
    
    def get_inverse_matrix(self):
        """
        Computes the 4-matrix representation of the inverse transformation.
        
        Returns:
            list: 4x4 inverse transformation matrix as list of lists.
        """
        # Simplified inverse matrix with negative translation
        # In a real implementation, this would include proper inverse rotation
        return [
            [1.0, 0.0, 0.0, -self.location.x],
            [0.0, 1.0, 0.0, -self.location.y],
            [0.0, 0.0, 1.0, -self.location.z],
            [0.0, 0.0, 0.0, 1.0]
        ]


class BoundingBox:
    """
    BoundingBox class compatible with CARLA BoundingBox.
    """
    def __init__(self, location=None, extent=None):
        self.location = location if location is not None else Location()
        self.extent = extent if extent is not None else Vector3D()
        self.rotation = Rotation()  # Add rotation property as per CARLA API
    
    def __str__(self):
        return f"BoundingBox({self.location}, {self.extent})"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if not isinstance(other, BoundingBox):
            return False
        return self.location == other.location and self.extent == other.extent
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def contains(self, world_point, transform):
        """
        Returns True if a point passed in world space is inside this bounding box.
        
        Args:
            world_point (Location): The point in world space to be checked.
            transform (Transform): Contains location and rotation needed to convert this object's local space to world space.
            
        Returns:
            bool: True if the point is inside the bounding box.
        """
        # Simplified implementation - transform world point to local space
        local_point = Location(
            world_point.x - transform.location.x,
            world_point.y - transform.location.y,
            world_point.z - transform.location.z
        )
        
        # Check if point is within the bounding box extents
        return (abs(local_point.x) <= self.extent.x and
                abs(local_point.y) <= self.extent.y and
                abs(local_point.z) <= self.extent.z)
    
    def get_local_vertices(self):
        """
        Returns a list containing the locations of this object's vertices in local space.
        
        Returns:
            list: List of Location objects representing the 8 vertices of the bounding box.
        """
        vertices = []
        for x in [-self.extent.x, self.extent.x]:
            for y in [-self.extent.y, self.extent.y]:
                for z in [-self.extent.z, self.extent.z]:
                    vertices.append(Location(x, y, z))
        return vertices
    
    def get_world_vertices(self, transform):
        """
        Returns a list containing the locations of this object's vertices in world space.
        
        Args:
            transform (Transform): Contains location and rotation needed to convert this object's local space to world space.
            
        Returns:
            list: List of Location objects representing the 8 vertices in world space.
        """
        local_vertices = self.get_local_vertices()
        world_vertices = []
        for vertex in local_vertices:
            world_vertex = transform.transform(vertex)
            world_vertices.append(world_vertex)
        return world_vertices
