import struct
from typing import Optional, List
import math


class DataPoint:
    """
    Represents a single signal detection with GPS coordinates and RSSI value.
    """
    
    def __init__(self, latitude: float, longitude: float, rssi: float):
        """
        Initialize a data point for signal detection.
        
        Args:
            latitude (float): Latitude in decimal degrees (-90 to 90)
            longitude (float): Longitude in decimal degrees (-180 to 180)
            rssi (float): RSSI value
            
        Raises:
            ValueError: If coordinates are out of valid range
            TypeError: If inputs cannot be converted to float
        """
        try:
            lat = float(latitude)
            lon = float(longitude)
            rssi_val = float(rssi)
        except (ValueError, TypeError) as e:
            raise TypeError(f"All parameters must be convertible to float: {e}")
            
        if not (-90 <= lat <= 90):
            raise ValueError(f"Latitude must be between -90 and 90 degrees, got {lat}")
        if not (-180 <= lon <= 180):
            raise ValueError(f"Longitude must be between -180 and 180 degrees, got {lon}")
            
        self.latitude = lat
        self.longitude = lon
        self.rssi = rssi_val
    
    def __eq__(self, other) -> bool:
        """Check equality with another DataPoint."""
        if not isinstance(other, DataPoint):
            return False
        return (abs(self.latitude - other.latitude) < 1e-9 and 
                abs(self.longitude - other.longitude) < 1e-9 and 
                abs(self.rssi - other.rssi) < 1e-6)
    
    def distance_to(self, other: 'DataPoint') -> float:
        """
        Calculate the distance to another DataPoint.
        
        Args:
            other (DataPoint): Another DataPoint to calculate distance to
            
        Returns:
            float: Distance in meters
            
        Raises:
            TypeError: If other is not a DataPoint instance
        """
        if not isinstance(other, DataPoint):
            raise TypeError("Can only calculate distance to another DataPoint")
            
        # Haversine formula
        R = 6371000  # Earth's radius in meters
        lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        lat2, lon2 = math.radians(other.latitude), math.radians(other.longitude)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return R * c
    
    def to_tunnel_payload(self) -> bytes:
        """
        Convert this data point to a 12-byte tunnel payload for transmission.
        
        Format:
        - Bytes 0-3: float latitude (little-endian)
        - Bytes 4-7: float longitude (little-endian)
        - Bytes 8-11: float rssi (little-endian)
        
        Returns:
            bytes: 12-byte payload ready for transmission
        """
        return struct.pack('<fff', self.latitude, self.longitude, self.rssi)


class DataPointCollection:
    """
    Data structure for holding and managing multiple DataPoints.
    """

    def __init__(self):
        """Initialize the data point collection."""
        self.data_points: List[DataPoint] = []

    def __len__(self) -> int:
        """Return the number of data points in the collection."""
        return len(self.data_points)

    def add(self, latitude: float, longitude: float, rssi: float) -> None:
        """
        Add a new DataPoint to the collection.

        Args:
            latitude (float): Latitude in decimal degrees (-90 to 90)
            longitude (float): Longitude in decimal degrees (-180 to 180)
            rssi (float): RSSI value
            
        Raises:
            ValueError: If coordinates are out of valid range
            TypeError: If inputs cannot be converted to float
        """
        point = DataPoint(latitude, longitude, rssi)
        self.data_points.append(point)
    
    def add_point(self, point: DataPoint) -> None:
        """
        Add an existing DataPoint to the collection.
        
        Args:
            point (DataPoint): DataPoint instance to add
            
        Raises:
            TypeError: If point is not a DataPoint instance
        """
        if not isinstance(point, DataPoint):
            raise TypeError("Can only add DataPoint instances")
        self.data_points.append(point)

    def latest(self) -> Optional[DataPoint]:
        """
        Get the most recent DataPoint.
        
        Returns:
            DataPoint: Most recently added DataPoint, or None if empty
        """
        return self.data_points[-1] if self.data_points else None

    def strongest(self) -> Optional[DataPoint]:
        """
        Find the DataPoint with the strongest signal (highest RSSI).

        Returns:
            DataPoint: DataPoint with strongest signal, or None if empty
        """
        if not self.data_points:
            return None
        return max(self.data_points, key=lambda point: point.rssi)
    
    def weakest(self) -> Optional[DataPoint]:
        """
        Find the DataPoint with the weakest signal (lowest RSSI).

        Returns:
            DataPoint: DataPoint with weakest signal, or None if empty
        """
        if not self.data_points:
            return None
        return min(self.data_points, key=lambda point: point.rssi)

    def clear(self) -> None:
        """Remove all DataPoints from the collection."""
        self.data_points.clear()
    
    def filter_by_rssi(self, min_rssi: Optional[float] = None, max_rssi: Optional[float] = None) -> 'DataPointCollection':
        """
        Filter DataPoints by RSSI range.
        
        Args:
            min_rssi (float, optional): Minimum RSSI value (inclusive)
            max_rssi (float, optional): Maximum RSSI value (inclusive)
            
        Returns:
            DataPointCollection: New collection with filtered points
        """
        filtered = DataPointCollection()
        for point in self.data_points:
            if min_rssi is not None and point.rssi < min_rssi:
                continue
            if max_rssi is not None and point.rssi > max_rssi:
                continue
            filtered.add_point(point)
        return filtered