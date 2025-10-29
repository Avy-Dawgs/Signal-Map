#!/usr/bin/env python3

import struct
from typing import Optional, List, Tuple
import math
import time
import argparse
import csv
import random
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2


# MAVLink TUNNEL constants
PAYLOAD_LEN = 12
PAYLOAD_BUF = 128
TYPE_RSSI_GLOBAL = 4  # Tunnel type for lat/lon/rssi heatmap data
TYPE_VICTIM_MARKER = 2  # Tunnel type for final victim marker


# Helper functions
def pack_triplet(lat: float, lon: float, rssi: float) -> bytes:
    """Pack lat/lon/rssi into tunnel payload format (12 bytes data + padding)."""
    body = struct.pack("<fff", float(lat), float(lon), float(rssi))
    return body + bytes(PAYLOAD_BUF - len(body))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance in meters between two GPS coordinates using Haversine formula."""
    R = 6371000.0  # Earth radius in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb/2)**2
    return 2*R*math.asin(math.sqrt(a))


def gaussian_rssi(d_m: float, peak: float, floor: float, sigma_m: float) -> float:
    """
    Calculate synthetic RSSI based on distance from beacon using Gaussian model.
    
    Args:
        d_m: Distance in meters
        peak: Peak RSSI at distance 0
        floor: Minimum RSSI at infinite distance
        sigma_m: Standard deviation in meters (controls falloff rate)
    
    Returns:
        float: RSSI value
    """
    strength = math.exp(-0.5*(d_m/sigma_m)**2)
    return floor + (peak - floor) * strength


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


class MAVLinkDataHandler:
    """
    Handles MAVLink communication with SITL, collects location data,
    computes RSSI, stores in DataPointCollection, and sends tunnel messages.
    """
    
    def __init__(self, rx_connection: str, tx_host: str, tx_port: int,
                 sysid: int = 255, compid: int = 0,
                 beacon_lat: float = 0.0, beacon_lon: float = 0.0,
                 peak_rssi: float = -30.0, floor_rssi: float = -95.0,
                 sigma_m: float = 40.0, sample_hz: float = 1.0,
                 min_dist_m: float = 2.5, noise_stddev: float = 3.0,
                 csv_file: Optional[str] = None):
        """
        Initialize MAVLink handler for 457 kHz avalanche beacon simulation.
        
        Args:
            rx_connection: MAVLink RX connection string
            tx_host: TX host for sending tunnel messages
            tx_port: TX UDP port
            sysid: MAVLink system ID
            compid: MAVLink component ID
            beacon_lat: Beacon latitude for RSSI calculation
            beacon_lon: Beacon longitude for RSSI calculation
            peak_rssi: Peak RSSI at beacon location (default: -30.0 dBm for 457 kHz)
            floor_rssi: Minimum RSSI at far distances (default: -95.0 dBm for 457 kHz)
            sigma_m: Standard deviation for Gaussian falloff in meters (default: 40.0m for 457 kHz)
            sample_hz: Sample rate in Hz (default: 1.0, typical for avalanche beacons)
            min_dist_m: Minimum distance (meters) drone must move before sending another tunnel message (default: 2.5m)
            noise_stddev: Standard deviation of Gaussian noise added to RSSI in dBm (default: 3.0 for 457 kHz)
            csv_file: Optional CSV file path for logging
        """
        self.collection = DataPointCollection()
        
        # Connection parameters
        self.rx_connection = rx_connection
        self.tx_host = tx_host
        self.tx_port = tx_port
        self.sysid = sysid
        self.compid = compid
        
        # Beacon model parameters
        self.beacon_lat = beacon_lat
        self.beacon_lon = beacon_lon
        self.peak_rssi = peak_rssi
        self.floor_rssi = floor_rssi
        self.sigma_m = sigma_m
        self.noise_stddev = noise_stddev
        
        # Sampling parameters
        self.sample_hz = sample_hz
        self.min_dist_m = min_dist_m
        self.min_dt = 1.0 / max(0.1, sample_hz)
        
        # State tracking
        self.last_emit_t = 0.0
        self.last_sent_lat = None  # Track last sent position for distance check
        self.last_sent_lon = None
        self.mission_active = False
        
        # CSV logging
        self.csv_file = csv_file
        self.csv_fp = None
        self.csv_writer = None
        
        # MAVLink connections (initialized in connect())
        self.rx = None
        self.tx = None
        self.mav = None
    
    def _add_noise_to_rssi(self, rssi: float) -> float:
        """
        Add Gaussian noise to RSSI value.
        
        Args:
            rssi: Base RSSI value in dBm
            
        Returns:
            float: RSSI with added noise
        """
        if self.noise_stddev > 0:
            noise = random.gauss(0, self.noise_stddev)
            return rssi + noise
        return rssi
    
    def _calculate_victim_location(self) -> Optional[Tuple[float, float, float]]:
        """
        Calculate victim location using weighted average of strongest signals.
        Returns (lat, lon, avg_rssi) or None if no data.
        """
        if not self.collection:
            return None
        
        # Get top 10% of strongest signals (minimum 3 points)
        all_points = sorted(self.collection.data_points, key=lambda p: p.rssi, reverse=True)
        top_count = max(3, len(all_points) // 10)
        top_points = all_points[:top_count]
        
        if not top_points:
            return None
        
        # Weighted average by RSSI (convert to linear scale for weighting)
        # Higher RSSI (less negative) gets more weight
        total_weight = 0.0
        weighted_lat = 0.0
        weighted_lon = 0.0
        rssi_sum = 0.0
        
        for point in top_points:
            # Convert dBm to linear scale: weight = 10^(rssi/10)
            weight = 10 ** (point.rssi / 10.0)
            total_weight += weight
            weighted_lat += point.latitude * weight
            weighted_lon += point.longitude * weight
            rssi_sum += point.rssi
        
        if total_weight > 0:
            avg_lat = weighted_lat / total_weight
            avg_lon = weighted_lon / total_weight
            avg_rssi = rssi_sum / len(top_points)
            return (avg_lat, avg_lon, avg_rssi)
        
        return None
    
    def connect(self):
        """Establish MAVLink connections."""
        print(f"[RX] Connecting to {self.rx_connection}")
        self.rx = mavutil.mavlink_connection(
            self.rx_connection, autoreconnect=True, force_mavlink2=True
        )
        
        print(f"[TX] Connecting to udpout:{self.tx_host}:{self.tx_port}")
        self.tx = mavutil.mavlink_connection(
            f"udpout:{self.tx_host}:{self.tx_port}",
            autoreconnect=False, force_mavlink2=True
        )
        
        # Setup MAVLink message encoder
        self.mav = mav2.MAVLink(None)
        self.mav.srcSystem = self.sysid
        self.mav.srcComponent = self.compid
        
        # Setup CSV logging if requested
        if self.csv_file:
            self.csv_fp = open(self.csv_file, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fp)
            self.csv_writer.writerow(["timestamp", "latitude", "longitude", "rssi", "distance_m"])
            print(f"[LOG] Writing to {self.csv_file}")
        
        # Send initial heartbeats to establish presence in QGC
        print("[INFO] Sending initial heartbeats...")
        for _ in range(3):
            self._send_heartbeat()
            time.sleep(0.2)
        
        print("[INFO] Waiting for vehicle heartbeat...")
        self.rx.wait_heartbeat()
        print(f"[INFO] Heartbeat received from system {self.rx.target_system}")
    
    def _send_heartbeat(self):
        """Send heartbeat to keep connection alive."""
        if self.mav and self.tx:
            hb = self.mav.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            self.tx.mav.send(hb)
            # Uncomment for debugging: print("[HB] Heartbeat sent")
    
    def _send_tunnel_message(self, lat: float, lon: float, rssi: float, msg_type: int = TYPE_RSSI_GLOBAL):
        """Send a tunnel message with lat/lon/rssi data."""
        if self.mav and self.tx:
            payload = pack_triplet(lat, lon, rssi)
            tmsg = self.mav.tunnel_encode(
                target_system=0,
                target_component=0,
                payload_type=msg_type,
                payload_length=PAYLOAD_LEN,
                payload=payload
            )
            self.tx.mav.send(tmsg)
            type_name = "HEATMAP" if msg_type == TYPE_RSSI_GLOBAL else "MARKER"
            print(f"[TX] TUNNEL type={msg_type} ({type_name}) lat={lat:.6f} lon={lon:.6f} rssi={rssi:.1f}")
    
    def send_victim_marker(self):
        """Send final victim marker using weighted average of strongest signals."""
        victim_loc = self._calculate_victim_location()
        if victim_loc:
            lat, lon, avg_rssi = victim_loc
            print(f"[MARKER] Sending final victim marker using weighted average of top signals:")
            print(f"[MARKER] Location: ({lat:.6f}, {lon:.6f}) Avg RSSI: {avg_rssi:.1f} dBm")
            self._send_tunnel_message(lat, lon, avg_rssi, TYPE_VICTIM_MARKER)
            
            # Show comparison with simple strongest point
            strongest = self.collection.strongest()
            if strongest:
                dist = haversine_m(lat, lon, strongest.latitude, strongest.longitude)
                print(f"[MARKER] Strongest single point was ({strongest.latitude:.6f}, {strongest.longitude:.6f}) at {strongest.rssi:.1f} dBm")
                print(f"[MARKER] Distance between methods: {dist:.2f}m")
        else:
            print("[MARKER] No data points collected, cannot send marker")
    
    def _process_position(self, lat: float, lon: float, now: float):
        """Process a position update, calculate RSSI, store, and send tunnel message."""
        # Rate limiting (1 sample per second)
        if (now - self.last_emit_t) < self.min_dt:
            return
        
        # Calculate RSSI based on distance from beacon
        distance = haversine_m(lat, lon, self.beacon_lat, self.beacon_lon)
        rssi_base = gaussian_rssi(distance, self.peak_rssi, self.floor_rssi, self.sigma_m)
        rssi = self._add_noise_to_rssi(rssi_base)
        
        # ALWAYS store in collection (every sample)
        self.collection.add(lat, lon, rssi)
        
        # Log to CSV
        if self.csv_writer:
            self.csv_writer.writerow([now, lat, lon, rssi, distance])
        
        # Check if we should send tunnel message (maintain consistent min_dist_m spacing)
        if self.last_sent_lat is None or self.last_sent_lon is None:
            # First sample - always send
            self._send_tunnel_message(lat, lon, rssi, TYPE_RSSI_GLOBAL)
            self.last_sent_lat = lat
            self.last_sent_lon = lon
            print(f"[DATA] Stored & sent (first): dist={distance:.1f}m from beacon (total: {len(self.collection)})")
        else:
            # Check distance from last sent position
            dist_from_last = haversine_m(lat, lon, self.last_sent_lat, self.last_sent_lon)
            
            if dist_from_last >= self.min_dist_m:
                # Moved enough to send
                # If we moved much more than min_dist_m in one step, send multiple interpolated points
                num_points = int(dist_from_last / self.min_dist_m)
                
                if num_points > 1:
                    # Interpolate intermediate points to maintain consistent spacing
                    for i in range(1, num_points + 1):
                        fraction = (i * self.min_dist_m) / dist_from_last
                        interp_lat = self.last_sent_lat + (lat - self.last_sent_lat) * fraction
                        interp_lon = self.last_sent_lon + (lon - self.last_sent_lon) * fraction
                        interp_dist = haversine_m(interp_lat, interp_lon, self.beacon_lat, self.beacon_lon)
                        interp_rssi_base = gaussian_rssi(interp_dist, self.peak_rssi, self.floor_rssi, self.sigma_m)
                        interp_rssi = self._add_noise_to_rssi(interp_rssi_base)
                        
                        self._send_tunnel_message(interp_lat, interp_lon, interp_rssi, TYPE_RSSI_GLOBAL)
                        self.last_sent_lat = interp_lat
                        self.last_sent_lon = interp_lon
                    
                    print(f"[DATA] Stored & sent {num_points} interpolated points: moved {dist_from_last:.1f}m (total: {len(self.collection)})")
                else:
                    # Normal case: just send current position
                    self._send_tunnel_message(lat, lon, rssi, TYPE_RSSI_GLOBAL)
                    self.last_sent_lat = lat
                    self.last_sent_lon = lon
                    print(f"[DATA] Stored & sent: dist={distance:.1f}m from beacon (total: {len(self.collection)})")
            else:
                print(f"[DATA] Stored only: dist={distance:.1f}m from beacon (total: {len(self.collection)}, moved {dist_from_last:.1f}m < {self.min_dist_m:.1f}m)")
        
        self.last_emit_t = now
    
    def run(self):
        """Main loop: receive location data, store points, send tunnel messages."""
        if not self.rx or not self.tx:
            raise RuntimeError("Must call connect() before run()")
        
        print("[INFO] Starting main loop. Press Ctrl+C to stop and send victim marker.")
        next_hb = 0.0
        
        try:
            while True:
                now = time.time()
                
                # Send periodic heartbeat
                if now >= next_hb:
                    self._send_heartbeat()
                    next_hb = now + 1.0
                
                # Read telemetry (check for GLOBAL_POSITION_INT and MISSION_ITEM_REACHED)
                msg = self.rx.recv_match(
                    type=["GLOBAL_POSITION_INT", "MISSION_ITEM_REACHED", "MISSION_CURRENT"],
                    blocking=True,
                    timeout=0.25
                )
                
                if not msg:
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    self._process_position(lat, lon, now)
                
                elif msg_type == "MISSION_ITEM_REACHED":
                    print(f"[MISSION] Reached waypoint {msg.seq}")
                
                elif msg_type == "MISSION_CURRENT":
                    if msg.seq == 0 and not self.mission_active:
                        print("[MISSION] Mission started")
                        self.mission_active = True
                    
        except KeyboardInterrupt:
            print("\n[INFO] Stopping...")
            self.send_victim_marker()
            print(f"[STATS] Total data points collected: {len(self.collection)}")
            if self.collection:
                victim_loc = self._calculate_victim_location()
                if victim_loc:
                    lat, lon, avg_rssi = victim_loc
                    print(f"[STATS] Calculated victim location (weighted avg): ({lat:.6f}, {lon:.6f}) at {avg_rssi:.1f} dBm")
        finally:
            if self.csv_fp:
                self.csv_fp.close()
                print(f"[LOG] Closed {self.csv_file}")


def main():
    """Main entry point with argument parsing."""
    parser = argparse.ArgumentParser(
        description="457 kHz Avalanche Beacon Simulator - Receive location data from SITL, calculate synthetic RSSI, send tunnel messages to QGC"
    )
    
    # Connection parameters
    parser.add_argument("--rx", default="udpin:0.0.0.0:14557",
                       help="Telemetry RX connection string (default: udpin:0.0.0.0:14557)")
    parser.add_argument("--tx-host", default="172.21.192.1",
                       help="TX host for QGC (default: 172.21.192.1)")
    parser.add_argument("--tx-port", type=int, default=14550,
                       help="TX UDP port for QGC (default: 14550)")
    parser.add_argument("--sysid", type=int, default=255,
                       help="MAVLink system ID (default: 255)")
    parser.add_argument("--compid", type=int, default=0,
                       help="MAVLink component ID (default: 0)")
    
    # Beacon model parameters (457 kHz avalanche beacon)
    parser.add_argument("--beacon-lat", type=float, required=True,
                       help="Beacon latitude (required)")
    parser.add_argument("--beacon-lon", type=float, required=True,
                       help="Beacon longitude (required)")
    parser.add_argument("--peak", type=float, default=-30.0,
                       help="Peak RSSI at beacon location (default: -30.0 dBm for 457 kHz)")
    parser.add_argument("--floor", type=float, default=-95.0,
                       help="Floor RSSI at far distance (default: -95.0 dBm for 457 kHz)")
    parser.add_argument("--sigma-m", type=float, default=40.0,
                       help="Gaussian falloff sigma in meters (default: 40.0m for 457 kHz)")
    parser.add_argument("--noise-stddev", type=float, default=3.0,
                       help="Standard deviation of Gaussian noise added to RSSI in dBm (default: 3.0 for 457 kHz, set to 0 for no noise)")
    
    # Sampling parameters
    parser.add_argument("--sample-hz", type=float, default=1.0,
                       help="Sample rate in Hz (default: 1.0, simulating avalanche beacon)")
    parser.add_argument("--min-dist-m", type=float, default=2.5,
                       help="Minimum distance to send tunnel message in meters (default: 2.5)")
    
    # Logging
    parser.add_argument("--csv", type=str, default=None,
                       help="CSV file path for logging (optional)")
    
    args = parser.parse_args()
    
    # Create handler
    handler = MAVLinkDataHandler(
        rx_connection=args.rx,
        tx_host=args.tx_host,
        tx_port=args.tx_port,
        sysid=args.sysid,
        compid=args.compid,
        beacon_lat=args.beacon_lat,
        beacon_lon=args.beacon_lon,
        peak_rssi=args.peak,
        floor_rssi=args.floor,
        sigma_m=args.sigma_m,
        sample_hz=args.sample_hz,
        min_dist_m=args.min_dist_m,
        noise_stddev=args.noise_stddev,
        csv_file=args.csv
    )
    
    # Connect and run
    handler.connect()
    handler.run()


if __name__ == "__main__":
    main()