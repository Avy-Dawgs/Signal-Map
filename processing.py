'''
For data processing.
'''
from numpy import interp, array, argmax
from DataPoint import DataPoint
from shared_types import RssiTime


def location_interpolate(
        lat: list[float],
        lon: list[float],
        time: list[float],
        rssi_time: RssiTime
        ) -> DataPoint: 
    '''
    Interpolate location with distance.

    args: (all lists must be size 2 monotonically increasing)
        lat: lattitude list 
        lon: longitude list
        time: times corresponding to lat and lon lists
        rssi_time: RssiTime instance
    '''
    rssi_lat = interp(rssi_time.time, time, lat)
    rssi_lon = interp(rssi_time.time, time, lon)

    return DataPoint(rssi_lat, rssi_lon, rssi_time.rssi)

def location_interpolate2(
        location_time0: LocationTime,
        location_time1: LocationTime,
        rssi_time: RssiTime
        ) -> DataPoint: 
    '''
    Interpolate location with distance.

    args: (all lists must be size 2 monotonically increasing)
        lat: lattitude list 
        lon: longitude list
        time: times corresponding to lat and lon lists
        rssi_time: RssiTime instance
    '''
    t = array([location_time0.time, location_time1.time])
    rssi_lat = interp(rssi_time.time, t, array([location_time0.lat, location_time1.lat]))
    rssi_lon = interp(rssi_time.time, t, array([location_time0.lon, location_time1.lon]))

    return DataPoint(rssi_lat, rssi_lon, rssi_time.rssi)
