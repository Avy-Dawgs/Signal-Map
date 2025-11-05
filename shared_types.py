'''
Types shared among files.
'''


class LocationTime: 
    '''
    Location with an associated time.
    '''
    rssi: float 
    lat: float 
    lon: float

    def __init__(self, lat: float, lon: float, time: float): 
        '''
        Constructor, simply takes values.
        '''
        self.lat = lat 
        self.lon = lon 
        self.time = time


class RssiTime: 
    '''
    RSSI with an associated time.
    '''
    rssi: float 
    time: float 

    def __init__(self, rssi: float, time: float): 
        '''
        Constructor.
        '''
        self.rssi = rssi 
        self.time = time

    def __str__(self):
        '''
        Converts to string.
        '''
        return f"rssi: {self.rssi}; time: {self.time}; "

