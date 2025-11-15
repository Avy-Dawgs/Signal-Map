'''
Simple logging library. 

Level values range from 0 to 100. Messages that have a lower level than the 
current logging level will be printed/logged.
For example, a message with a level of 0 will always be printed, and a message 
with a level of 100 will only be printed when the logging level is set to 100.
'''

from io import TextIOWrapper
from typing import Optional

__CURRENT_LOGGING_LEVEL: int = 0 # no logging by default
__LOG_FILE: Optional[TextIOWrapper] = None

def set_logging_level(
        level: int
        ) -> None: 
    '''
    Set logging level. (0 - 100)
    Logs that have a value that is less than or equal to this will be logged/printed.
    '''
    global __CURRENT_LOGGING_LEVEL

    if (level < 0):
        raise ValueError(f"logging level set to {level}, which is below the minimum value of 0.")
    if (level > 100):
        raise ValueError(f"logging level set to {level}, which is above the maximum value of 100.")

    __CURRENT_LOGGING_LEVEL = level


def open_log_file(
        filename: str
        ) -> None: 
    '''
    Initiate logging by opening the log file.
    '''
    global __LOG_FILE
    if __LOG_FILE:
        try:
            __LOG_FILE.close()
        except: 
            pass
    try:
        __LOG_FILE = open(filename)
    except: 
        __LOG_FILE = None

def close_log_file(
        ) -> None: 
    '''
    Deinitiate logging by closing the log file.
    '''
    global __LOG_FILE
    if __LOG_FILE:
        __LOG_FILE.close()
        __LOG_FILE = None

def log_msg(
        msg: str, 
        level: int = 100
        ) -> None: 
    '''
    Log a message.

    args: 
        msg: message to log 
        level: 0-100 value
    '''
    global __CURRENT_LOGGING_LEVEL
    global __LOG_FILE

    if not __LOG_FILE or level < 0: 
        return

    if level <= __CURRENT_LOGGING_LEVEL:
        __LOG_FILE.write(msg)


def logprint_msg(
        msg: str, 
        level: int = 100
        ) -> None:
    '''
    Logs and prints a message.

    args: 
        msg: message to log 
        level: 0-100 value
    '''
    global __CURRENT_LOGGING_LEVEL
    global __LOG_FILE

    if not __LOG_FILE or level < 0: 
        return
    if level <= __CURRENT_LOGGING_LEVEL:
        __LOG_FILE.write(msg)
        print(msg)


def print_msg(
        msg: str, 
        level: int = 100
        ) -> None:
    '''
    Prints a message.

    args: 
        msg: message to log 
        level: 0-100 value
    '''
    global __CURRENT_LOGGING_LEVEL
    global __LOG_FILE

    if not __LOG_FILE or level < 0: 
        return
    if level <= __CURRENT_LOGGING_LEVEL:
        print(msg)
