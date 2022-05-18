from ast import Global
import bisect
from vive_pb2 import *
import numpy as np
from transforms3d import quaternions
from sortedcontainers import SortedDict
from vive_utils import average_transforms


class ViveLog:
    def __init__(self, filename: str):
        self.filename:str = filename
        self.collection: GlobalCollection = GlobalCollection()

        f = open(filename, "rb")
        self.collection.ParseFromString(f.read())
        f.close()

        self.sorted_dict = SortedDict()
        for message in self.collection.messages:
            self.sorted_dict[message.time_since_epoch] = message

    def get_tagged_positions(self) -> list:
        """
        Retrieve tagged positions from t his log

        :return list: tagged positions
        """        
        positions:list = []
        for position in self.collection.tagged_positions:
            positions.append([position.x, position.y, position.z])

        return positions

    def get_first_last_timestamps(self) -> tuple:
        """
        Returns first and mast timestamps (UTC)

        :return tuple: a tuple containing first and last timestamp
        """
        return (self.collection.messages[0].time_since_epoch, self.collection.messages[-1].time_since_epoch)

    def get_trackers_serial_numbers(self) -> list:
        """
        Retrieve all tracker's serial numbers mentionned in the log

        :return list: list of (str) tracker's serial number
        """
        serial_numbers = set()

        for message in self.collection.messages:
            for tracker in message.trackers:
                if tracker.device_type == "tracker":
                    serial_numbers.add(tracker.serial_number)

        return list(serial_numbers)

    def _tracker_pose(self, message: GlobalMsg, serial_number: str) -> np.array:
        """
        Find a given tracker pose in a message

        :param str serial_number: tracker's serial
        :param GlobalMsg message: the message
        :raises ValueError: if the tracker is not found in the message
        :return np.array: tracker's pose
        """
        for tracker in message.trackers:
            if tracker.serial_number == serial_number:
                frame = np.eye(4)
                frame[:3, 3] = tracker.pos.x, tracker.pos.y, tracker.pos.z

                orientation = tracker.orientation
                frame[:3, :3] = quaternions.quat2mat((orientation.qw, orientation.qx, orientation.qy, orientation.qz))

                return frame

        raise ValueError(f"Tracker {serial_number} not found in the given message")

    def contains(self, timestamp:int) -> bool:
        """
        Checks if the log contains a given timestamp

        :param int timestamp: the timestamp
        :return bool: True if the timestamp is in this log
        """        
        min, max = self.get_first_last_timestamps()
        return min < timestamp < max

    def get_pose(self, serial_number: str, timestamp: int) -> np.array:
        """
        Get the logged pose at a given time

        :param float timestamp: the time since epoch timestamp (microseconds)
        :raises ValueError: if the timestamp is not in the range of this log
        :return np.array: tracker to field pose
        """
        if not self.contains(timestamp):
            raise ValueError(f"Timestamp {timestamp} is not in range of vive logs")

        # Using binary search to find before and after
        index = self.sorted_dict.bisect(timestamp)
        before = self.sorted_dict.iloc[index - 1]
        after = self.sorted_dict.iloc[index]
        alpha = (timestamp - before) / (after - before)

        frame_before = self._tracker_pose(self.sorted_dict[before], serial_number)
        frame_after = self._tracker_pose(self.sorted_dict[after], serial_number)

        return average_transforms(frame_before, frame_after, alpha)
