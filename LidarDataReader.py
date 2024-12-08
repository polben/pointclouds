import array
import os
from datetime import datetime

import numpy as np

from OxtsAlignmentUtility import OxtsAlignmentUtility
from utils import UNDEFINED


class LidarDataReader:

    root = UNDEFINED
    currentName = 0
    datamap = {}
    timestamps = []

    VELODYNE_DATA = "//velodyne_points//data//"
    TIMESTAMPS = "//velodyne_points//timestamps.txt"

    oxtsDataReader = None

    def getRoot(self, path):
        files = os.listdir(path)
        if len(files) == 1:
            return self.getRoot(path + "//" + files[0])
        else:
            return path



    def __init__(self, path, oxtsDataReader):
        self.root = self.getRoot(path)
        self.oxtsDataReader = oxtsDataReader
        count = 0

        self.timestamps = self.readTimestamps(self.root + self.TIMESTAMPS)

        files = os.listdir(self.root + self.VELODYNE_DATA)
        for filename in files:

            if filename.endswith(".bin"):
                np_points = self.eatBytes(filename)

                #np_points = OxtsAlignmentUtility.rotation(np.pi / 2, 0, 0) @ np_points.T # align data with xz gound
                #np_points = np_points.T

                # colors = self.calcDepth(np_points)
                colors = None

                self.datamap[filename.strip('.bin')] = (np_points, self.timestamps[count], colors)
                count += 1

            print(str(len(files)) + " / " + str(count))

        if not len(self.datamap.keys()) == len(self.timestamps):
            raise RuntimeError("Timestamps and farmes do not match: keys[" + str(len(self.datamap.keys())) + "] timestamps[" + str(len(self.timestamps)) + "]")
        else:
            print("Frames and timestamps loaded")


    def readTimestamps(self, filename):
        with open(filename, 'r') as file:
            lines = file.readlines()
        lines = [line.strip() for line in lines]

        date_format = "%Y-%m-%d %H:%M:%S.%f"  # Format string to match the input format
        timestamps = []
        for i in range(len(lines)):
            time = lines[i]
            time = time.split('.')
            time = time[0] + "." + time[1][:6]
            timestamps.append( datetime.strptime(time, date_format) )

        return timestamps

    def eatBytes(self, filename):
        with open(self.root + self.VELODYNE_DATA + filename, "rb") as f:
            bin_data = f.read()
            float_data = array.array('f', bin_data)
            numpy_array = np.frombuffer(float_data, dtype=np.float32)
            reshaped_array = numpy_array.reshape(-1, 4)
        return reshaped_array[:, :3].astype(np.float64)

    def getCurrentName(self):
        return self.getfilenames()[self.currentName]

    def getNextWait(self):
        currentTimestamp = self.datamap[self.getCurrentName()][1]
        nextFrame = self.peekNextName()
        if nextFrame == UNDEFINED:
            return 0

        nextTimestamp = self.datamap[nextFrame][1]

        delay = nextTimestamp - currentTimestamp
        ms = delay.total_seconds() * 1000
        return ms

    def getPoints(self, filename):
        return np.ascontiguousarray(self.datamap[filename][0]) # the actual points

    def getTimeStamp(self, filename):
        return self.datamap[filename][1] # the timestamp

    def getDepth(self, filename):
        return np.ascontiguousarray(self.datamap[filename][2]) # the timestamp

    def calcDepth(self, np_points):
        distances = np.linalg.norm(np_points, axis=1).astype(np.float64)

        dist_color = distances / np.max(distances)

        color_buffer = np.zeros((3, len(np_points)), np.float64)
        color = np.sqrt(dist_color)

        color_buffer[0] = color
        color_buffer[1] = color
        color_buffer[2] = color

        return np.ascontiguousarray(color_buffer.T)
        # had problems with passing this array to point_cloud.colors; might happen because of transpose

    def getfilenames(self):
        return list(self.datamap.keys())

    def peekNextName(self):
        if self.currentName < len(self.datamap.keys()) - 1:
            return self.getfilenames()[self.currentName + 1]
        else:
            return UNDEFINED

    def getNextName(self):
        names = self.getfilenames()

        ret = names[self.currentName]
        self.currentName += 1
        if self.currentName >= len(names):
            self.currentName = 0
            return False
        return ret

    def resetNameIterator(self):
        self.currentName = 0

    def getOxtsReader(self):
        return self.oxtsDataReader