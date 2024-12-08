import numpy as np


class OXTS:

    lat = None
    lon = None
    alt = None
    roll = None
    pitch = None
    yaw = None
    velNorth = None
    velEast = None
    velForward = None
    velLeft = None
    velUp = None
    accx = None
    accy = None
    accz = None
    accForward = None
    accLeft = None
    accUp = None
    angx = None
    angy = None
    angz = None
    angForward = None
    angLeft = None
    angUp = None
    posAcc = None
    velAcc = None
    navStat = None
    numSats = None
    posMode = None
    velMode = None
    oriMode = None
    time = None

    def __init__(self, latitude, longitude, altitude, roll, pitch, yaw, vel_north, vel_east, vel_forward, vel_left, vel_up, acc_x, acc_y, acc_z, acc_forward, acc_left, acc_up, angvel_x, angvel_y, angvel_z, angvel_forward, angvel_left, angvel_up, pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode, time):
        self.lat = latitude
        self.lon = longitude
        self.alt = altitude
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.velNorth = vel_north
        self.velEast = vel_east
        self.velForward = vel_forward
        self.velLeft = vel_left
        self.velUp = vel_up
        self.accx = acc_x
        self.accy = acc_y
        self.accz = acc_z
        self.accForward = acc_forward
        self.accLeft = acc_left
        self.accUp = acc_up
        self.angx = angvel_x
        self.angy = angvel_y
        self.angz = angvel_z
        self.angForward = angvel_forward
        self.angLeft = angvel_left
        self.angUp = angvel_up
        self.posAcc = pos_accuracy
        self.velAcc = vel_accuracy
        self.navStat = navstat
        self.numSats = numsats
        self.posMode = posmode
        self.velMode = velmode
        self.oriMode = orimode
        self.time = time

    def getWorldPos(self):
        return np.array([self.lon, self.lat, self.alt])

    def getVelocityNE(self):
        return np.array([self.velNorth, self.velEast, 0])

    def getVelocity(self):
        return np.array([self.velForward, self.velLeft, self.velUp])

    def getTime(self):
        return self.time