import math
import time
from time import sleep

import numpy as np
import open3d as o3d
import cupy as cp
from win32comext.shell.shellcon import SHARD_PATHA

from InputListener import InputListener
from ShapeUtility import ShapeUtility
from VoxelData import VoxelData


class PointcloudController:
    lidarDataReader = None
    pointCloudData = None
    oxtsAlignment = None

    camera = None

    vis = None
    width = None
    height = None

    xyz = None

    mouseSens = 0.005
    speed = 0.3

    inputListener = None
    mousePos = np.array([0, 0]).astype(np.float64)

    anim = False

    image = np.array([])
    objects_3d = {}

    blue = np.array([100, 255, 255])
    white = np.array([255, 255, 255])
    green = np.array([50, 255, 50])

    def __init__(self, lidarDataReader, pointCloudData, oxtsAlignmentUtility, camera):
        self.camera = camera
        self.oxtsAlignment = oxtsAlignmentUtility

        self.width = camera.resx
        self.height = camera.resy


        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(width=self.width, height=self.height)

        self.lidarDataReader = lidarDataReader
        self.pointCloudData = pointCloudData

        self.vis.get_render_option().load_from_json("renderopt.json")


        self.xyz = np.array([0, 0, 0]).astype(np.float64)



        # Register the custom callback for the 'D' key (ASCII 68)
        self.vis.register_key_callback(68, self.disable_depth_map)
        self.vis.register_key_callback(80, self.disable_depth_map)

        self.inputListener = InputListener()
        self.inputListener.listen()

    def disable_depth_map(self, vis):
        return False  # Return False to prevent default behavior

    def composeImage(self, *args):
        self.image[:] = 0

        for a in args:


            if self.image.any():
                self.image += self.camera.projectAsNumpy(a[0], a[1])
            else:
                self.image = self.camera.projectAsNumpy(a[0], a[1])

        for k in self.objects_3d:
            if self.image.any():
                self.image += self.camera.projectAsNumpy(self.objects_3d[k], self.blue)
            else:
                self.image = self.camera.projectAsNumpy(self.objects_3d[k], self.blue)

    def object(self, key, function, *args):
        if key not in self.objects_3d.keys():
            self.objects_3d[key] = function(*args)
            print(len(self.objects_3d[key]))
            print("new obj")


    def getGrid(self, *args):
        size = args[0]
        density = 10 # each grid cell has 100 points
        cells = 5 # per axis

        gridOrigin = np.array([0, 0, 0])#-self.camera.pos
        gridOrigin = np.int32(gridOrigin)
        # print(pos)

        h2 = int(float(cells)/2.0)


        boxes = ShapeUtility.box(gridOrigin, size, density)  # x-axis points toward facing of car

        for x in range(gridOrigin[0] - h2, gridOrigin[0] + h2):
            for y in range(gridOrigin[1] - h2, gridOrigin[1] + h2):
                for z in range(gridOrigin[2] - h2, gridOrigin[2] + h2):
                    boxes = ShapeUtility.np_append(boxes, ShapeUtility.box(np.array([x, y, z]), size, density))


        return boxes

    def getPlaneGrid(self, *args):
        size = args[0]
        cells = 50
        density = 10 # each grid cell has 100 points

        gridOrigin = np.array([0, 0, 0])  # -self.camera.pos
        h2 = int(float(cells)/2.0)

        squares = ShapeUtility.square(gridOrigin, size, density)  # x-axis points toward facing of car

        for x in range(gridOrigin[0] - h2, gridOrigin[0] + h2):
            for y in range(gridOrigin[1] - h2, gridOrigin[1] + h2):
                squares = ShapeUtility.np_append(squares, ShapeUtility.square(np.array([x, y, 0]) * size, size, density))

        return squares

    def axes(self):
        axlen = 10
        axpoints = 500


        xp = np.linspace(0, axlen, axpoints)
        yp = np.zeros(axpoints)
        zp = np.zeros(axpoints)

        points = np.column_stack((xp, yp, zp)) # x-axis points toward facing of car


        xp = np.zeros(axpoints)
        yp = np.linspace(0, axlen, axpoints)
        zp = np.zeros(axpoints)

        ypoints = np.column_stack((xp, yp, zp)) # positive y is to the right
        points = np.concatenate((points, ypoints), axis=0)

        xp = np.zeros(axpoints)
        yp = np.zeros(axpoints)
        zp = np.linspace(-1, int(axlen / 3), axpoints)

        zpoints = np.column_stack((xp, yp, zp))
        points = np.concatenate((points, zpoints), axis=0)
        return points

    def controller(self):
        sleep_fps = 60

        num_devices = cp.cuda.runtime.getDeviceCount()

        # List each device with its ID and name
        for i in range(num_devices):
            device = cp.cuda.Device(i)
            print(f"Device {i}: {device}")


        every_other_frame = True
        other = 0


        while self.vis.poll_events():
            pos3d = -self.camera.pos

            other += 1
            if self.anim and not self.inputListener.getPause():
                if other % 2 == 0 or not every_other_frame:
                    self.lidarDataReader.getNextName()
                    other = 0






            start_time_cycle = time.time()

            path_points = self.oxtsAlignment.getAlignedPathPoints()

            key = self.lidarDataReader.getCurrentName()

            points = self.pointCloudData.getLidarPoints(key) # points coming from the lidar data are spread out along x and y, z is their height

            points = self.oxtsAlignment.rotate(points, key)
            points = self.oxtsAlignment.transform(points, key)



            self.object("planeGrid", self.getPlaneGrid, 10)
            self.object("axes", self.axes)
            #self.object("bbgrid", self.pointCloudData.full_voxel.get_baked_grid)






            self.composeImage(
                (path_points, self.green),
                #(points, self.blue),
                #(self.pointCloudData.test_voxels_map[key].get_baked_grid(), self.white),
                (self.pointCloudData.test_voxels_map[key].get_baked_points(), self.blue)
            )




            #spheres = ShapeUtility.createSpheres(path_points, 0.05)
            #image_np = image_np + self.camera.projectAsNumpy(spheres)




            end_time_cycle = time.time()
            cycle_elapsed_ms = (end_time_cycle - start_time_cycle) * 1000
            fps = 1000 / cycle_elapsed_ms
            # print(f"projection fps: {fps}")

            o3d_image = o3d.geometry.Image(self.image)

            self.vis.clear_geometries()
            self.vis.add_geometry(o3d_image)
            self.vis.update_renderer()

            self.updateCamera()

            sleep(1 / sleep_fps)


        self.vis.destroy_window()


    def updateCamera(self):

        if self.inputListener.left:
            self.xyz -= self.camera.getLeft() * self.speed
        if self.inputListener.right: # captures depth xd
            self.xyz += self.camera.getLeft() * self.speed
        if self.inputListener.forward:
            self.xyz -= self.camera.getForward() * self.speed
        if self.inputListener.backward:
            self.xyz += self.camera.getForward() * self.speed

        if self.inputListener.up:
            self.xyz += self.camera.getUp() * self.speed
        if self.inputListener.down:
            self.xyz -= self.camera.getUp() * self.speed

        self.camera.setPosition(self.xyz)

        if self.inputListener.getLeft():
            delta = self.inputListener.getMouseDelta()
            delta *= self.mouseSens

            self.mousePos += delta

        self.camera.setRotation(np.pi/2 + self.mousePos[1], 0, self.mousePos[0] + np.pi/2)

    def toggleAnim(self):
        self.anim = not self.anim

