import math
from time import sleep

import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

from ShapeUtility import ShapeUtility


class OxtsAlignmentUtility:

    lidatData = None
    oxtsData = None
    datamap = {} # hold position and rotation data

    GPS = 0
    ROTATION_KEY = 1
    VELOCITY = 2
    ROTATIONAL = 3 # best
    MIXED = 4


    pathmode = ROTATIONAL

    def __init__(self, lidarData, oxtsData):
        self.lidatData = lidarData
        self.oxtsData = oxtsData




        aligned = self.rotateIntoAxis(self.oxtsData.getOxtsPosNp(), np.array([0, 0, 1]))
        aligned = self.alignWithVector(aligned, np.array([1, 0, 0]))
        aligned = self.mirrorToVector(aligned, np.array([1, 0, 0]))

        velocity_aligned = self.calcPath()
        rotation_aligned = self.calcPathRotation()

        key = self.lidatData.getCurrentName()
        rotationInfo = self.oxtsData.getRotation(key)  # oxts.roll, oxts.pitch, oxts.yaw
        self.datamap[key] = aligned[0], OxtsAlignmentUtility.rotation(rotationInfo[0], rotationInfo[1], rotationInfo[2]), velocity_aligned[0], rotation_aligned[0], (velocity_aligned[0] + rotation_aligned[0]) / 2
        count = 1
        while self.lidatData.getNextName():
            key = self.lidatData.getCurrentName()
            rotationInfo = self.oxtsData.getRotation(key)

            self.datamap[key] = aligned[count], OxtsAlignmentUtility.rotation(rotationInfo[0], rotationInfo[1], rotationInfo[2]), velocity_aligned[count], rotation_aligned[count], (velocity_aligned[count] + rotation_aligned[count]) / 2
            count += 1





        a = 0

    def calcPathRotation(self):
        rotations = []

        for k in self.oxtsData.datamap.keys():
            rot_inf = self.oxtsData.getRotation(k)
            rot = OxtsAlignmentUtility.rotation(rot_inf[0], rot_inf[1], rot_inf[2])
            rotations.append(rot)


        ox_datamap = self.oxtsData.datamap
        velocities = []
        times = []
        positions = []

        for k in ox_datamap.keys():
            ox = ox_datamap[k]
            velocity = ox.getVelocity()
            velocities.append(velocity)

            time = ox.getTime()
            times.append(time)


        positions.append(np.array([0, 0, 0]))
        last_pos = np.array([0, 0, 0])

        for i in range(len(times) - 1):
            elapsed = self.getElapsedSec(times[i], times[i+1])
            delta_pos = velocities[i] * elapsed

            rot = rotations[i]
            delta_pos = rot @ delta_pos.T
            delta_pos = delta_pos.T

            new_pos = last_pos + delta_pos
            positions.append(new_pos)
            last_pos = new_pos

        positions_np = np.array(positions)

        return positions_np

    def calcPath(self):
        ox_datamap = self.oxtsData.datamap
        velocities = []
        times = []
        positions = []

        for k in ox_datamap.keys():
            ox = ox_datamap[k]
            velocity = ox.getVelocityNE()
            velocities.append(velocity)

            time = ox.getTime()
            times.append(time)

        positions.append(np.array([0, 0, 0]))
        last_pos = np.array([0, 0, 0])

        for i in range(len(times) - 1):
            elapsed = self.getElapsedSec(times[i], times[i+1])
            delta_pos = velocities[i] * elapsed

            new_pos = last_pos + delta_pos
            positions.append(new_pos)
            last_pos = new_pos

        positions_np = np.array(positions)
        positions_np = self.alignWithVector(positions_np, np.array([1, 0, 0]))
        positions_np = self.mirrorToVector(positions_np, np.array([1, 0, 0]))

        return positions_np

    def getElapsedSec(self, before, after):
        delta = after - before
        return delta.total_seconds()

    def visCalcPath(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(height=800, width=1280)
        pcd = o3d.geometry.PointCloud()


        oxtsPoints = self.oxtsData.getOxtsPosNp()
        og_points = self.fitPlaneAndProject(oxtsPoints)


        oxtsPoints = self.rotateIntoAxis(og_points, np.array([0, 0, 1]))
        oxtsPoints = self.alignWithVector(oxtsPoints, np.array([1, 0, 0]))
        oxtsPoints = self.mirrorToVector(oxtsPoints, np.array([1, 0, 0]))


        path = self.calcPathRotation()
        oxtsPoints = ShapeUtility.np_append(oxtsPoints[::2], path)

        oxtsPoints = self.addAxis(oxtsPoints)




        pcd.points = o3d.utility.Vector3dVector(oxtsPoints)
        #pcd.colors = o3d.utility.Vector3dVector(colors2)

        vis.get_render_option().load_from_json("renderopt.json")

        vis.add_geometry(pcd)

        fps = 30

        while vis.poll_events():
            vis.update_geometry(pcd)
            vis.update_renderer()
            sleep(1 / fps)

        vis.destroy_window()




    def getAlignedPathPoints(self):
        points = []
        for k in self.datamap.keys():
            #points.append(self.datamap[k][0]) # pos (not rot)
            points.append(self.datamap[k][self.pathmode]) # pos (not rot)

        return np.array(points)

    def align(self, key, points):
        return self.transform(self.rotate(points, key), key)

    def transform(self, points, key):
        # return points + self.datamap[key][0] # pos, not rot
        return points + self.datamap[key][self.pathmode] # pos, not rot

    def rotate(self, points, key):
        rot = self.datamap[key][1]
        rotation = rot

        points = rotation @ points.T
        points = points.T

        return points


    def connect(self, point1, point2, num):
        points = []
        for i in range(num):
            points.append(point1 * (i / num) + point2 * (1 - (i / num)))
        return np.array(points)

    def getLine(self, origin, vector):
        points = []
        for i in range(20):
            points.append(origin + vector * i)

        return np.array(points)


    def addAxis(self, points):
        axlen = 10
        axpoints = 500


        xp = np.linspace(0, axlen, axpoints)
        yp = np.zeros(axpoints)
        zp = np.zeros(axpoints)

        xpoints = np.column_stack((xp, yp, zp)) # x-axis points toward facing of car
        points = np.concatenate((points, xpoints), axis=0)

        xp = np.zeros(axpoints)
        yp = np.linspace(0, axlen, axpoints)
        zp = np.zeros(axpoints)

        ypoints = np.column_stack((xp, yp, zp)) # positive y is to the right
        points = np.concatenate((points, ypoints), axis=0)

        xp = np.zeros(axpoints)
        yp = np.zeros(axpoints)
        zp = np.linspace(0, int(axlen/3), axpoints)

        zpoints = np.column_stack((xp, yp, zp))
        points = np.concatenate((points, zpoints), axis=0)
        return points

    def getDistanceToPlane(self, point, plane, d):
        return (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + d) / (
            math.sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]))

    def fitPlaneAndProject(self, np_points):
        plane = pyrsc.Plane()
        best_eq, best_inliers = plane.fit(np_points, 0.05, 3, 100)

        normal = np.array([best_eq[0], best_eq[1], best_eq[2]])

        projected = []
        unit_normal = normal / np.linalg.norm(normal)
        for p in np_points:
            proj = p - self.getDistanceToPlane(p, normal, best_eq[3]) * unit_normal
            projected.append(proj)

        return np.array(projected)

    def getNormal(self, np_points):
        first = np_points[0]
        point1_on_plane = np_points[int(len(np_points) / 2)]
        point2_on_plane = np_points[-1]

        p1norm = point1_on_plane - first
        p2norm = point2_on_plane - first

        p1norm = p1norm / np.linalg.norm(p1norm)
        p2norm = p2norm / np.linalg.norm(p2norm)

        norm = np.cross(p1norm, p2norm)
        norm = norm / np.linalg.norm(norm)

        return norm

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def checkRotationAxis(self, vec1, vec2):
        ang = self.angle_between(vec1, vec2)
        return ang == 0.0

    def rotateIntoAxis(self, points, axis):
        # get normal of plane
        origin = np.array([0, 0, 0])

        norm = self.getNormal(points)

        # rotate normal into Z axis (xy will be the plane)
        # using rodrigues' formula

        # 1. axis of rotation: normalized cross
        z = axis

        rotAx = np.linalg.cross(z, norm)
        rotAx = rotAx / np.linalg.norm(rotAx)

        # angle between norm and Z

        theta = self.angle_between(z, norm)  # Angle between vectors
        # note to self: crossing other way and changing the sign is the key
        # check if norm after rotation is paralell with Z, if no, change the sign of theta

        # Rodrigues'
        k = rotAx
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])

        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

        rotatedNormal = R @ norm.T
        if not self.checkRotationAxis(z, rotatedNormal):
            theta = -theta
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)


        rotated = R @ points.T
        rotated = rotated.T

        """
        oxtsPoints = ShapeUtility.np_append(points, rotated)

        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, np.array([0, 0, 1])))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, norm))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, rotAx))

        return oxtsPoints
        """


        return rotated

    def mirrorToVector(self, points, vector):
        reflected = []

        vector = vector / np.linalg.norm(vector)

        for p in points:
            dot = np.dot(p, vector)
            projection = vector * dot
            ref = p + 2 * (projection - p)
            reflected.append(ref)

        return np.array(reflected)

    def alignWithVector(self, points, vector):
        # get normal of plane


        firstDirectionOfData = points[1] - points[0]
        dirnorm = firstDirectionOfData / np.linalg.norm(firstDirectionOfData)

        # rotate normal into Z axis (xy will be the plane)
        # using rodrigues' formula

        # 1. axis of rotation: normalized cross
        dir = vector

        rotAx = np.linalg.cross(dir, dirnorm)
        rotAx = rotAx / np.linalg.norm(rotAx)

        # angle between norm and Z

        theta = self.angle_between(dir, dirnorm)  # Angle between vectors
        # note to self: crossing other way and changing the sign is the key
        # check if norm after rotation is paralell with Z, if no, change the sign of theta

        # Rodrigues'
        k = rotAx
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])

        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

        rotatedNormal = R @ dirnorm.T
        if not self.checkRotationAxis(dir, rotatedNormal):
            theta = -theta
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)


        rotated = R @ points.T
        rotated = rotated.T

        """
        oxtsPoints = ShapeUtility.np_append(points, rotated)

        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, np.array([0, 0, 1])))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, norm))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(origin, rotAx))

        return oxtsPoints
        """


        return rotated


    def visualize(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(height=800, width=1280)
        pcd = o3d.geometry.PointCloud()


        oxtsPoints = self.oxtsData.getOxtsPosNp()
        og_points = self.fitPlaneAndProject(oxtsPoints)

        oxtsPoints = self.rotateIntoAxis(og_points, np.array([1, 0, 0]))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.rotateIntoAxis(og_points, np.array([0, 1, 0])))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.rotateIntoAxis(og_points, np.array([0, 0, 1])))

        oxtsPoints = ShapeUtility.np_append(oxtsPoints, og_points)
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(np.array([0, 0, 0]), np.array([0, 0, 1])))

        oxtsPoints = self.addAxis(oxtsPoints)




        pcd.points = o3d.utility.Vector3dVector(oxtsPoints)
        #pcd.colors = o3d.utility.Vector3dVector(colors2)

        vis.get_render_option().load_from_json("renderopt.json")

        vis.add_geometry(pcd)

        fps = 30

        while vis.poll_events():
            vis.update_geometry(pcd)
            vis.update_renderer()
            sleep(1 / fps)

        vis.destroy_window()


    def visualizeInDirection(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(height=800, width=1280)
        pcd = o3d.geometry.PointCloud()


        oxtsPoints = self.oxtsData.getOxtsPosNp()
        og_points = self.fitPlaneAndProject(oxtsPoints)

        oxtsPoints = self.rotateIntoAxis(og_points, np.array([0, 0, 1]))
        oxtsPoints = self.alignWithVector(oxtsPoints, np.array([1, 0, 0]))
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.mirrorToVector(oxtsPoints, np.array([1, 0, 0])))



        oxtsPoints = ShapeUtility.np_append(oxtsPoints, og_points)
        oxtsPoints = ShapeUtility.np_append(oxtsPoints, self.getLine(np.array([0, 0, 0]), np.array([0, 0, 1])))
        oxtsPoints = self.addAxis(oxtsPoints)

        pcd.points = o3d.utility.Vector3dVector(oxtsPoints)
        #pcd.colors = o3d.utility.Vector3dVector(colors2)

        vis.get_render_option().load_from_json("renderopt.json")

        vis.add_geometry(pcd)

        fps = 30

        while vis.poll_events():
            vis.update_geometry(pcd)
            vis.update_renderer()
            sleep(1 / fps)

        vis.destroy_window()

    @staticmethod
    def rotation(x, y, z):


        xr = x
        yr = y
        zr = z

        R_z = np.array([
            [np.cos(zr), -np.sin(zr), 0],
            [np.sin(zr), np.cos(zr), 0],
            [0, 0, 1]
        ])

        R_y = np.array([
            [np.cos(yr), 0, np.sin(yr)],
            [0, 1, 0],
            [-np.sin(yr), 0, np.cos(yr)]
        ])

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(xr), -np.sin(xr)],
            [0, np.sin(xr), np.cos(xr)]
        ])


        return R_x @ R_y @ R_z