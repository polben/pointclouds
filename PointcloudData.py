import pyransac3d as pyr


import numpy as np


from VoxelData import VoxelData



class PointcloudData:
    lidarData = None
    oxtsAlignmentUtil = None


    datamap = {}


    gridcells = []

    voxel_minpoints = 50
    total_minpoints = 50
    voxel_size = 1


    test_voxels_map = {}
    full_voxel = None

    def __init__(self, lidarDataReader, oxtsAlignmentUtility):
        self.lidarData = lidarDataReader
        self.oxtsAlignmentUtil = oxtsAlignmentUtility

        self.lidarData.resetNameIterator()

        key = self.lidarData.getCurrentName()                    # load oxts and points into datamap
        #self.datamap[key] = self.segmentGround(self.lidarData.getPoints(key))
        self.datamap[key] = self.lidarData.getPoints(key)

        self.test_voxels_map[key] = VoxelData(self.voxel_size, self.voxel_minpoints)
        self.test_voxels_map[key].voxelize(self.oxtsAlignmentUtil.align(key, self.datamap[key]))
        self.test_voxels_map[key].clearNotDense()
        self.test_voxels_map[key].bake_points()
        self.test_voxels_map[key].bake(20)

        self.full_voxel = VoxelData(self.voxel_size, self.voxel_minpoints)
        self.full_voxel.voxelize(self.oxtsAlignmentUtil.align(key, self.datamap[key]))
        self.full_voxel.clearNotDense()

        while lidarDataReader.getNextName():
            key = self.lidarData.getCurrentName()
            #self.datamap[key] = self.segmentGround(self.lidarData.getPoints(key))
            self.datamap[key] = self.lidarData.getPoints(key)

            self.test_voxels_map[key] = VoxelData(self.voxel_size, self.voxel_minpoints)
            self.test_voxels_map[key].voxelize(self.oxtsAlignmentUtil.align(key, self.datamap[key]))
            self.test_voxels_map[key].clearNotDense()
            self.test_voxels_map[key].bake_points()
            self.test_voxels_map[key].bake(20)

            self.full_voxel.voxelize(self.oxtsAlignmentUtil.align(key, self.datamap[key]))
            self.full_voxel.clearNotDense()
            print(key)



        self.full_voxel.getStat()
        self.full_voxel.bake(5)

        a = 0

    def getLidarPoints(self, key):
        return self.datamap[key]

    def segmentGround(self, np_points):
        num_points = len(np_points)
        """


        minz = np.min(np_points[:, 2])
        maxz = np.max(np_points[:, 2])

        res = 0.2
        min_test = minz
        max_test = maxz

        prev_count = 0
        prev_diff = 0

        while min_test < max_test:
            count = np.count_nonzero(np_points[:, 2] < min_test)
            diff = count - prev_count
            if diff != 0 and prev_diff > diff and diff > 1000:
                min_test += res
                break

            min_test += res

            prev_count = count
            prev_diff = diff

        ground_points = np_points[np_points[:, 2] < min_test]
        """

        plane = pyr.Plane()
        best_eq, best_inliers = plane.fit(np_points, 0.2, 3, 100)
        mask = np.ones(num_points, dtype=bool)

        mask[best_inliers] = False

        return np_points[mask]

