from Camera import Camera
from LidarDataReader import LidarDataReader
from OxtsAlignmentUtility import OxtsAlignmentUtility
from OxtsDataReader import OxtsDataReader
from PointcloudController import PointcloudController
from PointcloudData import PointcloudData

path = "F://uni//3d-pointcloud//2011_09_26_drive_0005_sync"
pathOxts = "F://uni//3d-pointcloud//2011_09_26_drive_0005_sync"

path2 = "F://uni//3d-pointcloud//samle1"
path3 = "F://uni//3d-pointcloud//sample2"

calibration = "F://uni//3d-pointcloud//2011_09_26_calib//2011_09_26"


camera = Camera(calibration)
oxtsDataReader = OxtsDataReader(path)
lidarDataReader = LidarDataReader(path=path, oxtsDataReader=oxtsDataReader)

oxtsAlignment = OxtsAlignmentUtility(lidarDataReader, oxtsDataReader)
#oxtsAlignment.visCalcPath()
#oxtsAlignment.visualizeInDirection()

pointcloudData = PointcloudData(lidarDataReader, oxtsAlignment)

pointCloudController = PointcloudController(lidarDataReader, pointcloudData, oxtsAlignment, camera=camera)
pointCloudController.toggleAnim()
pointCloudController.controller()
