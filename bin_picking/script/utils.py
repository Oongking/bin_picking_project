# Ros
import rospy
import tf
from tf.transformations import *

# Utility
import numpy as np
import copy

# msg & convert
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import * # convert float to uint32

# Zivid
import dynamic_reconfigure.client
from zivid_camera.srv import *

# Image
import cv2

# 3D
import open3d as o3d


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

zivid_matrix_coefficients = np.array([  [ 1782.476318359375,        0.0,        965.43896484375], 
                                        [ 0.0,              1784.1812744140625, 590.5164184570312], 
                                        [ 0,                        0,              1]])
zivid_distortion_coefficients = np.array([[-0.08575305342674255, 0.1142171174287796, 0.00030625637737102807, -0.0007428471581079066, -0.048006460070610046]])
zivid_intrinsic = o3d.camera.PinholeCameraIntrinsic(1944, 1200, 1782.476318359375, 1784.1812744140625, 965.43896484375, 590.5164184570312)


# azure calibraby chessboard
azure_matrix_coefficients = np.array([  [610.89826648,   0.,         647.16398913],
                                        [  0.,         616.12187414, 366.37689302],
                                        [  0.,           0.,           1.        ]])
azure_distortion_coefficients = np.array([[ 0.09865886, -0.11209954, -0.00087517,  0.00436045,  0.06666987]])
azure_intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720, 610.89826648, 616.12187414, 647.16398913, 366.37689302)


def normalvector(base, cross1, cross2):
    vector1 = np.asarray(np.subtract(cross1,base),dtype = np.float64)
    vector2 = np.asarray(np.subtract(cross2,base),dtype = np.float64)
    normalVec = np.cross(vector1,vector2)
    UNormalVec = normalVec/np.linalg.norm(normalVec)
    print("Normal : ",UNormalVec)
    return UNormalVec

arucoParams = cv2.aruco.DetectorParameters_create() 
arucoDictA3 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)


zivid_boardA3 = cv2.aruco.GridBoard_create(14, 10, 0.02155, 0.0058, arucoDictA3)

# For Azure
azure_percentage_offset = 0.03
azure_ARsize = 0.02155 + (0.02155*azure_percentage_offset)
azure_ARgabsize = 0.0058 + (0.0058*azure_percentage_offset)
azure_boardA3 = cv2.aruco.GridBoard_create(14, 10, azure_ARsize, azure_ARgabsize, arucoDictA3)

#A4
# offset_x=0.5*((0.0315*7)+(0.0081*(7-1)))
# offset_y=0.5*((0.0315*5)+(0.0081*(5-1)))
#A3
A3_azure_offset_x=0.5*((azure_ARsize*14)+(azure_ARgabsize*(14-1)))
A3_azure_offset_y=0.5*((azure_ARsize*10)+(azure_ARgabsize*(10-1)))
A3_zivid_offset_x=0.5*((0.02155*14)+(0.0058*(14-1)))
A3_zivid_offset_y=0.5*((0.02155*10)+(0.0058*(10-1)))

bridge=CvBridge()

def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a = (vec1 / np.linalg.norm(vec1)).reshape(3)
    b = (vec2 / np.linalg.norm(vec2)).reshape(3)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    angle = np.arccos(c)
    return rotation_matrix,angle

class sphere:
    def __init__(self, center, radius = 0.01 , color = ( 1, 0.5, 0.5)):
        self.pcd = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.pcd.compute_vertex_normals()
        self.pcd.translate((center[0], center[1], center[2]), relative=False)
        self.pcd.paint_uniform_color(color)

class Azure_cam():
    def __init__(self,get_depth = False):
        

        rospy.loginfo(":: Starting Azure_class ::")

        rospy.Subscriber("/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/depth_to_rgb/image_raw",Image,self.depth_callback)
        if get_depth:
            rospy.Subscriber("/depth_to_rgb/points", PointCloud2, self.points_callback)  

        self.received_ros_cloud = None
        self.pcd = None
        self.rgb_image = None
        self.depth_image = None

    def get_pcd(self):

        while 1:
            if self.pcd is not None:
                break
        
        pcd = copy.deepcopy(self.pcd)

        self.pcd = None
        return pcd
    
    def get_rgbd(self):
        while 1:
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break
        
        rgb_image = copy.deepcopy(self.rgb_image)
        depth_image = copy.deepcopy(self.depth_image)

        self.rgb_image = None
        self.depth_image = None
        return rgb_image, depth_image

    def buildPCD(self):
        while 1:
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break
        depth = o3d.geometry.Image(self.depth_image)
        color = o3d.geometry.Image(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1000.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, azure_intrinsic)
        # downpcd = pcd.voxel_down_sample(voxel_size=0.01)
        # o3d.visualization.draw_geometries([downpcd,Realcoor])
        # return downpcd
        return pcd

    def points_callback(self, data):
        self.received_ros_cloud=data
        self.pcd = self.convertCloudFromRosToOpen3d()
        # if show_display and self.pcd is not None:
        #     o3d.visualization.draw_geometries([Realcoor,self.pcd])

    def rgb_callback(self, received_image):
        try:
            self.rgb_image = bridge.imgmsg_to_cv2(received_image, "bgr8")
        except CvBridgeError as e:
                print(e)
    
    def depth_callback(self,data):
        try:
            T_depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = T_depth_image.astype(np.uint16)
        except CvBridgeError as e:
            print(e)

    def convertCloudFromRosToOpen3d(self):
        print(":: convertCloudFromRosToOpen3d ::")
        open3d_cloud = o3d.geometry.PointCloud()
        if self.received_ros_cloud is not None:
            # Get cloud data from ros_cloud

            field_names=[field.name for field in self.received_ros_cloud.fields]
            cloud_data = list(pc2.read_points(self.received_ros_cloud, skip_nans=True, field_names = field_names))

            
            # Check empty
            
            if len(cloud_data)==0:
                print("Converting an empty cloud")
                return None  

            # Set open3d_cloud
            print("field_names : ",field_names)
            if "rgb" in field_names:
                IDX_RGB_IN_FIELD=3 # x, y, z, rgb
                
                # Get xyz
                xyz = [(x,y,z) for x,y,z,rgba in cloud_data ] # (why cannot put this line below rgb?)

                # Get rgb
                # Check whether int or float
                if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                    rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
                else:
                    rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

                # combine
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
                open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
            else:
                xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

            # return
            return open3d_cloud
        else:
            return None

class zivid_cam():
    def __init__(self):
        # rospy.init_node("Zivid_model", anonymous=True)

        rospy.loginfo(":: Starting Zivid_model ::")

        rospy.wait_for_service("/zivid_camera/capture", 30.0)

        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.points_callback)
        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.rgb_callback)
        rospy.Subscriber("/zivid_camera/depth/image", Image, self.depth_callback)
        self.received_ros_cloud = None
        self.pcd = None
        self.rgb_image = None
        self.depth_image = None
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

        rospy.loginfo("Enabling the reflection filter")
        settings_client = dynamic_reconfigure.client.Client("/zivid_camera/settings/")
        settings_config = {"processing_filters_reflection_removal_enabled": True}
        settings_client.update_configuration(settings_config)

        rospy.loginfo("Enabling and configure the first acquisition")
        acquisition_0_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/settings/acquisition_0"
        )
        acquisition_0_config = {
            # normal
            "enabled": True,
            # "aperture": 5.66,
            # "exposure_time": 20000,
            "aperture": 5.66,
            "brightness": 1.8,
            "exposure_time": 40000,
            "gain": 1,
        }
        acquisition_0_client.update_configuration(acquisition_0_config)

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.rgb_image = None
        self.depth_image = None
        self.pcd = None
        self.capture_service()

        while 1:
            # print("wait for input")
            if (self.rgb_image is not None) and (self.pcd is not None) and (self.depth_image is not None):
                break
        
        rgb_image = copy.deepcopy(self.rgb_image)
        pcd = copy.deepcopy(self.pcd)
        depth_image = copy.deepcopy(self.depth_image)
        self.rgb_image = None
        self.depth_image = None
        self.pcd = None
        return rgb_image, depth_image, pcd
    
    def get_rgbd(self):
        rospy.loginfo("Calling capture service")
        self.rgb_image = None
        self.depth_image = None
        self.capture_service()

        while 1:
            # print("wait for input")
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break
        
        rgb_image = copy.deepcopy(self.rgb_image)
        depth_image = copy.deepcopy(self.depth_image)
        self.rgb_image = None
        self.depth_image = None
        return rgb_image, depth_image
    
    def get_pcd(self):
        rospy.loginfo("Calling capture service")
        self.pcd = None
        self.capture_service()

        while 1:
            # print("wait for input")
            if self.pcd is not None:
                break
        
        pcd = copy.deepcopy(self.pcd)
        self.pcd = None
        return pcd

    def testMatrix(self):
        rospy.loginfo("Calling capture service")
        self.rgb_image = None
        self.depth_image = None
        self.pcd = None
        self.capture_service()

        while 1:
            # print("wait for input")
            if (self.rgb_image is not None) and (self.depth_image is not None) and (self.pcd is not None):
                break
        
        depth = o3d.geometry.Image(self.depth_image)
        color = o3d.geometry.Image(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
        make_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, zivid_intrinsic)

        pcd = copy.deepcopy(self.pcd)
        rgb_image = copy.deepcopy(self.rgb_image)
        depth_image = copy.deepcopy(self.depth_image)
        self.pcd = None
        self.rgb_image = None
        self.depth_image = None
        return pcd,rgb_image, depth_image,make_pcd

    def buildPCD(self):
        rospy.loginfo("Calling capture service")
        self.rgb_image = None
        self.depth_image = None
        self.capture_service()

        while 1:
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break

        depth = o3d.geometry.Image(self.depth_image)
        color = o3d.geometry.Image(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, zivid_intrinsic)
        
        self.rgb_image = None
        self.depth_image = None
        # downpcd = pcd.voxel_down_sample(voxel_size=0.01)
        # o3d.visualization.draw_geometries([downpcd,Realcoor])
        # return downpcd
        return pcd

    def depth_callback(self,data):
        try:
            T_depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = T_depth_image.astype(np.float32)

        except CvBridgeError as e:
            print(e)

    def points_callback(self, data):
        rospy.loginfo("PointCloud received")
        self.received_ros_cloud=data
        self.pcd = self.convertCloudFromRosToOpen3d()
        # if show_display and self.pcd is not None:
        #     o3d.visualization.draw_geometries([Realcoor,self.pcd])

    def rgb_callback(self, received_image):
        rospy.loginfo("Image received")
        try:
            self.rgb_image = bridge.imgmsg_to_cv2(received_image, "bgr8")
        except CvBridgeError as e:
                print(e)

    def convertCloudFromRosToOpen3d(self):
        print(":: convertCloudFromRosToOpen3d ::")
        open3d_cloud = o3d.geometry.PointCloud()
        if self.received_ros_cloud is not None:
            # Get cloud data from ros_cloud

            field_names=[field.name for field in self.received_ros_cloud.fields]
            cloud_data = list(pc2.read_points(self.received_ros_cloud, skip_nans=True, field_names = field_names))

            
            # Check empty
            
            if len(cloud_data)==0:
                print("Converting an empty cloud")
                return None  

            # Set open3d_cloud
            print("field_names : ",field_names)
            if "rgba" in field_names:
                IDX_RGB_IN_FIELD=3 # x, y, z, rgb
                
                # Get xyz
                xyz = [(x,y,z) for x,y,z,rgba in cloud_data ] # (why cannot put this line below rgb?)

                # Get rgb
                # Check whether int or float
                if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                    rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
                else:
                    rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

                # combine
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
                open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
            else:
                xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

            # return
            print("open3d_cloud : ",open3d_cloud)
            return open3d_cloud
        else:
            return None



class sim_cam():
    def __init__(self,get_depth = False):
        

        rospy.loginfo(":: Starting sim_cam ::")

        rospy.Subscriber("/rgbd/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/rgbd/depth/image_raw",Image,self.depth_callback)
        if get_depth:
            rospy.Subscriber("/rgbd/depth/points", PointCloud2, self.points_callback)  

        self.received_ros_cloud = None
        self.pcd = None
        self.rgb_image = None
        self.depth_image = None
        self.sim_intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720, 1024.249107649826, 1024.249107649826, 640.5, 360.5)

    def get_pcd(self):

        while 1:
            if self.pcd is not None:
                break
        
        pcd = copy.deepcopy(self.pcd)

        self.pcd = None
        return pcd
    
    def get_rgbd(self):
        while 1:
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break
        
        rgb_image = copy.deepcopy(self.rgb_image)
        depth_image = copy.deepcopy(self.depth_image)

        self.rgb_image = None
        self.depth_image = None
        return rgb_image, depth_image

    def buildPCD(self):
        while 1:
            if (self.rgb_image is not None) and (self.depth_image is not None):
                break
        depth = o3d.geometry.Image(self.depth_image)
        color = o3d.geometry.Image(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.sim_intrinsic)
        # downpcd = pcd.voxel_down_sample(voxel_size=0.01)
        # o3d.visualization.draw_geometries([downpcd,Realcoor])
        # return downpcd
        self.rgb_image = None
        self.depth_image = None

        return pcd

    def points_callback(self, data):
        self.received_ros_cloud=data
        self.pcd = self.convertCloudFromRosToOpen3d()
        # if show_display and self.pcd is not None:
        #     o3d.visualization.draw_geometries([Realcoor,self.pcd])

    def rgb_callback(self, received_image):
        try:
            self.rgb_image = bridge.imgmsg_to_cv2(received_image, "bgr8")
        except CvBridgeError as e:
                print(e)
    
    def depth_callback(self,data):
        try:
            T_depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = T_depth_image
        except CvBridgeError as e:
            print(e)

    def convertCloudFromRosToOpen3d(self):
        print(":: convertCloudFromRosToOpen3d ::")
        open3d_cloud = o3d.geometry.PointCloud()
        if self.received_ros_cloud is not None:
            # Get cloud data from ros_cloud

            field_names=[field.name for field in self.received_ros_cloud.fields]
            cloud_data = list(pc2.read_points(self.received_ros_cloud, skip_nans=True, field_names = field_names))

            
            # Check empty
            
            if len(cloud_data)==0:
                print("Converting an empty cloud")
                return None  

            # Set open3d_cloud
            print("field_names : ",field_names)
            if "rgb" in field_names:
                IDX_RGB_IN_FIELD=3 # x, y, z, rgb
                
                # Get xyz
                xyz = [(x,y,z) for x,y,z,rgba in cloud_data ] # (why cannot put this line below rgb?)

                # Get rgb
                # Check whether int or float
                if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                    rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
                else:
                    rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

                # combine
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
                open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
            else:
                xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
                open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

            # return
            return open3d_cloud
        else:
            return None

