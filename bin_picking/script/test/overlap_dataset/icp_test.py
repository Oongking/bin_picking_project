# Image
import cv2

# 3D
import open3d as o3d

import numpy as np
from utils import *

pcd_model = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/shampoo/shampoo.pcd")
pcd_target = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_3_sim.pcd")

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,[0,0,0])
# o3d.visualization.draw_geometries([Realcoor,pcd_model,pcd_target])


icp = icp_pose_estimate(pcd_model,pcd_target,t_down= False)
tfm = icp.estimate()

draw_registration_result(pcd_model, pcd_target, tfm)

'''
def preprocess_point_cloud(pcd, voxel_size,down = 0):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    if down == 1:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    else:
        pcd_down = pcd

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    source = pcd_model
    target = pcd_target

    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size,down = 1)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    return source, target, source_down, target_down, source_fpfh, target_fpfh

voxel_size = 0.003  # means 5cm for this dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    voxel_size)


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
# draw_registration_result(source_down, target_down, result_ransac.transformation)



trans_init = result_ransac.transformation
threshold = 0.002

### evaluate for check fitness value
print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(
    source_down, target_down, threshold, trans_init)
print(evaluation)
###

reg_p2p = o3d.pipelines.registration.registration_icp(
    source_down, target_down, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2p)
# print("Transformation is:")
# print(reg_p2p.transformation)

draw_registration_result(source_down, target_down, reg_p2p.transformation)

print("Apply point-to-plane ICP")
reg_p2l = o3d.pipelines.registration.registration_icp(
    source_down, target_down, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
draw_registration_result(source_down, target_down, reg_p2l.transformation)
'''