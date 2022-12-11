# Image
import cv2

# 3D
import open3d as o3d

import numpy as np
from utils import *
import threading

pcd_model_eraser = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/eraser/eraser.pcd")
pcd_model_shampoo = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/shampoo/shampoo.pcd")

# pcd_env = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/eraser_2/eraser_2_make_pcd.pcd")
# pcd_env = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_1/shampoo_1_make_pcd.pcd")
pcd_env = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/eraser_shampoo_2/eraser_shampoo_2_make_pcd.pcd")


Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,[0,0,0])
ws_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,[0,0,0])

ws_tf = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/workspace_pose.txt',delimiter=',')
ws_box = fixbox(ws_tf[:3, :3],ws_tf[:3,3],0, x = 0.3, y = 0.3, z = 0.1)
ws_coor.rotate(ws_tf[:3, :3],(0,0,0))
ws_coor.translate(np.asarray(ws_tf[:3,3],dtype=np.float64),relative=True)


o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box])

group_model = pcd_env.crop(ws_box)
group_model = group_model.voxel_down_sample(voxel_size=0.001)
group_model, ind = group_model.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)

o3d.visualization.draw_geometries([ws_coor,Realcoor,group_model,ws_box])

volume,pcds = Cluster(group_model)

o3d.visualization.draw_geometries([ws_coor]+pcds)


obj_tf,fitnesses = obj_pose_estimate(pcd_model_shampoo,pcds,point_p_obj = 4000)
shampoo_coor = coordinates(obj_tf)

obj_tf,fitnesses = obj_pose_estimate(pcd_model_eraser,pcds,point_p_obj = 800)
eraser_coor = coordinates(obj_tf)

o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box]+shampoo_coor+eraser_coor)


# xyz = np.asarray(pcds[0].points)

# shampoo_point = 4000
# eraser_point = 800

# clf = Kmeans(k = np.trunc(len(xyz)/shampoo_point).astype(int),tolerance=0.0001)
# labels = clf.predict(xyz)

# max_label = labels.max()
# pcd_cluster = []
# for i in range(0,max_label+1):
#     pcdcen = pcds[0].select_by_index(np.argwhere(labels==i))
#     # o3d.visualization.draw_geometries([ws_coor,pcdcen])
#     pcd_cluster.append(pcdcen)

# draw_labels_on_model(pcds[0],labels)

# obj_coor = []
# fitnesses = []
# for pcd in pcd_cluster:
#     icp = icp_pose_estimate(pcd_model,pcd,t_down= False)
#     tfm,fitness = icp.estimate()
#     coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,[0,0,0])
#     coor.rotate(tfm[:3, :3],(0,0,0))
#     coor.translate(np.asarray(tfm[:3,3],dtype=np.float64),relative=True)
#     if fitness > 0.4:
#         fitnesses.append(fitness)
#         obj_coor.append(coor)

#     draw_registration_result(pcd_model, pcd, tfm)

# fitnesses, obj_coor = zip(*sorted(zip(fitnesses, obj_coor),reverse=True))
# fitnesses = list(fitnesses)
# obj_coor = list(obj_coor)

# o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box]+shampoo_coor+eraser_coor)





# def icpppp(pcd_model,pcd):
#     icp = icp_pose_estimate(pcd_model,pcd,t_down= False)
#     tfm = icp.estimate()
#     coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,[0,0,0])
#     coor.rotate(tfm[:3, :3],(0,0,0))
#     coor.translate(np.asarray(tfm[:3,3],dtype=np.float64),relative=True)
#     obj_coor.append(coor)

# print("start threads")
# threads = []
# for pcd in pcd_cluster:
#     thread = threading.Thread(target=icpppp, args=(pcd_model,pcd,))
#     thread.start()
#     threads.append(thread)

# for thread in threads:
#     thread.join()

# t1 = threading.Thread(target=icpppp, args=(pcd_model,pcd_cluster[0],))
# t2 = threading.Thread(target=icpppp, args=(pcd_model,pcd_cluster[1],))
# t1.start()
# t2.start()
# t1.join()
# t2.join()


