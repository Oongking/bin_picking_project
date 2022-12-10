# Image
import cv2

# 3D
import open3d as o3d

import numpy as np
from utils import *

# float save problem
# T_depth_image = cv2.imread(f'/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_3/shampoo_3_dep_img.png') 

rgb_image = cv2.imread(f'/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_3/shampoo_3_rgb_img.png')
pcd_model = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/shampoo/shampoo.pcd")
pcd_env = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_3/shampoo_3_make_pcd.pcd")

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,[0,0,0])

# while True:
#     cv2.imshow("Original Image",resize(rgb_image,50))
#     if cv2.waitKey(1) & 0xFF==ord('q'):
#         break

ws_tf = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/workspace_pose.txt',delimiter=',')
ws_box = fixbox(ws_tf[:3, :3],ws_tf[:3,3],0, x = 0.3, y = 0.3, z = 0.1)

ws_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,[0,0,0])
ws_coor.rotate(ws_tf[:3, :3],(0,0,0))
ws_coor.translate(np.asarray(ws_tf[:3,3],dtype=np.float64),relative=True)

o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box,pcd_model])
group_model = pcd_env.crop(ws_box)
o3d.visualization.draw_geometries([ws_coor,Realcoor,group_model,ws_box,pcd_model])

group_model = group_model.voxel_down_sample(voxel_size=0.001)

o3d.visualization.draw_geometries([ws_coor,Realcoor,group_model,ws_box,pcd_model])

group_model, ind = group_model.remove_statistical_outlier(nb_neighbors=50,
                                                            std_ratio=1.0)

o3d.visualization.draw_geometries([ws_coor,Realcoor,group_model,ws_box,pcd_model])

def Cluster(pcd):
    pcds = []
    volume = []
    labels = np.array(pcd.cluster_dbscan(eps=0.01, min_points=20, print_progress=False))
    
    max_label = labels.max()
    for i in range(0,max_label+1):
        pcdcen = pcd.select_by_index(np.argwhere(labels==i))
        pcdcen, ind = pcdcen.remove_statistical_outlier(nb_neighbors=20,
                                                    std_ratio=1.0)
        print("pcdcen : ",np.asarray(pcdcen.points).shape)
        
        if 50000 >np.asarray(pcdcen.points).shape[0]>700:
            o3d.visualization.draw_geometries([pcdcen])
            box = pcdcen.get_oriented_bounding_box()
            volume.append(box.volume())
            pcdcen.estimate_normals()
            pcds.append(pcdcen)
    volume, pcds = zip(*sorted(zip(volume, pcds),reverse=True))
    volume = list(volume)
    pcds = list(pcds)

    return volume,pcds

volume,pcds = Cluster(group_model)

o3d.visualization.draw_geometries([ws_coor]+pcds)

xyz = np.asarray(pcds[0].points)

clf = Kmeans(k=2)
labels = clf.predict(xyz)

max_label = labels.max()
for i in range(0,max_label+1):
    pcdcen = pcds[0].select_by_index(np.argwhere(labels==i))
    o3d.visualization.draw_geometries([ws_coor,pcdcen])

draw_labels_on_model(pcds[0],labels)

# o3d.io.write_point_cloud(f"/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/shampoo_3_sim.pcd", pcdcen)
