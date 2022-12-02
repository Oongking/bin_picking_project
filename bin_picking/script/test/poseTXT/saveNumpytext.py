import numpy as np

# setuppose = np.array( [[ 0.41722179,  0.43891138,  0.39075829, -0.66082038,  0.23947232,  0.64650396, 0.29665142],
#  [ 0.42437606,  0.44341782,  0.47473083, -0.64075352,  0.25438193,  0.66649473, 0.28374202],
#  [ 0.27705915,  0.32772533,  0.6396222,  -0.64989973,  0.27415717,  0.64197236, 0.30056559]])

# np.savetxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/SetUpPose.out', setuppose, delimiter=',',header='SetUpPose')

# load = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/SetUpPose.out',delimiter=',')
# print("load : ",load)



# calibrationPose = np.array ( [[ 0.27708259, 0.32771506, 0.63961209, -0.64989793, 0.27418074, 0.64196402, 0.3005658 ],
#  [ 0.41966438, 0.01059623, 0.65863832, 0.73924836, -0.1711005, -0.65129093, 0.00752402],
#  [ 0.55495861, 0.04762791, 0.51709215, -0.77746765, -0.1975428, 0.59097734, 0.08524483],
#  [ 0.36889618, 0.1850511, 0.59114148, -0.70643833, -0.14293827, 0.69315351, 0.00719365],
#  [ 0.34851021, 0.21561124, 0.55890634, -0.34220109, 0.25306423, 0.76790337, 0.47872887],
#  [ 0.16933469, 0.37477346, 0.57751287, -0.4087771, 0.37705683, 0.6512474, 0.51633929],
#  [ 0.27966307, 0.37284331, 0.50981345, -0.70058955, 0.17064042, 0.53659677, 0.438315 ],
#  [ 0.00080227, 0.20387234, 0.6337485, -0.34248965, 0.23164082, 0.68442549, 0.60050406],
#  [ 0.03621563, 0.0964007, 0.67563679, -0.91819243, 0.22703443, 0.31516971, 0.0777565 ],
#  [ 0.00896625, 0.14446548, 0.65118082, -0.7942219, 0.02603473, 0.56392654, 0.22476796],
#  [ 0.12238119, 0.29612254, 0.62306578, -0.33609086, 0.25797591, 0.62994767, 0.65088962],
#  [ 0.00351919, 0.18102218, 0.59237474, -0.36448578, 0.3509594, 0.66394638, 0.55059316],
#  [ 0.05672287, 0.10886707, 0.6157037, -0.75003931, 0.40480583, 0.2468266, 0.46113979],
#  [ 0.12873947, 0.26099525, 0.37774206, -0.65027742, -0.12510497, 0.57661161, 0.47854685],
#  [ 0.17277162, 0.17763577, 0.39902863, -0.75325581, 0.29182459, 0.10165395, 0.58061224],
#  [ 0.05069532, 0.23557689, 0.44924277, -0.40746342, 0.01983951, 0.72868781, 0.55008547],
#  [ 0.24464494, 0.20230904, 0.58510011, 0.93200482, -0.1126942, -0.30022439, 0.16891521],
#  [ 0.1230709, 0.28874805, 0.58228238, -0.00746729, 0.23803624, 0.84367731, 0.48113572],
#  [ 0.26181431, 0.32869974, 0.40498154, -0.88686373, 0.37221378, 0.21761065, 0.16605788],
#  [0.26098317, 0.33606141, 0.39301881, 0.01224032, 0.09634008, 0.84063037, 0.53283143],
#  [ 0.14387604, 0.19564602, 0.61187661, -0.66617206, 0.21454747, 0.64311781, 0.3107791 ]])

# np.savetxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/calibrationPose.out', calibrationPose, delimiter=',',header='Calibration Pose')

# load = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/calibrationPose.out',delimiter=',')
# print("load : ",load)






# SIM ROBOT POSE
D2R = np.pi/180
R2D = 180/np.pi

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

cam_base_len = np.array([[1.,0.,0.,0.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,0.],
                            [0.,0.,0.,1.]])
cam_base_len[:3,:3] = np.matmul(Rz(-90),Rx(-90))

world_cam_base = np.array([[1.,0.,0.,-0.8],
                            [0,1,0,0],
                            [0,0,1,1.25],
                            [0,0,0,1]])
world_cam_base[:3,:3] = Ry(0.6*R2D)

world_table = np.array([[1,0,0,0.6],
                            [0,1,0,-0.3],
                            [0,0,1,0.005],
                            [0,0,0,1]])
world_table[:3,:3] = Rz(1.5707*R2D)

table_base = np.array([[1,0,0,0.27],
                        [0,1,0,0.16],
                        [0,0,1,0.698],
                        [0,0,0,1]])
table_base[:3,:3] = Rz(0.785375*R2D)

len_cam_base = np.linalg.inv(cam_base_len)
cam_base_world = np.linalg.inv(world_cam_base)

len_world = np.matmul(len_cam_base,cam_base_world)
len_table = np.matmul(len_world,world_table)
len_base = np.matmul(len_table,table_base)
cam_robotpose = len_base

np.savetxt('/home/oongking/RobotArm_ws/src/bin_picking/script/simRobotpose.txt', cam_robotpose, delimiter=',',header='sim_robotpose Pose')

load = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/simRobotpose.txt',delimiter=',')
print("load : ",load)