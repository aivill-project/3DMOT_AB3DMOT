
import numpy as np
import yaml
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R


import calib_logic
import utils



##//파일 parser =============================

def parse_conf_lidar(conf_pth, rot_dscr):
    # conf_lidar = conf_dir + "config0" + str(direction) + ".yaml"
    
    rot_selection = ["quat", "rmat", "rvec", "euler_deg", "euler_rad"]
    if rot_dscr not in rot_selection:
        print("오류: rot_dscr에 quat, rmat, rvec, euler_deg, euler_rad 선택하여 넣어주세요 ㅠㅜ")
        return    
    
    with open(conf_pth, 'r') as f:
        f.readline()
        conf_lidar = yaml.load(f, Loader=yaml.FullLoader)
        fx = float(conf_lidar['fx'])
        fy = float(conf_lidar['fy'])
        cx = float(conf_lidar['cx'])
        cy = float(conf_lidar['cy'])
        k1 = float(conf_lidar['k1'])
        k2 = float(conf_lidar['k2'])
        p1 = float(conf_lidar['p1/k3'])
        p2 = float(conf_lidar['p2/k4'])
        rot_lidar = conf_lidar[rot_dscr]
        tvec_lidar = conf_lidar['translation_vector']
    rot_lidar = np.asarray(rot_lidar) * 68
    
    rvec_lidar=None
    if rot_dscr==rot_selection[0]:
        rmat_lidar = calib_logic.quaternion_rotation_matrix(rot_lidar)
        rvec_lidar,_ = cv2.Rodrigues(rmat_lidar)
    elif rot_dscr==rot_selection[1]:
        rmat_lidar = rot_lidar
        rvec_lidar,_ = cv2.Rodrigues(rmat_lidar)
    elif rot_dscr==rot_selection[2]:
        rvec_lidar = rot_lidar
    elif rot_dscr==rot_selection[3]:
        rot_lidar = R.from_euler('xyz', rot_lidar, degrees=True)
        rmat_lidar = rot_lidar.as_matrix()
        rvec_lidar,_ = cv2.Rodrigues(rmat_lidar)
    elif rot_dscr==rot_selection[4]:
        rot_lidar = R.from_euler('xyz', rot_lidar, degrees=False)
        rmat_lidar = rot_lidar.as_matrix()
        rvec_lidar,_ = cv2.Rodrigues(rmat_lidar)
    # elif rot_dscr==rot_selection[4]:
    #     rot_lidar = R.from_mrp(rot_lidar)    
    #rvec_lidar,_ = cv2.Rodrigues(rmat_lidar)
    
    K = np.matrix([ [fx, 0.0, cx],
                    [0.0, fy, cy],
                    [0.0, 0.0, 1.0]])
    D = np.array([k1, k2, p1, p2])
    #rvec_lidar = np.asarray(rvec_lidar)
    tvec_lidar = np.asarray(tvec_lidar)
    return K, D, rvec_lidar, tvec_lidar


def parse_conf_radar(conf_pth, rot_dscr):
    # conf_radar = conf_dir + "config_radar.yaml"
    
    rot_selection = ["quat", "rmat", "rvec", "euler_deg", "euler_rad"]
    if rot_dscr not in rot_selection:
        print("오류: rot_dscr에 quat, rmat, rvec, euler_deg, euler_rad 선택하여 넣어주세요 ㅠㅜ")
        return    
    
    with open(conf_pth, 'r') as f:
        f.readline()
        conf_radar = yaml.load(f, Loader=yaml.FullLoader)
        rot_radar = conf_radar[rot_dscr]
        tvec_radar = conf_radar['translation_vector']
    
    rmat_radar=None
    if rot_dscr==rot_selection[0]:
        rmat_radar = calib_logic.quaternion_rotation_matrix(rot_radar)
        
    elif rot_dscr==rot_selection[1]:
        rmat_radar = rot_radar
        
    elif rot_dscr==rot_selection[2]:
        rmat_radar,_ = cv2.Rodrigues(rot_radar)
        
    elif rot_dscr==rot_selection[3]:
        rot_radar = R.from_euler('xyz', rot_radar, degrees=True)
        rmat_radar = rot_radar.as_matrix()
        
    elif rot_dscr==rot_selection[4]:
        rot_radar = R.from_euler('xyz', rot_radar, degrees=False)
        rmat_radar = rot_radar.as_matrix()
    
    rmat_radar = np.asarray(rmat_radar)
    tvec_radar = np.asarray(tvec_radar)
    return rmat_radar, tvec_radar


def parse_pcd(pcd_pth):
    ret = None
    
    if ".pcd" in pcd_pth:
        pcd_load = o3d.io.read_point_cloud(pcd_pth)
        ret = np.asarray(pcd_load.points)
        ret = ret.squeeze()
        ret = utils.flip_xyz(ret, "y")
    elif ".bin" in pcd_pth:
        bin_pcd = np.fromfile(pcd_pth, dtype=np.float32)
        ret = bin_pcd.reshape((-1, 4))[:, 0:3]
        ret = ret.squeeze()
    else: pass
    
    return ret

