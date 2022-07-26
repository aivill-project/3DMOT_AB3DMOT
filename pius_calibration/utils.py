import numpy as np
import sys
import math
import open3d as o3d
import pandas as pd
import os
import cv2

import calib_parser
import calib_logic


##//좌표축, 범위 검증용 utils =============================

def minmax_pcd(pcd_arr):
    ##// pcd 데이터 좌표 범위 검증
    ##// pcd_arr: (N, 3) pcd array
        x_min = sys.float_info.max
        y_min = sys.float_info.max
        z_min = sys.float_info.max
        x_max = sys.float_info.min
        y_max = sys.float_info.min
        z_max = sys.float_info.min
        
        for pt in pcd_arr:
            x = pt[0]
            y = pt[1]
            z = pt[2]
            
            if x < x_min: x_min = x
            if y < y_min: y_min = y
            if z < z_min: z_min = z
            
            if x > x_max: x_max = x
            if y > y_max: y_max = y
            if z > z_max: z_max = z
        
        print("\n x min max")
        print(x_min, x_max)
        
        print("\n y min max")
        print(y_min, y_max)
        
        print("\n z min max") 
        print(z_min, z_max)
        return x_min, x_max, y_min, y_max, z_min, z_max


def rotate_2d(target_pt, base_pt, theta, is_deg):
    ##// 2차원 평면에서 회전
    
    if is_deg == True:
        theta = theta*math.pi / 180
    x, y = target_pt[0], target_pt[1]
    base_x, base_y = base_pt[0], base_pt[1]    
    ret_x = (x-base_x)*math.cos(theta) - (y-base_y)*math.sin(theta) + base_x
    ret_y = (x-base_x)*math.sin(theta) + (y-base_y)*math.cos(theta) + base_y
    ret_pt = np.array([ret_x, ret_y])
    return ret_pt


def flip_xyz(pcd_arr, to_flip=""):
    ##// x, y, z축 기준 단순 flip (축방향 검증용)
    ##// pcd_arr: (N, 3) pcd array
    ##// to_flip: (str) 'xyz', 'xy'......
    
    #pcd_arr = pcd_arr.to_numpy()
    
    for i in range(len(pcd_arr)):
        if 'x' in to_flip:
            pcd_arr[i,0] = -(pcd_arr[i,0])    
        if 'y' in to_flip:
            pcd_arr[i,1] = -(pcd_arr[i,1])
        if 'z' in to_flip:
            pcd_arr[i,2] = -(pcd_arr[i,2])

    return pcd_arr


def show_lidar(lidar_pth):
    
    lidar_pcd_arr = calib_parser.parse_pcd(lidar_pth)
    
    lidar_color = np.zeros((len(lidar_pcd_arr), 3))
    
    lidar_color = pd.DataFrame(lidar_color, columns=['r', 'g', 'b'], dtype=np.int64)
    lidar_color['b'] = lidar_color['b'] + 255
    
    lidar_color = lidar_color.to_numpy()
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_pcd_arr) # array_of_points.shape = (N,3)
    point_cloud.colors = o3d.utility.Vector3dVector(lidar_color) # array_of_colors.shape = (N,3)
    o3d.visualization.draw_geometries([point_cloud])        
    pass


def show_fusion(lidar_pth, radar_pth, conf_dir=None, rot_dscr_radar=None, to_calib=False):
    
    lidar_pcd_arr = calib_parser.parse_pcd(lidar_pth)
    
    radar_pcd_arr = calib_parser.parse_pcd(radar_pth)
    
    if to_calib:
        rmat_radar, tvec_radar = calib_parser.parse_conf_radar(conf_dir, rot_dscr_radar)
        radar_pcd_arr = calib_parser.rotate_translate_pcd(radar_pcd_arr, rmat_radar, tvec_radar)
    
    lidar_color = np.zeros((len(lidar_pcd_arr), 3))
    radar_color = np.zeros((len(radar_pcd_arr), 3))
    lidar_color = pd.DataFrame(lidar_color, columns=['r', 'g', 'b'], dtype=np.int64)
    lidar_color['b'] = lidar_color['b'] + 255
    radar_color = pd.DataFrame(radar_color, columns=['r', 'g', 'b'], dtype=np.int64)
    radar_color['r'] = radar_color['r'] + 255
    lidar_color = lidar_color.to_numpy()
    radar_color = radar_color.to_numpy()

    pcd_array = np.concatenate((lidar_pcd_arr, radar_pcd_arr), axis=0)
    color_array = np.concatenate((lidar_color, radar_color), axis=0)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pcd_array) # array_of_points.shape = (N,3)
    point_cloud.colors = o3d.utility.Vector3dVector(color_array) # array_of_colors.shape = (N,3)
    o3d.visualization.draw_geometries([point_cloud])        
    pass


def show_lidar_bin(lidar_pth):
    bin_pcd = np.fromfile(lidar_pth, dtype=np.float32)
    
    lidar_pcd_arr = bin_pcd.reshape((-1, 4))[:, 0:3]
    lidar_pcd_arr = lidar_pcd_arr.squeeze()

    # depth = bin_pcd.reshape((-1, 4))[:, 3:]
    # depth = depth.reshape((len(depth),1))
    # depth = pd.DataFrame(depth, columns=['d'], dtype=np.int64)

    # lidar_color = np.zeros((len(lidar_pcd_arr), 3))
    # lidar_color = pd.DataFrame(lidar_color, columns=['r', 'g', 'b'], dtype=np.int64)
    # lidar_color['b'] = depth['d']
    
    
    # lidar_color = lidar_color.to_numpy()
    
    # print(lidar_pcd_arr.shape, lidar_color.shape)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_pcd_arr) # array_of_points.shape = (N,3)
    #point_cloud.colors = o3d.utility.Vector3dVector(lidar_color) # array_of_colors.shape = (N,3)
    o3d.visualization.draw_geometries([point_cloud])        
    pass



##//파일 export =============================

def to_pcd(out_path, pcd_array):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_array.squeeze())
    
    o3d.io.write_point_cloud(out_path, pcd)


def make_calib_radar(conf_pth, radar_dir, out_dir, rot_dscr_radar, start, end):
    rmat_radar, tvec_radar = calib_parser.parse_conf_radar(conf_pth, rot_dscr_radar)
    radar_files = os.listdir(radar_dir)
    for i in range(start, end):
        radar_pth = f'{radar_dir}/{radar_files[i]}'
        radar_np = calib_parser.parse_pcd(radar_pth)
        radar_np = calib_logic.rotate_translate_pcd(radar_np, rmat_radar, tvec_radar)
        
        out_pth = f'{out_dir}/calibed_{radar_files[i]}'
        
        to_pcd(out_pth, radar_np)


def make_vid(out_pth, cam_dir, start, end):
    
    imgs = []
    # for file in glob.glob(f'{cam_dir}/*.jpg'):
    #     image = cv2.imread(file)
    #     height, width, _ = image.shape
    #     size = (width, height)
    #     imgs.append(image)

    cam_files = os.listdir(cam_dir)
    for i in range(start, end):
        cam_pth = f'{cam_dir}/{cam_files[i]}'
        image = cv2.imread(cam_pth)
        if image is not None:
            height, width, _ = image.shape
            size = (width, height)
            imgs.append(image)

    
    out = cv2.VideoWriter(
        f'{out_pth}', cv2.VideoWriter_fourcc(*'DIVX'), 12, size)

    for img in imgs:
        out.write(img)
    out.release()

    print(f'video saved: {out_pth}')
