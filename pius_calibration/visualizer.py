import pandas as pd
import numpy as np
import math
from PIL import Image
import matplotlib.pyplot as plt




##//visualizer =============================

def cut_pcd(pcd_arr, direct):
    ##// FOV에 따라 pcd 데이터 일부 drop  
    ##// pcd_arr: (N, 3) pcd array
    ##// direction: (int) 카메라 시점 (-1이면 자르지 않음) 
    
    pcd_df = pd.DataFrame(pcd_arr, columns=['x', 'y', 'z'])

    if direct==1:
        pcd_df = pcd_df[pcd_df['x'] >= 0]
    elif direct==2:
        pcd_df = pcd_df[pcd_df['x'] <= 0]
    elif direct==3:
        pcd_df = pcd_df[pcd_df['y'] >= 0]
    elif direct==4:
        pcd_df = pcd_df[pcd_df['y'] <= 0]
        
    
    pcd_arr = pcd_df.to_numpy()
    
    return pcd_arr


# def pcd_cut(pcd_arr):
    
#     pcd_df = pd.DataFrame(pcd_arr, columns=['x', 'y', 'z'])
    
#     pcd_df = pcd_df[pcd_df['x'] >= 0]
    
#     pcd_arr = pcd_df.to_numpy()
    
#     return pcd_arr



def get_depth(pcd_arr):
    """_summary_

    Args:
        pcd_arr (_np.ndarray_): _(N,3)_

    Returns:
        _type_: _(np.ndarray)_: (N,)
    """
    depth_list = []
    for pt in pcd_arr:
        x = pt[0]
        y = pt[1]
        z = pt[2]
        
        dist = math.sqrt((x**2) + (y**2) + (z**2))
        dist = math.log((dist+1))
        
        # dist = -(1/(dist+0.1))
        depth_list.append(dist)
    ret_depth = np.array(depth_list).reshape((-1,1))
    return ret_depth


def make_img_lidar(img_pth, out_pth, projected_arr, d, show=False):
    image = Image.open(img_pth)
    image_compare = image.copy()
    background = image.copy()
    
    px = background.load()
    for i in range(image.size[0]):
        for j in range(image.size[1]):
            px[i, j] = (255, 255, 255)
    
    projected_arr = np.squeeze(projected_arr)
    u = projected_arr[:,0] 
    v = projected_arr[:,1] -100
    
    
    
    # plt.figure(figsize=(20,10))
    # plt.imshow(np.array(image_compare))
    # plt.savefig(f'{out_pth}origin_img_lidar.jpg', bbox_inches='tight', pad_inches=0)
    # if show: plt.show()
    
    plt.figure(figsize=(35,10))
    
    plt.subplot(121)
    plt.scatter(x=u, y=v, c=d, cmap='hsv', alpha=0.7, s=2)
    plt.imshow(np.array(image))
    plt.axis('off')
    #plt.savefig(f'{out_pth}overlay_img_lidar.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()        
    
    
    plt.subplot(122)
    #plt.figure(figsize=(20,10))
    plt.scatter(x=u, y=v, c=d, cmap='hsv', alpha=0.7, s=2)
    plt.imshow(background)
    plt.axis('off')
    
    plt.subplots_adjust(wspace=0.02)
    plt.savefig(f'{out_pth}overlay_lidar.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()
    
    """_summary_

    view_no = 0
    lidar_path = f'./input/lidar/{view_no:06d}.pcd'

    for i in range(1, 6):
        config_path = f'./input/calib/config0{i}.yaml'
        img_path = f'./input/camera/0{i}/{view_no:06d}.jpg'
        out_path = f'./output/0{i}/{view_no:06d}.jpg'

        origin, overlay, lidar = utils.get_overlay_lidar(
            config_path, img_path, lidar_path)

    fig = plt.figure(dpi=600)

    plt.subplot(131)
    plt.imshow(origin)
    plt.axis('off')

    plt.subplot(132)
    plt.imshow(overlay)
    plt.axis('off')

    plt.subplot(133)
    plt.imshow(lidar)
    plt.axis('off')

    plt.subplots_adjust(wspace=0.02)

    # plt.savefig(f'./test_{i}.jpg', bbox_inches='tight', pad_inches=0)

    break

    plt.show()
    
    """
    
    pass
    

def make_img_radar(img_pth, out_pth, projected_arr, d, show=False):
    image = Image.open(img_pth)
    image_compare = image.copy()
    background = image.copy()
    
    px = background.load()
    for i in range(image.size[0]):
        for j in range(image.size[1]):
            px[i, j] = (255, 255, 255)
    
    projected_arr = np.squeeze(projected_arr)
    u = projected_arr[:,0]
    v = projected_arr[:,1]
    
    
    
    
    # plt.figure(figsize=(20,10))
    # plt.imshow(np.array(image_compare))
    # plt.savefig(f'{out_pth}origin_img_radar.jpg', bbox_inches='tight', pad_inches=0)
    # if show: plt.show()
    
    plt.figure(figsize=(35,10))
    
    plt.subplot(121)
    plt.scatter(x=u, y=v, c=d, cmap='rainbow_r', alpha=1.0, s=20)
    plt.imshow(np.array(image))
    plt.axis('off')
    #plt.savefig(f'{out_pth}overlay_img_radar.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()        
    
    
    #plt.figure(figsize=(20,10))
    plt.subplot(122)
    plt.scatter(x=u, y=v, c=d, cmap='rainbow_r', alpha=1.0, s=20)
    plt.imshow(background)
    plt.axis('off')
    
    plt.subplots_adjust(wspace=0.02)
    plt.savefig(f'{out_pth}overlay_radar.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()


def make_img_fusion(img_pth, out_pth, projected_arr, projected_arr_radar, d, show=False):
    image = Image.open(img_pth)
    image_compare = image.copy()
    background = image.copy()
    
    px = background.load()
    for i in range(image.size[0]):
        for j in range(image.size[1]):
            px[i, j] = (255, 255, 255)
    
    projected_arr = np.squeeze(projected_arr)
    u = projected_arr[:,0]
    v = projected_arr[:,1]
    
    projected_arr_radar = np.squeeze(projected_arr_radar)
    u_radar = projected_arr_radar[:,0]
    v_radar = projected_arr_radar[:,1]
    
    # plt.figure(figsize=(60,10))
    
    
    # plt.imshow(np.array(image_compare))
    # plt.savefig(f'{out_pth}origin_img_fusion.jpg', bbox_inches='tight', pad_inches=0)
    # if show: plt.show()
    
    plt.figure(figsize=(35,10))
    
    plt.subplot(121)
    plt.scatter(x=u, y=v, c=d, cmap='hsv', alpha=0.7, s=2)
    plt.scatter(x=u_radar, y=v_radar, color='red', alpha=1.0, s=10)
    plt.imshow(np.array(image))
    plt.axis('off')
    # plt.savefig(f'{out_pth}overlay_img_fusion.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()        
    
    plt.subplot(122)
    # plt.figure(figsize=(20,10))
    plt.scatter(x=u, y=v, c=d, cmap='hsv', alpha=0.7, s=2)
    plt.scatter(x=u_radar, y=v_radar, color='red', alpha=1.0, s=10)
    plt.imshow(background)
    plt.axis('off')
    
    
    plt.subplots_adjust(wspace=0.02)
    plt.savefig(f'{out_pth}overlay_fusion.jpg', bbox_inches='tight', pad_inches=0)
    if show: plt.show()


