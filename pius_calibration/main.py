import calib_logic as calib
import calib_parser
import utils
import visualizer
import open3d as o3d


lidar_conf_pth1 = "./conf.yaml"  ## config 파일 라이다  
lidar_dir = "./dataset/lidar/"                  ## 라이다 pcd파일 디렉토리
img_dir = "./dataset/image_0/"                ## 이미지 파일 디렉토리
out_dir = "./dataset/output/"                 ## output 디렉토리


"""
    direction:
        1: x>0
        2: x<0
        3: y>0
        4: y<0
"""


"""
    ## !pip install opencv-python==3.4.17.61
    ## !pip install open3d

    ## core_dependencies: 
        ## python ver.__3.7.13 
        ## opencv ver.__3.4.17 
        ## open3d ver.__0.15.1 
        ## scipy ver.__1.7.3

    
    direction:
        1: x>0
        2: x<0
        3: y>0
        4: y<0
"""


##// 라이다만 오버레이
calib.overlay_lidar(lidar_conf_pth1, lidar_dir, img_dir, out_dir, 
                     direction=2, rot_dscr_lidar='euler_rad', 
                     start=0, end=1, show=False)

##// 레이다만 오버레이
#calib.overlay_radar(lidar_conf_pth1, radar_conf_pth, radar_dir, img_dir, out_dir, 'quat', 'euler_deg', 0, 1, False)

##// 라이다 레이다 둘 다 오버레이
#calib.overlay_fusion(lidar_conf_pth1, radar_conf_pth, lidar_dir, radar_dir, img_dir, out_dir, 4, 'quat', 'euler_deg', 0, 600, False)

#utils.make_vid("./output/vid_lidar_hub.mp4", "./output/",0,500)



# lidar_pth = "./dataset_sample\lidar/000599.pcd"
# radar_pth = "./dataset_sample/radar/000599.pcd"
# utils.show_lidar(lidar_pth)

# utils.show_fusion(lidar_pth, radar_pth)

# utils.show_fusion(lidar_pth, radar_pth, conf_dir, 'euler_deg', True)
# out_dir = "./output/radar_calibed/"
# before = calib_parser.parse_pcd(r"dataset_sample\radar\000599.pcd")
# utils.make_calib_radar(radar_conf_pth, radar_dir, out_dir, 'euler_deg', 0, 50)
# after = calib_parser.parse_pcd(r"output\radar_calibed\calibed_000599.pcd")

# print(before)
# print("\n")
# print(after)

# utils.show_lidar_bin(r"C:\Users\Admin\Downloads\Sample\3D 바운딩박스\1_원천데이터\1. 주간\2. 비\1. 구도심\20210907_021\lidar\total/1_1_1_20210907_021_00000018.bin")




# bin_pth = r"dataset_hub_inc\lidar\000000.pcd"

# bin_pcd = calib_parser.parse_pcd(bin_pth)

# bin_pcd = visualizer.cut_pcd(bin_pcd, 2)



# point_cloud = o3d.geometry.PointCloud()
# point_cloud.points = o3d.utility.Vector3dVector(bin_pcd) # array_of_points.shape = (N,3)
# #point_cloud.colors = o3d.utility.Vector3dVector(lidar_color) # array_of_colors.shape = (N,3)
# o3d.visualization.draw_geometries([point_cloud])        




