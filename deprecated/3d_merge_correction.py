#!/usr/bin/env python3
"""
Author : Emmanuel Gonzalez
Date   : 2020-11-28
Purpose: 3D geo correction
"""

import argparse
import os
import sys
import numpy as np
# import logging
# from typing import Optional
# import datetime
from terrautils.spatial import scanalyzer_to_latlon, scanalyzer_to_utm
import json
import glob
import utm
import open3d as o3d
from math import pi, cos, sin
import multiprocessing


# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='Rock the Casbah',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('pcd',
                        metavar='pcd',
                        nargs='+',
                        help='Point cloud file/s')

    parser.add_argument('-o',
                        '--outdir',
                        help='Output directory',
                        metavar='outdir',
                        type=str,
                        default='3d_geo_correction_out')

    parser.add_argument('-m',
                        '--metadata',
                        help='Metadata file',
                        metavar='metadata',
                        type=str,
                        required=True)

    parser.add_argument('-c',
                        '--cpu',
                        help='CPUs to be used for multiprocessing',
                        metavar='cpu',
                        type=int,
                        required=True)

    parser.add_argument('-z',
                        '--z_offset',
                        help='Gantry offset',
                        type=float,
                        default=0.76)

    return parser.parse_args()


# --------------------------------------------------
def get_bounding_box(meta, sensor):
    args = get_args()

    scan_direction = int(meta['sensor_variable_metadata']['current setting Scan direction (automatically set at runtime)'])
    scan_distance = float(meta['sensor_variable_metadata']['current setting Scan distance (automatically set at runtime) [mm]'])/1000
    #scan_distance = float(meta['gantry_system_fixed_metadata']['system scan area e-w [m]']) #+ abs(west_loc_gantry_y - east_loc_gantry_y)
    fov_y = meta['sensor_fixed_metadata']['field of view y [m]']
    fov_x = scan_distance
    theta = np.radians(90)
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 1]])
    #print(scan_direction)

    if sensor == 'east':
        # East
        east_loc_gantry_x = float(meta['sensor_fixed_metadata']['scanner east location in camera box x [m]'])
        east_loc_gantry_y = float(meta['sensor_fixed_metadata']['scanner east location in camera box y [m]'])
        east_loc_gantry_z = float(meta['sensor_fixed_metadata']['scanner east location in camera box z [m]'])

        gantry_x = float(meta['gantry_system_variable_metadata']['position x [m]']) #+ east_loc_gantry_x
        gantry_y = float(meta['gantry_system_variable_metadata']['position y [m]']) #+ east_loc_gantry_y
        gantry_z = float(meta['gantry_system_variable_metadata']['position z [m]']) + args.z_offset + east_loc_gantry_z#offset in m


        if scan_direction == 0:
            gantry_y = gantry_y - scan_distance/2 + east_loc_gantry_y



        elif scan_direction==1:
            gantry_y = gantry_y + scan_distance/2 + east_loc_gantry_y

        bbox_center = scanalyzer_to_latlon(gantry_x, gantry_y)

    elif sensor == 'west':
        # West
        west_loc_gantry_x = float(meta['sensor_fixed_metadata']['scanner west location in camera box x [m]'])
        west_loc_gantry_y = float(meta['sensor_fixed_metadata']['scanner west location in camera box y [m]'])
        west_loc_gantry_z = float(meta['sensor_fixed_metadata']['scanner west location in camera box z [m]'])

        gantry_x = float(meta['gantry_system_variable_metadata']['position x [m]']) #+ west_loc_gantry_x
        gantry_y = float(meta['gantry_system_variable_metadata']['position y [m]']) #+ west_loc_gantry_y
        gantry_z = float(meta['gantry_system_variable_metadata']['position z [m]']) + args.z_offset + west_loc_gantry_z#offset in m

        if scan_direction == 0:
            gantry_y = gantry_y - scan_distance/2 + west_loc_gantry_y

        elif scan_direction==1:
            gantry_y = gantry_y + scan_distance/2 + west_loc_gantry_y

        bbox_center = scanalyzer_to_latlon(gantry_x, gantry_y)

    B = gantry_z
    A_x = np.arctan((0.5*float(fov_x))/3.5)
    A_y = np.arctan((0.5*float(fov_y))/3.5)
    L_x = 2*B*np.tan(A_x)
    L_y = 2*B*np.tan(A_y)

    x_n = gantry_y + (L_x/2)
    x_s = gantry_y - (L_x/2)
    y_w = gantry_x + (L_y/2)
    y_e = gantry_x - (L_y/2)

    bbox_nw_latlon = scanalyzer_to_latlon(y_w, x_n)
    bbox_se_latlon = scanalyzer_to_latlon(y_e, x_s)

    lon_shift = 0.000020308287
    lat_shift = 0.000018292 #0.000015258894

    b_box =  (bbox_se_latlon[0] - lat_shift,
              bbox_nw_latlon[0] - lat_shift,
              bbox_nw_latlon[1] + lon_shift,
              bbox_se_latlon[1] + lon_shift)

    return b_box, bbox_center, rotation_matrix, gantry_z


# --------------------------------------------------
def load_point_clouds(pcd_list, voxel_size=0.05):
    pcds = []
    file_name_list = []
    for i in pcd_list:

        file_name = i.split('/')[-1]
        file_name_list.append(file_name)
        pcd = o3d.io.read_point_cloud(i)
        #pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        #pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pcds.append(pcd)

    return pcds


# --------------------------------------------------
def latlong_to_utm_bbox(b_box):
    l_w = (b_box[2], b_box[0])
    u_w = (b_box[2], b_box[1])
    u_e = (b_box[3], b_box[1])
    l_e = (b_box[3], b_box[0])

    l_w = utm.from_latlon(float(l_w[1]), float(l_w[0]))[:2]
    u_w = utm.from_latlon(float(u_w[1]), float(u_w[0]))[:2]
    u_e = utm.from_latlon(float(u_e[1]), float(u_e[0]))[:2]
    l_e = utm.from_latlon(float(l_e[1]), float(l_e[0]))[:2]

    return [l_w, u_w, u_e, l_e]


# --------------------------------------------------
def latlong_to_utm_center(center):
    lat, long = center[0], center[1]
    center_utm = utm.from_latlon(lat, long)

    return center_utm[:2]


# --------------------------------------------------
def get_corners(bbox):
    u_w = latlong_to_utm_bbox(bbox)[1]
    l_e = latlong_to_utm_bbox(bbox)[3]
    center_x = l_e[0] - ((l_e[0] - u_w[0])/2)
    center_y = u_w[1] - ((u_w[1] - l_e[1])/2)

    return u_w, l_e, center_x, center_y


# --------------------------------------------------
def load_pcd(pcd_path, voxel_size=0.05):
    pcd = o3d.io.read_point_cloud(pcd_path)
    #pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    #pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    return pcd


# --------------------------------------------------
def geo_reference_pcd(pcd, utm_x, utm_y, gantry_z, rotation_matrix):
    #center_z = float(pcd.get_center()[2])
    corrected_pcd = pcd.translate([utm_x, utm_y, gantry_z], relative=False)
    rotated_pcd = corrected_pcd.rotate(rotation_matrix, center=[utm_x, utm_y, gantry_z])

    return rotated_pcd


# --------------------------------------------------
def scale_pcd(pcd, scale_parameter, utm_x, utm_y, gantry_z):
    scaled_pcd = pcd.scale(float(scale_parameter), center=[utm_x, utm_y, gantry_z])

    return scaled_pcd


# --------------------------------------------------
def get_scale_parameter(pcd, real_scale):
    max_x, max_y, _ = pcd.get_max_bound()
    min_x, min_y, _ = pcd.get_min_bound()

    over_scale = abs(max_x - min_x)
    scale_parameter = real_scale/over_scale

    return scale_parameter


# --------------------------------------------------
def main():
    """Make a jazz noise here"""

    args = get_args()

    out_path = args.outdir
    os.makedirs(out_path) if not os.path.isdir(out_path) else None

    meta_path = args.metadata
    with open(meta_path) as f:
        meta = json.load(f)['lemnatec_measurement_metadata']

    fname = str(os.path.splitext(os.path.basename(args.pcd[0]))[-2]).split('__')[-2]

    e_bbox, e_center, rot_matrix, gantry_z = get_bounding_box(meta, 'east')
    w_bbox, w_center, _, _ = get_bounding_box(meta, 'west')
    _, east_l_e, _, _ = get_corners(e_bbox)
    west_u_w, _, _, _= get_corners(w_bbox)

    real_scale = abs(east_l_e[0] - west_u_w[0])
    # down_pcd = load_pcd(pcd)
    print('Loading/merging point clouds.')
    down_pcd = load_point_clouds(args.pcd)

    center_x = float(east_l_e[0] - abs(east_l_e[0] - west_u_w[0])/2)
    center_y = float(west_u_w[1] - abs(east_l_e[1] - west_u_w[1])/2)

    print('Rotating and translating merged point cloud.')
    corr_pcd = geo_reference_pcd(down_pcd, center_x, center_y, gantry_z, rot_matrix)
    scale_parameter = get_scale_parameter(corr_pcd, real_scale)

    print('Scaling point cloud.')
    scaled_pcd = scale_pcd(corr_pcd, float(scale_parameter), center_x, center_y, gantry_z)
    out_name = os.path.join(out_path, fname+'_corrected.ply')

    print('Saving corrected point cloud.')
    # o3d.io.write_point_cloud(out_name, scaled_pcd)

    print(f'Done. See output in {out_path}')
    return None



# --------------------------------------------------
if __name__ == '__main__':
    main()
