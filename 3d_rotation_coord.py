#!/usr/bin/env python3
"""
Author : Emmanuel Gonzalez
Date   : 2020-12-15
Purpose: 3D point cloud rotation and initial coordinate registration
"""

import argparse
import os
import sys
import open3d as o3d
import numpy as np
import json
import utm
from terrautils.spatial import scanalyzer_to_utm, scanalyzer_to_latlon
from geographiclib.constants import Constants
from geographiclib.geodesic import Geodesic


# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='Geo-registration of point cloud data',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('pcd',
                        metavar='pcd',
                        help='Merged point cloud')

    parser.add_argument('-o',
                        '--outdir',
                        help='Output directory',
                        metavar='outdir',
                        type=str,
                        default='rotation_registration_out')

    parser.add_argument('-r',
                        '--rotation_theta',
                        help='Degrees by which to rotate PCD',
                        metavar='rotation_theta',
                        type=int,
                        default=90)

    parser.add_argument('-m',
                        '--meta_path',
                        help='Metadata path',
                        metavar='meta_path',
                        required=True)

    return parser.parse_args()


# --------------------------------------------------
def open_pcd(pcd_path):

    pcd = o3d.io.read_point_cloud(pcd_path, print_progress=True)

    return pcd


# --------------------------------------------------
def scale_pcd(pcd):

    scaled_pcd = pcd.scale(0.001, pcd.get_center())

    return scaled_pcd


# --------------------------------------------------
def rotate_pcd(scaled_pcd, rotation_theta):

    theta = np.radians(rotation_theta)

    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]])

    rotated_pcd = scaled_pcd.rotate(rotation_matrix, center=scaled_pcd.get_center())

    return rotated_pcd


# --------------------------------------------------
def get_distance(rotated_pcd):

    min_x, min_y, z = rotated_pcd.get_min_bound()
    max_x, max_y, z = rotated_pcd.get_max_bound()

    distance = (max_x-min_x)/2

    return distance


# --------------------------------------------------
def get_meta_info(meta_path):

    with open(meta_path) as f:
        meta = json.load(f)['lemnatec_measurement_metadata']

    scan_dir = int(meta['sensor_variable_metadata']['current setting Scan direction (automatically set at runtime)'])

    east_x = float(meta['sensor_fixed_metadata']['scanner east location in camera box x [m]'])
    east_y = float(meta['sensor_fixed_metadata']['scanner east location in camera box y [m]'])

    west_x = float(meta['sensor_fixed_metadata']['scanner west location in camera box x [m]'])
    west_y = float(meta['sensor_fixed_metadata']['scanner west location in camera box y [m]'])

    y_diff = abs(east_y - west_y)/2
    x_diff = east_x

    x = float(meta['gantry_system_variable_metadata']['position x [m]']) #+ x_diff
    y = float(meta['gantry_system_variable_metadata']['position y [m]']) + y_diff if scan_dir==1 else float(meta['gantry_system_variable_metadata']['position y [m]']) - y_diff
    z = float(meta['gantry_system_variable_metadata']['position z [m]'])

    lat, lon = scanalyzer_to_latlon(x, y)

    return lat, lon, scan_dir, z


# --------------------------------------------------
def get_endpoint(lat1, lon1, bearing, d):


    geod = Geodesic(Constants.WGS84_a, Constants.WGS84_f)
    d = geod.Direct(lat1, lon1, bearing, d)#* 1852.0)

    return d['lat2'], d['lon2']


# --------------------------------------------------
def to_utm(end_lat, end_lon):

    utm_x, utm_y, _, _ = utm.from_latlon(end_lat, end_lon, 12, 'N')

    return utm_x, utm_y


# --------------------------------------------------
def translate_pcd(rotated_pcd, utm_x, utm_y, scan_dir, z):

    trans_pcd = rotated_pcd.translate([utm_x-0.15, utm_y+0.25, z], relative=False) if scan_dir==0 else rotated_pcd.translate([utm_x+0.15, utm_y+0.25, z], relative=False)

    return trans_pcd


# --------------------------------------------------
def write_pcd(out_path, trans_pcd):
    args = get_args()
    o3d.io.write_point_cloud(out_path, trans_pcd)


# --------------------------------------------------
def main():
    """Make a jazz noise here"""

    args = get_args()

    # Generate output filename and make output directory
    f_name = os.path.splitext(os.path.basename(args.pcd))[-2] + '_registered.ply'
    out_path = os.path.join(args.outdir, f_name)

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    # Open point cloud
    pcd = open_pcd(args.pcd)

    print(f_name)
    # Scale and rotate point cloud
    scaled_pcd = scale_pcd(pcd)
    rotated_pcd = rotate_pcd(scaled_pcd, args.rotation_theta)

    # Calculate distance traveled and calculate the center point of the scan
    distance = get_distance(rotated_pcd)
    lat, lon, scan_dir, z = get_meta_info(args.meta_path)
    end_lat, end_lon = get_endpoint(lat, lon, 90, distance) if scan_dir==0 else get_endpoint(lat, lon, 270, distance)
    utm_x, utm_y = to_utm(end_lat, end_lon)

    # Translate point cloud to center point (UTM)
    trans_pcd = translate_pcd(rotated_pcd, utm_x, utm_y, scan_dir, z)

    # Write point cloud to file
    write_pcd(out_path, trans_pcd)


# --------------------------------------------------
if __name__ == '__main__':
    main()
