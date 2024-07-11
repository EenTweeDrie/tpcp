import math
import statistics
import time
from datetime import date

import circle_fit as cf
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pandas as pd
import plotly.graph_objects as go
from django.contrib.auth import logout
from django.contrib.auth.views import LoginView
from django.http import HttpResponseNotFound
from django.shortcuts import redirect
from django.shortcuts import render
from django.views.generic import DetailView, ListView
from plotly.offline import plot
from pyntcloud import PyntCloud
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from sklearn.cluster import OPTICS

from .forms import *
from .models import *


def check_direction(center, direction_vector_length, num_cyl, nums_cyl_to_del):
    l_mic_line_1 = math.sqrt(
        (center[num_cyl][0] - center[num_cyl - 1][0]) ** 2 + (center[num_cyl][1] - center[num_cyl - 1][1]) ** 2 + (
                    center[num_cyl][2] - center[num_cyl - 1][2]) ** 2)
    direction_mic_line_1 = [(center[num_cyl][0] - center[num_cyl - 1][0]) / l_mic_line_1,
                            (center[num_cyl][1] - center[num_cyl - 1][1]) / l_mic_line_1,
                            (center[num_cyl][2] - center[num_cyl - 1][2]) / l_mic_line_1]
    arccos_min_line_1_to_length = (180 / math.pi) * math.acos((direction_mic_line_1[0] * direction_vector_length[0] +
                                                               direction_mic_line_1[1] * direction_vector_length[1] +
                                                               direction_mic_line_1[2] * direction_vector_length[2]) / (
                                                                      math.sqrt(direction_mic_line_1[0] ** 2 +
                                                                                direction_mic_line_1[1] ** 2 +
                                                                                direction_mic_line_1[2] ** 2) *
                                                                      math.sqrt(direction_vector_length[0] ** 2 +
                                                                                direction_vector_length[1] ** 2 +
                                                                                direction_vector_length[2] ** 2)))
    if arccos_min_line_1_to_length > 35:
        nums_cyl_to_del = np.append(nums_cyl_to_del, [num_cyl - 1], axis=0)
    return nums_cyl_to_del


def make_circle_tree(center, radius, kpol):
    circle = np.array([[0, 0, 0]])
    lines = np.array([[0, 0]])
    k = -1
    for i in range(0, radius.size):
        circle = np.append(circle, [[center[i][0], center[i][1], center[i][2]]], axis=0)
        k += 1
        for j in range(0, kpol + 1):
            a = (math.pi * j * 2) / kpol
            x = center[i][0] + radius[i] * math.cos(a)
            y = center[i][1] + radius[i] * math.sin(a)
            circle = np.append(circle, [[x, y, center[i][2]]], axis=0)
            k += 1
            if j != kpol - 1:
                lines = np.append(lines, [[k, k + 1]], axis=0)
            else:
                lines = np.append(lines, [[k, k - kpol + 1]], axis=0)
        circle = np.append(circle, [[center[i][0], center[i][1], center[i][2]]], axis=0)
        if i != radius.size - 1:
            lines = np.append(lines, [[k - kpol, k + 1]], axis=0)
    circle = np.delete(circle, 0, axis=0)
    lines = np.delete(lines, 0, axis=0)
    return circle, lines, kpol


def down_pcd(data):
    if data.shape[0] > 40000:
        factor = data.shape[0] // 40000
    else:
        factor = 1
    down_data = data[::factor]
    return down_data


def np_to_pcd(points):
    colors = np.zeros(points.shape)
    for i in range(0, points.shape[0]):
        colors[i][0] = 0.12156863
        colors[i][1] = 0.46666667
        colors[i][2] = 0.70588235
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    return point_cloud


def preparedata(file_path):
    print("Load a pcd point cloud, print it, and render it")
    if file_path.endswith('.txt'):
        print('TXT')
        data = pd.read_csv(file_path, sep=' ', dtype=np.float64)
        data = down_pcd(data)
        try:
            points = np.asarray(data[['//X', 'Y', 'Z']])
        except KeyError:
            points = np.asarray(data[['X', 'Y', 'Z']])
        try:
            try:
                intensity = np.asarray(data['Intensity'])
            except KeyError:
                intensity = np.asarray(data['//Intensity'])
            intensity_exist = True
            RGBint = intensity / max(intensity)
        except KeyError:
            intensity_exist = False
            RGBint = [0]
        point_cloud = np_to_pcd(points)

    if file_path.endswith('.pcd'):
        try:
            print('PCD1')
            point_cloud = PyntCloud.from_file(file_path)
            data = point_cloud.points
            data = down_pcd(data)
            points = np.asarray(data[['x', 'y', 'z']])
            print('!!!', points.shape[0])
            try:
                intensity = np.asarray(data['Intensity'])
                intensity_exist = True
                RGBint = intensity / max(intensity)
            except KeyError:
                intensity_exist = False
                RGBint = [0]
            point_cloud = np_to_pcd(points)
            intensity_exist = True
            print("intensity_exist = True")
        except (NotImplementedError, ValueError) as e:
            print('PCD2')
            pcd = o3d.io.read_point_cloud(file_path)
            print(pcd)
            points = np.asarray(pcd.points)
            decimated_points = down_pcd(points)
            pcd = np_to_pcd(decimated_points)
            print('downpcd:', pcd)
            point_cloud = pcd
            points = np.asarray(pcd.points)
            intensity_exist = False
            RGBint = [0]
            print("intensity_exist = False")
    print("=======================")
    print(point_cloud)
    print(points)
    print(intensity_exist)
    print(RGBint)
    print("=======================")
    return point_cloud, points, intensity_exist, RGBint


def clustering_dbscan(point_cloud, my_eps, my_min_points):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(point_cloud.cluster_dbscan(eps=my_eps, min_points=my_min_points, print_progress=True))
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return point_cloud, labels


def open_txt(file_path):
    data = pd.read_csv(file_path, sep=' ', header=None)
    if str(data[0][0]).find('//') != -1:
        f = open(file_path, 'r')
        column_names = f.readline().split()
        print(column_names)
        f.close()
        column_names[0] = column_names[0].replace('//', '')
        data = data.drop(labels=[0], axis=0)
        data.columns = column_names
    else:
        column_names = ['X', 'Y', 'Z', 'R', 'G', 'B', 'Intensity', 'Illuminance_(PCV)']
        data = data.drop(labels=[0], axis=0)
        data.columns = column_names
    print(data)
    return data


def toFixed(numObj, digits=0):
    return f"{numObj:.{digits}f}"


def processing(file_path, EPS, MINPTS, ITER, ONNORMALSZ):
    print('использовано!!!!', EPS, MINPTS, ITER, ONNORMALSZ)
    ITER = ITER + 1

    start = time.time()
    point_cloud, points, intensity_exist, RGBint = preparedata(file_path)

    x_min, y_min, z_min = points.min(axis=0)
    x_max, y_max, z_max = points.max(axis=0)
    print("min:", x_min, y_min, z_min)
    print("max:", x_max, y_max, z_max)
    for i in range(points.shape[0]):
        points[i][0] -= x_min
        points[i][1] -= y_min
        points[i][2] -= z_min
    height_tree = z_max - z_min

    arg_x_min, arg_y_min, arg_z_min = points.argmin(axis=0)
    arg_x_max, arg_y_max, arg_z_max = points.argmax(axis=0)
    x_min, y_min, z_min = points.min(axis=0)
    x_max, y_max, z_max = points.max(axis=0)
    print("arg_min:", arg_z_min)
    print("arg_max:", arg_z_max)
    print("min_point_z:", points[arg_z_min])
    print("max_point_z:", points[arg_z_max])
    length_tree = math.sqrt(
        (points[arg_z_max][0] - points[arg_z_min][0]) ** 2 + (points[arg_z_max][1] - points[arg_z_min][1]) ** 2 + (
                    points[arg_z_max][2] - points[arg_z_min][2]) ** 2)
    direction_vector_length = [(points[arg_z_max][0] - points[arg_z_min][0]) / length_tree,
                               (points[arg_z_max][1] - points[arg_z_min][1]) / length_tree,
                               (points[arg_z_max][2] - points[arg_z_min][2]) / length_tree]
    print("length:", length_tree)
    print("direction_vector_length:", direction_vector_length)

    print('height_tree:', height_tree)

    idx_all = np.empty((points.shape[0], 1), dtype=int)
    for i in range(points.shape[0]):
        idx_all[i][0] = i
    pointsres = points


    # удаление земли
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    normals = np.asarray(point_cloud.normals)
    # normals=pptk.estimate_normals(points,k=6,r=np.inf)
    print(normals)

    print('remove ground...')
    idx_normals = np.where((abs(normals[:, 2]) < 0.7))
    idx_ground = np.where(points[:, 2] > np.min(points[:, 2] + 0.5))
    idx_wronglyfiltered = np.setdiff1d(idx_ground, idx_normals)
    idx_retained = np.append(idx_normals, idx_wronglyfiltered)
    points = points[idx_retained]
    print(points.shape[0])
    idx_retained = np.asarray(idx_retained)
    idx_retained = np.transpose(idx_retained)

    print(idx_retained.shape)
    print(idx_all.shape)

    idx_inv = np.setdiff1d(idx_all, idx_retained)
    print(idx_inv)
    points_ground = pointsres[idx_inv]

    point_cloud = np_to_pcd(points)


    # удаление шумов
    print('remove noise...')
    point_cloud, labels = clustering_dbscan(point_cloud, 0.5, 10)

    idx_labels = np.where(labels > -1)
    points_no_noise = points[idx_labels]
    if len(RGBint) > 1:
        RGBint = RGBint[idx_labels]
    point_cloud_no_noise = np_to_pcd(points_no_noise)

    idx_labels_inv = np.where(labels < 0)
    points_noise = points[idx_labels_inv]
    point_cloud_noise = np_to_pcd(points_noise)

    cur_points = points_no_noise
    cur_points_inv = points_noise


    if intensity_exist == False:
        print('remove branch...')
        # удаление веток
        for i in range(1, ITER):
            clustering = OPTICS(max_eps=0.5, cluster_method='xi', min_cluster_size=100 // i).fit(cur_points)

            idx_labels = np.where(clustering.labels_ > -1)
            if i == 1:
                cur_points_inv = cur_points[idx_labels]
            else:
                cur_points_inv = np.vstack((cur_points_inv, cur_points[idx_labels]))

            idx_labels_inv = np.where(clustering.labels_ < 0)
            cur_points = cur_points[idx_labels_inv]

        points_branch = cur_points_inv


        # удаление листвы
        print('remove foliage...')
        for i in range(1, ITER):
            clustering = OPTICS(max_eps=0.5, cluster_method='xi', min_cluster_size=12, min_samples=3).fit(cur_points)

            idx_labels = np.where(clustering.labels_ > -1)
            if i == 1:
                cur_points_inv = cur_points[idx_labels]
            else:
                cur_points_inv = np.vstack((cur_points_inv, cur_points[idx_labels]))

            idx_labels_inv = np.where(clustering.labels_ < 0)
            cur_points = cur_points[idx_labels_inv]

        points_foliage = cur_points_inv

        print('clustering trunk...')

        P0 = pd.DataFrame(cur_points, columns=['X', 'Y', 'Z'])
        if ONNORMALSZ == True:
            cur_p_cloud = np_to_pcd(cur_points)
            cur_p_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            normals = np.asarray(cur_p_cloud.normals)
            P0['NormalsZ'] = normals[:, 2]  # on/off
        X = np.asarray(P0)
        clustering = DBSCAN(eps=EPS, min_samples=MINPTS).fit(X)
        labels = clustering.labels_

        idx_labels = np.where(labels > -1)
        points_trunk = cur_points[idx_labels]

        idx_labels = np.where(labels < 0)
        points_no_trunk = cur_points[idx_labels]

        points_foliage = np.vstack((points_foliage, points_no_trunk))

    P = pd.DataFrame(cur_points, columns=['X', 'Y', 'Z'])

    if intensity_exist == True:
        P['Intensity'] = RGBint  # on/off

        if ONNORMALSZ == True:
            cur_p_cloud = np_to_pcd(cur_points)
            cur_p_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            normals = np.asarray(cur_p_cloud.normals)
            P['NormalsZ'] = normals[:, 2]  # on/off
        X = np.asarray(P)

        clustering = DBSCAN(eps=EPS, min_samples=MINPTS).fit(X)
        labels = clustering.labels_
        max_label = labels.max()
        print(f"Point cloud has {max_label + 1} clusters")
        idx_layer = np.where(labels > -1)
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0

        points_trunk = cur_points[idx_layer]

        idx_labels = np.where(labels < 0)
        points_no_trunk = cur_points[idx_labels]

        # удаление веток
        print('remove branch...')
        cur_points = points_no_trunk
        idx_ground = np.where(cur_points[:, 2] > np.min(cur_points[:, 2] + 0.5))
        idx_ground_inv = np.where(cur_points[:, 2] <= np.min(cur_points[:, 2] + 0.5))
        points_ground = np.vstack((points_ground, cur_points[idx_ground_inv]))
        cur_points = cur_points[idx_ground]

        for i in range(1, ITER):
            clustering = OPTICS(max_eps=0.5, cluster_method='xi', min_cluster_size=100 // i).fit(cur_points)

            idx_labels = np.where(clustering.labels_ > -1)
            if i == 1:
                cur_points_inv = cur_points[idx_labels]
            else:
                cur_points_inv = np.vstack((cur_points_inv, cur_points[idx_labels]))
            cur_point_cloud_inv = np_to_pcd(cur_points_inv)

            idx_labels_inv = np.where(clustering.labels_ < 0)
            cur_points = cur_points[idx_labels_inv]
            cur_point_cloud = np_to_pcd(cur_points)

        points_branch = cur_points_inv
        points_no_branch = cur_points
        points_foliage = points_no_branch

    points = points_trunk
    print("new")
    print(z_max)
    lst1 = np.unique(points[:, 2])
    print("No of unique elements in the list are:", len(lst1))
    toch = 0.05

    labelsPoint = []
    for i in range(0, labels.shape[0]):
        labelsPoint.append(0)

    center = np.array([[0, 0, 0]])
    radius = np.array([0])
    r_prev = 0
    r_prevH = 0
    h = 0
    prev_median = 0
    prev_medianH = 0
    fh = False
    num_cyl = 0
    nums_cyl_to_del = []
    h_list = np.array([0])

    centerH = np.array([[0, 0, 0]])
    radiusH = np.array([0])

    while h < height_tree:
        idx_layer = np.where((points[:, 2] >= h) & (points[:, 2] < h + toch))
        save_z = (2 * h + toch) / 2
        h += toch
        slice_points = points[idx_layer]
        if len(slice_points) > 2:
            xc, yc, r, _ = cf.least_squares_circle(slice_points)
            xcH, ycH, rH, _ = cf.hyper_fit(slice_points)

            if ((fh) & (save_z > 0.25 * height_tree) & (r > 2 * prev_median)):
                continue

            if (r > 2.5) | (rH > 2.5):
                continue

            if ((save_z > 0.5 * height_tree) & (save_z < 0.75 * height_tree) & (r > 1.5 * prev_median)):
                r = 0.99 * r_prev
            if ((save_z > 0.5 * height_tree) & (save_z < 0.75 * height_tree) & (rH > 1.5 * prev_medianH)):
                rH = 0.99 * r_prevH

            r_prev = r
            r_prevH = rH

            center = np.append(center, [[xc, yc, save_z]], axis=0)
            radius = np.append(radius, [r], axis=0)
            h_list = np.append(h_list, [round(h, 2)], axis=0)

            centerH = np.append(centerH, [[xcH, ycH, save_z]], axis=0)
            radiusH = np.append(radiusH, [rH], axis=0)

            num_cyl += 1
            prev_median = statistics.median(radius)
            prev_medianH = statistics.median(radiusH)
            fh = True

    h_list = np.delete(h_list, 0, axis=0)
    center = np.delete(center, 0, axis=0)
    radius = np.delete(radius, 0, axis=0)

    centerH = np.delete(centerH, 0, axis=0)
    radiusH = np.delete(radiusH, 0, axis=0)


    # проверка углов несколько раз
    fdel = True
    while fdel:
        num_cyl = 0
        for i in range(radius.size):
            if (num_cyl > 1):
                nums_cyl_to_del = check_direction(center, direction_vector_length, num_cyl, nums_cyl_to_del)
            num_cyl += 1
        ii = 0
        if len(nums_cyl_to_del) != 0:
            for i in range(nums_cyl_to_del.size):
                center = np.delete(center, int(nums_cyl_to_del[i] - ii), axis=0)
                radius = np.delete(radius, int(nums_cyl_to_del[i] - ii), axis=0)
                centerH = np.delete(centerH, int(nums_cyl_to_del[i] - ii), axis=0)
                radiusH = np.delete(radiusH, int(nums_cyl_to_del[i] - ii), axis=0)
                h_list = np.delete(h_list, int(nums_cyl_to_del[i] - ii), axis=0)
                ii += 1
            nums_cyl_to_del = []
        else:
            fdel = False


    # результаты
    hr = pd.DataFrame({"h": h_list, "radius": radius})
    hr_filter = hr['h'].isin([1.25, 1.30, 1.35])
    hrb = np.asarray(hr[hr_filter]['radius'])

    hrH = pd.DataFrame({"h": h_list, "radiusH": radiusH})
    hr_filterH = hrH['h'].isin([1.25, 1.30, 1.35])
    hrbH = np.asarray(hrH[hr_filterH]['radiusH'])

    breast_diameter_tree = 2 * statistics.median(hrb)
    breast_diameter_tree_hyper = 2 * statistics.median(hrbH)
    print("Diameter:", breast_diameter_tree)
    print("Diameter_hyper:", breast_diameter_tree_hyper)
    print("Length:", length_tree)
    print("Height:", height_tree)
    print(type(breast_diameter_tree), type(breast_diameter_tree_hyper), type(length_tree), type(height_tree))

    idd_center = np.where((center[:, 2] >= 1.25) & (center[:, 2] <= 1.35))
    d_center = center[idd_center]
    d_radius = radius[idd_center]
    x_center, y_center, z_center = center[0]
    print(x_center, y_center, z_center)
    kpol = 48
    circle, lines, kpol = make_circle_tree(center, radius, kpol)
    kpol = 64
    d_circle, d_lines, kpol = make_circle_tree(d_center, d_radius, kpol)

    idd_centerH = np.where((centerH[:, 2] >= 1.25) & (centerH[:, 2] <= 1.35))
    d_centerH = centerH[idd_centerH]
    d_radiusH = radiusH[idd_centerH]
    x_centerH, y_centerH, z_centerH = centerH[0]
    print(x_centerH, y_centerH, z_centerH)
    kpol = 64
    d_circleH, d_linesH, kpol = make_circle_tree(d_centerH, d_radiusH, kpol)

    breast_diameter_tree = 100 * float(toFixed(breast_diameter_tree, 4))
    breast_diameter_tree_hyper = 100 * float(toFixed(breast_diameter_tree_hyper, 4))
    breast_diameter_tree = float(toFixed(breast_diameter_tree, 2))
    breast_diameter_tree_hyper = float(toFixed(breast_diameter_tree_hyper, 2))
    length_tree = float(toFixed(length_tree, 5))
    height_tree = float(toFixed(height_tree, 5))
    print("Diameter:", breast_diameter_tree)
    print("Diameter_hyper:", breast_diameter_tree_hyper)
    print("Length:", length_tree)
    print("Height:", height_tree)

    if (points_branch.shape[0] > 0):
        cur_p = np.vstack((points_foliage, points_branch))
        print('this0')
    else:
        cur_p = points_foliage
        print('this')
    P1 = pd.DataFrame(cur_p, columns=['X', 'Y', 'Z'])

    cur_p_cloud = np_to_pcd(cur_p)
    cur_p_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    normals = np.asarray(cur_p_cloud.normals)
    P1['NormalsX'] = normals[:, 0]  # on/off
    P1['NormalsY'] = normals[:, 1]  # on/off
    # P1['NormalsZ'] = normals[:,2] #on/off
    X = np.asarray(P1)

    clustering = DBSCAN(eps=0.95, min_samples=200).fit(X)
    labels = clustering.labels_

    idx_labels = np.where(labels > -1)
    points_foliage0 = cur_p[idx_labels]

    x_min0, y_min0, z_min0 = points_foliage0.min(axis=0)
    print("min0:", x_min0, y_min0, z_min0)

    idx_labels00 = np.where(cur_p[:, 2] < z_min0)
    if (len(idx_labels00) > 0):
        points_trunk = np.vstack((points_trunk, cur_p[idx_labels00]))
        P = pd.DataFrame(points_trunk, columns=['X', 'Y', 'Z'])
        X = np.asarray(P)
        clustering = DBSCAN(eps=2 * EPS, min_samples=MINPTS).fit(X)
        labels = clustering.labels_
        idx_labels = np.where(labels < 0)
        points_no_trunk0 = points_trunk[idx_labels]
        idx_labels = np.where(labels > -1)
        points_trunk = points_trunk[idx_labels]

        print(points_no_trunk0.shape)

    idx_labels00 = np.where(cur_p[:, 2] >= z_min0)
    points_foliage = cur_p[idx_labels00]

    x_minF, y_minF, z_minF = points_foliage.min(axis=0)
    x_maxF, y_maxF, z_maxF = points_foliage.max(axis=0)
    print("minF:", x_minF, y_minF, z_minF)
    print("maxF:", x_maxF, y_maxF, z_maxF)
    crown_height = z_maxF - z_minF
    crown_height = toFixed(crown_height, 5)
    print('Crown height:', crown_height)

    if (points_branch.shape[0] > 250):
        pts = np.vstack((points_branch, points_foliage))
    else:
        pts = points_foliage
    if (points_no_trunk0.shape[0] > 250):
        points_foliage = np.vstack((points_foliage, points_no_trunk0))
    hull = ConvexHull(pts)
    crown_volume = hull.volume
    crown_volume = toFixed(crown_volume, 5)
    crown_square = hull.area
    crown_square = toFixed(crown_square, 5)

    fig = go.Figure(data=[
        go.Scatter3d(x=points_trunk[:, 0], y=points_trunk[:, 1], z=points_trunk[:, 2], mode='markers', name='Ствол',
                     marker=dict(
                         size=2,
                         color='#8B4513',
                         opacity=0.9
                     ))])
    fig.add_trace(
        go.Scatter3d(x=points_foliage[:, 0], y=points_foliage[:, 1], z=points_foliage[:, 2], mode='markers', name='Листва',
                   marker=dict(
                       size=2,
                       color='#93EE7D',
                       colorscale='Viridis',
                       opacity=0.9
                   )))
    fig.add_trace(
        go.Scatter3d(x=points_branch[:, 0], y=points_branch[:, 1], z=points_branch[:, 2], mode='markers', name='Ветки',
                     marker=dict(
                         size=2,
                         color='#266B00',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    fig.add_trace(
        go.Scatter3d(x=points_noise[:, 0], y=points_noise[:, 1], z=points_noise[:, 2], mode='markers', name='Шум',
                     marker=dict(
                         size=2,
                         color='#171717',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    fig.add_trace(
        go.Scatter3d(x=points_ground[:, 0], y=points_ground[:, 1], z=points_ground[:, 2], mode='markers', name='Земля',
                     marker=dict(
                         size=2,
                         color='#fcbc2d',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    fig.add_trace(
        go.Scatter3d(x=circle[:, 0], y=circle[:, 1], z=circle[:, 2], mode='lines', name='Сбег ствола',
                    marker=dict(
                        size=2,
                        color='#FD2000',
                        colorscale='Viridis',
                        opacity=0.9
                    )))
    fig.add_trace(go.Scatter3d(x=d_circle[:, 0], y=d_circle[:, 1], z=d_circle[:, 2], mode='markers', name='Диаметр',
                               marker=dict(
                                   size=6,
                                   color='#FD9000',
                                   colorscale='Viridis',
                                   opacity=1
                               )))
    fig.add_trace(go.Mesh3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], color='green', name='Объем кроны',
                            opacity=0.15, alphahull=0))
    fig.add_trace(go.Scatter3d(x=[x_max, x_max], y=[y_max, y_max], z=[z_minF, z_maxF], mode='lines', name='Высота',
                               marker=dict(
                                   size=6,
                                   color='#FF0080',
                                   opacity=1
                               )))
    fig.add_trace(go.Scatter3d(x=[x_min, x_min], y=[y_min, y_min], z=[z_min, z_max], mode='lines', name='Высота кроны',
                               marker=dict(
                                   size=6,
                                   color='#AD0680',
                                   opacity=1
                               )))
    fig.update_layout(scene_aspectmode='data')
    fig.update_layout(margin=dict(l=5, r=5, t=5, b=5))
    fig.update_layout(showlegend=False)
    camera = dict(
        up=dict(x=0, y=0, z=1),
        center=dict(x=0, y=0, z=0),
        eye=dict(x=0, y=3, z=0)
    )
    fig.update_layout(scene_camera=camera)
    fig.update_layout(template="plotly_white")

    graphs = []
    graphs.append(
        go.Scatter3d(x=points_trunk[:, 0], y=points_trunk[:, 1], z=points_trunk[:, 2], mode='markers', name='Ствол',
                     marker=dict(
                         size=2,
                         color='#8B4513',
                         opacity=0.9
                     )))
    graphs.append(go.Scatter3d(x=points_foliage[:, 0], y=points_foliage[:, 1], z=points_foliage[:, 2], mode='markers',
                               name='Листва',
                               marker=dict(
                                   size=2,
                                   color='#93EE7D',
                                   colorscale='Viridis',
                                   opacity=0.9
                               )))
    graphs.append(
        go.Scatter3d(x=points_branch[:, 0], y=points_branch[:, 1], z=points_branch[:, 2], mode='markers', name='Ветки',
                     marker=dict(
                         size=2,
                         color='#266B00',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    graphs.append(
        go.Scatter3d(x=points_noise[:, 0], y=points_noise[:, 1], z=points_noise[:, 2], mode='markers', name='Шум',
                     marker=dict(
                         size=2,
                         color='#171717',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    graphs.append(
        go.Scatter3d(x=points_ground[:, 0], y=points_ground[:, 1], z=points_ground[:, 2], mode='markers', name='Земля',
                     marker=dict(
                         size=2,
                         color='#fcbc2d',
                         colorscale='Viridis',
                         opacity=0.9
                     )))
    graphs.append(go.Scatter3d(x=circle[:, 0], y=circle[:, 1], z=circle[:, 2], mode='lines', name='Сбег ствола',
                               marker=dict(
                                   size=2,
                                   color='#FD2000',
                                   colorscale='Viridis',
                                   opacity=0.9
                               )))
    graphs.append(go.Scatter3d(x=d_circle[:, 0], y=d_circle[:, 1], z=d_circle[:, 2], mode='markers', name='Диаметр',
                               marker=dict(
                                   size=6,
                                   color='#FD9000',
                                   colorscale='Viridis',
                                   opacity=1
                               )))
    graphs.append(
        go.Scatter3d(x=d_circleH[:, 0], y=d_circleH[:, 1], z=d_circleH[:, 2], mode='markers', name='Диаметр Hyper',
                     marker=dict(
                         size=6,
                         color='#4B0082',
                         colorscale='Viridis',
                         opacity=1
                     )))
    graphs.append(go.Mesh3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], color='green', name='Объем кроны', showlegend=True,
                            opacity=0.15, alphahull=0))
    graphs.append(go.Scatter3d(x=[x_max, x_max], y=[y_max, y_max], z=[z_minF, z_maxF], mode='lines', name='Высота',
                               marker=dict(
                                   size=6,
                                   color='#FF0080',
                                   opacity=1
                               )))
    graphs.append(go.Scatter3d(x=[x_min, x_min], y=[y_min, y_min], z=[z_min, z_max], mode='lines', name='Высота кроны',
                               marker=dict(
                                   size=6,
                                   color='#AD0680',
                                   opacity=1
                               )))

    layout = dict(
        height=950,
        width=1000,
        template="plotly_white",
        scene_aspectmode='data',
        margin=dict(l=4, r=4, t=20, b=4),
        paper_bgcolor='#E4E9F7',
        legend={"title": "Части облака точек:"}
    )


    plot_div = plot({'data': graphs, 'layout': layout},
                    output_type='div')
    end = time.time()
    print('t:', end - start)

    return plot_div, fig, breast_diameter_tree, breast_diameter_tree_hyper, length_tree, height_tree, crown_height, crown_volume, crown_square


def createlog(request):
    if request.user.is_anonymous:
        return render(request, 'index.html')
    else:
        namelg = []
        diameterlg = []
        diameterhyperlg = []
        heightlg = []
        lengthlg = []
        crownheight = []
        crownvolume = []
        crownsquare = []
        namelg = np.asarray(namelg)
        diameterlg = np.asarray(diameterlg)
        diameterhyperlg = np.asarray(diameterhyperlg)
        heightlg = np.asarray(heightlg)
        lengthlg = np.asarray(lengthlg)
        crownheight = np.asarray(crownheight)
        crownvolume = np.asarray(crownvolume)
        crownsquare = np.asarray(crownsquare)
        for item in Tree.objects.all():
            namelg = np.append(namelg, item.name)
            diameterlg = np.append(diameterlg, item.tree_diameter)
            diameterhyperlg = np.append(diameterhyperlg, item.tree_diameter_hyper)
            heightlg = np.append(heightlg, item.tree_height)
            lengthlg = np.append(lengthlg, item.tree_length)
            crownheight = np.append(crownheight, item.tree_crown_height)
            crownvolume = np.append(crownvolume, item.tree_crown_volume)
            crownsquare = np.append(crownsquare, item.tree_crown_square)

        lt = False
        lt1 = True
        if Log.objects.exists():
            lastlog = Log.objects.last()
            lastlg = pd.read_excel(lastlog.logfile)
            lt = True
            lastlgdrop = lastlg.drop(columns=['Name'], axis=1)
            nplastlg = lastlgdrop.to_numpy()

        lg = pd.DataFrame(
            {"Name": namelg, "Diameter_LS, cm": diameterlg, "Diameter_HLS, cm": diameterhyperlg, "Height, m": heightlg,
             "Length, m": lengthlg, "Crown_height, m": crownheight, "Crown_volume, m2": crownvolume,
             "Crown_square, m3": crownsquare})
        current_datetime = str(date.today())
        t = time.localtime()
        current_datetime = current_datetime + '-' + str(time.strftime("%H-%M-%S", t))
        lgdrop = lg.drop(columns=['Name'], axis=1)
        nplg = lgdrop.to_numpy()

        if lt:
            if nplastlg.shape[0] == nplg.shape[0]:
                uniquelg = np.unique(np.isclose(nplastlg, nplg))
                if len(uniquelg) == 1:
                    lt1 = False
        if lt1:
            lg_path = os.path.join(settings.MEDIA_ROOT, "logs")
            lg_path = os.path.join(lg_path, f"{current_datetime}.xlsx")
            lg.to_excel(lg_path, index=False)
            log = Log()
            relative_path = f"logs/{current_datetime}.xlsx"
            log.logfile = relative_path
            log.log_name = current_datetime
            log.save()
        logs = Log.objects.all()
        context = {'logs': logs}
        return render(request, "download.html", context)


def index(request):
    return render(request, "index.html")


def upload(request):
    if request.user.is_anonymous:
        return render(request, 'index.html')
    else:
        message = 'Upload as many files as you want!'
        if request.method == 'POST':
            form = TreeForm(request.POST, request.FILES)
            if form.is_valid():
                form.save()
                return redirect('upload')
            else:
                message = 'The form is not valid. Fix the following error:'
        else:
            form = TreeForm()

        trees = Tree.objects.all()

        context = {'trees': trees, 'form': form, 'message': message}
        return render(request, 'upload.html', context)


class LoginUser(LoginView):
    form_class = LoginUserForm
    template_name = 'login.html'


def RegisterUser(request):
    if request.method == 'POST':
        form = RegisterUserForm(request.POST)
        if form.is_valid():
            form.save()
    else:
        form = RegisterUserForm()

    context = {'form': form}
    return render(request, 'login.html', context)


def login_or_reg(request):
    if 'password2' in request.POST:
        print("1")
        return RegisterUser(request)
    print("3")
    return LoginUser.as_view()(request)


def logout_view(request):
    logout(request)
    return render(request, 'login.html')


class TreeListView(ListView):
    model = Tree
    paginate_by = 24


class TreeDetailView(DetailView):
    model = Tree


def TreeList(request):
    # Load trees for the list page
    trees = Tree.objects.all()

    # Render list page with the trees and the form
    context = {'trees': trees}
    return render(request, 'tree_list.html', context)


def treedetail(request, id):
    if request.user.is_anonymous:
        return render(request, 'index.html')
    else:
        try:
            tree = Tree.objects.get(id=id)
            if request.method == 'POST' and request.is_ajax():
                print('сохранение')
                eps = request.POST.get("eps")
                if type(eps) is str:
                    edited_eps = eps.replace(",", ".")
                tree.tree_eps = float(edited_eps)
                tree.tree_minpts = request.POST.get("minpts")
                tree.tree_iter = request.POST.get("iter")
                znz = request.POST.get("onnormalsz")
                if znz == 'on':
                    tree.tree_onnormalsz = True
                else:
                    tree.tree_onnormalsz = False
                tree.save()
                print('сохранено!!!!', tree.tree_eps, tree.tree_minpts, tree.tree_iter)
            else:
                print('по умолчанию')

            return render(request, 'tree//tree_detail.html', context={'plot_div': tree.tree_plot,
                                                                      'D': tree.tree_diameter,
                                                                      'DH': tree.tree_diameter_hyper,
                                                                      'H': tree.tree_height, 'L': tree.tree_length,
                                                                      'CH': tree.tree_crown_height,
                                                                      'CV': tree.tree_crown_volume,
                                                                      'CS': tree.tree_crown_square, 'name': tree.name,
                                                                      'docfile_name': tree.docfile.name,
                                                                      'docfile_url': tree.docfile.url,
                                                                      'docfile_size': tree.docfile.size,
                                                                      "tree_id": tree.id,
                                                                      'eps': tree.tree_eps, 'minpts': tree.tree_minpts,
                                                                      'iter': tree.tree_iter,
                                                                      'onnormalsz': tree.tree_onnormalsz
                                                                      })

        except Tree.DoesNotExist:
            return HttpResponseNotFound("<h2>Tree not found</h2>")


import os
from django.conf import settings


def treeprocessing(request, id):
    if request.user.is_anonymous:
        return render(request, 'index.html')
    else:
        try:
            tree = Tree.objects.get(id=id)
            file = tree.docfile
            print(file.name)
            print("начат")
            print(id)
            plot_div, fig, breast_diameter_tree, breast_diameter_tree_hyper, length_tree, height_tree, crown_height, crown_volume, crown_square = processing(
                file.path, tree.tree_eps, tree.tree_minpts, tree.tree_iter, tree.tree_onnormalsz)
            img_path = os.path.join(settings.MEDIA_ROOT, "images")
            img_path = os.path.join(img_path, f"{id}.png")
            fig.write_image(img_path)
            relative_path = f"images/{id}.png"
            tree.tree_img = relative_path
            tree.tree_length, tree.tree_height, tree.tree_diameter, tree.tree_diameter_hyper, tree.tree_crown_height, tree.tree_crown_volume, tree.tree_crown_square = length_tree, height_tree, breast_diameter_tree, breast_diameter_tree_hyper, crown_height, crown_volume, crown_square
            tree.tree_plot = plot_div
            print("окончен")
            tree.save()

            return render(request, 'processing.html',
                          context={'plot_div': plot_div, 'D': breast_diameter_tree, 'DH': breast_diameter_tree_hyper,
                                   'H': height_tree, 'L': length_tree, 'CH': crown_height, 'CV': crown_volume,
                                   'CS': crown_square,
                                   'name': tree.name, "tree_id": tree.id,
                                   'docfile_name': tree.docfile.name, 'docfile_url': tree.docfile.url,
                                   'docfile_size': tree.docfile.size,
                                   'eps': tree.tree_eps, 'minpts': tree.tree_minpts, 'iter': tree.tree_iter,
                                   'onnormalsz': tree.tree_onnormalsz
                                   })
        except Tree.DoesNotExist:
            return HttpResponseNotFound("<h2>Дерево не найдено</h2>")
