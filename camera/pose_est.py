import numpy as np
from sklearn.cluster import KMeans, DBSCAN

target_dims = {'Balls':0.0342*2}

def target_pose_est(self, int_matrix, detection, rob_pose):
        # get information
        focal_length = int_matrix[0][0]
        target_class = detection[0]
        target_box = detection[1]       # get bounding box measures: [x,y,width,height]
        true_height = target_dims[target_class][2]

        # compute target pose based on pixel dims
        pix_h = target_box[3]
        pix_c = target_box[0]
        distance = true_height/pix_h * focal_length

        x_shift = 320/2 - pix_c
        theta = np.arctan(x_shift/focal_length)
        ang = theta + rob_pose[2]

        # relative object location
        distance_obj = distance/np.cos(theta)
        x_relative = distance_obj * np.cos(theta)
        y_relative = distance_obj * np.sin(theta)
        relative_pose = {'x': x_relative, 'y': y_relative}


        # location of object in the world frame using rotation matrix
        delta_x_world = x_relative * np.cos(ang) - y_relative * np.sin(ang)
        delta_y_world = x_relative * np.sin(ang) + y_relative * np.cos(ang)
        # add robot pose with delta target pose
        target_pose = {'y': (rob_pose[1]+delta_y_world)[0],
                        'x': (rob_pose[0]+delta_x_world)[0]}

        return target_pose

def merge_ests(target_dict):
        target_est = {}
        targets_keys = [key for key in target_dict.keys()]
        points = [[point['x'], point['y']] for point in targets_keys.values()]
        if len(points) == 0: return

        # finding and discarding outliers
        dbscan = DBSCAN(eps=0.2, min_samples=3)
        labels = dbscan.fit_predict(points)

        j = 0
        for i in np.where(labels==1)[0]:
            points.pop(i-j)
            j += 1


        return target_est

def calc_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)