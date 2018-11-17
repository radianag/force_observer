import numpy as np
import math as m
import csv
import rospkg

def save_csv_data(folder, name, data):
    rospack = rospkg.RosPack()
    path = rospack.get_path('force_observer')
    with open(path + '/' + folder + name + '.csv', 'w') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(np.size(data, 0)):
            wr.writerow(data[i])

def modified_dh(_dh_alpha, _dh_a, _dh_d, _dh_theta):
    transform = np.matrix([[m.cos(_dh_theta), -m.sin(_dh_theta), 0, _dh_a],
    [m.sin(_dh_theta) * m.cos(_dh_alpha), m.cos(_dh_theta) * m.cos(_dh_alpha), -m.sin(_dh_alpha), -m.sin(_dh_alpha) * _dh_d],
    [m.sin(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_theta) * m.sin(_dh_alpha), m.cos(_dh_alpha), m.cos(_dh_alpha) * _dh_d],
    [0, 0, 0, 1]])

    return transform