import numpy as np
import scipy.io


force_array = [1,2,3]
pos_array = [1,2,3]
dataz = np.array(force_array)
dataz.reshape(3,1)
pos = np.array(pos_array)
pos.reshape(3,1)

file_path1 = 'force_z.mat'
file_path2 = 'pos.mat'

scipy.io.savemat(file_path1, {'data': dataz})
scipy.io.savemat(file_path2, {'data': pos})