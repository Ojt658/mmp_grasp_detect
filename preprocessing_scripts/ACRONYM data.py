"""
This script is used to create the .sdf files so that the models can be used in Gazebo
for simulation. It makes use of multi-threading to speed up the process.
Author: olt13
"""

import h5py
import os
import shutil
from multiprocessing.dummy import Pool

os.chdir("/home/ollie/mmp_ws/src/mmp_grasp_detect/models/")

tf = open("template_sdf.txt", 'r')
template_sdf = tf.read()
tf.close()

tf = open("template_config.txt", 'r')
template_config = tf.read()
tf.close()

grasp_location = "/home/ollie/Documents/Major project/acronym data/grasps/"
mesh_location = "/home/ollie/Documents/Major project/acronym data/meshes/"
new_loc = "/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3/"

models = []


def thread(file):
    grasp_file = h5py.File(grasp_location + file, 'r')
    mesh_file = grasp_file['object/file'][()]
    # print(mesh_file)
    obj = mesh_file.split('/')[1]
    mesh_name = mesh_file.split('/')[2].split('.')[0]

    name = ""
    if obj not in models:
        name = obj
        os.mkdir(new_loc + name)
    else:
        i = 2
        while True:
            name = obj + "_" + str(i)
            i += 1
            if not os.path.exists(new_loc + name):
                try:
                    os.mkdir(new_loc + name)
                    break
                except FileExistsError:
                    continue

    models.append(name)
    os.mkdir(new_loc + name + "/meshes")

    mass = grasp_file['object/mass'][()]
    scale = grasp_file['object/scale'][()]
    # com = grasp_file['object/com'][()]

    with open(new_loc + name + "/model.sdf", "w") as f:
        f.write(template_sdf.format(m=mass, s=scale, v=1, xx=1, xy=1, xz=1, yy=1, yz=1, zz=1, n=name, mesh=mesh_name+".obj"))

    with open(new_loc + name + "/model.config", "w") as f:
        f.write(template_config.format(n=name))

    shutil.copy(mesh_location + mesh_name + ".obj", new_loc + name + "/meshes")


def main():
    files = [f.name for f in os.scandir(grasp_location)]

    pool = Pool(12)
    pool.map(thread, files)


if __name__ == '__main__':
    main()
