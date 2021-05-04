"""
This script makes watertight versions of the .obj files so that the ACRONYM scripts
can make use of them. It makes use of the Manifold library suggested on the ACRONYM page.
Author: olt13
"""

import os
import subprocess

m_loc = "/home/ollie/Documents/Major project/acronym data/meshes/"
new = "/home/ollie/Documents/Major project/acronym data/new_meshes/meshes/"
os.chdir(m_loc)
files = [f.name for f in os.scandir(os.getcwd()) if str(f.name).split('.')[1] == 'obj']

os.chdir(new)
done_files = [f.name for f in os.scandir(os.getcwd()) if str(f.name).split('.')[1] == 'obj']

remaining = [f for f in files if f not in done_files]

m_cmd = "./manifold  '{}' temp.obj -s"
s_cmd = "./simplify -i temp.obj -o '{}' -m -r 0.02"


def cmds(file):
    try:
        subprocess.run(m_cmd.format(m_loc + file), shell=True, timeout=150)
        subprocess.run(s_cmd.format(new + file), shell=True, timeout=150)
    except subprocess.TimeoutExpired:
        print("TIMEOUT")
    except KeyboardInterrupt:
        return


os.chdir("/home/ollie/Documents/Major project/Manifold/build")
index = 0
num = len(remaining)

for name in remaining:
    index += 1
    cmds(name)
    print(index, "/", num, "percentage complete: ", str(index * 100 / num) + "%")
