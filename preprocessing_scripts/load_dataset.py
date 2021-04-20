"""
This script uses the image/.csv pairs created by using the modified ACRONYM tools to create one large dataset of
image / grasp pairs. This dataset is then saved as a .csv.
Author: olt13
"""

import os
import pandas as pd
from pandas.errors import EmptyDataError
import glob
import progressbar
from datetime import datetime


class LoadData:
    def __init__(self):
        self.grasp_loc = "/home/ollie/Documents/Major project/acronym data/dataset_grasps2/"
        self.grasp_files = glob.glob(os.path.join(self.grasp_loc, "*.csv"))
        self.columns = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'image']
        self.grasp_data = pd.DataFrame()

    def _load_grasps(self):
        dfs = []
        bar = progressbar.ProgressBar(max_value=len(self.grasp_files))
        for index, file in enumerate(self.grasp_files):
            try:
                temp = pd.read_csv(file, header=None)
                temp['image'] = file.split('/')[7].split('.')[0] + ".png"
                dfs.append(temp)
            except EmptyDataError:
                continue
            bar.update(index)
        self.grasp_data = pd.concat(dfs, axis=0, ignore_index=True)
        self.grasp_data.columns = self.columns

    def load(self):
        start = datetime.now()
        self._load_grasps()
        print("\nTime to load: " + str((datetime.now()) - start))
        self.grasp_data.to_csv('/home/ollie/Documents/Major project/acronym data/dataset.csv', index=False)
        return self.grasp_data


def main():
    ld = LoadData()
    grasps = ld.load()
    print(grasps)


if __name__ == "__main__":
    main()
