"""
This script loads in the main dataset, which is then randomised and reduced by randomly selecting
no more than 100,000 images.
This is done so that there is not too much data for the CNN, but also not too many files uploaded to the SCW.
Tests are done to ensure that all the images quoted in the dataset are actually files
to avoid errors when training the model.
Author: olt13
"""

import pandas as pd
import os
import shutil
import progressbar
import random


def main():
    img_loc = '/home/ollie/Documents/Major project/acronym data/dataset_images2/'
    new_loc = '/home/ollie/Documents/Major project/scw1780/images/'
    df = pd.read_csv('/home/ollie/Documents/Major project/acronym data/dataset.csv')
    new_df = pd.DataFrame(columns=['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'image'])
    df_images = df['image'].values
    df_images = set(df_images)

    # Select from 100,000 images with grasps
    file_images = [f.name for f in os.scandir(img_loc)]
    random.shuffle(file_images)
    bar = progressbar.ProgressBar(max_value=len(file_images[:100000]))
    for i, f in enumerate(file_images[:100000]):
        if f in df_images:
            shutil.copy(img_loc + f, new_loc + f)
        bar.update(i + 1)
    bar.finish()

    # Generate new dataset from reduced images
    done_images = [f.name for f in os.scandir(new_loc)]
    done_images = set(done_images)
    bar = progressbar.ProgressBar(max_value=len(df.values))
    for index, row in df.iterrows():
        if df.at[index, 'image'] in done_images:
            new_df.loc[index] = row
        bar.update(index+1)
    bar.finish()

    print(len(new_df['x'].values))
    new_df.to_csv('/home/ollie/Documents/Major project/scw1780/new_dataset.csv', index=False)

    # Test for missing images
    print(len(df['image']))
    bar = progressbar.ProgressBar(max_value=len(df_images))
    done_images = [f.name for f in os.scandir(new_loc)]
    done_images = set(done_images)
    index = 0
    for img in df_images:
        bar.update(index + 1)
        index += 1
        # print(index)
        if img in done_images:
            continue
        else:
            print("ERROR")


if __name__ == '__main__':
    main()
