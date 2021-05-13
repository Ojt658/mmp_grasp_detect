Major Project: Grasp Detection
This is the catkin package in which the ROS code is stored. This is used to run the experimnets on the models.

Two example models have been proved in the models folder. They are:  and

This package also includes four exta folders:

    *  data_subset

        Includes a subset of the dataset used to train the CNN.

    *  preprocessing_scripts

        Includes the scripts used to build the dataset:

           * ACRONYM data.py
           * ACRONYM OBJ WATERTIGHT FILES.py
           * load_dataset.py
           * rationalise_images.py
           * ACRONYM_SCRIPTS/scripts/generate_dataset.py
           * ACRONYM_SCRIPTS/scripts/save_grasps.py

        Also inculdes the modified acronym tools package.

    *  tensorflow_models

        Includes the scripts used to build and train the CNNs. Also the SBatch file used to run on SCW.

    *  results

        Includes the cleaned up results tables form the experiments and the visualisations (with the Python script).

There are also the files used by the git service.

The main code for running the experiments is in the src and the scripts folders.

Some scripts require external libraries outside of ROS. These include: Tensorflow 2 (CPU or GPU), Numpy, and Pandas. These can be downloaded using pip.

To run the main CNN experiment:

  roscore
  roslaunch mmp_grasp_detect mmp.launch
  roslaunch mmp_grasp_detect get_model_success.launch

To run the ground truth experiment:

  roscore
  roslaunch mmp_grasp_detect mmp.launch
  roslaunch mmp_grasp_detect get_ground_success.launch

