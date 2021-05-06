#!/bin/bash --login

###

#SBATCH --partition=gpu
#SBATCH --gres=gpu:2

#SBATCH --job-name=TRAINING

#SBATCH --output=grasp.out.%J

#SBATCH --error=grasp.err.%J

#maximum job time in D-HH:MM
#SBATCH --time=1-00:00

#SBATCH --ntasks=1

#SBATCH --mem-per-cpu=8000

#SBATCH --ntasks-per-node=1

#SBATCH -A scw1780

###

#now run normal batch commands 

module load anaconda/3
conda activate ../.conda/envs/cenv

python3 model_training.py
