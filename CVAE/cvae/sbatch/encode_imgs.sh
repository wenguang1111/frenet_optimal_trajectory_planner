#!/bin/bash

#SBATCH -J encode_imgs
#SBATCH -o cvae/log/log_2.out
#SBATCH -e cvae/log/errors_2.err
#SBATCH -D /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE
#SBATCH --clusters=hpda2
#SBATCH --partition=hpda2_compute_gpu
#SBATCH --cpus-per-task=2
#SBATCH --gres=gpu:1
#SBATCH --mem=64gb
#SBATCH --time=24:00:00
#SBATCH --mail-type=none
#SBATCH --mail-user=karem.mohamed@dlr.de

module load slurm_setup
module load python

# source poetry env
source /dss/dsshome1/07/di97xub/.cache/pypoetry/virtualenvs/cvae-3j1Yb4w3-py3.10/bin/activate

cd /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE/cvae/utils

python3 encode_img_parquet.py
