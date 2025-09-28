#!/bin/bash

#SBATCH -J cvae_training
#SBATCH -o cvae/log/log.out
#SBATCH -e cvae/log/errors.err
#SBATCH -D /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE
#SBATCH --clusters=hpda2
#SBATCH --partition=hpda2_compute_gpu
#SBATCH --cpus-per-task=1
#SBATCH --gres=gpu:1
#SBATCH --mem=64gb
#SBATCH --time=01:00:00
#SBATCH --mail-type=none
#SBATCH --mail-user=karem.mohamed@dlr.de

module load slurm_setup
module load python

# source poetry env
source /dss/dsshome1/07/di97xub/.cache/pypoetry/virtualenvs/cvae-3j1Yb4w3-py3.10/bin/activate

cd /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE

python3 cvae/model/inference.py
