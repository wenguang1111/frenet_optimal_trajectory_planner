#!/bin/bash

#SBATCH -J run_cvae
#SBATCH -o cvae/log/log_cvae.out
#SBATCH -e cvae/log/errors_cvae.err
#SBATCH -D /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE
#SBATCH --clusters=hpda2
#SBATCH --partition=hpda2_compute
#SBATCH --cpus-per-task=2

#SBATCH --mem=64gb
#SBATCH --time=24:00:00
#SBATCH --mail-type=none
#SBATCH --mail-user=karem.mohamed@dlr.de

# load python3.10
module load slurm_setup
module load python

# source poetry env
source /dss/dsshome1/07/di97xub/.cache/pypoetry/virtualenvs/cvae-3j1Yb4w3-py3.10/bin/activate

cd /dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE/commonroad-reactive-planner

python3 run_planner.py --cvae --mode test
