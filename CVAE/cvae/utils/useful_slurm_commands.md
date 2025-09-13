## Useful Slurm Commands

__ssh to login node__

ssh di97xub@login.terrabyte.lrz.de

__Allocate resouces in interactive shell__

salloc --cluster=hpda2 --partition=hpda2_testgpu --nodes=1 --ntasks-per-node=1 --gres=gpu:1 --time=00:10:00

This will open the interactive shell

__See how many nodes are free__

sinfo

scontrol

__See which jobs you are running__

squeue

__Monitor cluster resources__

module use /dss/dsstbyfs01/pn56su/pn56su-dss-0020/usr/share modules/files/

module load check_cluster
 
check_cluster

__OR__

cluster-smi

cluster-smi -u di97xub

cluster-smi -p -t _(more detailed)_

cluster-smi -n hpdar01c05s02 _(particular node)_

__Transfer files / folders to the login node__

rsync -r {src_path} di97xub@login.terrabyte.lrz.de:{dest_path}

__Load python3.10 instead of 3.6__

module load python

__Search for jobs and cancel__

squeue -u di97xub

scancel {JOB_ID}

__Send a job__

sbatch {SLURM_SCRIPT.sh}

__Check storage__

dssusrinfo all

__Get Tensorboard on local machine__

On the login node run: tensorboard --logdir {runs_dir} --host 0.0.0.0 --port 6006

On local machine ssh to cluster: ssh -L 6006:localhost:6006 di97xub@login.terrabyte.lrz.de

Then tensorboard will be available at: http://localhost:6006

