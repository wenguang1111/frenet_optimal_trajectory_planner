## Wenguang Onboarding

### Setting up the venv
I use __poetry__ as a package manager for this repo. You should have __poetry__ installed. Run the following commands to install, create the venv and install dependencies:

<pre>
cd /to/your/path/frenet_optimal_trajectory_planner/CVAE
pip install poetry==2.1.4
poetry install
</pre>

### Activating the venv
To activate the venv, run:
<pre>
poetry env activate
</pre>
then it will paste a line like:
<pre>
source /dss/dsshome1/07/di97xub/.cache/pypoetry/virtualenvs/cvae-3j1Yb4w3-py3.10/bin/activate
</pre>
copy it and run it and now you're in the virtual environment.

### Directory Structure
- __commonroad-reactive-planner/__ : My fork of CommonRoad Reactive Planner with CVAE integration. This a submodule that I update if I make commits there.
    - __commonroad_rp/__ : The main code of CommonRoad Reactive Planner.
    - __run_planner.py__ : Script to run the reactive planner.
    - __output/__ : Folder where the planner outputs results (evaluation csv, gifs). If _EVAL_ is set to True in the __run_planner.py__, it will output evaluation results here. If _save_plots_ is set to True in config files, it will save gifs of the planned scenarios here (this takes more time). This is not tracked in git. You have to create it and mkdir cvae/ and rp/ in __output__ to save the results in the respective folder it you're running CVAE or RP.
- __cvae/__ : CVAE model code and training scripts.
    - __all_scenarios/__ : Folder that contains all the CommonRoad scenarios. This is not tracked in git. You have to create it and put the scenarios there (from Zenodo as discussed below).
        - __planning_scenarios__ : Folder that contains the scenarios that RP planned and are part of our dataset.
    - __cvae/model/__ : CVAE model architecture and training scripts code.
        - __weights/__ : Folder to save model weights. The weights in cvae config file are the latest/best weights I have trained.
        - __runs/__ : Folder to save training runs and logs. Not tracked in git. You have to create it if you want to train the model.
    - __cvae/sbatch/__ : To submit a job to slurm you use sbatch scripts.
        - __training_job_script.sh__ : Script to train the CVAE model on the cluster.
        - __run_planner_script_cvae.sh__: Script to run the reactive planner __with__ CVAE on the cluster. If _--cvae_ flag is set, it will use CVAE. Otherwise, it will use the original reactive planner. The flag _--mode_ determines whether you run the planner on 'train', 'val', or 'test' sets.
        - __run_planner_script_rp.sh__ : Script to run the reactive planner __without__ CVAE on the cluster.
    - __config/__ : Config files for cvae planner and rp planner.
    - __utils/__ : Utility functions like beta annealing and preparing the dataset.
    - __data/__ : Folder to save the preprocessed dataset. Not tracked in git. You have to create it and download data in it (data_v2).
        - __data_v2/__ : Preprocessed dataset folder. You have to download it from Zenodo (discussed later).

### Downloading the dataset
The preprocessed dataset (17.5 GB) is available on Zenodo at this [link](https://zenodo.org/records/17851245?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6ImZjZWJlNzExLTU2NGUtNDA5MC05YzY1LTdhYzVhN2NlN2EwMiIsImRhdGEiOnt9LCJyYW5kb20iOiJhNmE4MmNhZDMyOTk4NmJjYTdlMjU4NTJiNGQzZjIzYiJ9.1916XCRYQCsoaulhaUI16sx2s-K7GlroRAXbBMEF9tKGz8268JMVb9bJ7HwSYlx1vXeeoDQKBVQ314cOOgZ21Q). Download the dataset, extract it, and paste __data_v2/__ in __cvae/data/__ folder so that the path to the data becomes __cvae/data/data_v2/__. Paste __all_scenarios/__ in __cvae/__ folder. _fiss_scenarios.tar.gz_ contains the scenarios for FISS (explained in the meeting).

I recommend creating an account on Zenodo for me to add you as a collaborator so that you can download the dataset easily in the future if I upload a new version.

For later reference, I used this repo to upload the dataset to Zenodo: https://github.com/jhpoelen/zenodo-upload

__Note__: Their are hardcoded paths in many scripts, as I prefer global paths to avoid python path issues. We can change them later.

## Running the planner on 1 scenario

To run the planner on 1 scenario:

<pre>
# pull the repo again
git pull

# make a directory that has all the imgs from train, val, test sets
cd cvae/data/data_v2/
mkdir -p all/imgs

# copy all imgs into all/imgs
cp -r train/imgs/* all/imgs/
cp -r val/imgs/* all/imgs/
cp -r test/imgs/* all/imgs/

# the command to run the planner on 1 scenario
cd CVAE/commonroad-reactive-planner/
python3 run_planner.py --cvae --scenario ARG_Carcarana-4_7_T-1.xml
</pre>

## Comments on latest experiments
- Data consists of the samples (x) that generated the optimal trajectory for a certain timestep and the conditions (c) which are the initial/goal states and the image of the scenario at each timestep. If a scenario has 100 timesteps, then it will have the same initial/goal state duplicated 100 times, and a 512 feature vector for the image at each timestep that was pre-encoded with frozen ResNet-18. 

- Model is an encoder of 2 hidden layers of 512 neurons and ReLU activations. Plus two other layers that map from 512 to z dimension which is 16, one layer outputs the mean and the other the log of the variance.

- Decoder is one hidden layer that maps from c dimension (6+512) + z (16) to 512 neurons. And an output layer maps 512 to dimension of x (3).

- It was trained for 20 epochs, with batch size of 128, and learning rate 1e-4. optimizer is Adam with weight decay of 1e-5. 

- CVAE loss is two components, reconstruction loss (how close are the samples ) + KL divergence (that measures the similarity between two probability distributions, approximate posterior q(z/x,c) and the actual posterior p(z/x,c) which is not known but approximated to be a normal distribution N~(0, 1)).

- Validation/Test losses might be deceiving. During training the validation loss was reasonable and did not indicate overfitting and during testing the batch (128 samples) loss was usually on the order of 0.005 (roughly). That indicated good samples reconstruction abilities on data never seen before.

- When testing with CommonRoad frenet all scenarios fail due to either kinematic feasibility of the trajectory or collisions.

- However, 3/~800 scenarios were planned successfully which shows potential for improvement.

## Problems/Recommendations for current setup

- The KL divergence term is in practice weighted by a tunable parameter beta which in my initial training was 1e-4. When printing the KL divergence term before weighting it is big (~27) and beta was too small that makes it has no contribution to the loss.

- The previous point might indicate posterior collapse (posterior becomes the prior q(z/x,c) = p(z) = N(0, 1) and z will carry no information about x therefore the decoder will reconstruct x only from c).

- The dataset included only the set of samples (t, d, v) that led to the optimal trajectory. Adding also other feasible samples for each timestep might be more helpful.

- The condition does not have any temporal information. Maybe conditioning on the timestep might be useful.

- The design choices for the number of layers, neurons, and latent space dimentionality is also a question mark but I cannot be sure that something is wrong with it.

- Maybe incorporate physics-informed losses. Borrow the feasibility checks from frenet and check it while training and penalize if samples produce infeasible trajectories. This would need generating trajectories for each reconstructed batch during training and check feasibility and collisions. Very expensive!

## Comparing Commonroad and CVAE samples

#### CommonRoad Reactive Planner Trajectory (Scenario __DEU_Lohmar-53_1_T-1__)
![CommonRoad RP Trajectory](https://raw.githubusercontent.com/kareem4200/commonroad-reactive-planner/refs/heads/cvae-dev/gifs/DEU_Lohmar-53_1_T-1_rp.gif)

#### CVAE Trajectory (Scenario __DEU_Lohmar-53_1_T-1__)
![CVAE Trajectory](https://raw.githubusercontent.com/kareem4200/commonroad-reactive-planner/refs/heads/cvae-dev/gifs/DEU_Lohmar-53_1_T-1_cvae.gif)

- CommonRoad RP sampler took ~58.66 seconds to plan this scenario (98 timesteps), while CVAE took ~65.433 seconds.

- Average time to encode 1 scenario image with ResNet-18 was 0.18 seconds. For 98 timesteps, time taken to encode images was ~17.6 seconds out of the 65.433 seconds.

- CommonRoad RP had 5, 10, 6 samples for lon_v, t, d respectively. Which makes it 5x10x6=300 trajectories to try.

- CVAE outputs 3 samples for each variable which makes possibilities are 3x3x3=27 trajectories.

- By intuition, time reduction in planning should be much more since number of trajectories to try in reduced from 300 to 27. But it is not the case?

## Track experiments

- Track the performance on 142 test scenarios.
- First get as much scenarios planned with CVAE as possible (reliability). Set the samples to 256 as an upper bound. Hierarchial sampling should be implemented later (robustness).

### Experiment 1

#### Setup

- LR = 1e-4, Batch size = 1024, z_dim = 64, epochs = 5.
- Cosine beta annealing from 0 to 0.3 over the training steps.
- Trainable resnet18 as img feature extractor.

#### Results

- Solved 86/142 test scenarios.

### Experiment 2

#### Setup

- z_dim = 32

#### Results

- Solved 88/142 test scenarios

### Experiment 3

#### Setup

- No image masking

#### Results

- Solved 90/142 test scenarios

### Experiment 4

#### Setup

- Image size 256 instead of 128

#### Results

- Solved 89/142 test scenarios

### Experiment 5

#### Setup

- Image size 128 again
- z_dim = 32
- No image masking
- end kl_beta = 0.1 cosine annealing from 0.0 over 5 epochs

#### Results

- Solved 104/142 test scenarios
- Apparently there was a posterior collapse

### Experiment 6

#### Setup

- Image size 128 again
- z_dim = 32
- No image masking
- end kl_beta = 0.1 cosine annealing from 0.0 over 3 epochs

#### Results

- Solved 99/142 test scenarios

### Experiment 7

#### Setup

- Image size 128 again
- z_dim = 32
- No image masking
- end kl_beta = 0.1 cosine annealing from 0.0 over 5 epochs
- Sample 512 samples

#### Results

- Solved 98/142 test scenarios
- Next, experiments with the model

### Experiment 8

#### Setup

- Image size 128 again
- z_dim = 32
- No image masking
- end kl_beta = 0.1 cosine annealing from 0.0 over 10 epochs
- Sample 256 samples

#### Results

- Solved 95/142 test scenarios

### Experiment 9

#### Setup

- Image size 128 again
- z_dim = 32
- No image masking
- end kl_beta = 0.05 cosine annealing from 0.0 over 5 epochs
- Sample 256 samples

#### Results

- Solved 102/142 test scenarios
