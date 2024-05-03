source ~/anaconda3/etc/profile.d/conda.sh
conda activate tf-gpu-cuda10
cd ~/csl_uav_simulator_ws	
source devel/setup.bash
rosrun img_seg_cnn vgg_unet_predict.py
