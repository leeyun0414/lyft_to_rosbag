#matplotlib inline
from nuscenes.nuscenes import NuScenes

# Load the dataset
# Adjust the dataroot parameter below to point to your local dataset path.
# Note that using "~" for your home directory typically won't work here, thus specify the complete pathname.
# The correct dataset path contains at least the following four folders (or similar): images, lidar, maps, v1.0-mini
# In case you didn't download the 'v1.0-mini' version of the dataset, also adjust the version parameter below.
level5data = NuScenes(version='v1.01-train', dataroot='/home/ee904-pc4/dataset/v1.01-train', verbose=True)
my_scene = level5data.scene[0]
my_sample_token = my_scene["first_sample_token"]
level5data.render_sample(my_sample_token)
