# Tuni Metrics Competition

The repository presents the task completed for a robotics competition - [1st ADAPT FIELD CAMPAIGN](https://metricsproject.eu/agile-production/1st-field-campaign/)

Update:

Our team from [Cognitive Robotics](https://research.tuni.fi/cogrob/) research group at Tampere university won the 2nd place for the Video track and the Open-source award for the robotics competition


### Results

- The video for the complete assembly task :
  
[![Fidget Assembly Task](http://img.youtube.com/vi/YQirOBqAwiE/maxresdefault.jpg)](https://www.youtube.com/watch?v=YQirOBqAwiE&ab_channel=KulunuOsanda "Frank Emika Panda Robot with Realsense d435")

- The rosbags can be downloaded [here](https://tuni-my.sharepoint.com/:u:/g/personal/kulunu_samarawickrama_tuni_fi/EVfPOJ6b3a1JiUHJhAJ59TEB1Et6zy6F_NeNTQw7rIN8ww?e=dQjQjU)


### The structure of the repo

```

├── CAD_Models
├── .stl files
└── .ply files
├── Readme.md
│    ├── link to download dataset
│    ├── result videos
│    └── link to download pre-trained models
├── metrics_bringup
    └── starts everything
├── metrics_msgs
    └── defines all messages for the project
├── metrics_control
│   └── ros controllers/path planning etc
└── metrics_vision
    ├── detection
    ├── segmentation
    ├── pose estimation
    └── grasp detection
```

