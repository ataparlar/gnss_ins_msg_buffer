### Description 
This package converts GNSS-INS data (lattitude,longitude,altitude) to cartesian coordinate system and publishes the pose (pose_with_covariance_stamped).

### How it works
The first incoming location information is set as the map frame. The next incoming location information is published according to the map frame. Using tf2, map frame and new pose frame are published. 

### Usage

* `ros2 run gnss_ins_cartesian_converter_nodes gnss_ins_cartesian_converter_nodes`

#### Subscribed Topics
* /lvx_client/gsof/ins_solution_49 (applanix_msgs::msg::NavigationSolutionGsof49)
* /lvx_client/gsof/ins_solution_rms_50 (applanix_msgs::msg::NavigationPerformanceGsof50)

#### Published Topics
* /pose_with_covariance_stamped (geometry_msgs::msg::PoseWithCovarianceStamped)

### External Dependencies
* GeographicLib
* applanix_msgs
* Eigen3
