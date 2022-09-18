# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

**EKF implementation**

Following are the highlights of this Tracking Project : 


- Implements EKF (Extended Kalman Filter) algorithm which take into account Camera non-linear measurement model. 
- - Please review the F(), Q() , gamma() and S() implementations that mathematically provide a kalman filter functionality.
- Initially, EKF will track a single target with lidar measurement . RMSE is quite high as shown below :

![rmse_initial](https://user-images.githubusercontent.com/11416834/190886612-070ac73b-0316-44d4-8524-34aeb128dc0f.PNG)

**Track Management**

Track management process involves the following steps : 

- Intialize the track with the lidar measurement and a high position estimation error. 
- Based on the intitialization track score and state - either increase or reduce the score depending on the measurement returned by the sensor.
- Classify the tracks based on their track score as either initialized, tentative or confirmed.
- Also finally delete the tracks and update the meas list and track list.
The RMSE plot is below : 

![rmse_2](https://user-images.githubusercontent.com/11416834/190886827-f2f9b005-7014-4e61-832f-a14d30900fd5.PNG)

**Data Association**

- Here, we implement two important functions : Mahalanobis calculation and the Data Association Matrix based on Single Nearest Neighbour(SNN).
- Gating function can also help achieve disassociate highly unlikely measurments from the tracks.

The results from this step are below :
![capt_1](https://user-images.githubusercontent.com/11416834/190886910-5b388ffb-42b5-4e1b-955c-d342aa46a068.PNG)
![rmse_only_lidar](https://user-images.githubusercontent.com/11416834/190886912-ebb539d7-7bdc-4a9a-9f3c-f98387a439e1.PNG)

**Camera Fusion**

In the last step, we implement a nonlinear camera measurement model . We fuse the camera data and the lidar data to overcome the limitations w.r.t FOV of the lidar and also the additional sensor proves useful in a much efficient tracking model. 

Please review the tracking video under `results/my_tracking_results.avi`
The results from this step are below : 

![capt_final](https://user-images.githubusercontent.com/11416834/190887054-72b10c9d-9d38-4e95-bd8c-d4fe4a68b169.PNG)
![capt_final_gts](https://user-images.githubusercontent.com/11416834/190887077-c4522160-c162-4c22-952c-98f2b43cc5e6.PNG)

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

