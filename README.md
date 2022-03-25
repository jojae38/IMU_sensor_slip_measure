# IMU_sensor_slip_measure

Sensor_imu와 같이 실행해야 합니다. ROBOT_pose (orientation) 와 IMU_sensor(orientation) callback 이 모두 들어오는 시점에서 실행됩니다.

1. callback 이 모두 들어온 경우 1초간 양쪽의 orientation 값을 이용해 차를 구한 후 해당 값을 0으로 맞춰줍니다.
![image](https://user-images.githubusercontent.com/58541374/160034868-fa3bb3fe-fa37-4f09-8b36-ed8616f15fd5.png)

IMU와 로봇 간의 각도 차이가 1.943정도 나오기에 제거해주는 모습

2. geometry_msgs::Twist 를 이용해 /acc_diff 토픽명으로 publish 를 진행하며 다음과 같이 나타냅니다.
* [angular.x = dθ] 
* [angular.y = dθ'] 
* [angular.z = dθ"] 
3. rqt_plot을 실행 후 y 축을 적절히 조정하면 로봇과 IMU 센서 간의 각도차이 및 각도 변화량이 측정 가능합니다.
![image](https://user-images.githubusercontent.com/58541374/160034918-8005b8ba-b8f8-4acd-b125-33e26b54124e.png)


#주의사항

* 30HZ이기에 1/30초 변화량을 생각하면 dθ에서 dθ'로 변환 시 30을 곱해야 하지만 값이 너무 크게 변하여 관측이 어렵기에 곱을 하지 않았습니다.
* launch 파일 내 변화량이 얼마정도 나오면 슬립으로 인식할 것인지 슬립 민감도를 설정하는 파라미터가 있습니다. -> 현재 0.5
* 현재 IMU drift 현상 (가만히 놔 둬도 각도가 찔끔찔끔 변하는 현상) 이 10초에 (+-) 0.0002~0.0003 정도 관측되고 있습니다.

처음 실행시

![image](https://user-images.githubusercontent.com/58541374/160035016-56a5da68-631f-49f8-87fa-3455789f9c81.png)

움직이지 않았는데 20초 후 변화한 각도

![image](https://user-images.githubusercontent.com/58541374/160035045-2b010514-8ba8-4992-8e84-7172cca5cd06.png)
