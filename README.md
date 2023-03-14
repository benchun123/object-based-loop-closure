# object-based-loop-closure

**Related Paper:**  

+ Benchun Zhou, Yongqi Meng, Furmans Kai. **Object-based Loop Closure with Directional Histogram Descriptor**[C]//2022 IEEE 18th International Conference on Automation Science and Engineering (CASE). IEEE, 2022.: 1346-1351. [[**PDF**](https://ieeexplore.ieee.org/abstract/document/9926687)] [[**Video**]()]

+ If you use the code in your academic work, please cite the above paper. 


## 1. Prerequisites 
* Ubuntu (18.04.5)
* CMake (3.10.2)
* Eigen (3)
* OpenCV (3.2.0)
* PCL (1.8.1)
* Note: the code builds on the top of [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM2), if you meet compile problems, please refer to [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM2).

## 2. Running 
Clone the repository and catkin_make:
```
    git clone https://github.com/benchun123/object-based-loop-closure.git
    cd object-based-loop-closure
    sh build.sh
```

Launch it as follows:
```
./mono_ustc ~/path/to/dataset

```
## 3. Dataset 

the open-source code runs on a [USTC Sequence](https://bwsyncandshare.kit.edu/s/7keQF3zowPsdajf) sequences of [USTC RGB-D Dataset](https://rec.ustc.edu.cn/share/bd180320-bc6d-11eb-9675-6d646d5cf84a)(password:epya), we only unzip 2020-11-13-15-41-05, other six sequence are not explored. 


## 4. Note 
The main contribution of the code is to add a function **DetectObjectLoop()** in "LoopClosing.cc" to detect object loop, once the loop is detected, we use **CorrectObjectLoop()** to close the loop. 

please check "LoopClosing.h" and "LocalMapping.h" for more details. 

## 5. Acknowledgement 

Thanks for the great work: [**Topology Loop Closure**](https://ieeexplore.ieee.org/document/9484819), [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM2), [**Cube SLAM**](https://github.com/shichaoy/cube_slam), and [**EAO-SLAM**](https://github.com/yanmin-wu/EAO-SLAM)

+ Lin, S., Wang, J., et al. **Topology aware object-level semantic mapping towards more robust loop closure**[J]. IEEE Robotics and Automation Letters, 2021: 7041-7048. [PDF](https://ieeexplore.ieee.org/document/9484819). 
+ Mur-Artal R, Tardós J D. **Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras**[J]. IEEE Transactions on Robotics, 2017, 33(5): 1255-1262. [PDF](https://arxiv.org/abs/1610.06475), [Code](https://github.com/raulmur/ORB_SLAM2)
+ Yang S, Scherer S. **Cubeslam: Monocular 3-d object slam**[J]. IEEE Transactions on Robotics, 2019, 35(4): 925-938. [PDF](https://arxiv.org/abs/1806.00557), [Code](https://github.com/shichaoy/cube_slam)
+ Wu, Y., Zhang, Y., et al. **Eao-slam: Monocular semi-dense object slam based on ensemble data association**[C].//In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 4966-4973. IEEE, 2020. [[**PDF**](https://ieeexplore.ieee.org/abstract/document/9341757)] [[**Code**](https://github.com/yanmin-wu/EAO-SLAM)]

## 6. Description
<div align=center><img src="./README_Picture/loop_correction.png"/></div>