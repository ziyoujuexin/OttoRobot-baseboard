## Otto Robot - Baseboard Project

这个工程中包含的是Otto机器人的底层控制板。
主要用于接受来自上位机ESP32S3（小智）的串口信号，进一步控制舵机、屏幕，完成机器人的动作与表情。

### 工程环境

在ESP-IDF v5.5.0下测试良好，在5.5.1下Sdkconfig需要进行小幅度的修改，修复启动时容易重启以及屏幕显示问题。

### 硬件配置

使用微雪ESP32P4-WIFI6板卡。主处理器为ESP32P4，其中WIFI6是附带的网卡。在调试阶段，其会启动一个Web Server，在其中可以利用网页调试机器人的动作、表情。

#### 舵机驱动

共计14个舵机。使用16舵机的拓展版PCA9685。其使用I2C控制，每一个Channel对应一个舵机，具体到机器人的部位上，参考下表。**这里需要更新，范围不一定是对的**

| Part Name      | Channel (L/R) | Angle Range        | Notes                                      |
|----------------|---------------|--------------------|--------------------------------------------|
| Ear (Front/Back) | 1 / 3         | 0-180              | Right ear angle needs adjustment.          |
| Ear (Up/Down)  | 0 / 2         | 0-180              | Left ear is completely stuck (stalling).   |
| Head (Up/Down) | 4             | 80-140             | -                                          |
| Head (Left/Right)| 5             | 0-180              | -                                          |
| Arm (Front/Back) | 6 / 8         | 0-180              | Left arm is reversed. Right arm gets stuck.|
| Arm (Up/Down)  | 7 / 9         | 80-120 / 60-100    | Left arm is reversed. Note opposite limits.|
| Leg (Rotation) | 10 / 12       | 60-150 / 30-120    | Feet are parallel when angles are complementary. |
| Ankle (Lift)   | 11 / 13       | 50-120             | Center of gravity foot ~100, lifted foot ~120. |


#### 网页调参

具体来说，在启动板卡后，其自动连接公司的WIFI（名称与密码在config.h中配置），随后发起一个网页服务并且提供一个IP。此时观察串口可以看到其IP，形如：`192.168.1.114`。在同样连接了公司WIFI（即处于一个内网的环境中），可以在浏览器中输入IP访问

#### 声源定位

板卡利用两路I2S连接了4个麦克风，由两个左声道两个右声道组成。<br>
麦克风布局
```
i2s_data0 -> Left Back
i2s_data1 -> Right Back
i2s_data2 -> Right Front
i2s_data3 -> Left Front
```
在测试的时候，使用边长约为60.8mm的距离比较合适。

具体原理与测试可以见另一仓库@ESP32-GCC-PHAT<br>
这个仓库的特点是也使用了网页调参，类似本工程，可以直观的调试声源的方位。<br>
直接替换成这个工程进行下载可以调试声源定位。

#### 屏幕显示

目前屏幕使用LVGL框架进行GIF显示。单SPI驱动双屏，利用CS时分复用

目前遇到比较大的问题是有一定的撕裂（Tear）问题。使用全屏幕+双缓冲区可以解决。但是我开辟到SPIRAM的空间好像不是很好用，会出现传输问题。

同时，切换成双SPI会使得刷新率提高很多，目前SPI速率80M，但是刷新率约只有10fps。

同时，注意总是在Core 0进行屏幕相关的工作，Core 1 进行其他工作。这样的分配下ESP32P4的性能绰绰有余，主要还是被时分复用与SPI速率限制。

#### 运动调参

本系统有两套运动逻辑。一种基于一阶SIN拟合的曲线。在Web中可以直接调整参数。

第二种是基于KF（key frame）的运动逻辑，用于解决上面的逻辑的周期性与单调性的问题，目前运动参数被硬编码到代码中
