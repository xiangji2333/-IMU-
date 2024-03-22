对六轴imu的一个信息提取，通过Windows的串口协议
new data figure 是通过imu自身获取的加速度和角度对imu在世界坐标系下的位置解算，需用使用c++ eigen线性代数库，可通过[官网](https://eigen.tuxfamily.org/index.php?title=Main_Page)下载后在visual studio内设置包含库
