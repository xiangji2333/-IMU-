#include<Eigen/eigen>
#include<iostream>
#include<fstream>
#include<iomanip>

const double ARC_TO_DEG = 57.29577951308238;//弧度转度数
const double DEG_TO_ARC = 0.0174532925199433;//度数转弧度制

class imu_data {
private:
    //局部坐标系下加速度
    Eigen::Vector3d localacceleration;
    //世界坐标系下加速度
    Eigen::Vector3d worldacceleration;
    // 欧拉角,pitch——x，roll——y，yaw——z
    Eigen::Vector3d euler_angle;
    //局部坐标系转到全局坐标系的旋转矩阵
    Eigen::Matrix3d rotation_matrix;
    //全局坐标系下速度
    Eigen::Vector3d velocity;
    //全局坐标系位置
    Eigen::Vector3d position;
    //传感器时间间隔

    double dt = 0.05;

public:
    imu_data() {
        velocity << 0, 0, 0;
        position << 0, 0, 0;
    }

    void update(double acc_x, double acc_y, double acc_z, double pitch, double roll, double yaw) {

        //修改imu_data
        euler_angle << pitch * DEG_TO_ARC, roll* DEG_TO_ARC, yaw* DEG_TO_ARC;
        localacceleration << acc_x, acc_y, acc_z;

        // 使用Eigen库将欧拉角转换为旋转矩阵
        rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());

        updateposition();
    }

    void updateposition() {//更新位置
        worldacceleration = rotation_matrix * localacceleration;//更新世界速度,将旋转矩阵乘以局部加速度转到世界加速度
        worldacceleration[2] -= 9.8;//消除重力加速度影响
        velocity += worldacceleration * dt;
        position += velocity * dt;
    }

    const Eigen::Matrix3d& getremote() {
        return rotation_matrix;
    }


    const Eigen::Vector3d& getposition() {
        return position;
    }
};

int main() {

    //初始化imu
    imu_data imu;

    std::ifstream file("out.txt");
    std::ofstream outfile("ok.txt");

    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        long long timestamps;
        double accx, accy, accz, angx, angy, angz;
        if (!(iss >>timestamps>> accx >> accy >> accz >> angx >> angy >> angz)) {
            std::cerr << "文件格式错误！\n";
            continue;
        }
        //测试读取
        //std::cout << timestamps << ' ' << accx << ' ' << accy << ' ' << accz << ' ' << angx << ' ' << angy << ' ' << angz << std::endl;

        imu.update(accx, accy, accz, angx, angy, angz);
        Eigen::Vector3d pos = imu.getposition();
        Eigen::Matrix3d remote = imu.getremote();
        //std::cout << std::fixed << std::setprecision(10) << timestamps << ' ' << pos[0] << ' ' << pos[1] << ' ' << pos[2] << std::endl;
        outfile << std::fixed << std::setprecision(10) << timestamps 
            << ' ' << remote(0 ,0) << ' ' << remote(0, 1) << ' ' << remote(0, 2) << ' ' << pos[0] 
            << ' ' << remote(1, 0) << ' ' << remote(1, 1) << ' ' << remote(1, 2) << ' '<<pos[1] 
            << ' ' << remote(2, 0) << ' ' << remote(2, 1) << ' ' << remote(2, 2) << ' ' << pos[2] 
            << std::endl;

    }
}

