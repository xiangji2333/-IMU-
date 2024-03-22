#include<Eigen/eigen>
#include<iostream>
#include<fstream>
#include<iomanip>

const double ARC_TO_DEG = 57.29577951308238;//����ת����
const double DEG_TO_ARC = 0.0174532925199433;//����ת������

class imu_data {
private:
    //�ֲ�����ϵ�¼��ٶ�
    Eigen::Vector3d localacceleration;
    //��������ϵ�¼��ٶ�
    Eigen::Vector3d worldacceleration;
    // ŷ����,pitch����x��roll����y��yaw����z
    Eigen::Vector3d euler_angle;
    //�ֲ�����ϵת��ȫ������ϵ����ת����
    Eigen::Matrix3d rotation_matrix;
    //ȫ������ϵ���ٶ�
    Eigen::Vector3d velocity;
    //ȫ������ϵλ��
    Eigen::Vector3d position;
    //������ʱ����

    double dt = 0.05;

public:
    imu_data() {
        velocity << 0, 0, 0;
        position << 0, 0, 0;
    }

    void update(double acc_x, double acc_y, double acc_z, double pitch, double roll, double yaw) {

        //�޸�imu_data
        euler_angle << pitch * DEG_TO_ARC, roll* DEG_TO_ARC, yaw* DEG_TO_ARC;
        localacceleration << acc_x, acc_y, acc_z;

        // ʹ��Eigen�⽫ŷ����ת��Ϊ��ת����
        rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());

        updateposition();
    }

    void updateposition() {//����λ��
        worldacceleration = rotation_matrix * localacceleration;//���������ٶ�,����ת������Ծֲ����ٶ�ת��������ٶ�
        worldacceleration[2] -= 9.8;//�����������ٶ�Ӱ��
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

    //��ʼ��imu
    imu_data imu;

    std::ifstream file("out.txt");
    std::ofstream outfile("ok.txt");

    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        long long timestamps;
        double accx, accy, accz, angx, angy, angz;
        if (!(iss >>timestamps>> accx >> accy >> accz >> angx >> angy >> angz)) {
            std::cerr << "�ļ���ʽ����\n";
            continue;
        }
        //���Զ�ȡ
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

