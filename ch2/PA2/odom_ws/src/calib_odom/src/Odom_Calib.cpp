#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //构建超定方程组
        // Ai * xi = bi
        Eigen::Matrix<double,3,9> Ai;
        Ai << Odom[0], Odom[1], Odom[2], 0, 0, 0, 0, 0, 0,
              0, 0, 0, Odom[0], Odom[1], Odom[2], 0, 0, 0,                    
              0, 0, 0, 0, 0, 0, Odom[0], Odom[1], Odom[2];
        
        Eigen::Vector3d bi;
        bi << scan[0], scan[1], scan[2];
        
        A.block<3,9>(now_len*3, 0) = Ai;
        b.block<3,1>(now_len*3, 0) = bi;

        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //求解线性最小二乘
    Eigen::Matrix<double,9,1> x = A.colPivHouseholderQr().solve(b);
    correct_matrix << x[0], x[1], x[2], 
                      x[3], x[4], x[5], 
                      x[6], x[7], x[8];

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
