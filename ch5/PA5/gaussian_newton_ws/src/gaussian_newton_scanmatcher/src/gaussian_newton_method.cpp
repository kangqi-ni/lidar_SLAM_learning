#include <map.h>
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

//进行角度正则化 [-pi, pi]
double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans = Eigen::Vector3d::Zero();
    //TODO
    // Get map indices of the 4 corners
    double x = coords[0];
    double y = coords[1];

    int ind_x = floor((x - map->origin_x) / map->resolution) + map->size_x / 2;
    int ind_y = floor((y - map->origin_y) / map->resolution) + map->size_y / 2;

    if (!MAP_VALID(map, ind_x, ind_y) or !MAP_VALID(map, ind_x + 1, ind_y + 1)) {
        std::cout << "Map indices are outside the map!\n";
        return ans;
    }

    // Get cell values of the 4 corners
    double z1 = map->cells[MAP_INDEX(map, ind_x, ind_y)].score;
    double z2 = map->cells[MAP_INDEX(map, ind_x + 1, ind_y)].score;
    double z3 = map->cells[MAP_INDEX(map, ind_x, ind_y + 1)].score;
    double z4 = map->cells[MAP_INDEX(map, ind_x + 1, ind_y + 1)].score;

    // Get world coodinates of the 4 corners
    double x0 = MAP_WXGX(map, ind_x);
    double x1 = MAP_WXGX(map, ind_x + 1);
    double y0 = MAP_WYGY(map, ind_y);
    double y1 = MAP_WYGY(map, ind_y + 1);

    // Bilinear interpolate the map value
    ans[0] = (x - x1) / (x0 - x1) * (y - y1) / (y0 - y1) * z1
        + (x - x0) / (x1 - x0) * (y - y1) / (y0 - y1) * z2
        + (x - x0) / (x1 - x0) * (y - y0) / (y1 - y0) * z3
        + (x - x1) / (x0 - x1) * (y - y0) / (y1 - y0) * z4;
    ans[1] = 1 / (x0 - x1) * (y - y1) / (y0 - y1) * z1
        + 1 / (x1 - x0) * (y - y1) / (y0 - y1) * z2
        + 1 / (x1 - x0) * (y - y0) / (y1 - y0) * z3
        + 1 / (x0 - x1) * (y - y0) / (y1 - y0) * z4;
    ans[2] = (x - x1) / (x0 - x1) * 1 / (y0 - y1) * z1
        + (x - x0) / (x1 - x0) * 1 / (y0 - y1) * z2
        + (x - x0) / (x1 - x0) * 1 / (y1 - y0) * z3
        + (x - x1) / (x0 - x1) * 1 / (y1 - y0) * z4;
    //END OF TODO

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO
    double theta = now_pose[2];
    for (Eigen::Vector2d &pt: laser_pts) {
        // Transform point from the world frame to current frame
        Eigen::Vector2d pt_cur = GN_TransPoint(pt, GN_V2T(now_pose));
        // Get the map value and gradient at the current point 
        Eigen::Vector3d M = InterpMapValueWithDerivatives(map, pt_cur);   
        
        // Compute fi(x)
        double fx = 1 - M[0];
        
        // Compute Hi and bi
        Eigen::Matrix<double, 2, 3> dS_dT;
        dS_dT << 1, 0, -sin(theta)*pt[0] - cos(theta)*pt[1],
                  0, 1, cos(theta)*pt[0] - sin(theta)*pt[1];
        Eigen::Matrix<double, 1, 2> grad_M(M[1], M[2]);
        Eigen::Matrix<double, 1, 3> J = grad_M * dS_dT;

        H += J.transpose() * J;
        b += fx * J.transpose();
    }
    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    for(int i = 0; i < maxIteration;i++) {
        //TODO
        Eigen::Matrix3d H;
        Eigen::Vector3d b;
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);

        // Make sure H is not singular
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H);
        double ratio = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
        
        if (ratio > 1000) {
            std::cout << "Matrix H is almost singular!\n";
            break;
        }

        Eigen::Vector3d x = H.colPivHouseholderQr().solve(b);
        if(std::fabs(x(2)) >= 0.17){
            std::cout << "Rotation is too large!\n";
            break;
        }
        now_pose += x;

        if(std::sqrt(std::pow(x(0),2) + std::pow(x(1),2)) < 0.001 && x(2) < (0.01/57.295)){
            std::cout << "delta T converges" << std::endl;
            break;
        }
        //END OF TODO
    }
    init_pose = now_pose;
}
