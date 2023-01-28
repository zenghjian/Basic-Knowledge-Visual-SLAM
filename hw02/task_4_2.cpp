#include <iostream>
#include <cmath>
using namespace std;
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

using namespace Eigen;
int main ( int argc, char** argv )
{
    Vector3d p_c (0.3, 0.2, 1.2);
    Quaterniond q_wr (0.55, 0.3, 0.2, 0.2);
    Quaterniond q_rb (0.99, 0.0, 0.0, 0.01);    
    Quaterniond q_bc (0.8, 0.2, 0.1, 0.1);
    Quaterniond q_bl  (0.3, 0.5, 0.0, 20.1);
    Vector3d t_wr (0.1, 0.2, 0.3);
    Vector3d t_rb (0.05, 0.0, 0.5);    
    Vector3d t_bc (0.5, 0.1, 0.5);
    Vector3d t_bl ( 0.4, 0.0, 0.5 );    


    q_bc.normalize();
    cout << "norm is \n" << q_bc.coeffs() << endl;
    // 使用前必须归一化
    Isometry3d T_bc=Eigen::Isometry3d::Identity();
    AngleAxisd rv_bc (q_bc);
    T_bc.rotate(q_bc);
    T_bc.pretranslate ( t_bc );

    Vector3d p_b = T_bc * p_c;
    // 也可以 pb = qbc *pc +tbc 直接乘
    cout << "pb is \n" << p_b << endl;

    q_bl.normalize();   
    Isometry3d T_bl=Eigen::Isometry3d::Identity();
    AngleAxisd rv_bl (q_bl);
    T_bl.rotate(q_bl);
    T_bl.pretranslate ( t_bl );

    Vector3d p_l = T_bl.inverse() * p_b;
    cout << "pl(激光下坐标) is \n" << p_l << endl;

    q_rb.normalize();   
    Isometry3d T_rb=Eigen::Isometry3d::Identity();
    AngleAxisd rv_rb (q_rb);
    T_rb.rotate(q_rb);
    T_rb.pretranslate ( t_rb ); 

    q_wr.normalize(); 
    Isometry3d T_wr=Eigen::Isometry3d::Identity();
    AngleAxisd rv_wr (q_wr);
    T_wr.rotate(q_wr);
    T_wr.pretranslate ( t_wr ); 

    Vector3d p_w = T_wr * T_rb * p_b;

    cout << "pw(世界下坐标) is \n" << p_w << endl;



    return 0;
}
