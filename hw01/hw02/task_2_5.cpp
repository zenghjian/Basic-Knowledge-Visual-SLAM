#include <iostream>
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;
#define MATRIX_SIZE 100

int main( int argc, char** argv )
{
    
    MatrixXd matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    MatrixXd tmp = matrix_NN * matrix_NN.transpose();    
    // create dynmaic size matrix A (N,N)
    VectorXd v_Nd = VectorXd:: Random(MATRIX_SIZE);    
    // create vector b (N,1)

    // llt require input matrix is symmetrix + positive defined 
    // 通过矩阵乘以自己的转置构建对称矩阵 A = A^T
    VectorXd qr = tmp.colPivHouseholderQr().solve(v_Nd);
    // compute qr decomposition
    VectorXd llt = tmp.llt().solve(v_Nd);

    cout <<"QR decompose: "<< qr << endl;

    cout <<"Cholesky decompose: "<< llt << endl;

    return 0;
}
