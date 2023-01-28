#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <sophus/se3.h>
#include <unistd.h>
#include <cassert>
using namespace std;

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;
// path to trajectory file
string trajectory_file_estimated = "../estimated.txt";
string trajectory_file_gt = "../groundtruth.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(TrajectoryType, TrajectoryType);

TrajectoryType ReadTrajectory(const string &path) {

    TrajectoryType poses;
    
    ifstream input(path);
    std::array<double, 7> tmp{};
    double time = 0.0;
    while(input >> time){  
        for( auto& i :tmp){
            input >> i;
        }
        Eigen::Quaterniond q(tmp[6], tmp[3], tmp[4], tmp[5]);
        Eigen::Vector3d t(tmp[0], tmp[1], tmp[2]);      
        Sophus::SE3 point (q, t); 
        poses.push_back(point);
    }
    return poses;
}

int main(int argc, char **argv) {


    /* 另一种写法 */
    // while(!input.eof()){
    //     double time, tx, ty, tz, qx, qy, qz, qw;
    //     input >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    //     Eigen::Quaterniond q(qw, qx, qy, qz);
    //     Eigen::Vector3d t(tx, ty, tz);
    //     Sophus::SE3 point (q, t); 
    //     poses.push_back(point);
    // }

    /// implement pose reading code
    // start your code here (5~10 lines)

    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(ReadTrajectory(trajectory_file_gt), ReadTrajectory(trajectory_file_estimated));
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(TrajectoryType gt,TrajectoryType estimated) {
    if (gt.empty() || estimated.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    // caculate RMSE
    double sum_error_quad = 0.0;
    for(auto i = 0; i < gt.size(); ++i){
        double error = 0.0;
        error = (gt[i].inverse() * estimated[i]).log().norm();
        sum_error_quad += pow(error,2);
    }
    double rmse = 0.0;
    rmse = sqrt(sum_error_quad / gt.size());
    cout << "RMSE is: " << rmse << endl;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    int count = 0;
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < estimated.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = estimated[i], p2 = estimated[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }        
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}