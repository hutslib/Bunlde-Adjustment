#include <iostream>
#include <ceres/ceres.h>
#include "common/common.h"
#include "BAReprojectionError.h"
#include "sophus/se3.hpp"
using namespace std;

void SolveBA(BALProblem &bal_problem);
int main(int argc, char **argv) {
  std::string filename;
  if (argc == 2) {
    filename = argv[1];
  }
  if (argc != 2) {
    cout << "usage: bundle_adjustment_ceres bal_data.txt" << endl;
    filename = "/home/udi/01hts/Bundle_Adjustment/dataset/problem-16-22106-pre.txt";
    //return 1;
  }
  //step1定义BALProblem类对象从文件读入BAL dataset
  //step2:

  BALProblem bal_problem(filename);
  bal_problem.Normalize();
  bal_problem.Perturb(0.1, 0.5, 0.5);
  bal_problem.WriteToPLYFile("initial.ply");
  SolveBA(bal_problem);
  bal_problem.WriteToPLYFile("final.ply");

  return 0;
}

void SolveBA(BALProblem &bal_problem) {
  //得到相机维度和路标维度，相机维度如果用四元数的话就是10维，旋转向量的话就是9维，路标维度3维
  const int point_block_size = bal_problem.point_block_size();
  const int camera_block_size = bal_problem.camera_block_size();
  //返回数据中路标位姿数据列的开头位置，这个是可以更改的那一组
  double *points = bal_problem.mutable_points();
  //返回数据中相机位姿数据列的开头位置 这个是可以更改的那一组
  double *cameras = bal_problem.mutable_cameras();

  // Observations is 2 * num_observations long array observations
  // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
  // and y position of the observation.
  const double *observations = bal_problem.observations();
  ceres::Problem problem;
  ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering;

  for (int i = 0; i < bal_problem.num_cameras(); i++) {
    double *camera = cameras + camera_block_size * i;
    problem.AddParameterBlock(camera, 6, new PoseSE3Parameterization());
    //if(i < 2)
    //{
    //  problem.SetParameterBlockConstant(camera);
    //}
  }
  for (int i = 0; i < bal_problem.num_points(); i++) {
    double *point = points + point_block_size * i;
    problem.AddParameterBlock(point, 3);
    ordering->AddElementToGroup(point, 0);
  }
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    ceres::CostFunction *cost_function;

    double *camera = cameras + camera_block_size * bal_problem.camera_index()[i];
    double *point = points + point_block_size * bal_problem.point_index()[i];

    cost_function = new BAReprojectionError(observations[2 * i + 0], observations[2 * i + 1]);

    // If enabled use Huber's loss function.
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    problem.AddResidualBlock(cost_function, loss_function, camera, point);
  }
  for (int i = 0; i < bal_problem.num_cameras(); i++) {
    double *camera = cameras + camera_block_size * i;
    ordering->AddElementToGroup(camera, 1);
  }
  // show some information here ...
  std::cout << "bal problem file loaded..." << std::endl;
  std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
            << bal_problem.num_points() << " points. " << std::endl;
  std::cout << "Forming " << bal_problem.num_observations() << " observations. " << std::endl;

  std::cout << "Solving ceres BA ... " << endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  options.linear_solver_ordering.reset(ordering);
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

}