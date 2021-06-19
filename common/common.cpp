#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "common.h"
#include "sophus/rotation.h"
#include "random.h"

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

template<typename T>
void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "Invalid UW data file. ";
}
//给一个三维向量加入噪声，很简单xyz依次加入随机值就好了。定义这个的目的是为了后面的Perturb()函数在增加噪声时，
// 是分开对路标点，相机的旋转，相机的平移分别加入噪声的，并且这三个量都是三维的，所以定义一个三维向量添加噪声的函数
void PerturbPoint3(const double sigma, double *point) {
    for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
}
//取一个数组的中位数，主要用在Normalize()函数中。
double Median(std::vector<double> *data) {
    int n = data->size();
    std::vector<double>::iterator mid_point = data->begin() + n / 2;
    std::nth_element(data->begin(), mid_point, data->end());
  //nth_element作用为求第n小的元素，并把它放在第n位置上:
  /**
   * inline void
   * nth_element(_RandomAccessIterator __first, _RandomAccessIterator __nth,
   *             _RandomAccessIterator __last)
   * 这个函数有点半排序的感觉，它能够让第n个大小的值在第n个位置，左边的都比它小，右边的都比它大，但是不保证其他的顺序。
   */
  //由于这里mid_point指向数组的中间位置，所以下一句的*mid_point肯定就是一个数组的中位数了。
    return *mid_point;
}
//构造函数，主要是把优化数据读进程序
BALProblem::BALProblem(const std::string &filename, bool use_quaternions) {
    FILE *fptr = fopen(filename.c_str(), "r");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };
//  problem-16-22106-pre.txt文件中的头三个数字，表征了相机数，路标点数，观测数。
  //这里就直接读进类成员中去了：num_cameras_、 num_points_、 num_observations_
    // This wil die horribly on invalid files. Them's the breaks.
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    std::cout << "Header: " << num_cameras_
              << " " << num_points_
              << " " << num_observations_<<std::endl;
    //观测数据中的路标点编号那一列
    point_index_ = new int[num_observations_];
    //观测数据中的相机编号那一列
    camera_index_ = new int[num_observations_];
    //观测数据中的观测值那两列
    observations_ = new double[2 * num_observations_];
    //所有要优化的参数量，相机个数*9维，路标点个数*3维。
    num_parameters_ = 6 * num_cameras_ + 3 * num_points_;
    //将这些值存进parameters_数组中。这里也就是优化变量的值，只不过全部一列码开，在txt中也就是后半部分，所有优化变量的初始值
    parameters_ = new double[num_parameters_];

    //这里开始对.txt文件读取了，按文件中数据循序读，第i个相机，第j个路标，然后两个像素坐标。循环读完所有观测。
    //用num_observations控制循环读取所有观测，每次读取一行。
    for (int i = 0; i < num_observations_; ++i) {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        for (int j = 0; j < 2; ++j) {
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
        }
    }
  //这里就读到了txt的后半部分，就是所有优化变量的具体值，当然这里也就是初始值了。
    for (int i = 0; i < num_parameters_; ++i) {
        FscanfOrDie(fptr, "%lf", parameters_ + i);
    }

    fclose(fptr);
  //使用四元数表示旋转，camera 相机内外参，9维数组（四元数10维）。0-2旋转向量、3-5平移向量、6-8相机内参(f,二阶径向畸变系数,四阶径向畸变系数)
    use_quaternions_ = use_quaternions;
    if (use_quaternions) {
        // Switch the angle-axis rotations to quaternions.
//      R(4),t(3),f(1),k1(1) and k2(1)
        num_parameters_ = 10 * num_cameras_ + 3 * num_points_;
        double *quaternion_parameters = new double[num_parameters_];
        double *original_cursor = parameters_;
        double *quaternion_cursor = quaternion_parameters;
        for (int i = 0; i < num_cameras_; ++i) {
            AngleAxisToQuaternion(original_cursor, quaternion_cursor);
            quaternion_cursor += 4;
            original_cursor += 3;
            for (int j = 4; j < 10; ++j) {
                *quaternion_cursor++ = *original_cursor++;
            }
        }
        // Copy the rest of the points.
        for (int i = 0; i < 3 * num_points_; ++i) {
            *quaternion_cursor++ = *original_cursor++;
        }
        // Swap in the quaternion parameters.
        delete[]parameters_;
        parameters_ = quaternion_parameters;
    }
}

void BALProblem::WriteToFile(const std::string &filename) const {
    FILE *fptr = fopen(filename.c_str(), "w");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    }
  //对比data中txt文件的第一行：相机个数，路标个数，观测个数
    fprintf(fptr, "%d %d %d %d\n", num_cameras_, num_cameras_, num_points_, num_observations_);
  //这个循环是每行循环，每行四部分，相机，路标，两维的观测
    for (int i = 0; i < num_observations_; ++i) {
        fprintf(fptr, "%d %d", camera_index_[i], point_index_[i]);
        for (int j = 0; j < 2; ++j) {
            fprintf(fptr, " %g", observations_[2 * i + j]);
        }
        fprintf(fptr, "\n");
    }

  //这里应该到了输出9维的相机参数
  //用相机个数控制总循环
    for (int i = 0; i < num_cameras(); ++i) {
        double angleaxis[6];
        if (use_quaternions_) {
            //OutPut in angle-axis format.
            //先把四元数转成轴角，拷贝3个参数
            QuaternionToAngleAxis(parameters_ + 10 * i, angleaxis);
            //再拷贝剩下的6个参数，txtytzfk1k2
            memcpy(angleaxis + 3, parameters_ + 10 * i + 4, 6 * sizeof(double));
        } else {
            //直接将9个参数拷贝出去
            memcpy(angleaxis, parameters_ + 6 * i, 6 * sizeof(double));
        }
        //输出到txts
        for (int j = 0; j < 6; ++j) {
            fprintf(fptr, "%.16g\n", angleaxis[j]);
        }
    }
  //这里输出3维的路标参数
  //这里用points指针，跳过相机参数部分，指向路标的首位置
  //那么问题来了，为什么不用points()直接定位？？
    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
      //每个路标3维，这里的3维用的是point_block_size()的维度值，上方的相机参数输出时，用的直接就是9.有点小乱
      const double *point = points + i * point_block_size();
        for (int j = 0; j < point_block_size(); ++j) {
            fprintf(fptr, "%.16g\n", point[j]);
        }
    }

    fclose(fptr);
}

// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
//将相机位姿和路标点写入文件，这里也就是生成.PLY文件
void BALProblem::WriteToPLYFile(const std::string &filename) const {
    std::ofstream of(filename.c_str());
  //这里是写入一些头字符，一般是文件的一些说明，比如有顶点数量，表示方式xyz，rgb颜色等
    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << num_cameras_ + num_points_
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << std::endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_cameras(); ++i) {
        const double *camera = cameras() + camera_block_size() * i;
      //用CameraToAngelAxisAndCenter()函数将从相机参数中解析出来相机姿势和相机位置。当然这里只用位置了。
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      //坐标依次写入文件，再加上颜色数据。
      of << center[0] << ' ' << center[1] << ' ' << center[2]
           << " 0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points + i * point_block_size();
      //维度循环，写入xyz
      for (int j = 0; j < point_block_size(); ++j) {
            of << point[j] << ' ';
        }
        of << " 255 255 255\n";
    }
    of.close();
}

/**
 * 由camera数据中的旋转向量和平移向量解析出相机世界坐标系下的姿态(依旧是旋转向量)和位置(世界坐标系下的xyz)，也是用于生成点云用的
 * @param camera 要解析的相机参数，前三维旋转，接着三维平移，这里只用到这6维
 * @param angle_axis 解析出的相机姿态承接数组，也是旋转向量形式
 * @param center 解析出来的相机原点在世界坐标系下的坐标承接数组，XYZ
 */
void BALProblem::CameraToAngelAxisAndCenter(const double *camera,
                                            double *angle_axis,
                                            double *center) const {
    VectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_) {
        QuaternionToAngleAxis(camera, angle_axis);
    } else {
        angle_axis_ref = ConstVectorRef(camera, 3);
    }
//[(Rcamera_world)^(-1) -(Rcamera_world)^(-1)tcamera_world
//    0                          1                        ]
    // c = -R't
//  center是指相机原点在世界坐标系下的坐标，那么定义一下：
//Tcamera_world*Pworld_center = Pcamera_center
//pworld_center = (Tcamera_world)^(-1)*Pcamera_center
//因为Pcamra_center = (0,0,0)，所以求一下Pworld_center = -Rworld_camera*tcamera_world
//相机的外参中就是将世界坐标系中的点转化到相机坐标系Tcamera_world
//轴角取负号就是求逆
  Eigen::VectorXd inverse_rotation = -angle_axis_ref;
    //第一个量是Rworld_camera 第二个量是tcamera_world(取得是camera参数中的t的部分) center得到的结果和最终的result差一个负号
    AngleAxisRotatePoint(inverse_rotation.data(),
                         camera + camera_block_size() - 6,
                         center);
    //添加上负号
    VectorRef(center, 3) *= -1.0;
}

/**
 * 反向过程，由世界坐标系下的相机姿态和原点位置，生成一个camera数据
 * @param angle_axis 旋转向量数据世界坐标系下的旋转向量
 * @param center 相机中心在世界坐标系下的位置坐标
 * @param camera 承接数据的camera数组，由于这里只是生成旋转和平移，所以是camera的前6维
 */

void BALProblem::AngleAxisAndCenterToCamera(const double *angle_axis,
                                            const double *center,
                                            double *camera) const {
    ConstVectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_) {
        AngleAxisToQuaternion(angle_axis, camera);
    } else {
        VectorRef(camera, 3) = angle_axis_ref;
    }

    // t = -R * c
  /**
   * 这里再说一下，同样是上面的过程：
   * angle_axis为R
   * center为PW_center
   * 结果t放在camera+camera_block_size()-6位置，也就是camera中的平移位置。
**/

//  已知Tworld_camera = [Rworld_camera tworld_camera
//                            0           1]
//  求tcamera_world  根据逆矩阵方法 Tcamera_world = [Rcamera_world tcamera_world  =  [Rworld_camera^(-1) -Rworld_camera^(-1)*tworld_camera
//                                                      0                  1]       0                               1       ]
//即tcamera_world =  -Rworld_camera^(-1)*tworld_camera
  AngleAxisRotatePoint(angle_axis, center, camera + camera_block_size() - 6);
  VectorRef(camera + camera_block_size() - 6, 3) *= -1.0;
}

void BALProblem::Normalize() {
  // Compute the marginal median of the geometry (每一位的中位数 边缘中位数)
  std::vector<double> tmp(num_points_);
  Eigen::Vector3d median;
  double *points = mutable_points();
  //求所有路标点的xyz方向的中位数 median(i)
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < num_points_; ++j) {
          tmp[j] = points[3 * j + i];
      }
      median(i) = Median(&tmp);
  }
//每一个点的坐标减去中位数  并取绝对值
//这步完成之后，空间点相对于中值点的偏差的L1范数被求出来了，放在tmp[]数组中
  for (int i = 0; i < num_points_; ++i) {
      VectorRef point(points + 3 * i, 3);
      //l1范数 取绝对值
      tmp[i] = (point - median).lpNorm<1>();
  }
//偏差的中位数
//然后再对tmp[]求中值，命名为绝对偏差的中值
  const double median_absolute_deviation = Median(&tmp);

  // Scale so that the median absolute deviation of the resulting
  // reconstruction is 100
  //（数据的大小顺序没变，原来在中值位置的点还是中值，原来与在median _absolute_deviation处的点还在那个位置
  // 与media 的差距还是那个数，所以scale是这样设定的）

  const double scale = 100.0 / median_absolute_deviation;

  // X = scale * (X - median)
  for (int i = 0; i < num_points_; ++i) {
      VectorRef point(points + 3 * i, 3);
      point = scale * (point - median);
  }

  double *cameras = mutable_cameras();
  double angle_axis[3];
  double center[3];
  for (int i = 0; i < num_cameras_; ++i) {
      double *camera = cameras + camera_block_size() * i;
      //将Tcamera_world转换成Tworld_camera因为下面的这个media 是世界坐标系中的
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      // center = scale * (center - median)
      VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);
      //将Tworld_camera转换成Tcamera_world
      AngleAxisAndCenterToCamera(angle_axis, center, camera);
  }
}
//添加噪声
void BALProblem::Perturb(const double rotation_sigma,
                       const double translation_sigma,
                       const double point_sigma) {
  assert(point_sigma >= 0.0);
  assert(rotation_sigma >= 0.0);
  assert(translation_sigma >= 0.0);

  double *points = mutable_points();
  if (point_sigma > 0) {
      for (int i = 0; i < num_points_; ++i) {
          PerturbPoint3(point_sigma, points + 3 * i);
      }
  }

  for (int i = 0; i < num_cameras_; ++i) {
      double *camera = mutable_cameras() + camera_block_size() * i;

      double angle_axis[3];
      double center[3];
      // Perturb in the rotation of the camera in the angle-axis
      // representation
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      if (rotation_sigma > 0.0) {
          PerturbPoint3(rotation_sigma, angle_axis);
      }
      AngleAxisAndCenterToCamera(angle_axis, center, camera);

      if (translation_sigma > 0.0)
          PerturbPoint3(translation_sigma, camera + camera_block_size() - 6);
  }
}
