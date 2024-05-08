#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}
  
    template <typename T>
    bool operator()(const T* const r,
                    const T* const t,
                    const T* const i,
                    const T* const point,
                    T* residuals) const {
        T p[3];
        p[0] = point[0] - t[0];
        p[1] = point[1] - t[1];
        p[2] = point[2] - t[2];

        T p_new[3];
        ceres::QuaternionRotatePoint(r, p, p_new);

        T xp = p_new[0] / p_new[2];
        T yp = p_new[1] / p_new[2];
        
        const T& fx = i[0];
        const T& fy = i[1];

        const T& cx = i[2];
        const T& cy = i[3];

        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        return true;
                  
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
        return new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 3, 4, 3>(new SnavelyReprojectionError(observed_x, observed_y));
    }

    double observed_x;
    double observed_y;
};

struct Camera {
    double cx;
    double cy;

    double fx;
    double fy;

    double k1;
    double k2;
    double k3;

    double p1;
    double p2;

    Eigen::Quaterniond q;
    Eigen::Vector3d t;

    Eigen::Vector2d project(Eigen::Vector3d p_world) {
        Eigen::Vector3d p_temp = p_world - t;
        Eigen::Vector3d p_camera = q.toRotationMatrix() * p_temp;

        Eigen::Vector2d p_cameraPlane;
        p_cameraPlane << p_camera.x() / p_camera.z(), p_camera.y() / p_camera.z();

        Eigen::Vector2d pixel;
        pixel << p_cameraPlane.x() * fx + cx, p_cameraPlane.y() * fy + cy;

        return pixel;
    }
};

int main() {
  std::random_device rd; 
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-20.0, 20.0);

  std::vector<Eigen::Vector3d> points;

  for (int i = 0; i < 4000; ++i) {
      Eigen::Vector3d point(dis(gen), dis(gen), dis(gen));
      points.push_back(point);
  }

  // for (const auto& point : points) {
  //     std::cout << "Point: (" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
  // }

  // create the cameras
  int width = 320;
  int height = 240;
  Camera c1, c2;

  c1.fx = 201;
  c1.fy = 199;
  c1.cx = width / 2 - 1;
  c1.cy = height / 2 + 2;

  c2.fx = 200;
  c2.fy = 198;
  c2.cx = width / 2 - 3;
  c2.cy = height / 2 - 2;

  c1.q = Eigen::Quaterniond::Identity();
  c1.t << 0, 0, -40;
  c2.q.w() = 0.4;
  c2.q.x() = 0.1;
  c2.q.y() = 0.1;
  c2.q.normalize();
  c2.t << 12, 10, -40;

  std::vector<Camera> cameras;
  cameras.push_back(c1);
  cameras.push_back(c2);

  std::vector<std::vector<Eigen::Vector2d>> pixels(cameras.size());
  int i=0;
  for (Eigen::Vector3d point : points) {
    pixels[0].push_back(c1.project(point));
    pixels[1].push_back(c2.project(point));
  }

  // without noise, check if the ceres can get the camera's properties correctly
  double c1_r[4] = {1, 0, 0, 0};
  double c1_t[3] = {0, 0, -20};
  double c1_i[4] = {0, 0, 0, 0};
  double c2_r[4] = {1, 0, 0, 0};
  double c2_t[3] = {0, 0, -20};
  double c2_i[4] = {0, 0, 0, 0};

  ceres::Problem problem;

  ceres::Manifold *q_parameterization0 =
      new ceres::EigenQuaternionManifold();
  problem.AddParameterBlock(c1_r, 4, q_parameterization0);
  problem.AddParameterBlock(c1_t, 3);

  problem.SetParameterBlockConstant(c1_r);

  ceres::Manifold *q_parameterization1 =
      new ceres::EigenQuaternionManifold();
  problem.AddParameterBlock(c2_r, 4);
  problem.SetManifold(c2_r, q_parameterization1);

  i=0;
  for (Eigen::Vector2d pixel : pixels[0]) {
    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(pixel.x(), pixel.y());
        problem.AddResidualBlock(cost_function,
                                nullptr /* squared loss */,
                                c1_r,
                                c1_t,
                                c1_i,
                                points[i].data());
    i++;
  }

  i=0;
  for (Eigen::Vector2d pixel : pixels[1]) {
    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(pixel.x(), pixel.y());
        problem.AddResidualBlock(cost_function,
                                nullptr /* squared loss */,
                                c2_r,
                                c2_t,
                                c2_i,
                                points[i].data());
    i++;
  }

  for (i =0; i<points.size(); i++) {
    // problem.SetParameterBlockConstant(points[i].data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  options.max_num_iterations = 100000;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  std::cout << "Original c1 property: " << c1.q.w() << ", " << c1.q.x() << ", " << c1.q.y() << ", " << c1.q.z() << ", " << c1.t.transpose() << ", " << c1.fx << ", " << c1.fy << ", " << c1.cx << ", " << c1.cy << std::endl;
  std::cout << "Optimized c1 property: " << c1_r[0] << ", " << c1_r[1] << ", " << c1_r[2] << ", " << c1_r[3] << ", "
                                          << c1_t[0] << ", " << c1_t[1] << ", " << c1_t[2] << ", "
                                          << c1_i[0] << ", " << c1_i[1] << ", " << c1_i[2] << ", " << c1_i[3] << std::endl;
  
  std::cout << "Original c2 property: " << c2.q.w() << ", " << c2.q.x() << ", " << c2.q.y() << ", " << c2.q.z() << ", " << c2.t.transpose() << ", " << c2.fx << ", " << c2.fy << ", " << c2.cx << ", " << c2.cy << std::endl;
  std::cout << "Optimized c2 property: " << c2_r[0] << ", " << c2_r[1] << ", " << c2_r[2] << ", " << c2_r[3] << ", "
                                          << c2_t[0] << ", " << c2_t[1] << ", " << c2_t[2] << ", "
                                          << c2_i[0] << ", " << c2_i[1] << ", " << c2_i[2] << ", " << c2_i[3] << std::endl;
  
  return 0;
}