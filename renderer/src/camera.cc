#include "../include/camera.h"

Camera::Camera(float fov, float aspectRatio, float near, float far)
    : _fov(fov), _aspectRatio(aspectRatio), _near(near), _far(far) {
  _position = Eigen::Vector3f(0.0f, 0.0f, 1.5f);

  Eigen::Vector3f cameraTarget(0.0f, 0.0f, 0.0f);
  _direction = _position - cameraTarget;
  _direction.normalize();

  _up = Eigen::Vector3f(0.0, 1.0, 0.0);
  _right = _up.cross(_direction);
  _right.normalize();

  float yScale = 1.0 / std::tan(fov / 2);
  float xScale = yScale / aspectRatio;
  // 透视变换矩阵公式参考
  // 实时渲染中的坐标系变换（4）：投影变换-2 - IgorKakarote的文章 - 知乎
  // https://zhuanlan.zhihu.com/p/114729671
  _perspectiveMatrix << xScale, 0, 0, 0, 0, yScale, 0, 0, 0, 0,
      -(far + near) / (far - near), -2.0 * near * far / (far - near), 0, 0,
      -1.0, 0.0;

  updateViewMatrix();
}

Eigen::Matrix4f Camera::getProjectMatrix() { return _perspectiveMatrix; }

Eigen::Matrix4f Camera::getViewMatrix() { return _viewMatrix; }

void Camera::setDirection(Eigen::Vector3f d, Eigen::Vector3f up,
                          Eigen::Vector3f right) {
  _direction = d;
  _up = up;
  _right = right;
  updateViewMatrix();
}

void Camera::updateViewMatrix() {
  _viewMatrix = Eigen::Matrix4f::Identity();
  _viewMatrix.block<1, 3>(0, 0) = _right.transpose();
  _viewMatrix.block<1, 3>(1, 0) = _up.transpose();
  _viewMatrix.block<1, 3>(2, 0) = _direction.transpose();

  Eigen::Affine3f translation = Eigen::Affine3f::Identity();
  translation.translate(-_direction);

  _viewMatrix *= translation.matrix();
}
