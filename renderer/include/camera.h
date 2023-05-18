#pragma once
#ifndef _USE_GLAD
#define _USE_GLAD
#include <glad/glad.h>
#endif

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <iostream>

/**
 * Perspective camera.
 */
class Camera {
 private:
  float _fov;
  float _aspectRatio;
  float _near;
  float _far;
  Eigen::Matrix4f _perspectiveMatrix;
  Eigen::Matrix4f _viewMatrix;

  Eigen::Vector3f _position;
  Eigen::Vector3f _direction;
  Eigen::Vector3f _right;
  Eigen::Vector3f _up;

  void updateViewMatrix();

 public:
  /**
   * construct the camera's view frustum
   *
   * \param fov vertical field of view
   * \param aspectRatio aspect ratio
   * \param near near plane
   * \param far far plane
   */
  Camera(float fov, float aspectRatio, float near, float far);
  Eigen::Matrix4f getViewMatrix();
  Eigen::Matrix4f getProjectMatrix();
  void setDirection(Eigen::Vector3f d, Eigen::Vector3f up,
                    Eigen::Vector3f right);
};
