/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once

#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace crisp::render {
class Camera;
class Debug;
class Geometry;
class Ground;
class Lights;
class Shader;
}  // namespace crisp::render

namespace crisp {
struct data_t;
struct model_t;

struct camera_t {
  Eigen::Vector3f target;
  float distance, azimuth, elevation, fovy;
};

struct scene_t {
  struct option_t {
    bool shadow = true;
    bool reflection = true;

    bool clear_color = true;
    bool clear_depth = true;
  } opt;

  struct debug_t {
    float point_size = 10.0f;
    float force_scale = 20.0f;
    float force_width = 2.0f;
    float force_head_size = 0.01f;
    float bbox_width = 2.0f;
    float collider_width = 1.0f;
    float collider_alpha = 0.2f;

    bool contact_point = false;
    bool contact_force = false;
    bool bounding_box = false;
    bool collider = false;
  } dbg;

  struct {
    Eigen::Vector4f bg;
    Eigen::Vector4f contact_point = {1.0f, 0.0f, 0.0f, 1.0f};
    Eigen::Vector4f contact_force = {0.0f, 0.0f, 1.0f, 1.0f};
    Eigen::Vector4f bounding_box = {1.0f, 1.0f, 0.0f, 1.0f};
    Eigen::Vector4f collider = {0.0f, 1.0f, 0.0f, 0.2f};
  } rgba;

  struct rect_t {
    int left = 0, bottom = 0;
    int width, height;
  } viewport;

  std::unique_ptr<render::Camera> cam;
  std::unique_ptr<render::Debug> debug;
  std::unique_ptr<render::Lights> lights;
  std::unique_ptr<render::Ground> ground;
  std::vector<render::Geometry> geoms;
  std::unordered_map<int, render::Shader> shaders;

  CRISP_API scene_t();
  CRISP_API ~scene_t();
};

struct mouse_t {
  bool left_btn = false, right_btn = false, scroll_btn = false;
  int x, y;
  float dx, dy, sx, sy;
};

CRISP_API std::unique_ptr<camera_t> load_camera(model_t const& m);
CRISP_API void move_camera(camera_t& cam, mouse_t const& mouse);

CRISP_API std::unique_ptr<scene_t> make_scene(model_t const& m);
CRISP_API void update_scene(model_t const& m, camera_t& cam, scene_t& scn);

CRISP_API void render_scene_forward_only(
  model_t const& m, data_t const& d, scene_t& scn, render::Camera const& cam,
  scene_t::rect_t const& vp);
CRISP_API void render_scene(
  model_t const& m, data_t const& d, scene_t& scn, render::Camera const& cam,
  scene_t::rect_t const& vp);
CRISP_API void render_scene(model_t const& m, data_t const& d, scene_t& scn);
}  // namespace crisp
