#pragma once

#include "geometry/circle.hh"
#include "geometry/circular_section.hh"
#include "geometry/line.hh"
#include "geometry/plane.hh"

#include <Eigen/Dense>

namespace sonder {

// A wonky coordinate system
void draw_coordinate_system();

//
// This is kinda cool, but isn't ~~quite~~ useful yet
//
void draw_coordinate_system(const Eigen::Vector3f &position, const Eigen::Matrix3f &orientation);

void draw_line(const Eigen::Vector3f &from, const Eigen::Vector3f &to);
void draw_line2d(const Eigen::Vector2f &from, const Eigen::Vector2f &to);

void draw_point(const Eigen::Vector3f &point, const float radius = 1.0);
void draw_point2d(const Eigen::Vector2f &point, const float radius = 1.0);

// Draw a dotted circle
//
// @param center The center of the circle
// @param normal The normal vector for the plane in which the circle lies
// @param radius The radius of the circle
//
// (Solid circle is a one-line change, this just looks cooler)
void draw_circle(const Eigen::Vector3f &normal, const Eigen::Vector3f &center, const float radius);

void draw_circle(const circle &ge_circle);

void draw_circular_section(const circular_section &section);

void draw_line(const line &ge_line);

void draw_plane(const plane &ge_plane);

void draw_cube();

void draw_sonar_view(const se3 &pose, const float max_bearing, const float max_elevation);
}
