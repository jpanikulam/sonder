#include "visualization/gl_shapes.hh"
#include "visualization/opengl.hh"

#include "geometry/geometry.hh"

#include <Eigen/Dense>

namespace sonder {

void draw_coordinate_system() {
  draw_coordinate_system(Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity());
}

void draw_coordinate_system(const se3 &pose) {
  draw_coordinate_system(pose.translation(), pose.rotationMatrix());
}

void draw_coordinate_system(const Eigen::Vector3f &position, const Eigen::Matrix3f &orientation) {
  glPushMatrix();

  const Eigen::Quaternionf q(orientation);
  glTranslate(position);
  glRotate(q);

  constexpr float narrow_scale = 0.05;
  constexpr float wide_scale   = 0.5;
  // Z
  glPushMatrix();
  glScalef(narrow_scale, narrow_scale, wide_scale);
  glColor3f(0.0, 0.0, 1.0);
  glTranslatef(0.0, 0.0, 1.0);
  draw_cube();
  glPopMatrix();

  // Y
  glPushMatrix();
  glScalef(narrow_scale, wide_scale, narrow_scale);
  glColor3f(0.0, 1.0, 0.0);
  glTranslatef(0.0, 1.0, 0.0);
  draw_cube();
  glPopMatrix();

  // X
  glPushMatrix();
  glScalef(wide_scale, narrow_scale, narrow_scale);
  glColor3f(1.0, 0.0, 0.0);
  glTranslatef(1.0, 0.0, 0.0);
  draw_cube();
  glPopMatrix();

  glPopMatrix();
}

void draw_line(const Eigen::Vector3f &from, const Eigen::Vector3f &to) {
  glBegin(GL_LINES);
  {
    glVertex(from);
    glVertex(to);
  }
  glEnd();
}

void draw_line2d(const Eigen::Vector2f &from, const Eigen::Vector2f &to) {
  glBegin(GL_LINES);
  {
    glVertex2f(from.x(), 1.0 - from.y());
    glVertex2f(to.x(), 1.0 - to.y());
  }
  glEnd();
}

void draw_point2d(const Eigen::Vector2f &point, const float radius) {
  glPushMatrix();
  glTranslatef(point.x(), point.y(), -0.5f);
  glutSolidSphere(radius, 16, 16);
  glPopMatrix();
}

void draw_point(const Eigen::Vector3f &point, const float radius) {
  glPushMatrix();
  glTranslate(point);
  glutSolidSphere(radius, 16, 16);
  glPopMatrix();
}

void draw_circle(const Eigen::Vector3f &normal, const Eigen::Vector3f &center, const float radius) {
  constexpr int num_vertices = 100;

  //
  // Build the circle in 2D on the x-y plane
  //
  Eigen::Array<float, 3, num_vertices> xy_plane_vertices;
  for (int k = 0; k < num_vertices; ++k) {
    const float t            = 2 * M_PI * ((float)k / num_vertices);
    xy_plane_vertices.col(k) = Eigen::Vector3f(radius * std::sin(t), radius * std::cos(t), 0.0);
  }

  //
  // Transform the x-y plane circle to the normal plane
  //

  // A rotation between the Z axis and the circle normal
  Eigen::Quaternionf rotation = create_rotation_to(Eigen::Vector3f::UnitZ(), normal);

  const se3 transform(rotation, center);

  // Draw the lines (dotted)
  // ((IF SOLID: GL_LINE_STRIP))
  glBegin(GL_LINES);
  {
    for (int k = 0; k < num_vertices; ++k) {
      glVertex(transform * xy_plane_vertices.col(k));
    }
  }
  glEnd();
}

void draw_circle(const circle &ge_circle) {
  draw_circle(ge_circle.normal, ge_circle.center, ge_circle.radius);
}

void draw_circular_section(const circular_section &section) {
  constexpr int num_vertices = 100;

  //
  // Build the circle in 2D on the x-y plane
  //
  Eigen::Array<float, 3, num_vertices> xz_plane_vertices;
  const float angle_per_vertex = section.arc_rads / static_cast<float>(num_vertices);
  for (int k = 0; k < num_vertices; ++k) {
    const float angle = (-section.arc_rads * 0.5f) + k * angle_per_vertex;
    xz_plane_vertices.col(k) =
        Eigen::Vector3f(section.radius * std::cos(angle), 0.0f, section.radius * std::sin(angle));
  }

  //
  // Transform the x-z plane circle to the normal plane
  //
  Eigen::Quaternionf rotation = rotation_from_xy(section.direction, section.normal);
  const se3          transform(rotation, section.center);

  // Draw the lines (dotted)
  // ((IF SOLID: GL_LINE_STRIP))
  glBegin(GL_LINE_STRIP);
  {
    for (int k = 0; k < num_vertices; ++k) {
      glVertex(transform * xz_plane_vertices.col(k));
    }
  }
  glEnd();
}

void draw_line(const line &ge_line) {
  const Eigen::Vector3f scaled_direction = 10.0 * ge_line.direction;
  draw_line(ge_line.point + scaled_direction, ge_line.point - scaled_direction);
}

void draw_plane(const plane &ge_plane) {
  constexpr float       scale   = 5.0;
  const Eigen::Vector3f x_basis = any_perpendicular(ge_plane.normal).normalized();
  const Eigen::Vector3f y_basis = x_basis.cross(ge_plane.normal).normalized();

  glBegin(GL_QUADS);
  {
    glVertex((scale * x_basis) + ge_plane.point);
    glVertex((scale * y_basis) + ge_plane.point);
    glVertex(-(scale * x_basis) + ge_plane.point);
    glVertex(-(scale * y_basis) + ge_plane.point);
  }
  glEnd();
}

void draw_cube() {
  glBegin(GL_QUADS);
  {
    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);

    glVertex3f(-1.0, 1.0, -1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);

    glVertex3f(1.0, 1.0, -1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);

    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(-1.0, -1.0, 1.0);

    glVertex3f(-1.0, -1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);

    glVertex3f(-1.0, -1.0, -1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glVertex3f(-1.0, 1.0, -1.0);
  }
  glEnd();
}

//
// This isn't quite correct
//
void draw_sonar_view(const se3 &pose, const float max_bearing, const float max_elevation) {
  constexpr float range = 20.0f;

  const float x_sym = range * cos(max_bearing);
  const float y_sym = range * sin(max_bearing);
  const float z_sym = range * sin(max_elevation);

  const Eigen::Vector3f symmetric_point(x_sym, y_sym, z_sym);

  // draw_coordinate_system(pose.translation(), pose.rotationMatrix());
  draw_coordinate_system(pose);

  glPushMatrix();
  glTranslate(pose.translation());
  glRotate(pose.unit_quaternion());

  glColor3f(0.9f, 0.0f, 0.0f);
  glBegin(GL_LINE_STRIP);
  {
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, y_sym, z_sym);
    glVertex3f(x_sym, -y_sym, z_sym);

    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, y_sym, -z_sym);
    glVertex3f(x_sym, -y_sym, -z_sym);
    glVertex3f(0.0, 0.0, 0.0);

    glVertex3f(x_sym, -y_sym, -z_sym);
    glVertex3f(x_sym, -y_sym, z_sym);

    glVertex3f(x_sym, y_sym, z_sym);
    glVertex3f(x_sym, y_sym, -z_sym);
  }
  glEnd();

  glColor4f(0.2f, 0.7f, 0.0f, 0.2f);
  // This does some weird stuff, meh
  glBegin(GL_TRIANGLES);
  {
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, y_sym, z_sym);
    glVertex3f(x_sym, -y_sym, z_sym);

    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, -y_sym, -z_sym);
    glVertex3f(x_sym, y_sym, -z_sym);

    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, -y_sym, -z_sym);
    glVertex3f(x_sym, -y_sym, z_sym);

    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(x_sym, y_sym, z_sym);
    glVertex3f(x_sym, y_sym, -z_sym);
  }
  glEnd();
  glPopMatrix();
}
}