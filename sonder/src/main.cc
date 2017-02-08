#include <math.h>  // Math library (Higher math functions )
#include <iostream>
#include <map>

#include <Eigen/Dense>
#include <sonder/opengl.hh>

#include <sonder/simulation/rendering.hh>

#include <sonder/geometry.hh>
#include <sonder/geometry/circle.hh>
#include <sonder/geometry/line.hh>
#include <sonder/geometry/plane.hh>

#include <sonder/gl_shapes.hh>
#include <sonder/types.hh>

#include <sophus/se3.hpp>

enum ManipulationMode : int16_t {
  kNORMAL,  // Manipulate the camera view
  kSONAR    // Manipulate the sonar view
};

struct SonarParams {
  float max_bearing   = 1.1f;  // Max bearing for the sonar view
  float max_elevation = 0.4f;  // Max elevation for the sonar view
};

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Lighting configuration
  //

  GLfloat light_ambient[4]  = {0.2f, 0.2f, 0.2f, 1.0f};
  GLfloat light_diffuse[4]  = {0.5f, 0.5f, 0.5f, 1.0f};
  GLfloat light_position[4] = {5.0f, 5.0f, -10.0f, 1.0f};
  GLfloat mat_specular[4]   = {0.2f, 0.2f, 0.2f, 1.0f};

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // View configuration
  //
  bool use_orthographic_projection = false;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Camera pose
  //

  // Camera position management
  float velocity_scaling         = 0.01f;
  float angular_velocity_scaling = 0.002f;

  float           view_velocity_decay = 0.9f;
  Eigen::Vector3f view_velocity       = Eigen::Vector3f::Zero();

  // Camera orientation management
  float           view_rotation_decay   = 0.9f;
  Eigen::Vector3f view_angular_velocity = Eigen::Vector3f::Zero();
  se3             view_pose             = se3(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Sonar state
  //

  se3         sonar_pose = se3(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
  SonarParams sonar_params;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Belief state
  //
  std::vector<sonder::circular_section> sections;
  EigStdVector<Eigen::Vector3f>         intersection_estimates;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // View State
  //

  // Fixed width/height
  int width = 800;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Input state
  //

  // Store which object we are manipulating
  ManipulationMode manipulation_mode = ManipulationMode::kNORMAL;

  // This list is also the precedence ordering for single-input commands
  bool left_mouse_held   = false;
  bool right_mouse_held  = false;
  bool scroll_mouse_held = false;

  // Store mouse state to compute relative motion
  Eigen::Vector2f mouse_down_screen_pos    = Eigen::Vector2f(0.0f, 0.0f);
  Eigen::Vector2f mouse_current_screen_pos = Eigen::Vector2f(0.0f, 0.0f);

  // Store pose states to apply relative motions
  se3 view_pose_at_start  = se3(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
  se3 sonar_pose_at_start = se3(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
  se3 last_capture_pose   = se3(Eigen::Matrix3f::Random(), Eigen::Vector3f::Random());

  // Keys
  std::map<unsigned char, bool> held_keys;
  std::map<int, bool>           held_specials;
};

// Instantiate program global state
State gstate;

// Step forward the camera physics
void view_physics() {
  //
  // view_Orientation decay
  //
  // Build a rotation matrix from that
  gstate.view_pose.so3() = so3::exp(gstate.view_angular_velocity) * gstate.view_pose.so3();

  // Apply decay
  gstate.view_angular_velocity = gstate.view_rotation_decay * gstate.view_angular_velocity;

  //
  // Translation decay
  //
  gstate.view_pose.translation() += gstate.view_velocity;
  gstate.view_velocity *= gstate.view_velocity_decay;
}

// Trigger a draw event, step forward camera physics
static void timer_event(const int te) {
  glutTimerFunc(10, timer_event, 1);
  view_physics();

  Eigen::Vector3f view_acceleration(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f view_angular_acceleration(0.0f, 0.0f, 0.0f);

  //
  // Handle normal keys that are currently held down
  //
  for (const auto& key_el : gstate.held_keys) {
    if (!key_el.second) {
      // Skip if key not held
      continue;
    }
    switch (key_el.first) {
      case 27:
        glutDestroyWindow(glutGetWindow());
        return;

      case 'w':
        view_acceleration += Eigen::Vector3f(0.0f, 0.0f, gstate.velocity_scaling);
        break;

      case 'a':
        view_acceleration += Eigen::Vector3f(gstate.velocity_scaling, 0.0f, 0.0f);
        break;

      case 's':
        view_acceleration += Eigen::Vector3f(0.0f, 0.0f, -gstate.velocity_scaling);
        break;

      case 'd':
        view_acceleration += Eigen::Vector3f(-gstate.velocity_scaling, 0.0f, 0.0f);
        break;

      case 'c':
        view_acceleration += Eigen::Vector3f(0.0f, -gstate.velocity_scaling, 0.0f);
        break;

      case 'z':
        view_acceleration += Eigen::Vector3f(0.0f, gstate.velocity_scaling, 0.0f);
        break;

      case 'q':
        view_angular_acceleration += -Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        break;

      case 'e':
        view_angular_acceleration += Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        break;

      default:
        break;
    }
  }

  // Transform "thrust" from view frame to world frame
  // (Must force evaluation)
  const Eigen::Vector3f delta = gstate.view_pose.so3().inverse() * view_acceleration;
  gstate.view_velocity += delta;

  //
  // Handle special keys that are currently held down
  //
  for (const auto& special_key_el : gstate.held_specials) {
    if (!special_key_el.second) {
      // Skip if key not held
      continue;
    }

    switch (special_key_el.first) {
      case GLUT_KEY_LEFT: {
        view_angular_acceleration += -Eigen::Vector3f(0.0f, 1.0f, 0.0f);
      } break;

      case GLUT_KEY_RIGHT: {
        view_angular_acceleration += Eigen::Vector3f(0.0f, 1.0f, 0.0f);

      } break;

      case GLUT_KEY_UP: {
        view_angular_acceleration += Eigen::Vector3f(1.0f, 0.0f, 0.0f);

      } break;

      case GLUT_KEY_DOWN: {
        view_angular_acceleration += -Eigen::Vector3f(1.0f, 0.0f, 0.0f);

      } break;

      default:
        break;
    }
  }
  gstate.view_angular_velocity += gstate.angular_velocity_scaling * view_angular_acceleration;

  glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////

// Setup our Opengl world, called once at startup.
void default_init() {
  // When screen cleared, use black.
  glClearColor(0.2f, 0.2f, 0.2f, 0.0f);

  // How the object color will be rendered smooth or flat
  glShadeModel(GL_SMOOTH);

  // Check depth when rendering
  glEnable(GL_DEPTH_TEST);

  // Lighting is added to scene
  glLightfv(GL_LIGHT1, GL_AMBIENT, gstate.light_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, gstate.light_diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, gstate.light_position);

  // Turn on lighting
  glEnable(GL_LIGHTING);

  // Turn on light 1
  glEnable(GL_LIGHT1);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Callbacks
/////////////////////////////////////////////////////////////////////////////////////////////////

void keyboard(unsigned char key, int x, int y, bool held) {
  //
  // Handle single-touch input keys (No special behavior if held down)
  //
  if (held) {
    switch (key) {
      case 'p':
        std::cout << "Switching projection mode" << std::endl;
        gstate.use_orthographic_projection = !gstate.use_orthographic_projection;
        break;

      case 'P':
        std::cout << "Pose::" << std::endl;
        std::cout << gstate.view_pose.translation().transpose() << std::endl;
        std::cout << gstate.view_pose.rotationMatrix() << std::endl;

      case 'v':
        if (gstate.manipulation_mode == ManipulationMode::kNORMAL) {
          gstate.manipulation_mode = ManipulationMode::kSONAR;
          std::cout << "Switching mode to sonar" << std::endl;
        } else {
          gstate.manipulation_mode = ManipulationMode::kNORMAL;
          std::cout << "Switching mode to normal" << std::endl;
        }
        break;

      case 'G':
        std::cout << "Clearing observation history" << std::endl;
        gstate.sections.clear();
        gstate.intersection_estimates.clear();
        break;

      case 'Q':
        std::cout << "Attempting to exit" << std::endl;
        glutDestroyWindow(glutGetWindow());
        return;
    }
  }
  gstate.held_keys[key] = held;
  glutPostRedisplay();
}
void keyboard_down(unsigned char key, int x, int y) {
  keyboard(key, x, y, true);
}
void keyboard_up(unsigned char key, int x, int y) {
  keyboard(key, x, y, false);
}

void process_special_keys(const int key, const int x, const int y, bool held) {
  gstate.held_specials[key] = held;
}
void special_keys_down(const int key, const int x, const int y) {
  process_special_keys(key, x, y, true);
}
void special_keys_up(const int key, const int x, const int y) {
  process_special_keys(key, x, y, false);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Mouse motion callbacks
/////////////////////////////////////////////////////////////////////////////////////////////////

void held_mouse_motion(const int x, const int y) {
  const Eigen::Vector2f pos       = Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y)) / gstate.width;
  gstate.mouse_current_screen_pos = pos;

  const Eigen::Vector2f relative_motion = pos - gstate.mouse_down_screen_pos;

  if (gstate.right_mouse_held) {
    //
    // Right mouse drag
    //
    // Locally perturb something by treating mouse motion as a left-tangent scaling

    const so3::Tangent w(relative_motion.y(), relative_motion.x(), 0.0f);
    const auto         perturbation = so3::exp(w / 1.0f);

    //
    // Manipulate orientation of the selected object (View or sonar head)
    //
    if (gstate.manipulation_mode == ManipulationMode::kNORMAL) {
      gstate.view_pose.so3() = perturbation * gstate.view_pose_at_start.so3();

    } else if (gstate.manipulation_mode == ManipulationMode::kSONAR) {
      // Transform the perturbation so that it appears to happen relative to our own view
      const so3 view_orientation                  = gstate.view_pose.so3();
      const so3 perturbation_apparent_local_frame = view_orientation.inverse() * perturbation * view_orientation;

      gstate.sonar_pose.so3() = perturbation_apparent_local_frame * gstate.sonar_pose_at_start.so3();
    }

  } else if (gstate.left_mouse_held) {
    //
    // Left mouse drag
    //
    // Second in precedence ordering

    // Get movement in screen coordinates
    const Eigen::Vector3f expressed_motion(relative_motion.x(), -relative_motion.y(), 0.0f);

    // Transform screen motion into world motion
    const Eigen::Vector3f delta = gstate.view_pose.rotationMatrix().transpose() * (5.0 * expressed_motion);

    //
    // Manipulate translation of the selected object (View or sonar head)
    //
    if (gstate.manipulation_mode == ManipulationMode::kNORMAL) {
      gstate.view_pose.translation() = delta + gstate.view_pose_at_start.translation();

    } else if (gstate.manipulation_mode == ManipulationMode::kSONAR) {
      gstate.sonar_pose.translation() = delta + gstate.sonar_pose_at_start.translation();
    }
  }

  glutPostRedisplay();
}

void update_mouse_state(const int button, const bool held) {
  switch (button) {
    case 0:
      gstate.left_mouse_held = held;
      break;
    case 1:
      gstate.scroll_mouse_held = held;
      break;
    case 2:
      gstate.right_mouse_held = held;
      break;
    default:
      break;
  }
}

void mouse(const int button, const int state, const int x, const int y) {
  const Eigen::Vector2f pos = Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y)) / gstate.width;
  update_mouse_state(button, state == GLUT_DOWN);

  //
  // Handle storing state for a mouse press
  //
  if (state == GLUT_DOWN) {
    gstate.mouse_down_screen_pos    = pos;
    gstate.mouse_current_screen_pos = pos;

    gstate.view_pose_at_start  = gstate.view_pose;
    gstate.sonar_pose_at_start = gstate.sonar_pose;

  } else if (state == GLUT_UP) {
  }

  //
  // Handle scroll-wheel events
  //
  if ((button == 3) || (button == 4)) {
    if (state == GLUT_UP) {
      return;
    }

    // The direction is governed which direction the wheel is being turned
    const float direction = (button == 3) ? 1.0f : -1.0f;

    // Project the motion in screen coordinates into world coordinates
    const Eigen::Vector3f delta    = gstate.view_pose.so3().inverse() * Eigen::Vector3f::UnitZ() * direction;
    gstate.view_pose.translation() = gstate.view_pose.translation() + delta;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Generic geometry
/////////////////////////////////////////////////////////////////////////////////////////////////

EigStdVector<Eigen::Vector3f> intersect_all_sections(const sonder::circular_section&              section,
                                                     const std::vector<sonder::circular_section>& other_sections) {
  constexpr float MIN_DIST_TO_ADD = 1e-3;

  EigStdVector<Eigen::Vector3f> all_intersections;
  all_intersections.reserve(10);

  //
  // Seek an intersection among `other_sections`
  //
  for (const auto& other_section : other_sections) {
    const auto intersections = section.intersect(other_section);
    for (const auto& pt : intersections) {
      //
      // Don't add a point if it is <1mm away from an existing point
      //
      // todo(jpanikulam): Factor into a new function
      bool add = true;
      for (const auto& other_pt : all_intersections) {
        if ((pt - other_pt).norm() < MIN_DIST_TO_ADD) {
          add = false;
        }
      }
      // If this point is truly unique, we add it
      if (add) {
        all_intersections.push_back(pt);
      }
    }
  }
  return all_intersections;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Display
/////////////////////////////////////////////////////////////////////////////////////////////////

void draw_sonar_data() {
  //
  // Rendering a field of points
  //

  // Minimum constants for updating the belief
  constexpr float MIN_TRANSLATION = 0.5f;
  constexpr float MIN_ROTATION    = 0.4f;

  {
    const EigStdVector<Eigen::Vector3f> plane         = sonder::sim::make_plane_blanket();
    const EigStdVector<Eigen::Vector2f> range_bearing = sonder::sim::to_range_bearing(
        plane, gstate.sonar_pose, gstate.sonar_params.max_bearing, gstate.sonar_params.max_elevation);

    if (false) {
      for (const auto& pt : plane) {
        sonder::draw_point(pt, 0.1f);
      }
    }

    if (true) {
      for (const auto& pt : gstate.intersection_estimates) {
        glColor3f(0.0, 0.4, 0.8);
        sonder::draw_point(pt, 0.05f);
      }
    }

    const float delta_position = (gstate.last_capture_pose.translation() - gstate.sonar_pose.translation()).norm();
    const float delta_orientation =
        (so3::log(gstate.last_capture_pose.so3() * gstate.sonar_pose.so3().inverse())).norm();

    if ((delta_position > MIN_TRANSLATION) || (delta_orientation > MIN_ROTATION)) {
      gstate.last_capture_pose = gstate.sonar_pose;

      for (const auto& rb : range_bearing) {
        Eigen::AngleAxisf rotation(rb.y(), Eigen::Vector3f::UnitZ());

        const Eigen::Vector3f normal =
            gstate.sonar_pose.rotationMatrix() * rotation.toRotationMatrix() * Eigen::Vector3f::UnitY();
        const Eigen::Vector3f direction =
            gstate.sonar_pose.rotationMatrix() * rotation.toRotationMatrix() * Eigen::Vector3f::UnitX();

        const sonder::circular_section circ_sec(direction, normal, gstate.sonar_pose.translation(), rb.x(),
                                                2.0f * gstate.sonar_params.max_elevation);

        gstate.sections.push_back(circ_sec);

        // --> TODO: Try std::moveing this
        const EigStdVector<Eigen::Vector3f> intersections = intersect_all_sections(circ_sec, gstate.sections);
        if (intersections.size() > 0) {
          for (const auto& pt : intersections) {
            gstate.intersection_estimates.push_back(pt);
          }
        }
      }
    }
  }

  glColor3f(0.0f, 0.7f, 0.1f);
  for (const auto& circ_sec : gstate.sections) {
    sonder::draw_circular_section(circ_sec);
  }

  glColor3f(0.9f, 0.4f, 0.2f);
  for (const auto& circ_sec : gstate.sections) {
    sonder::draw_circle(circ_sec.spanning_circle);
  }

  //
  // Render the sonar viewing frustum
  //
  // This is not the real limits of the sonar view, but makes a reasonable approximation
  sonder::draw_sonar_view(gstate.sonar_pose, gstate.sonar_params.max_bearing, gstate.sonar_params.max_elevation);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (gstate.use_orthographic_projection) {
    glOrtho(-16.0f, 16.0f, -16.0f, 16.0f, 0.0f, 30.0f);
  } else {
    gluPerspective(60.0f, 1.0f, 1.0f, 1000.0f);
  }

  //
  // Set up the current view
  //
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  Eigen::Quaternionf q(gstate.view_pose.unit_quaternion());
  glRotate(q);
  glTranslate(gstate.view_pose.translation());

  //
  // Enable lighting & material settings
  //
  if (true) {
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

  } else {
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT);
    glColor4f(0.65f, 0.65f, 0.65f, 0.4f);
    glColorMaterial(GL_FRONT, GL_EMISSION);
    glColor4f(0.10f, 0.10f, 0.10f, 0.0f);
    glColorMaterial(GL_FRONT, GL_SPECULAR);
    glColor4f(0.5f, 0.5f, 0.5f, 0.4f);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glColor4f(0.85f, 0.85f, 0.85f, 0.4f);
  }

  //
  // Draw the universal origin
  //
  sonder::draw_coordinate_system();

  //
  // Draw the sonar and sonar related nonsense
  //
  draw_sonar_data();

  // Draw on the "HUD" as it were, with a second mvp setup
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    sonder::draw_line2d(gstate.mouse_down_screen_pos, gstate.mouse_current_screen_pos);
  }

  glutSwapBuffers();
}

void reshape(int w, int h) {
  int size = std::max(w, h);
  glViewport(0, 0, (GLsizei)size, (GLsizei)size);
  glutReshapeWindow(size, size);

  gstate.width = size;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
}

int main(int argc, char** argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  // Square
  glutInitWindowSize(gstate.width, gstate.width);
  glutInitWindowPosition(10, 10);
  glutTimerFunc(10, timer_event, 1);

  int discrete_belief_view = glutCreateWindow(argv[0]);
  glutSetWindowTitle("Sonder -- Discrete Belief View");
  default_init();

  //
  // Standard GLUT functions
  //
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);

  //
  // Input
  //
  glutKeyboardFunc(keyboard_down);
  glutKeyboardUpFunc(keyboard_up);
  glutSpecialFunc(special_keys_down);
  glutSpecialUpFunc(special_keys_up);
  glutMouseFunc(mouse);

  // Active motion
  glutMotionFunc(held_mouse_motion);

  //
  // View Initialization
  //
  Eigen::Matrix3f view_orientation;
  view_orientation << 0.978117, 0.207935, -0.00695645, 0.006618, 0.00232671, 0.999976, 0.207946, -0.978139, 0.000898672;
  Eigen::Vector3f view_position(-1.40469, 6.51158, -0.350347);

  gstate.view_pose = se3(view_orientation, view_position);

  std::cout << "Starting viewer" << std::endl;
  glutMainLoop();
  std::cout << "Ending main loop" << std::endl;
  return 0;
}