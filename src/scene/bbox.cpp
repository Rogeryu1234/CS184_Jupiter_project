#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  double new_t0 = -INF_D;
  double new_t1 = INF_D;

  // Check each XYZ plane of bbox
  for (int i = 0; i < 3; ++i) {
    double inv_ray_dir = 1.0 / r.d[i];
    double t_near = (min[i] - r.o[i]) * inv_ray_dir;
    double t_far = (max[i] - r.o[i]) * inv_ray_dir;

    // Ensure t_near is less than t_far
    if (t_near > t_far) std::swap(t_near, t_far);

    new_t0 = std::max(new_t0, t_near);
    new_t1 = std::min(new_t1, t_far);

    // If the interval becomes empty, return false
    if (new_t0 > new_t1) return false;
  }

  // check if the intersect points are within ray's range (note that if ray starts or ends inside bbox it is still an intersection
  if (new_t0 > r.max_t || new_t1 < r.min_t) {
      return false;
  }

  t0 = new_t0;
  t1 = new_t1;
  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
