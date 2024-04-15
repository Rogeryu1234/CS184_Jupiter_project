#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  
  // Compute coefficients of the quadratic equation
  Vector3D L = r.o - o;
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.d, L);
  double c = dot(L, L) - r2;

  // Compute discriminant
  double disc = b * b - 4 * a * c;

  // If the discriminant is negative, there are no real roots
  if (disc < 0) return false;

  // Compute the two possible solutions for t
  t1 = (-b + sqrt(disc)) / (2 * a);
  t2 = (-b - sqrt(disc)) / (2 * a);

  // Ensure t1 is the smaller of the two intersection times
  if (t1 > t2) std::swap(t1, t2);

  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if (!test(r, t1, t2)) {
      return false;
  }

  // we found an intersection, test if it is in range
  bool t1_valid = false;
  bool t2_valid = false;
  if (t1 >= r.min_t && t1 <= r.max_t) {
      // t1 is valid
      t1_valid = true;
  }
  if (t2 >= r.min_t && t2 <= r.max_t) {
      // t2 is valid
      t2_valid = true;
  }

  if (t1_valid && t2_valid) {
      // find the closer one
      if (t1 < t2) {
          // t1 is closer
          r.max_t = t1;
          return true;
      }
      else {
          // t2 is closer
          r.max_t = t2;
          return true;
      }
  }
  else if (t1_valid) {
      // only t1 is valid
      r.max_t = t1;
      return true;
  }
  else if (t2_valid) {
      // only t2 is valid
      r.max_t = t2;
      return true;
  }

  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  if (has_intersection(r)) {
    // intersects!
    // Update intersection structure
    i->t = r.max_t;
    i->primitive = this;
    i->bsdf = get_bsdf();
    i->n = (r.o + r.max_t * r.d - o).unit();
    return true;
  }
  return false;

  //double t1, t2;
  //if (test(r, t1, t2) && (t1 >= r.min_t && t1 <= r.max_t)) {
  //  // Update max_t accordingly
  //  r.max_t = t1;
  //  
  //  // Update intersection structure
  //  i->t = t1;
  //  i->primitive = this;
  //  i->bsdf = get_bsdf();
  //  i->n = (r.o + t1 * r.d - o).unit();
  //  return true;
  //}
  //return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
