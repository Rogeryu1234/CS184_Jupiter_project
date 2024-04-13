#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include "scene/object.h"
#include "vector3D.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  
  // Compute the triangle's normal
  Vector3D N = cross(p2 - p1, p3 - p1).unit();
  
  // Compute the ray's direction and origin
  Vector3D D_vec = r.d;
  Vector3D O = r.o;

  // Compute intersection point between the ray and the plane of the triangle
  double t = dot(N, p1 - O) / dot(N, D_vec);
  
  // Check if the intersection point lies within the valid range
  if (t >= r.min_t && t <= r.max_t) {
    // Compute barycentric coordinates
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D s = O - p1;
    Vector3D s1 = cross(D_vec, e2);
    Vector3D s2 = cross(s, e1);
    double divisor = dot(s1, e1);
    double u = dot(s1, s) / divisor;
    double v = dot(s2, D_vec) / divisor;

    // Check if the intersection point lies within the triangle
    if (u >= 0 && v >= 0 && (u + v) <= 1 && u <= 1 && v <= 1 && (u + v) >= 0) {
      // Update max_t accordingly
      r.max_t = t;
      return true;
    }
  }
  return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  // Compute the triangle's normal
  Vector3D N = cross(p2 - p1, p3 - p1).unit();
  
  // Compute the ray's direction and origin
  Vector3D D_vec = r.d;
  Vector3D O = r.o;

  // Compute intersection point between the ray and the plane of the triangle
  double t = dot(N, p1 - O) / dot(N, D_vec);

  // Check if the intersection point lies within the valid range
  if (t >= r.min_t && t <= r.max_t) {
    // Compute barycentric coordinates
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D s = O - p1;
    Vector3D s1 = cross(D_vec, e2);
    Vector3D s2 = cross(s, e1);
    double divisor = dot(s1, e1);
    double u = dot(s1, s) / divisor;
    double v = dot(s2, D_vec) / divisor;

    // Check if the intersection point lies within the triangle
    if (u >= 0 && v >= 0 && (u + v) <= 1 && u <= 1 && v <= 1 && (u + v) >= 0) {
      // Update max_t accordingly
      r.max_t = t;

      // Populate intersection structure
      isect->t = t;
      isect->primitive = this;
      isect->bsdf = get_bsdf();

      // Interpolate the surface normal using barycentric coordinates
      Vector3D n = (1 - u - v) * n1 + u * n2 + v * n3;
      isect->n = n.unit();

      return true;
    }
  }

  return false;



}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
