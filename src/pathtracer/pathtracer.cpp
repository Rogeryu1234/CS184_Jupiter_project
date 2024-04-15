#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "vector3D.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  
  // sample incoming light uniformly over a hemisphere about hit_p
  Vector3D sum_vec = Vector3D(0.0);
  for (int i = 0; i < num_samples; ++i) {
      // get a uniform sample (in object-space)
      Vector3D hemi_samp = hemisphereSampler->get_sample();

      // check if this sample's direction from hit_p intersects a light source
      Intersection light_intersect = Intersection();
      Ray ray_samp = Ray(hit_p, (o2w * hemi_samp));  // convert to world-space
      ray_samp.min_t = EPS_F;  // prevent ray from intersecting the surface it came from
      
      bool has_intersection = bvh->intersect(ray_samp, &light_intersect);
      if (has_intersection) {
          // this sample intersects with a primitive, check if it is a light source
          // retrieve BSDF
          BSDF* intersect_bsdf = light_intersect.bsdf;
          if (intersect_bsdf->get_emission() == Vector3D()) {
              // no emission, so it's not a light source, skip this sample
              continue;
          }

          // hit a light source!
          Vector3D brdf = isect.bsdf->f(w_out.unit(), -hemi_samp.unit());  // pass in object-space vectors
          double pdf = 1.0 / (2 * PI);  // reflected light can go in any full circular direction, so 360 degrees

          Vector3D radiance = intersect_bsdf->get_emission();
          sum_vec += brdf * radiance * cos_theta(hemi_samp) / pdf;
      }
  }

  L_out = sum_vec / (double)num_samples;
  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  
  // go over each light source
  for (auto light : scene->lights) {
      int num_samples = 0;
      Vector3D sum_vec = Vector3D(0.0);

      // check if this is a point light source
      bool point_light = light->is_delta_light();
      if (point_light) {
          num_samples = 1;
      }
      else {
          num_samples = ns_area_light;
      }

      for (int i = 0; i < num_samples; ++i) {
          // sample this light source only once
          Vector3D samp_dir = Vector3D();  // world-space unit vector, sampled direction between hit_p and light source
          double dist_to_light;
          double pdf;  // pdf evaluated at samp_dir direction
          Vector3D radiance = light->sample_L(hit_p, &samp_dir, &dist_to_light, &pdf);

          // check if light is behind the surface at the hit point
          if ((w2o * samp_dir).z < 0.0) {
              continue;
          }

          // check if samp_dir intersects with obstacle
          Intersection obstacle_inter = Intersection();
          Ray ray_samp = Ray(hit_p, samp_dir);
          ray_samp.min_t = EPS_F;
          ray_samp.max_t = dist_to_light - EPS_F;
          bool has_intersection = bvh->intersect(ray_samp, &obstacle_inter);

          if (!has_intersection) {
              // no obstacle intersection, count this light!
              Vector3D brdf = isect.bsdf->f(w_out.unit(), -(w2o * samp_dir).unit());  // pass in object-space vectors
              sum_vec += brdf * radiance * cos_theta(w2o * samp_dir) / pdf;  // convert to object-space
          }
      }
      L_out += sum_vec / (double)num_samples;
  }

  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.

  /*
  call the one_bounce_radiance function, and then recursively call itself to 
  estimate the higher bounces. This recursive call should take one random sample of a 
  direction based on the BSDF at the hit point, trace a ray in that sample direction, 
  and recursively call itself on the new hit point.
  */

  /*
  If isAccumBounces is set to true (default), it will accumulate all light along 
  the path up to max_ray_depth. If it is set to false, then we want to only return the 
  light at the max_ray_depth bounce
  */

  if (r.depth == 0) {
      // 0 bounce
      return L_out;
  }

  L_out = one_bounce_radiance(r, isect);
  
  if (r.depth == 1) {
      return L_out;
  }
  
  // sample a direction based on the BSDF at the hit point
  Vector3D w_in;  // object-space
  double pdf;
  Vector3D reflectance = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  
  // trace a ray in this direction
  Intersection new_intersect = Intersection();
  Ray ray_samp = Ray(hit_p, o2w * w_in);
  ray_samp.min_t = EPS_F;
  ray_samp.depth = r.depth - 1;
  bool has_intersection = bvh->intersect(ray_samp, &new_intersect);

  if (has_intersection) {
      // RUSSIAN ROULETTE
      if (r.depth < max_ray_depth || max_ray_depth <= 1) {
          bool to_terminate = coin_flip(0.35);
          if (to_terminate) {
              // stop recursion
              return L_out;
          }
      }
      
      // indirect bounces
      // recursively call this function on the new hit point
      if (isAccumBounces) {
          // accumulate all light along the path up to max_ray_depth
          L_out += at_least_one_bounce_radiance(ray_samp, new_intersect) * reflectance * w_in.z / pdf;
      }
      else {
          // only return the light at the max_ray_depth bounce
          L_out = at_least_one_bounce_radiance(ray_samp, new_intersect) * reflectance * w_in.z / pdf;
      }
  }
  else {
      L_out = Vector3D(0.0);
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  //L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  if (isAccumBounces) {
      L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
      //L_out = at_least_one_bounce_radiance(r, isect);
  }
  else {
      if (r.depth == 0) {
          L_out = zero_bounce_radiance(r, isect);
      }
      else {
          L_out = at_least_one_bounce_radiance(r, isect);
      }
  }

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int num_samples = ns_aa;          // total samples to evaluate
  // Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  double s1 = 0;
  double s2 = 0;

  // Initialize Monte Carlo Estimator to be 0;
  Vector3D MonteCarlo(0,0,0);
  int n;
  for (n = 0; n<num_samples; n++) {
    // Check if the number of sample is a multiple of samplesPerBatch,
    // if so, check for convergence.
    if (n % samplesPerBatch == 0 && n != 0) {
      double mean = s1/double(n);
      double var = (s2 - (s1*s1)/double(n))/(n - 1.0);
      if (1.96*sqrt(var/double(n)) <= maxTolerance*mean)
      {
        // if converge, break, and report number of samples used;
        break;
      }
    }

    // Generate random point;
    Vector2D randomSample = gridSampler->get_sample();
    // Create random ray from that normalized point;
    Ray r = camera->generate_ray((x + randomSample.x)/(double)sampleBuffer.w, (y + randomSample.y)/(double)sampleBuffer.h);
    r.depth = max_ray_depth;
    // Evaluate radiance and update the estimator;
    Vector3D tmp = est_radiance_global_illumination(r);
    MonteCarlo += tmp;
    // update s1 and s2;
    s1 += tmp.illum();
    s2 += tmp.illum() * tmp.illum();
  }
  // Update pixel;
  MonteCarlo = MonteCarlo/double(n);
  sampleBuffer.update_pixel(MonteCarlo, x, y);
  // Update sample rate;
  sampleCountBuffer[x + y * sampleBuffer.w] = n;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
