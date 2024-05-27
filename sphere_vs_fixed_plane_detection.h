#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{
  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc, // sphere time position in timestep
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_ds,
    [[maybe_unused]] types::Point3 const&           fplane_q,  // plane pos
    [[maybe_unused]] types::Vector3 const&          fplane_n,
   
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  
  {
    auto const epsilon = 10e-5;
    
    auto const n = blaze::normalize(fplane_n);
    auto const d = (fplane_q + sphere_r * n) - sphere_p;
    auto const R = blaze::inner(sphere_ds, n); // * fplane_n;
    auto const Q = blaze::inner(d, n); // * fplane_n;
    
             // Check singularities first
    if (std::abs(R) < epsilon) {
      return std::nullopt; // Sphere moves in parallel to the plane
    }
    
    if (std::abs(Q) < epsilon) {
      return std::nullopt; // Sphere is touching the plane
    }
    
             // Condition for collision: xR = Q ===> x = Q/R
    auto const x = Q/R;
    
             // Check if collision is in the past or in the future
    if (x <= 0 or x > 1) {
      return std::nullopt;
    }
    
             // find collision position in time (this does take sphere_tc into account)
    auto const tp = sphere_tc + utils::toDuration(x * (timestep - (sphere_tc - t_0)));
    return tp;
  }
  




  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
    
    // compute ds by the computeTrajectory
    auto const [ds, a] = computeLinearTrajectory(sphere_v, external_forces, timestep - (sphere_tc - t_0));

    return detectCollisionSphereFixedPlane(sphere_tc, sphere_p, sphere_r, ds, fplane_q, fplane_n, t_0, timestep);

    /*auto const epsilon = 10e-5;

             //Compute ds by the compute trajectory
    auto const [ds,a] = computeLinearTrajectory(sphere_v, external_forces, timestep-(sphere_tc-t_0));

    auto const n = blaze::normalize(fplane_n);
    auto const d = (fplane_q + sphere_r * n) - sphere_p;

    auto const Q = blaze::inner(d, n);
    auto const R = blaze::inner(ds, n);

             //Check Singularities first
    if (std::abs(R)<epsilon)
      return std::nullopt;//Sphere moves in parrallel to the plane


    if (std::abs(Q)<epsilon)
      return std::nullopt;//Sphere touches the plane

             //Condition for collition xR=Q;
    auto const x = Q/R;

    if (x <= 0 or x>1)
      return std::nullopt;

             //find collision position in time (this does takes sphere_tc into account)
    auto const tp = t_0 + utils::toDuration(x * timestep - (sphere_tc - t_0));
    return tp; */

  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
