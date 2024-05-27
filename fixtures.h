#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "types.h"

// stl
#include <variant>

namespace dte3607::physengine::fixtures
{


  struct FixtureLevel1 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;


    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const { return m_spheres.size(); }
    void   setGravity([[maybe_unused]] Forces G) {m_gravity = G;}

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return m_spheres.at(rid).translation;

     // return {};
    }

    /*** Fixture unit-test setup API ***/
    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      m_spheres.push_back({radius, velocity, translation});
      return m_spheres.size();
     // return {};
    }

    /*** END API requirements ***/

    struct Sphere {
      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
    };

    std::vector<Sphere>m_spheres;
    Forces m_gravity;

  };



  struct FixtureLevel2 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;


    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces G ) {m_gravity = G;}
    const Forces &getGravity () const {return m_gravity;}

    // RBs
    size_t              noRigidBodies() const { return m_rigid_bodies.size(); }
    std::vector<size_t> nonFixedSphereRBs() const { return m_sphere_indexes; }
    std::vector<size_t> fixedInfPlaneRBs() const { return m_plane_indexes; }

    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const {
      return m_rigid_bodies.at(s_rid).radius;
    }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const {
      return  m_rigid_bodies.at(p_rid).normal;
    }

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return  m_rigid_bodies.at(rid).translation;
    }

    void setglobalFramePosition(size_t rid, const Vector3& pos){
      m_rigid_bodies[rid].translation = pos;
    }

    types::Vector3 rbSphereVelocity([[maybe_unused]] size_t rid) const{
      return m_rigid_bodies.at(rid).velocity;
    }

    void setRbSphereVelocity(size_t rid, const Vector3& vel){
      m_rigid_bodies[rid].velocity = vel;
    }

    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({radius,velocity,translation, {}});
      m_sphere_indexes.push_back(m_rigid_bodies.size() - 1);
      return m_rigid_bodies.size() - 1;
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({{}, {}, normal, translation});
      m_plane_indexes.push_back(m_rigid_bodies.size() - 1);
      return m_rigid_bodies.size() - 1;
    }

    /*** END API requirements ***/

    struct RigidBody {

      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
      Vector3 normal;
    };
    std::vector<RigidBody>  m_rigid_bodies;
    std::vector<size_t>     m_sphere_indexes;
    std::vector<size_t>     m_plane_indexes;

    Forces m_gravity;
  };


  struct FixtureLevel3 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;


    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces G ) {m_gravity = G;}
    //const Forces &getGravity () const {return m_gravity;}

    ValueType rbMaxFrictionCoef(size_t rid) const { return m_rigid_bodies.at(rid).firction_coef; }
    ValueType rbPlaneMaxFrictionCoef(size_t pid) const { return m_rigid_bodies.at(pid).firction_coef; }
    ValueType rbSphereMaxFrictionCoef(size_t sid) const { return m_rigid_bodies.at(sid).firction_coef; }

    RBState rbState([[maybe_unused]] size_t rid) const { return m_rigid_bodies.at(rid).state; }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0},
                        [[maybe_unused]] RBState initial_state = RBState::Free,
                        [[maybe_unused]] ValueType friction_coef = 0.,
                        [[maybe_unused]] ValueType mass          = 1.)
    {
      m_rigid_bodies.push_back({radius,velocity,translation, {}, initial_state, friction_coef, mass});
      m_sphere_indexes.push_back(m_rigid_bodies.size() - 1);
      return m_rigid_bodies.size() - 1;

    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0},
                               [[maybe_unused]] ValueType friction_coef = 0.)
    {
      m_rigid_bodies.push_back({{}, {}, normal, translation, {}, friction_coef, {}});
      m_plane_indexes.push_back(m_rigid_bodies.size() - 1);
      return m_rigid_bodies.size() - 1;
    }

    /*** END API requirements ***/

    struct RigidBody {

      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
      Vector3 normal;
      RBState state;
      ValueType firction_coef;
      ValueType mass;
    };

    std::vector<RigidBody>  m_rigid_bodies;
    std::vector<size_t>     m_sphere_indexes;
    std::vector<size_t>     m_plane_indexes;

    Forces m_gravity;

    ValueType const max_p_friction = {0.9};
    ValueType const max_s_friction = {0.9};
  };


  struct FixtureLevel4 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;

    /*** END API requirements ***/

  };


}   // namespace dte3607::physengine::fixtures


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
