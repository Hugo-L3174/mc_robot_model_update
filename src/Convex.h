/*
 * Copyright 2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/Visual.h>
#include <mc_rtc/gui/Polyhedron.h>
#include <SpaceVecAlg/PTransform.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "mc_rtc/gui/types.h"
#include <memory>
#include <sch/S_Object/S_Object.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Cylinder.h>


namespace mc_rtc::gui
{
  /** Helper to convert an sch::S_Polyhedron to a Polyhedron */
  inline std::vector<std::array<Eigen::Vector3d, 3>> convexToPolyhedron(std::shared_ptr<sch::S_Polyhedron> poly_,
                                        const sva::PTransformd & poseWorld)
  {
    // mc_rtc::log::info("Visual from convex: {}", fmt::ptr(poly_));
    auto poly = std::dynamic_pointer_cast<sch::S_Polyhedron>(poly_);
    const auto & pa = *poly->getPolyhedronAlgorithm();
    const auto triangles = pa.triangles_;
    const auto vertexes = pa.vertexes_;
    auto polyhedron = std::vector<std::array<Eigen::Vector3d, 3>>{};
    polyhedron.reserve(triangles.size());
    for(unsigned int i = 0; i < triangles.size(); i++)
    {
      const auto a = vertexes[triangles[i].a]->getCoordinates();
      const auto b = vertexes[triangles[i].b]->getCoordinates();
      const auto c = vertexes[triangles[i].c]->getCoordinates();
      sva::PTransformd va(Eigen::Vector3d(a[0], a[1], a[2]));
      sva::PTransformd vb(Eigen::Vector3d(b[0], b[1], b[2]));
      sva::PTransformd vc(Eigen::Vector3d(c[0], c[1], c[2]));
      const auto normal = triangles[i].normal;
      auto cross = (a - b) ^ (a - c);
      auto dot = normal * (cross / cross.norm());
      bool reversePointOrder = dot < 0;
      std::array<sva::PTransformd, 3> vertexOrder = {va, vb, vc};
      if(reversePointOrder) { vertexOrder = {vc, vb, va}; }
      for(size_t j = 0; j < vertexOrder.size(); j++)
      {
        vertexOrder[j] = vertexOrder[j] * poseWorld;
      }
      polyhedron.push_back(
          {
          vertexOrder[0].translation(),
          vertexOrder[1].translation(),
          vertexOrder[2].translation()
          }
          );
    }
    return polyhedron;
  }

  inline rbd::parsers::Visual convexToVisual(const sch::S_Box & box)
  {
    rbd::parsers::Visual visual;
    visual.origin = sva::PTransformd::Identity();
    visual.geometry.type = rbd::parsers::Geometry::Type::BOX;
    auto boxGeom = rbd::parsers::Geometry::Box{};
    box.getBoxParameters(boxGeom.size.x(), boxGeom.size.y(), boxGeom.size.z());
    visual.geometry.data = boxGeom;
    return visual;
  }

  inline rbd::parsers::Visual convexToVisual(const sch::S_Cylinder & cylinder)
  {

    rbd::parsers::Visual visual;
    visual.origin = sva::PTransformd::Identity(); 
    visual.geometry.type = rbd::parsers::Geometry::Type::CYLINDER;
    auto cylinderGeom = rbd::parsers::Geometry::Cylinder{};
    cylinderGeom.radius = cylinder.getRadius();
    cylinderGeom.length = (cylinder.getP2() - cylinder.getP1()).norm();
    visual.geometry.data = cylinderGeom;
    return visual;
  }

  /** Helper function to create a Polyhedron from an sch::S_Polyhedron */
  // Returns either a Polyhedron element of a Visual element depending on the type of the
  // type of convex object
  // Problem: this can only be known at runtime
  template<typename GetConvex, typename GetPos>
  auto Convex(const std::string & name, const PolyhedronConfig & config, GetConvex get_convex_fn, GetPos get_pos_fn)
  {
    auto convert = [get_convex_fn, get_pos_fn]() {
      return convexToPolyhedron(get_convex_fn(), get_pos_fn());
    };
    return details::PolyhedronTrianglesListImpl(name, config, convert);
  }

  /** Helper to create a Polyhedron from an sch::S_Polyhedron with a default configuration */
  template<typename GetConvex, typename GetPos>
  auto Convex(const std::string & name, GetConvex get_convex_fn, GetPos get_pos_fn)
  {
    PolyhedronConfig config;
    config.triangle_color = {0, 0.9, 0, 0.5};
    config.vertices_config.color = {0, 1, 0, 0.5};
    config.show_vertices = false;
    config.edge_config.color = {0, 1, 0, 0.5};
    config.show_edges = false;
    return Convex(name, config, get_convex_fn, get_pos_fn);
  }

  template<typename GetConvex, typename GetPos>
  auto ConvexBox(const std::string & name, GetConvex get_convex_fn, GetPos get_pos_fn)
  {
    auto convert = [get_convex_fn]() {
      return convexToVisual(get_convex_fn());
    };
    return details::VisualImpl(name, convert, get_pos_fn);
  }

  template<typename GetConvex, typename GetPos>
  auto ConvexCylinder(const std::string & name, GetConvex get_convex_fn, GetPos get_pos_fn)
  {
    auto convert = [get_convex_fn]() {
      return convexToVisual(get_convex_fn());
    };
    return details::VisualImpl(name, convert, get_pos_fn);
  }
}
//
// visualization_msgs::Marker fromCylinder(const std::string & frame_id,
//                                         const std::string & name,
//                                         size_t id,
//                                         sch::S_Cylinder & cylinder,
//                                         const sva::PTransformd & colTrans)
// {
//   auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::CYLINDER);
//   marker.scale.x = 2 * cylinder.getRadius();
//   marker.scale.y = 2 * cylinder.getRadius();
//   marker.scale.z = (cylinder.getP2() - cylinder.getP1()).norm();
//   auto midP = cylinder.getP1() + (cylinder.getP2() - cylinder.getP1()) / 2;
//   Eigen::Vector3d midPV3{midP.m_x, midP.m_y, midP.m_z};
//   sva::PTransformd cylinderCenter{Eigen::Matrix3d::Identity(), midPV3};
//   setMarkerPose(marker, colTrans * cylinderCenter);
//   return marker;
// }
//
// visualization_msgs::Marker fromSphere(const std::string & frame_id,
//                                       const std::string & name,
//                                       size_t id,
//                                       sch::S_Sphere & sphere,
//                                       const sva::PTransformd & colTrans)
// {
//   auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::SPHERE);