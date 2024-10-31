#ifndef GEOMETRIES_HPP
#define GEOMETRIES_HPP

#include <algorithm>

#include <boost/geometry.hpp>

using point_2d = boost::geometry::model::d2::point_xy<double>;
using box_2d = boost::geometry::model::box<point_2d>;
using ring_2d = boost::geometry::model::ring<point_2d>;

box_2d intersect(const box_2d & r1, const box_2d & r2) {
    //*
    const double left_x = std::max(r1.min_corner().x(), r2.min_corner().x());
    const double right_x = std::min(r1.max_corner().x(), r2.max_corner().x());
    const double top_y = std::max(r1.min_corner().y(), r2.min_corner().y());
    const double bottom_y = std::min(r1.max_corner().y(), r2.max_corner().y());
    return box_2d(point_2d(left_x, top_y), point_2d(right_x, bottom_y));
    /*/
    box_2d b;
    boost::geometry::intersection(r1, r2, b);
    return b;
    //*/
}

auto translated(const ring_2d & ring, const point_2d & point) {
    ring_2d translated_ring;
    translated_ring.reserve(ring.size());
    for(auto & ring_p : ring) {
        translated_ring.emplace_back(ring_p.x() + point.x(),
                                     ring_p.y() + point.y());
    }
    return translated_ring;
}

auto translated(const box_2d & b, const point_2d & p) {
    return box_2d(
        point_2d(b.min_corner().x() + p.x(), b.min_corner().y() + p.y()),
        point_2d(b.max_corner().x() + p.x(), b.max_corner().y() + p.y()));
}

#endif  // GEOMETRIES_HPP