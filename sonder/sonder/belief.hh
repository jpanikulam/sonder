#pragma once

#include <Eigen/Dense>
#include <sonder/types.hh>

#include <vector>

namespace sonder {

struct IntersectionVotes {
  IntersectionVotes() = default;
  IntersectionVotes(const PointList &_points, const std::vector<std::size_t> &_votes) : points(_points), votes(_votes) {
  }

  inline void clear() {
    points.clear();
    votes.clear();
  }

  inline void add_point(const Eigen::Vector3f &point, const std::size_t point_votes = 1) {
    constexpr float MIN_DIST_TO_ADD = 1e-3;

    for (std::size_t k = 0; k < points.size(); ++k) {
      if ((point - points[k]).norm() < MIN_DIST_TO_ADD) {
        votes[k] += point_votes;
        _max_vote = votes[k] > _max_vote ? votes[k] : _max_vote;
        return;
      }
    }
    points.push_back(point);
    votes.push_back(point_votes);
    _max_vote = point_votes > _max_vote ? point_votes : _max_vote;
  }

  float max_votes() const {
    return static_cast<float>(_max_vote);
  }

  PointList                points;
  std::vector<std::size_t> votes;

private:
  size_t _max_vote = 0;
};

// A gazillion potential optimizations
//
// Potential improvements:
// - Bias the search by already which arcs the points belong to (Does that make sense?)
// - Implement as kdtree
//
void add_points(const IntersectionVotes &new_votes, Out<IntersectionVotes> old_votes) {
  for (std::size_t new_index = 0; new_index < new_votes.points.size(); ++new_index) {
    const Eigen::Vector3f &pt = new_votes.points[new_index];
    old_votes->add_point(pt, new_votes.votes[new_index]);
  }
}
}