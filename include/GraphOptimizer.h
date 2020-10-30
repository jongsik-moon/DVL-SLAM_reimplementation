//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H
#define DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/vertex_se3.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

class GraphOptimizer{
public:
  GraphOptimizer();
  ~GraphOptimizer();

  void AddEdge();
  void Optimize();

private:
  g2o::SparseOptimizer graphOptimizer;

};


#endif //DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H
