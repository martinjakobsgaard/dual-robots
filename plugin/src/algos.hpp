#ifndef ALGOS_HPP
#define ALGOS_HPP

#include <rw/rw.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>

// This file is horribly named.
// It is intended to contain the various robotics-related
// methods and algorithms used for the dual-robot project.
// This file contains algorithms for sampling, planning,
// etc.
// It is not named 'algorithms.hpp' due to the
// similarity with standard <algorithm> header.

rwlibs::pathplanners::RRTTree<rw::math::Q> findConstrainedTree(rw::math::Q rootQ, rw::models::Device::CPtr device);

#endif
