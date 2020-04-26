#include "algos.hpp"

void findConstrainedTree(rwlibs::pathplanners::RRTTree<rw::math::Q> &tree, rw::models::Device::CPtr device)
{
    rw::math::Q newQ(1, 1, 1, 1, 1, 1);
    tree.add(newQ, &tree.getRoot());
}
