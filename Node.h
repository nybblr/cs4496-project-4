#ifndef __NODE_H__
#define __NODE_H__

#include <vector>
#include "vl/VLd.h"
#include <iostream>

using namespace std;

class Node
{
 public:
  virtual void Draw() = 0;

  virtual ~Node() {}
  virtual void UpdateUpMatrix(Mat4d, Mat4d){}

  virtual std::vector<Vec4d> ComputeJacobian(Matd* J, C3dFileInfo* c3d, int frameNum);

  Vec3d mColor;
  int mIndex;
};

#endif
