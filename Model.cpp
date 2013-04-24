#ifndef __MODEL_H__
#include "Model.h"
#endif //__MODEL_H__

#ifndef __TRANSFORMNODE_H__
#include "TransformNode.h"
#endif //__TRANSFORMNODE_H__

void Model::ComputeJacobian(int frameNum)
{
  mJacobian.SetSize(GetDofCount(), GetHandleCount() * 3);

  // High level: call limb asking it to fill in part of the Jacobian
  // Each limb calls its children asking them to fill it it in.
  // When they return, add your own entry for all constraints and your DOFs.

  std::vector<Vec4d*> handles = mLimbs.front()->ComputeJacobian(&mJacobian, mOpenedC3dFile, frameNum);
}
