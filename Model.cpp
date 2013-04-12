#ifndef __MODEL_H__
#include "Model.h"
#endif //__MODEL_H__

void Model::ComputeJacobian()
{
  mJacobian.SetSize(GetDofCount(), GetHandleCount() * 3);
}
