#ifndef __TRANSFORMNODE_H__
#include "TransformNode.h"
#endif //__TRANSFORMNODE_H__

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif //__TRANSFORM_H__

#ifndef __DOF_H__
#include "Dof.h"
#endif //__DOF_H__

#ifndef __GLPRIMITIVES_H__
#include "GLPrimitives.h"
#endif //__GLPRIMITIVES_H__


#include <FL/gl.h>
#include <assert.h>

TransformNode::TransformNode(char* name)
{
  mName = name;
  mIndex = -1;

  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( char* name, Node* child,Transform* t1 ... )
{
  mName = name;
  mIndex = -1;
  if ( child ){
    //mChildren.push_back( child );
    mPrimitive.push_back(child);
  }

  if(t1){
    mTransforms.push_back( t1 );
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg( ap, Transform* );
      if(t)
  mTransforms.push_back( t );
      else
  break;
    }
    va_end(ap);
  }
  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( Node* child, Transform* t1 ... )
{
  mName = 0;
  mIndex = -1;
  mPrimitive.push_back(child);

  if (t1){
    mTransforms.push_back( t1 );
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg(ap, Transform*);
      if(t)
  mTransforms.push_back(t);
      else
  break;
    }
    va_end( ap );
  }
  mCurrentTransform = vl_I;
}

TransformNode::TransformNode( int index, Node* child, Transform* t1 ... )
{
  mName = 0;
  mIndex = index;
  mPrimitive.push_back(child);

  if(t1){
    mTransforms.push_back(t1);
    va_list ap;
    va_start( ap, t1 );
    while(1){
      Transform* t = va_arg( ap, Transform* );
      if(t)
  mTransforms.push_back( t );
      else
  break;
    }
    va_end(ap);
  }
  mCurrentTransform = vl_I;
}

void TransformNode::Draw()
{
  glPushMatrix();

  for( int i = 0; i < mTransforms.size(); i++ )
    mTransforms[i]->Apply();

  for(int i = 0; i < mPrimitive.size(); i++){
    mPrimitive[i]->Draw();
  }

  for(int i = 0; i < mChildren.size(); i++ )
    mChildren[i]->Draw();

  glPopMatrix();
}

void TransformNode::Draw(int frameNum)
{
  glPushMatrix();

  for( int i = 0; i < mTransforms.size(); i++ )
    mTransforms[i]->Apply();

  for(int i=0;i<mPrimitive.size();i++){
    mPrimitive[i]->Draw();
  }

  for(int i = 0; i < mChildren.size(); i++ )
    ((TransformNode*)mChildren[i])->Draw(frameNum);

  glPopMatrix();
}

void TransformNode::DrawHandles()
{
    for(int i = 0; i < mHandles.size(); i++)
        mHandles[i]->Draw();

    for(int i = 0; i < mChildren.size(); i++ )
        ((TransformNode*)mChildren[i])->DrawHandles();
}

TransformNode::~TransformNode()
{
  int size = mTransforms.size();
  for(int i = 0; i < size; i++)
    delete mTransforms[i];

  size = mPrimitive.size();
  for(int i = 0; i < size; i++)
    delete mPrimitive[i];

  size = mHandles.size();
  for(int i = 0; i < size; i++)
    delete mHandles[i];

}

void TransformNode::UpdateUpMatrix(Mat4d currTransform, Mat4d invHeadMatrix)
{
  mParentTransform = currTransform;
  mLocalTransform = vl_I;
  for( int i = 0; i < mTransforms.size(); i++ )
    mLocalTransform *= mTransforms[i]->GetTransform();

  currTransform *= mLocalTransform;

  mCurrentTransform = currTransform;
  for(int i = 0; i < mHandles.size(); i++){
    Vec3d tempVec3;
    tempVec3 = mHandles[i]->mOffset;

    Vec4d tempVec4 = Vec4d(tempVec3[0], tempVec3[1], tempVec3[2], 1);
    tempVec4 = currTransform * tempVec4;

    mHandles[i]->mGlobalPos = Vec3d(tempVec4[0], tempVec4[1], tempVec4[2]);
  }

  for(int i = 0; i < mChildren.size(); i++ )
    mChildren[i]->UpdateUpMatrix(currTransform, invHeadMatrix);

  for(int i = 0; i < mPrimitive.size(); i++ )
    mPrimitive[i]->UpdateUpMatrix(currTransform, invHeadMatrix);
}

std::vector<Vec4d> TransformNode::ComputeJacobian(Matd* J, C3dFileInfo* c3d, int frameNum) {
	// Initialize handles list (transformed)
	std::vector<Vec4d> hLocals;
	// for (int i = 0; i < hLocals.size(); i++) {
	// 	hLocals[i] = Vec4d();
	// }

	// Call all children first
	for (int i = 0; i < mChildren.size(); i++) {
		// We should get all child handles and their transforms
		std::vector<Vec4d> cLocals = static_cast<TransformNode*>(mChildren[i])->ComputeJacobian(J, c3d, frameNum);
		// Copy handle transforms
		for (int j = 0; j < cLocals.size(); j++) {
			hLocals.push_back(cLocals[j]);
		}
	}

	// Now add all our handles to the list
	for (int i = 0; i < mHandles.size(); i++) {
		hLocals[mHandles[i]->mMarkerOrder] = Vec4d(mHandles[i]->mOffset, 1);
	}

	for (int i = 0; i < GetSize(); i++) {
		for (int j = 0; j < mTransforms[i]->GetDofCount(); j++) {
			// Which column of J should this DOF be in?
			int c = mTransforms[i]->GetDof(j)->mId;
			
			// Derivative of this local transform
			// Some transforms have many DOFs
			// Multiply all transforms;
			// If we come to this current transform i,
			// Take the derivative of it with respect to jth DOF.
			// Otherwise, just get regular transform
			Mat4d T = vl_I;
			for (int k = 0; k < GetSize(); k++) {
				if (k == i)
					T *= mTransforms[i]->GetDeriv(j);
				else
					T *= mTransforms[i]->GetTransform();
			}

			// Now T is our local transform with derivative
			// taken with respect to the jth DOF.

			// Now the partial transforms of all handles
			// on children is populated on model for use.
			for (int k = 0; k < hLocals.size(); k++) {
				// Make homogeneous coordinate of child handles
				// Vec4d h = Vec4d(c3d->GetMarkerPos(frameNum, r), 1);
				Vec4d h = T * hLocals[k];
				int r = hLocals[k].mMarkerOrder;

				// Each row is an x,y, or z of a handle
				// All child handles use the transform
				// Otherwise, all 0s are output
				(*J)[r*3+0][c] = h[0];
				(*J)[r*3+1][c] = h[1];
				(*J)[r*3+2][c] = h[2];
			}
		}
	}
  return hLocals;
}
