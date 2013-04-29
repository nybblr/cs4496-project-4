#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__



int readSkelFile( FILE* file, ArticulatedBody* skel );

extern RealTimeIKUI *UI;

void LoadModel(void *v)
{
  char *params = (char*)v;
  if(!params){
    params = (char*)fl_file_chooser("Open File?", "{*.skel}", "../src/skels" );
  }

  if(!params)
    return;

  FILE *file = fopen(params, "r");

  if(file == NULL){
    cout << "Skel file does not exist" << endl;
    return;
  }

  ArticulatedBody *mod = new ArticulatedBody();
  UI->mData->mModels.push_back(mod);
  UI->mData->mSelectedModel = mod;

  readSkelFile(file, mod);
  UI->CreateDofSliderWindow();

  mod->InitModel();
  UI->mGLWindow->mShowModel = true;
  UI->mShowModel_but->value(1);
  UI->mGLWindow->refresh();

  cout << "number of dofs in model: " << UI->mData->mModels[0]->GetDofCount() << endl;
}

void Solution(void *v)
{
    Model* model = UI->mData->mSelectedModel;
    C3dFileInfo* c3d = model->mOpenedC3dFile;

    int numDofs = model->GetDofCount();
    int numCons = model->GetHandleCount() * 3;

    int frameNum = 0;
    double alpha = 0.1;

    cout << "The Jacobian will be " << numCons << " by " << numDofs << endl;

    for (int i = 0; i < 100; i++) {
      std::vector<Vec4d*> handles = model->ComputeJacobian(frameNum);

      Matd J = *model->mJacobian;

      Matd Jt = trans(J);

      // cout << Jt << endl;

      Vecd C;
      C.SetSize(numCons);

      for (int i = 0; i < model->GetHandleCount(); i++) {
        for (int j = 0; j < 3; j++) {
          // cout << "Prevec " << i << ":" << j << C << endl;
          Vec4d v = *(handles[i]);
          // cout << "Homogeneous " << v << endl;
          double h = v[j]/v[3];
          double m = c3d->GetMarkerPos(frameNum, i)[j];
          // cout << "Calculated m" << endl;
          C[i*3 + j] = h - m;
        }
      }

      // cout << "Objective vector " << C << endl;

      Vecd dF = 2 * Jt * C;

      Vecd q;
      q.SetSize(numDofs);
      model->mDofList.GetDofs(&q);
      Vecd qnew = q - alpha*dF;

      model->SetDofs(qnew);
    }

    for (int i = 0; i < UI->mData->mSelectedModel->GetDofCount(); i++) {
      cout << UI->mData->mSelectedModel->mDofList.mDofs[i]->GetName() << endl;
    }
}

void Exit(void *v)
{
  exit(0);
}

void LoadC3d(void *v)
{
  if(!UI->mData->mSelectedModel){
    cout << "Load skeleton first";
    return;
  }
  char *params = (char*)v;
  if(!params){
    params = fl_file_chooser("Open File?", "{*.c3d}", "mocap/" );
  }

  if(!params)
    return;

  char *c3dFilename = new char[80];

  // load single c3d file

  C3dFileInfo *openFile = new C3dFileInfo(params);
  openFile->LoadFile();
  UI->mData->mSelectedModel->mOpenedC3dFile = openFile;
  cout << "number of frames in c3d: " << openFile->GetFrameCount() << endl;

  UI->InitControlPanel();
  UI->mGLWindow->mShowConstraints = true;
  UI->mShowConstr_but->value(1);
}
