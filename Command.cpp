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
    double alpha = 0.02;

    cout << "The Jacobian will be " << numCons << " by " << numDofs << endl;

    double F;
    do {
      F = 0;

      model->ComputeJacobian(frameNum);

      Matd J = *model->mJacobian;

      Matd Js;
      Js = vl_0;
      Js.SetSize(numCons, numDofs);
      sub(Js, 0, 0, 3, numDofs) = sub(J, 0, 0, 3, numDofs);

      Matd Jt = trans(J);

      cout << Jt << endl;

      Vecd C;
      C.SetSize(numCons);

      for (int i = 0; i < model->GetHandleCount(); i++) {
        Vec3d h = model->mHandleList[i]->mGlobalPos;
        Vec3d m = c3d->GetMarkerPos(frameNum, i);
        Vec3d c = h - m;

        F += sqrlen(c);
        sub(C, i*3, 3) = c;
      }

      cout << "Objective function " << F << endl;


      Vecd dF = 2 * Jt * C;

      Vecd q;
      q.SetSize(numDofs);
      model->mDofList.GetDofs(&q);
      Vecd qnew = q - alpha*dF;

      model->SetDofs(qnew);
    } while (F > 1E-5);
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
