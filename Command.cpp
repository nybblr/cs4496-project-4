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

Matd* frames = new Matd();

// ArticulatedBody* aBody;

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
    // aBody = (ArticulatedBody*)model;

    int numDofs = model->GetDofCount();
    int numCons = model->GetHandleCount() * 3;
    int numFrames = c3d->GetFrameCount();

    // int frameNum = 0;
    double alpha = 0.02;

    frames->SetSize(numFrames, numDofs);
    *frames = vl_0;

    cout << "The Jacobian will be " << numCons << " by " << numDofs << endl;

    double F = 0;
    double Fl = 0;
    double Fb = 10000;
    Vecd qnew;
    Vecd qbest;
    int i;
    for (int frameNum = 0; frameNum < 10; frameNum++) {
      cout << "On frame " << frameNum << endl;
      i = 0;
    // for (int curr = 0; curr < model->GetHandleCount(); curr++) {
      // cout << "Converging " << curr << endl;
    do {
      Fl = F;
      F = 0;
      i++;

      model->ComputeJacobian(frameNum);

      Matd J = *model->mJacobian;

      Matd Jt = trans(J);

      // cout << Jt << endl;

      Vecd C;
      C.SetSize(numCons);
      C = vl_0;

      for (int i = 0; i < model->GetHandleCount(); i++) {
        Vec3d h = model->mHandleList[i]->mGlobalPos;
        Vec3d m = c3d->GetMarkerPos(frameNum, i);
        if (m[0] == 0.0 && m[1] == 0.0 && m[2] == 0.0)
          model->mHandleList[i]->mWeight = 0;
        Vec3d c = h - m;
        c *= model->mHandleList[i]->mWeight;
        // cout << "Weight " << model->mHandleList[i]->mWeight;

        // if (i != 0) c = vl_0;

        F += sqrlen(c);
        C[i*3+0] = c[0];
        C[i*3+1] = c[1];
        C[i*3+2] = c[2];
      }

      // cout << "Objective vector " << C << endl;
      cout << "Objective function " << F << endl;


      Vecd dF = 2 * Jt * C;

      Vecd q;
      q.SetSize(numDofs);
      model->mDofList.GetDofs(&q);
      qnew = q - alpha*dF;

      model->SetDofs(qnew);

      if (F < Fb) qbest = qnew;
    } while ((F > 1E-5 || abs(F-Fl) < 1E-3) && i < 1000);
    // }
      (*frames)[frameNum] = qbest;
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
