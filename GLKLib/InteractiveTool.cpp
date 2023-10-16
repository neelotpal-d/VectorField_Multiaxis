#include "InteractiveTool.h"
#include "../QMeshLib/QMeshNode.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/PolygenMesh.h"
#include "GLKGeometry.h"
#include "GLKCameraTool.h"
#include <QMouseEvent>
#include <QApplication>
#include <qDebug>

InteractiveTool::InteractiveTool(GLKLib *pGLKLib, QMeshPatch *meshPatch_,  GLKMouseTool *mouseTool, select_type selectType, bool Select)
{
    pGLK = pGLKLib;
    meshPatch = meshPatch_;
    type = selectType;
    mouse_tool = mouseTool;
    tool_type = 1;
	isSelect = Select;

	handleNode = _getHandleNode();
}

InteractiveTool::InteractiveTool(GLKLib *pGLKLib, GLKObList *polygenMeshList,  GLKMouseTool *mouseTool, select_type selectType, bool Select)
{
    pGLK = pGLKLib;
    polygenList = polygenMeshList;
    type = selectType;
    mouse_tool = mouseTool;
    tool_type = 1;
	isSelect = Select;

	handleNode = _getHandleNode();
}

InteractiveTool::~InteractiveTool()
{

}

int InteractiveTool::process_event(QEvent *event, mouse_event_type event_type)
{
    QMouseEvent *e = (QMouseEvent*)event;
	if (type == VECTOR) {
		if (event_type == MOUSE_PRESS) {
			if (!handleNode) return 1;
			pGLK->m_drawPolylinePoints.clear();
			double xx, yy, zz, sx, sy, sz;
			handleNode->GetCoord3D(xx, yy, zz);
			pGLK->wcl_to_screen_new(xx, yy, zz, sx, sy, sz);
			
			QPoint handlePoint((int)sx, (int)sy);
			pGLK->m_drawPolylinePoints << handlePoint;
			pos = handlePoint;
			lastPos = handlePoint;
			pGLK->update();
		}
		if (event_type == MOUSE_MOVE && pGLK->m_drawPolylinePoints.size() >= 1) {
			pGLK->m_drawLine << lastPos;
			pGLK->m_drawLine << pos;

			
			pGLK->update();

			pos = e->pos();
			pGLK->m_drawLine << lastPos;
			pGLK->m_drawLine << pos;

			double dist_ = ((double)pos.x() - (double)lastPos.x()) * ((double)pos.x() - (double)lastPos.x());
			dist_ += ((double)pos.y() - (double)lastPos.y()) * ((double)pos.y() - (double)lastPos.y());
			dist_ = sqrt(dist_);
			double anglePoint = atan2(((double)pos.y() - (double)lastPos.y()), ((double)pos.x() - (double)lastPos.x()));
			pGLK->centerPoint_ << lastPos;
			for (int ii = 0; ii <= 20; ii++) {
				double x_Stride = dist_ * cos(anglePoint * ii / 20);
				double y_Stride = dist_ * sin(anglePoint * ii / 20);
				int pointX = (int)(lastPos.x() + (int)x_Stride);
				int pointY = (int)(lastPos.y() + (int)y_Stride);
				QPoint _Point((int)pointX, (int)pointY);
				pGLK->centerPoint_ << _Point;
			}


			pGLK->update();
		}
		if (event_type == MOUSE_PRESS && (e->buttons() & Qt::RightButton)) {
			
			pGLK->m_drawPolylinePoints.clear();
			pos = e->pos();
			std::cout << pos.x() << " " << pos.y() << std::endl;
			double sx, sy, sz;
			double xx, yy, zz;
			double xo, yo, zo;
			handleNode->GetCoord3D(xo, yo, zo);
			pGLK->wcl_to_screen_new(xo, yo, zo, sx, sy, sz);
			sx = pos.x();
			sy = pos.y();
			pGLK->screen_to_wcl_new(sx, sy, sz, xx, yy, zz);
		;
			double mag = (xx - xo) * (xx - xo) + (yy - yo) * (yy - yo) + (zz - zo) * (zz - zo);

			mag = sqrt(mag);
			if (mag < 1e-5) mag = 1e-5;

			handleNode->vectorDir[0] = (xx-xo)/mag;
			handleNode->vectorDir[1] = (yy-yo)/mag;
			handleNode->vectorDir[2] = (zz-zo)/mag;
			
			if (mouse_tool)
				mouse_tool->process_event(event, event_type);
			pGLK->set_tool(mouse_tool);
			pGLK->refresh(true);
		}
	}
	else {
		if (event_type == MOUSE_PRESS) {
			pos = e->pos();
			pGLK->m_drawPolylinePoints << pos;
			lastPos = e->pos();
			pGLK->update();
		}
		if (event_type == MOUSE_MOVE && pGLK->m_drawPolylinePoints.size() >= 1) {
			pGLK->m_drawLine << lastPos;
			pGLK->m_drawLine << pos;
			pGLK->update();

			pos = e->pos();
			pGLK->m_drawLine << lastPos;
			pGLK->m_drawLine << pos;
			pGLK->update();
		}
		if (event_type == MOUSE_PRESS && (e->buttons() & Qt::RightButton)) {
			Qt::KeyboardModifiers modifier = QApplication::keyboardModifiers();
			if (modifier == Qt::ShiftModifier)
				bShiftPressed = true;
			else
				bShiftPressed = false;
			if (modifier == Qt::AltModifier)
				bAltPressed = true;
			else
				bAltPressed = false;

			switch (type) {
			case NODE:
				_selectNodes(event, event_type);
				break;
			case EDGE:
				_selectEdges(event, event_type);
				break;
			case FACE:
				_selectFaces2(event, event_type);
				break;
			case FIX:
				_selectFixed(event, event_type);
				break;
			case NHANDLE:
				_selectHandle2(event, event_type);
				break;
			}
			pGLK->m_drawPolylinePoints.clear();
			if (mouse_tool)
				mouse_tool->process_event(event, event_type);
			pGLK->set_tool(mouse_tool);
			pGLK->refresh(true);
		}
	}
//    for (GLKPOSITION polyPos=polygenList->GetHeadPosition(); polyPos!=nullptr;){
//        PolygenMesh *polygen = (PolygenMesh*)polygenList->GetNext(polyPos);
//        for (GLKPOSITION posPatch=polygen->GetMeshList().GetHeadPosition(); posPatch!=nullptr;){
//            QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetNext(posPatch);
//            for (GLKPOSITION pos=patch->GetFaceList().GetHeadPosition(); pos!=nullptr;) {
//                QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
//                face->m_nIdentifiedPatchIndex = -1;
//            }
//        }
//    }
	return 1;
}

void InteractiveTool::_selectNodes(QEvent *event, mouse_event_type event_type)
{
    int pointNum = pGLK->m_drawPolylinePoints.size()+1;
    double *xp, *yp;
    xp = new double[pointNum];
    yp = new double[pointNum];
    for (int i=0; i<pointNum-1; i++) {
        xp[i] = pGLK->m_drawPolylinePoints.at(i).x();
        yp[i] = pGLK->m_drawPolylinePoints.at(i).y();
    }
    xp[pointNum-1] = pGLK->m_drawPolylinePoints.at(0).x();
    yp[pointNum-1] = pGLK->m_drawPolylinePoints.at(0).y();
    GLKGeometry geo;

	PolygenMesh *polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double xx, yy, zz, sx, sy;
		TempNode->GetCoord3D(xx, yy, zz);
		pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		if (geo.JugPointInsideOrNot(pointNum, xp, yp, sx, sy)) {
			if(isSelect == false){
				TempNode->selected = true;
			}
			else TempNode->selected = false;
		}
	}

	delete []xp;	delete []yp;
}

void InteractiveTool::_selectEdges(QEvent *event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size() + 1;
	double *xp, *yp;
	xp = new double[pointNum];
	yp = new double[pointNum];
	for (int i = 0; i<pointNum - 1; i++) {
		xp[i] = pGLK->m_drawPolylinePoints.at(i).x();
		yp[i] = pGLK->m_drawPolylinePoints.at(i).y();
	}
	xp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).x();
	yp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).y();
	GLKGeometry geo;

	PolygenMesh *polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double xx, yy, zz, sx, sy;
		TempNode->GetCoord3D(xx, yy, zz);
		pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		if (geo.JugPointInsideOrNot(pointNum, xp, yp, sx, sy)) {
			TempNode->selectedforEdgeSelection = true;
		}
	}
	delete[]xp;	delete[]yp;

	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		if (Edge->GetStartPoint()->selectedforEdgeSelection == true 
			&& Edge->GetStartPoint()->selectedforEdgeSelection == true)
		{
			if (isSelect == false) {
				Edge->selected = true;
			}
			else Edge->selected = false;
		}
	}

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		TempNode->selectedforEdgeSelection = false;
	}
}

void InteractiveTool::_selectFaces(QEvent *event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size() + 1;
	double *xp, *yp;
	xp = new double[pointNum];
	yp = new double[pointNum];
	for (int i = 0; i<pointNum - 1; i++) {
		xp[i] = pGLK->m_drawPolylinePoints.at(i).x();
		yp[i] = pGLK->m_drawPolylinePoints.at(i).y();
	}
	xp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).x();
	yp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).y();
	GLKGeometry geo;

	PolygenMesh *polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double xx, yy, zz, sx, sy;
		TempNode->GetCoord3D(xx, yy, zz);
		pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		if (geo.JugPointInsideOrNot(pointNum, xp, yp, sx, sy)) {
			TempNode->selectedforFaceSelection = true;
		}
	}
	delete[]xp;	delete[]yp;

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->selectedforFaceSelection == true &&
			Face->GetNodeRecordPtr(1)->selectedforFaceSelection == true &&
			Face->GetNodeRecordPtr(2)->selectedforFaceSelection == true)
		{
			if (isSelect == false) {
				Face->selected = true;
			}
			else Face->selected = false;
		}
	}

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		TempNode->selectedforFaceSelection = false;
	}
}

void InteractiveTool::_selectFaces2(QEvent* event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size();
	if (pointNum < 1) return;
	std::cout << "size " << pointNum << std::endl;
	double* xp, * yp;
	xp = new double[4];
	yp = new double[4];
	double x_m, y_m;
	x_m = pGLK->m_drawPolylinePoints.at(pointNum - 1).x();
	y_m = pGLK->m_drawPolylinePoints.at(pointNum - 1).y();


	
	GLKGeometry geo;
	

	PolygenMesh* polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch* patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	double min = 9e9, max = -9e9;
	QMeshNode* minNode;
	
	double px, py, pz;
	pGLK->screen_to_wcl(x_m, y_m, px, py, pz);

	double v[3];
	pGLK->GetViewVector(v[0], v[1], v[2]);
	
	QMeshFace* maxFace;
	double maxDis = -9e10;

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* TempFace = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (!TempFace->dispFace) continue;
		double xx, yy, zz, sx, sy, sz;

		double nor[3];
		TempFace->CalPlaneEquation();
		TempFace->GetNormal(nor[0], nor[1], nor[2]);

		double checkVal = geo.VectorProject(nor, v);
		if (checkVal < 0) continue;

		for (int i = 0; i < 3; i++) {
			QMeshNode* faceNode = TempFace->GetNodeRecordPtr(i);
			faceNode->GetCoord3D(xx, yy, zz);
			//pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
			pGLK->wcl_to_screen_new(xx, yy, zz, sx, sy, sz);
			xp[i] = sx;
			yp[i] = sy;

			
		}
		
		xp[3] = xp[0];
		yp[3] = yp[0];

		
		
		if (geo.JugPointInsideOrNot(4, xp, yp, x_m, y_m)) {
			
			double centr_[3];
			TempFace->GetCenterPos(centr_[0], centr_[1], centr_[2]);
			double dist = geo.VectorProject(centr_, v);
			
			if (maxDis < dist) {
				maxDis = dist;
				maxFace = TempFace;
			}
		}
	

	}

	if (maxFace) {
		std::cout << "Face Selected...\n";
		maxFace->isSelectedTemp = true;
	}

	delete[]xp;	delete[]yp;
}

void InteractiveTool::_selectFixed(QEvent *event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size() + 1;
	double *xp, *yp;
	xp = new double[pointNum];
	yp = new double[pointNum];
	for (int i = 0; i<pointNum - 1; i++) {
		xp[i] = pGLK->m_drawPolylinePoints.at(i).x();
		yp[i] = pGLK->m_drawPolylinePoints.at(i).y();
	}
	xp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).x();
	yp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).y();
	GLKGeometry geo;

	PolygenMesh *polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double xx, yy, zz, sx, sy;
		TempNode->GetCoord3D(xx, yy, zz);
		pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		if (geo.JugPointInsideOrNot(pointNum, xp, yp, sx, sy)) {
			if (isSelect == false) {
				TempNode->isFixed = true;
			}
			else TempNode->isFixed = false;
		}
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
			else Face->isFixedDraw = false;	
	}
	delete[]xp;	delete[]yp;
}

void InteractiveTool::_selectHandle(QEvent *event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size() + 1;
	double *xp, *yp;
	xp = new double[pointNum];
	yp = new double[pointNum];
	for (int i = 0; i<pointNum - 1; i++) {
		xp[i] = pGLK->m_drawPolylinePoints.at(i).x();
		yp[i] = pGLK->m_drawPolylinePoints.at(i).y();
	}
	xp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).x();
	yp[pointNum - 1] = pGLK->m_drawPolylinePoints.at(0).y();
	GLKGeometry geo;

	PolygenMesh *polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	double min = 9e9, max = -9e9;

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		double xx, yy, zz, sx, sy, sz;
		TempNode->GetCoord3D(xx, yy, zz);
		//pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		pGLK->wcl_to_screen_new(xx, yy, zz, sx, sy, sz);
		if (geo.JugPointInsideOrNot(pointNum, xp, yp, sx, sy)) {
			if (max < sz) max = sz;
			if (min > sz) min = sz;
			//if (sz > 0.5) continue;
			if (isSelect == false) {
				TempNode->isHandle = true;
			}
			else TempNode->isHandle = false;
		}
	}
	std::cout << "Max z " << max << std::endl;
	printf("Min z %f\n", min);
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
		
		Face->isHandleDraw = true;		
		else Face->isHandleDraw = false;	
	}
	delete[]xp;	delete[]yp;
}


void InteractiveTool::_selectHandle2(QEvent* event, mouse_event_type event_type)
{
	int pointNum = pGLK->m_drawPolylinePoints.size();
	if (pointNum < 1) return;
	std::cout << "size " << pointNum << std::endl;
	double* xp, * yp;
	xp = new double[5];
	yp = new double[5];
	double x_m, y_m;
	x_m = pGLK->m_drawPolylinePoints.at(pointNum -1).x();
	y_m = pGLK->m_drawPolylinePoints.at(pointNum - 1).y();

	xp[0] = x_m - 5;
	yp[0] = y_m - 5;

	xp[1] = x_m + 5;
	yp[1] = y_m - 5;

	xp[2] = x_m + 5;
	yp[2] = y_m + 5;

	xp[3] = x_m - 5;
	yp[3] = y_m + 5;

	xp[4] = xp[0];
	yp[4] = yp[0];
	
	
	GLKGeometry geo;

	PolygenMesh* polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch* patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	double min = 9e9, max = -9e9;
	QMeshNode* minNode;
	double minDistance = 99e9;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* TempNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (!TempNode->displayNode) continue;
		double xx, yy, zz, sx, sy, sz;
		TempNode->GetCoord3D(xx, yy, zz);
		//pGLK->wcl_to_screen(xx, yy, zz, sx, sy);
		pGLK->wcl_to_screen_new(xx, yy, zz, sx, sy, sz);
		if (geo.JugPointInsideOrNot(5, xp, yp, sx, sy)) {
			double dist = (sx - x_m) * (sx - x_m) + (sy - y_m) + sz * sz;
			if (dist < minDistance) {
				minDistance = dist;
				minNode = TempNode;
			}
			//		if (max < sz) max = sz;
	//		if (min > sz) min = sz;
			//if (sz > 0.5) continue;
			/*if (isSelect == false) {
				TempNode->isHandle = true;
			}
			else TempNode->isHandle = false;*/
		}

	}

	if (minNode) minNode->isHandle = true;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)

			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;
	}
	delete[]xp;	delete[]yp;
}
void InteractiveTool::GetVisibleMesh(QMeshPatch *patch)
{
    double centroid[3] = {0.0, 0.0, 0.0};
    for (GLKPOSITION PosNode=patch->GetNodeList().GetHeadPosition();PosNode!=NULL;) {
        QMeshNode *node=(QMeshNode *)(patch->GetNodeList().GetNext(PosNode));
        double xx, yy, zz;
        node->GetCoord3D(xx, yy, zz);
        centroid[0] += xx;
        centroid[1] += yy;
        centroid[2] += zz;
    }
    int totalNodeNum = patch->GetNodeNumber();
    centroid[0] /= (double)totalNodeNum;
    centroid[1] /= (double)totalNodeNum;
    centroid[2] /= (double)totalNodeNum;

    double xmin, ymin, zmin, xmax, ymax, zmax;
    xmin=1.0e+32;	ymin=1.0e+32;	zmin=1.0e+32;
    xmax=-1.0e+32;	ymax=-1.0e+32;	zmax=-1.0e+32;
    int selectNum = 0;
    for (GLKPOSITION pos=patch->GetNodeList().GetHeadPosition(); pos!=NULL;){
        QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
        double xx, yy, zz;
        node->GetCoord3D(xx,yy,zz);
        if (xx > xmax) xmax = xx;
        if (yy > ymax) ymax = yy;
        if (zz > zmax) zmax = zz;
        if (xx < xmin) xmin = xx;
        if (yy < ymin) ymin = yy;
        if (zz < zmin) zmin = zz;
    }
    double *range = new double[6];
    range[0]=xmin;range[1]=xmax;range[2]=ymin;range[3]=ymax;range[4]=zmin;range[5]=zmax;

    int faceNum = patch->GetFaceNumber();
    QMeshFace **faceArray = (QMeshFace **)new long[faceNum];
    int arrayNum = 0;
    for (GLKPOSITION pos=patch->GetFaceList().GetHeadPosition(); pos!=NULL;arrayNum++){
        QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
        faceArray[arrayNum] = face;
        faceArray[arrayNum]->visible = false;
    }
    int w, h;
    pGLK->GetSize(w,h);
    unsigned char *pixelColor = pGLK->GetColorImage(patch,w,h,range,centroid);
    unsigned char rr, bb, gg;
    for (int i=0; i<h; i++){
        for (int j=0; j<w; j++){
            rr = pixelColor[(i*w+j)*3];
            gg = pixelColor[(i*w+j)*3+1];
            bb = pixelColor[(i*w+j)*3+2];
            int index = 256*256*rr+256*gg+bb;
            if (index < faceNum)
                faceArray[index]->visible = true;
        }
    }
    delete [](QMeshFace**)faceArray;
    delete []range;
    delete []pixelColor;
}

QMeshNode* InteractiveTool::_getHandleNode()
{
	PolygenMesh* polygen = (PolygenMesh*)polygenList->GetHead();
	QMeshPatch* patch = (QMeshPatch*)polygen->GetMeshList().GetHead();
	for (GLKPOSITION nodePos = patch->GetNodeList().GetHeadPosition(); nodePos;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(nodePos);
		if (node->isHandle) return node;
	}
	return nullptr;
}



