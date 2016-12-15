#ifdef WIN32
#include <windows.h>
#endif
#include "glut.h"
#include "mesh.h"
#include "matrix.h"
#include "OpenGLProjector.h"
#include <fstream>//20110819


using namespace std;
// Enumeration
enum EnumDisplayMode { HIDDENLINE, FLATSHADED, SMOOTHSHADED, COLORSMOOTHSHADED, DELETESELECTEDVERTEX, WIREFRAME };

enum Mode
{
	Viewing,
	Selection
};
Mode currentMode = Viewing;


// variables
int displayMode = FLATSHADED;	// current display mode
int mainMenu, displayMenu;		// glut menu handlers
int winWidth, winHeight;		// window width and height
double winAspect;				// winWidth / winHeight;
int lastX, lastY;				// last mouse motion position
int currSelectedVertex = -1;         // current selected vertex
bool leftDown, leftUp, rightUp, rightDown, middleDown, middleUp, shiftDown;		// mouse down and shift down flags
double sphi = 90.0, stheta = 45.0, sdepth = 10;	// for simple trackball
double xpan = 0.0, ypan = 0.0;				// for simple trackball
double zNear = 1.0, zFar = 100.0;
double g_fov = 45.0;
Vector3d g_center;
double g_sdepth;
Mesh mesh;	// our mesh

// functions
void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax);
void InitGL();
void InitMenu();
void InitGeometry();
void MenuCallback(int value);
void ReshapeFunc(int width, int height);
void DisplayFunc();
void DrawWireframe();
void DrawHiddenLine();
void DrawFlatShaded();
void DrawSmoothShaded();
void DrawColorSmoothShaded();
void DrawSelectedVertices();
//void Partition();

void KeyboardFunc(unsigned char ch, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void SelectVertexByPoint();
void DeleteSelectedVertex(int vertex);


void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax) {
	double PI = 3.14159265358979323846;
	double radius = bmax.Distance(bmin);
	g_center = 0.5 * (bmin + bmax);
	zNear = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
	zFar = zNear + 2.0 * radius;
	g_sdepth = zNear + radius;
	zNear *= 0.1;
	zFar *= 10;
	sdepth = g_sdepth;
}

// init openGL environment
void InitGL() {
	GLfloat light0Position[] = { 0, 1, 0, 1.0 };

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Mesh Viewer");
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glPolygonOffset(1.0, 1.0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glLightfv(GL_LIGHT0, GL_POSITION, light0Position);
	glEnable(GL_LIGHT0);

	glutReshapeFunc(ReshapeFunc);
	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
}

// init right-click menu
void InitMenu() {
	displayMenu = glutCreateMenu(MenuCallback);
	glutAddMenuEntry("Wireframe", WIREFRAME);
	glutAddMenuEntry("Hidden Line", HIDDENLINE);
	glutAddMenuEntry("Flat Shaded", FLATSHADED);
	glutAddMenuEntry("Smooth Shaded", SMOOTHSHADED);
	glutAddMenuEntry("Color Smooth Shaded", COLORSMOOTHSHADED);
	glutAddMenuEntry("Delete Selected Vertex", DELETESELECTEDVERTEX);
	mainMenu = glutCreateMenu(MenuCallback);
	glutAddSubMenu("Display", displayMenu);
	glutAddMenuEntry("Exit", 99);
	glutAttachMenu(GLUT_RIGHT_BUTTON);//glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

// init geometry (if no input argument is provided)
void InitGeometry() {
	const int VSIZE = 4;
	const int HESIZE = 12;
	const int FSIZE = 4;
	int i;
	Vertex *v[VSIZE];
	HEdge *he[HESIZE];
	Face *f[FSIZE];

	for (i = 0; i<VSIZE; i++) {
		v[i] = new Vertex();
		mesh.vList.push_back(v[i]);
	}
	v[0]->SetPosition(Vector3d(0.0, 0.0, 0.0));
	v[1]->SetPosition(Vector3d(10.0, 0.0, 0.0));
	v[2]->SetPosition(Vector3d(0.0, 10.0, 0.0));
	v[3]->SetPosition(Vector3d(0.0, 0.0, 10.0));

	v[0]->SetNormal(Vector3d(-0.577, -0.577, -0.577));
	v[1]->SetNormal(Vector3d(0.0, -0.7, -0.7));
	v[2]->SetNormal(Vector3d(-0.7, 0.0, -0.7));
	v[3]->SetNormal(Vector3d(-0.7, -0.7, 0.0));

	for (i = 0; i<FSIZE; i++) {
		f[i] = new Face();
		mesh.fList.push_back(f[i]);
	}

	for (i = 0; i<HESIZE; i++) {
		he[i] = new HEdge();
		mesh.heList.push_back(he[i]);
	}
	for (i = 0; i<FSIZE; i++) {
		int base = i * 3;
		SetPrevNext(he[base], he[base + 1]);
		SetPrevNext(he[base + 1], he[base + 2]);
		SetPrevNext(he[base + 2], he[base]);
		SetFace(f[i], he[base]);
	}
	SetTwin(he[0], he[4]);
	SetTwin(he[1], he[7]);
	SetTwin(he[2], he[10]);
	SetTwin(he[3], he[8]);
	SetTwin(he[5], he[9]);
	SetTwin(he[6], he[11]);
	he[0]->SetStart(v[1]); he[1]->SetStart(v[2]); he[2]->SetStart(v[3]);
	he[3]->SetStart(v[0]); he[4]->SetStart(v[2]); he[5]->SetStart(v[1]);
	he[6]->SetStart(v[0]); he[7]->SetStart(v[3]); he[8]->SetStart(v[2]);
	he[9]->SetStart(v[0]); he[10]->SetStart(v[1]); he[11]->SetStart(v[3]);
	v[0]->SetHalfEdge(he[3]);
	v[1]->SetHalfEdge(he[0]);
	v[2]->SetHalfEdge(he[1]);
	v[3]->SetHalfEdge(he[2]);
}

// GLUT menu callback function
void MenuCallback(int value) {
	switch (value) {
	case 99: exit(0); break;
	default:
		displayMode = value;
		glutPostRedisplay();
		break;
	}
}

// GLUT reshape callback function
void ReshapeFunc(int width, int height) {
	winWidth = width;
	winHeight = height;
	winAspect = (double)width / (double)height;
	glViewport(0, 0, width, height);
	glutPostRedisplay();
}

// GLUT display callback function
void DisplayFunc() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, winAspect, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(xpan, ypan, -sdepth);
	glRotatef(-stheta, 1.0, 0.0, 0.0);
	glRotatef(sphi, 0.0, 1.0, 0.0);
	glTranslatef(-g_center[0], -g_center[1], -g_center[2]);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	switch (displayMode) {
	case WIREFRAME:
		DrawWireframe();
		break;
	case HIDDENLINE:
		DrawHiddenLine();
		break;
	case FLATSHADED:
		DrawFlatShaded();
		DrawSelectedVertices();
		break;
	case SMOOTHSHADED:
		DrawSmoothShaded();
		DrawSelectedVertices();
		break;
	case COLORSMOOTHSHADED:
		DrawColorSmoothShaded();
		break;
	case DELETESELECTEDVERTEX: {
								   DrawFlatShaded();
								   //	DeleteSelectedVertex(currSelectedVertex);
	}
		break;
	}

	glutSwapBuffers();
}

// Wireframe render function
void DrawWireframe() {
	HEdgeList heList = mesh.Edges();
	HEdgeList bheList = mesh.BoundaryEdges();
	glColor3f(0.3, 0.3, 1.0);
	glBegin(GL_LINES);
	size_t i;
	for (i = 0; i<heList.size(); i++) {
		glVertex3dv(heList[i]->Start()->Position().ToArray());
		glVertex3dv(heList[i]->End()->Position().ToArray());
	}

	glColor3f(1, 0.0, 0.0);
	for (i = 0; i<bheList.size(); i++) {
		glVertex3dv(bheList[i]->Start()->Position().ToArray());
		glVertex3dv(bheList[i]->End()->Position().ToArray());
	}

	glEnd();
	glPointSize(5.0f);
	glColor3f(1, 0.1, 0.1);
	glBegin(GL_POINTS);
	for (i = 0; i<heList.size(); i++) {
		glVertex3dv(heList[i]->Start()->Position().ToArray());
	}
	glEnd();
}

// Hidden Line render function
void DrawHiddenLine() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glColor3f(0, 0, 0);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_POLYGON_OFFSET_FILL);

	DrawWireframe();
}

// Flat Shaded render function
void DrawFlatShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		if (fList[i] != NULL && fList[i]->HalfEdge()->LeftFace() != NULL)
		{
			Face *f = fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			normal /= normal.L2Norm();

			f->SetNormal_f(normal);//1007

			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	mesh.ComputeVertexNormals();
	mesh.ComputeVertexCurvatures();
	glEnd();
	
}

// Smooth Shaded render function
/*
double AreaOfTriangle(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3)
{
	Vector3d sideVector;
	sideVector = p1 - p2;
	double a = sideVector.L2Norm();
	sideVector = p3 - p2;
	double b = sideVector.L2Norm();
	sideVector = p1 - p3;
	double c = sideVector.L2Norm();
	double p = (a + b + c) / 2;
	return sqrt(p*(p-a)*(p-b)*(p-c));
}
double AngleOfVectors(const Vector3d & p1, const Vector3d & p2)
{
	return acos(p1.Dot(p2)/p1.L2Norm()/p2.L2Norm());
}*/
void DrawSmoothShaded() {
//	mesh.ComputeVertexNormals();
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		glNormal3dv(v1->Normal().ToArray());
		glVertex3dv(v1->Position().ToArray());
		glNormal3dv(v2->Normal().ToArray());
		glVertex3dv(v2->Position().ToArray());
		glNormal3dv(v3->Normal().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawColorSmoothShaded() {

//	mesh.ComputeVertexCurvatures();
//	cout << "the colored smooth model is drawn" << endl;
/*	VertexList vList = mesh.vList;
//	double PI = 3.14159265358979323846;
	double max_H = 0, max_G = 0, max_K1 = 0, max_K2 = 0;
	for (size_t i = 0; i < vList.size(); i++) {
		Vertex* v = vList[i];
			OneRingHEdge ringHE0(v);
		double angle_sum = 0, area_sum = 0,h_sum=0;
		Vector3d H_sum(0, 0, 0);
		vector<HEdge*> vRingHE(v->Valence());
		for (int j = 0; j < (v->Valence()); j++){
			vRingHE[j] = ringHE0.NextHEdge();
		}
		for (int j = 0; j < (v->Valence()); j++){
			HEdge *adjHE = vRingHE[j];
			HEdge *prevHE = vRingHE[(j - 1 + v->Valence()) % (v->Valence())];
		//	HEdge *nextHE = vRingHE[(j + 1) % (v->Valence())];
			Face *adjF = adjHE->LeftFace();
			Face *prevF = prevHE->LeftFace();
			if (adjF != NULL&&prevF!=NULL){
				Vertex * vadj = adjF->HalfEdge()->Start();
				Vertex * vcur = adjF->HalfEdge()->End();//pointer same as v
				Vertex * vnext = adjF->HalfEdge()->Next()->End();
				Vertex * vprev = prevHE->End();

				double area = AreaOfTriangle(vadj->Position(), vcur->Position(), vnext->Position());
				area_sum += area;
				double angle = AngleOfVectors(vadj->Position() - vcur->Position(), vnext->Position() - vcur->Position());
				angle_sum += angle;
				double alpha = AngleOfVectors(vcur->Position() - vprev->Position(), vadj->Position() - vprev->Position());
				double beta = AngleOfVectors(vcur->Position() - vnext->Position(), vadj->Position() - vnext->Position());
				Vector3d addition = (1.0 / tan(alpha) + 1.0 / tan(beta))*(vadj->Position() - vcur->Position());
				H_sum += addition;
				h_sum += addition.L2Norm();
			}
		}
	//	v->H = abs(-1.0 / 2 / area_sum*H_sum.L2Norm());
		v->H = abs(-1.0 / 2 / area_sum * h_sum);
		v->G = abs(2 / area_sum*(2 * PI - angle_sum));
		v->K1 = abs(v->H - sqrt(abs((v->H)*(v->H) - v->G)));
		v->K2 = abs(v->H + sqrt(abs((v->H)*(v->H) + v->G)));
		
		if (max_H < (v->H)){
			max_H = v->H;
		}
		if (max_G < (v->G)){
			max_G = v->G;
		}
		if (max_K1 < (v->K1)){
			max_K1 = v->K1;
		}
		if (max_K2 < (v->K2)){
			max_K2 = v->K2;
		}
	}
*/
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	Vector3d rgbColor;
	for (size_t i = 0; i<fList.size(); i++) {
		glColor3f(0.4f, 0.4f, 1.0f);
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		rgbColor = v1->Color();
		glColor3f(rgbColor[0], rgbColor[1], rgbColor[2]);
	//	glColor3f((float)i / fList.size(), (float)i / fList.size(), (float)i / fList.size());
		glNormal3dv(v1->Normal().ToArray());
		glVertex3dv(v1->Position().ToArray());
	//	glColor3f(v2->H / max_H, v2->H / max_H, v2->H / max_H);
		rgbColor = v2->Color();
		glColor3f(rgbColor[0], rgbColor[1], rgbColor[2]);
	//	glColor3f((float)i / fList.size(), (float)i / fList.size(), (float)i / fList.size());
		glNormal3dv(v2->Normal().ToArray());
		glVertex3dv(v2->Position().ToArray());
	//	glColor3f(v3->H / max_H, v3->H / max_H, v3->H / max_H);
		rgbColor = v3->Color();
		glColor3f(rgbColor[0], rgbColor[1], rgbColor[2]);
	//	glColor3f((float)i / fList.size(), (float)i / fList.size(), (float)i / fList.size());
		glNormal3dv(v3->Normal().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}


/*********/
// draw the selected ROI vertices on the mesh
void DrawSelectedVertices()
{
//	DrawFlatShaded();
//	DisplayFunc();
	if (currSelectedVertex != -1)
	{
		VertexList vList = mesh.Vertices();
		glPointSize(5.0f);
		glColor3f(1.0, 0.1, 0.1);
		glBegin(GL_POINTS);
		glVertex3dv(vList[currSelectedVertex]->Position().ToArray());
		glEnd();
		cout << "The selected vertex has been drawn" << endl;
	}
	else
	{
		//	cout << "You haven't selected any vertex" << endl;
	}
	glDisable(GL_LIGHTING);
//	cout << "the selected vertex is drawn " << endl;
}
void DeleteHEdge_in_mesh(HEdge * he)
{
	vector<HEdge*>::iterator it = find(mesh.heList.begin(), mesh.heList.end(), he);
	int index = distance(mesh.heList.begin(), it);
	if (index > mesh.heList.size() - 1)
		return;
//	if (mesh.heList[index] != NULL)
//		delete mesh.heList[index];
	mesh.heList[index] = NULL;
	mesh.heList.erase(it);
}
void DeleteFace_in_mesh(Face * face)
{
	vector<Face*>::iterator itf = find(mesh.fList.begin(), mesh.fList.end(), face);
	int index = distance(mesh.fList.begin(), itf);
	if (index > mesh.fList.size() - 1)
		return;
//	if (mesh.fList[index]!=NULL)
//		delete mesh.fList[index];
	mesh.fList[index] = NULL;
	mesh.fList.erase(itf);
}
//delete selected vertex and its incident faces and half-edges
void DeleteSelectedVertex(int vertex)
{
	int index;
	HEdge * currHEdge = mesh.vList[vertex]->HalfEdge();
	int adjNum = mesh.vList[vertex]->Valence();
	int i = 0;
	do{
		i++;
		vector<HEdge*>::iterator it;
		SetPrevNext(currHEdge->Twin()->Prev(), currHEdge->Next());
		mesh.bheList.push_back(currHEdge->Twin()->Prev());
		mesh.bheList.push_back(currHEdge->Twin()->Prev()->Twin());
		currHEdge = currHEdge->Twin();//****************
		HEdge * currTwin = currHEdge->Twin();
		DeleteHEdge_in_mesh(currTwin);
		if (i<adjNum){
			if (currHEdge == currHEdge->Start()->HalfEdge()){
				currHEdge->Start()->SetHalfEdge(currHEdge->Twin()->Next());
			}
			currHEdge = currHEdge->Next();//****************
			HEdge* currPrev = currHEdge->Prev();
			DeleteHEdge_in_mesh(currPrev);
			Face* currFace = currHEdge->LeftFace();
			DeleteFace_in_mesh(currFace);
		}
		else
			break;
	} while (1);
	Face* currFace = currHEdge->LeftFace();
	DeleteFace_in_mesh(currFace);
	DeleteHEdge_in_mesh(currHEdge);
//	delete mesh.vList[vertex];
	mesh.vList[vertex] = NULL;
	vector<Vertex*>::iterator itv = mesh.vList.begin()+vertex;
	mesh.vList.erase(itv);
	for (int i = 0; i < mesh.bheList.size(); i++){
		vector<HEdge*>::iterator it = find(mesh.heList.begin(), mesh.heList.end(), mesh.bheList[i]);
		if (it != mesh.heList.end()){
			index = distance(mesh.heList.begin(), it);
			mesh.heList[index] = NULL;
			mesh.heList.erase(it);
		}
	}
	cout << "the selected vertex is gone, its index is " << currSelectedVertex << endl;

	/*
	Face * face = currHEdge->LeftFace();
	HEdge *curr = face->HalfEdge();
	do{
	//	curr->End()->;
	curr->SetBoundary(TRUE);
	curr = curr->Next();
	} while (curr != face->HalfEdge());

	face = NULL;
	delete face;
	*/


	/*
	FaceList fList = mesh.Faces();
	VertexList vList = mesh.Vertices();
	Vertex *currHEdgerVert = vList[vertex];
	for (size_t i = 0; i <(currHEdgerVert->adjHEdges).size(); i++)//遍历所有与该顶点相连的边
	{
		HEdge *currHEdge = currHEdgerVert->adjHEdges[i];
		if(currHEdge->Next()->Start()->HalfEdge()==currHEdge->Twin())
		{
			cout << "需要修改点的所在半边" << endl;
			currHEdge->Next()->Start()->SetHalfEdge(currHEdge->Next()->Twin()->Prev());
		}

		SetPrevNext(currHEdge->Twin()->Prev(), currHEdge->Next());//修改半边之间的前后关系

		currHEdge->SetFace(NULL);
		currHEdge->Twin()->SetFace(NULL);
		currHEdge->Next()->SetFace(NULL);
	}
	

	*/
}
/****/

// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y) {
	switch (ch) {
	case '3':

		cout << "0003" << endl;
		for (int i = 0; i < 10;i++)
			mesh.UmbrellaSmooth(0.00005);
		break;
	case '4':

		cout << "0004" << endl;;

		break;
	case '5':
		cout << "0005" << endl;
		break;
	case '6':
		cout << "0006" << endl;
		mesh.ImplicitUmbrellaSmooth(0.0001);
		break;

	case '1':	// key '1'
		currentMode = Viewing;
		cout << "Viewing mode" << endl;
		break;
	case '2':	// key '2'
		currentMode = Selection;
		cout << "Selection mode" << endl;
		break;

	case '9':
		cout << "0009" << endl;
		DeleteSelectedVertex(currSelectedVertex);
		break;
		//cout<<"test000 "<<currSelectedVertex<<endl;
	case 27:
		exit(0);
		break;
	}
	glutPostRedisplay();
}
// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {

	lastX = x;
	lastY = y;
	leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
	leftUp = (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP);
	rightDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
	rightUp = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_UP);
	middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
	middleUp = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_UP);
	shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);

	if ((currentMode == Selection) && state == GLUT_UP)
	{
		if (middleUp)//middle button
		{
			if (currSelectedVertex != -1)
			{
				mesh.Vertices()[currSelectedVertex]->SetFlag(0);
				currSelectedVertex = -1;
			}
		}
		else{
			SelectVertexByPoint();
			DrawSelectedVertices();
		}


		//if (leftUp)
		//{
		//  SelectVertexByPoint(); 

		//}
		//if(middleUp)
		//{DeleteSelectedVertex(currSelectedVertex);}

		lastX = lastY = 0;
		glutPostRedisplay();
	}

}

// GLUT mouse motion callback function
void MotionFunc(int x, int y) {
	if (leftDown)
	if (!shiftDown) { // rotate
		sphi += (double)(x - lastX) / 4.0;
		stheta += (double)(lastY - y) / 4.0;
	}
	else { // pan
		xpan += (double)(x - lastX)*sdepth / zNear / winWidth;
		ypan += (double)(lastY - y)*sdepth / zNear / winHeight;
	}
	// scale
	if (middleDown) sdepth += (double)(lastY - y) / 10.0;

	lastX = x;
	lastY = y;
	glutPostRedisplay();
}

/*******/
// select a mesh point
void SelectVertexByPoint()
{
	// get the selection position
	int x = lastX, y = winHeight - lastY;
	OpenGLProjector projector;
	double z = projector.GetDepthValue(x, y);
	Vector3d u(x, y, z);
	Vector3d selected_pos = projector.UnProject(u);
	int nearest_index = 0;
	double nearest_dist = (mesh.vList[0]->Position() - selected_pos).L2Norm();
	for (size_t i = 1; i < mesh.vList.size(); i++){
		double cur_dist = (mesh.vList[i]->Position() - selected_pos).L2Norm();
		if (cur_dist < nearest_dist){
			nearest_index = i;
			nearest_dist = cur_dist;
		}
	}
	//	cout << nearest_index << endl;
	currSelectedVertex = nearest_index;
}
/*****/

// main function
int main(int argc, char **argv) {
	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	if (argc>=2) mesh.LoadObjFile(argv[1]);
	else InitGeometry();
	SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());
	
	/************************************************************************/
	/* activate the following code if you finish the corresponding functions*/
 	mesh.DisplayMeshInfo();
	/************************************************************************/

	glutMainLoop();
	return 0;
}

