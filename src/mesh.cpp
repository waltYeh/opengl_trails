#include "mesh.h"
#include "matrix.h"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <cmath>
#include <float.h>
#include <queue>
using namespace std;

/////////////////////////////////////////
// helping inline functions
inline double Cot(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p1 - p2;
	Vector3d v2 = p3 - p2;

	v1 /= v1.L2Norm();
	v2 /= v2.L2Norm();
	double tmp = v1.Dot(v2);
	return 1.0 / tan(acos(tmp));
}

inline double Area(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p2 - p1;
	Vector3d v2 = p3 - p1;
	return v1.Cross(v2).L2Norm() / 2.0;
}


/////////////////////////////////////////
// implementation of OneRingHEdge class
OneRingHEdge::OneRingHEdge(const Vertex * v) {
	if (v == NULL) start = next = NULL;
	else start = next = v->HalfEdge();
}

HEdge * OneRingHEdge::NextHEdge() {
	HEdge *ret = next;
	if (next && next->Prev()->Twin() != start)
		next = next->Prev()->Twin();
	else
		next = NULL;
	return ret;
}

/////////////////////////////////////////
// implementation of Mesh class
//
// function AddFace
// it's only for loading obj model, you do not need to understand it
void Mesh::AddFace(int v1, int v2, int v3) {
	int i;
	HEdge *he[3], *bhe[3];
	Vertex *v[3];
	Face *f;

	// obtain objects
	for (i=0; i<3; i++) he[i] = new HEdge();
	for (i=0; i<3; i++) bhe[i] = new HEdge(true);
	v[0] = vList[v1];
	v[1] = vList[v2];
	v[2] = vList[v3];
	f = new Face();

	// connect prev-next pointers
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	SetPrevNext(bhe[0], bhe[1]);
	SetPrevNext(bhe[1], bhe[2]);
	SetPrevNext(bhe[2], bhe[0]);

	// connect twin pointers
	SetTwin(he[0], bhe[0]);
	SetTwin(he[1], bhe[2]);
	SetTwin(he[2], bhe[1]);

	// connect start pointers for bhe
	bhe[0]->SetStart(v[1]);
	bhe[1]->SetStart(v[0]);
	bhe[2]->SetStart(v[2]);
	for (i=0; i<3; i++) he[i]->SetStart(v[i]);

	// connect start pointers
	// connect face-hedge pointers
	for (i=0; i<3; i++) {
		v[i]->SetHalfEdge(he[i]);
		v[i]->adjHEdges.push_back(he[i]);
		SetFace(f, he[i]);
	}
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// merge boundary if needed
	for (i=0; i<3; i++) {
		Vertex *start = bhe[i]->Start();
		Vertex *end   = bhe[i]->End();
		for (size_t j=0; j<end->adjHEdges.size(); j++) {
			HEdge *curr = end->adjHEdges[j];
			if (curr->IsBoundary() && curr->End()==start) {
				SetPrevNext(bhe[i]->Prev(), curr->Next());
				SetPrevNext(curr->Prev(), bhe[i]->Next());
				SetTwin(bhe[i]->Twin(), curr->Twin());
				bhe[i]->SetStart(NULL);	// mark as unused
				curr->SetStart(NULL);	// mark as unused
				break;
			}
		}
	}

	// finally add hedges and faces to list
	for (i=0; i<3; i++) heList.push_back(he[i]);
	for (i=0; i<3; i++) bheList.push_back(bhe[i]);
	fList.push_back(f);
}

// function LoadObjFile
// it's only for loading obj model, you do not need to understand it
bool Mesh::LoadObjFile(const char *filename) {
	if (filename==NULL || strlen(filename)==0) return false;
	ifstream ifs(filename);
	if (ifs.fail()) return false;

	Clear();

	char buf[1024], type[1024];
	do {
		ifs.getline(buf, 1024);
		istrstream iss(buf);
		iss >> type;

		// vertex
		if (strcmp(type, "v") == 0) {
			double x, y, z;
			iss >> x >> y >> z;			
            AddVertex(new Vertex(x,y,z));

		}
		// face
		else if (strcmp(type, "f") == 0) {
			int index[3];
			iss >> index[0] >> index[1] >> index[2];
			AddFace(index[0]-1, index[1]-1, index[2]-1);
		}
	} while (!ifs.eof());
	ifs.close();

	size_t i;
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());

	Vector3d tot;
	for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i=0; i<bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i=0; i<vList.size(); i++) 
	{
		vList[i]->adjHEdges.clear(); 
		vList[i]->SetIndex((int)i);
		vList[i]->SetFlag(0);
	}

	return true;
}

void Mesh::DisplayMeshInfo()
{
	int NO_VERTICES = (int)vList.size();
	int NO_FACES = (int)fList.size();
	int NO_HEDGES = (int)heList.size()+(int)bheList.size();
	
	int NO_B_LOOPS = CountBoundaryLoops();
	int NO_COMPONENTS = CountConnectedComponents();
	int NO_GENUS = NO_COMPONENTS - (NO_VERTICES - NO_HEDGES/2 +  NO_FACES + NO_B_LOOPS)/2;
}

int Mesh::CountBoundaryLoops()
{
    int no_loop =0;//count the number of boundary loops
	size_t i;
    for (i=0; i< bheList.size(); i++)
	{
	   HEdge *cur=bheList[i];
       HEdge *nex=cur;
	   while(nex->Start()->visit!=1)
	   {
	     nex->Start()->visit=1;
         nex=nex->Next();
		 if (nex==cur)
		 {no_loop++;break;} 
	   }
	}
	return no_loop;

}
#define DFS 0
#define BFS 1
int Mesh::CountConnectedComponents()
{
#if BFS	
	int no_component =0;

	size_t i,j;
	for(i=0; i<vList.size(); i++){
		Vertex *cur=vList[i];
		cur->SetValid(true);
	}
	
	for(i=0; i<vList.size(); i++){
		Vertex *start=vList[i];
		if (start->IsValid()==false)
			continue;
		else{
			no_component++;
			//invalidate all the vertices of the same
			// component with vertix start
			start->SetValid(false);
		//	cout<<"invalidate start"<<i<<endl;
			queue<Vertex*> q;
			q.push(start);
			while(!q.empty()){
			//	cout<<"in loop"<<endl;
				Vertex *curr=q.front();
				q.pop();
				OneRingVertex ring(curr);
				for(j=0;j<(curr->Valence());j++){
					Vertex *adj=ring.NextVertex();
					if(adj->IsValid()==true){
						adj->SetValid(false);
					//	cout<<"invalidate"<<j<<endl;
						q.push(adj);
					}
				}
			}
		}
	}
	cout << "number of components" << endl;
	cout << no_component << endl;
	return no_component;

#elif DFS
	int no_component =0;

	size_t i,j;
	for(i=0; i<vList.size(); i++){
		Vertex *cur=vList[i];
		cur->SetValid(true);
	}
	
	for(i=0; i<vList.size(); i++){
		Vertex *start=vList[i];
		if (start->IsValid()==false)
			continue;
		else{
			no_component++;
			DFSVisit(start);
			
		}
	}
	cout << "number of components" << endl;
	cout << no_component << endl;
	return no_component;
#endif
}

void Mesh::DFSVisit(Vertex * v)
{
	cout << "visit once" << endl;
	v->SetValid(false);
	OneRingVertex ring(v);
	for(int j=0;j<(v->Valence());j++){
		Vertex *adj=ring.NextVertex();
		if(adj->IsValid()==true){
			DFSVisit(v);
		}
	}
}


// -------------------------------------------------------
// DO NOT TOUCH THE FOLLOWING FOR NOW
// -------------------------------------------------------
void Mesh::ComputeVertexNormals()
{
	cout<<"vertex normal"<<endl;
}

void Mesh::UmbrellaSmooth() 
{
	cout<<"Umbrella Smooth starts..."<<endl;
}

void Mesh::ImplicitUmbrellaSmooth()
{
    cout<< "Implicit Umbrella Smooth starts..."<<endl;
}
void Mesh::ComputeVertexCurvatures()
{
	cout<< "Vertex Curvatures"<<endl;
}