#include "mesh.h"
#include "matrix.h"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <cmath>
#include <float.h>
#include <queue>
#include <algorithm>
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
	//rescale the model -- unify
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());
	//re-place the model's origin
	Vector3d tot;
	for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i=0; i<bheList.size(); i++)//boundary half edge
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

	cout << "number of vertices" << endl;
	cout << NO_VERTICES << endl;
	cout << "number of faces" << endl;
	cout << NO_FACES << endl;
	cout << "number of edges" << endl;
	cout << NO_HEDGES << endl;
	cout << "number of components" << endl;
	cout << NO_COMPONENTS << endl;

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
#define DFS 1
#define BFS 0
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

	size_t i;
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
//	cout << "visit once" << endl;
	v->SetValid(false);
	OneRingVertex ring(v);
	for(int j=0;j<(v->Valence());j++){
		Vertex *adj=ring.NextVertex();
		if(adj->IsValid()==true){
			DFSVisit(adj);
		}
	}
}
// -------------------------------------------------------
// DO NOT TOUCH THE FOLLOWING FOR NOW
// -------------------------------------------------------
void Mesh::ComputeVertexNormals()
{
	for (size_t i = 0; i < vList.size(); i++) {
		Vertex* v = vList[i];
		OneRingHEdge ringHE1(v);
		Vector3d norm1(0, 0, 0);
		for (int j = 0; j < (v->Valence()); j++){
			HEdge *adj = ringHE1.NextHEdge();
			Face *ringF = adj->LeftFace();
			if (ringF != NULL){
				//	double area;
				Vertex * v1 = ringF->HalfEdge()->Start();
				Vertex * v2 = ringF->HalfEdge()->End();
				Vertex * v3 = ringF->HalfEdge()->Next()->End();
				double area = Area(v1->Position(), v2->Position(), v3->Position());
				norm1 += ringF->Normal_f()*area;
			}
		}
		norm1 /= norm1.L2Norm();
		v->SetNormal(norm1);
	}
}

void Mesh::UmbrellaSmooth(double lamda)
{
#define GAUSS_CURV_SCHEME 0
#define MEAN_CURV_SCHEME 0
#define MAX_PRINC_CURV_SCHEME 0
#define MIN_PRINC_CURV_SCHEME 1
//	VertexList vListNew = vList;
	//Gaussian curvature
	size_t i;
	double min_c = 200000;//initialize the maximum value of curvature
	double max_c = -200000;//initialize the minimum value of curvature
	//	VertexList vList = mesh.vList;
	for (i = 0; i < vList.size(); i++)
	{
		if (!vList[i]->IsBoundary()) //for interior vertices only
		{
			double  v_A_1 = 0.0; //initialize the area of any face incident to each vertex v
			double  v_A = 0.0;//the area of all face incident to vertex v
			double  Angle_v_1 = 0.0; //initialize any angle incident on each vertex v
			double  Angle_v = 0.0;//the angles incident on vertex v

			double cot_sum = 0.0;//coefficient of (vj - vi) in the mean curvature function
			Vector3d color_v(0.0, 0.0, 0.0);

			HEdge *cur = vList[i]->HalfEdge();
			HEdge *nex = cur;
			while (nex && nex->Prev()->Twin() != cur)
			{
				if (nex && !nex->IsBoundary())//exclude the holes formed by boundary loops
				{
					const Vector3d & pos1 = vList[i]->Position();//p1
					const Vector3d & pos2 = nex->End()->Position();//p2
					const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
					if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
					{
						const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
						cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
						color_v = color_v + cot_sum*(pos3 - pos1);

					}
					v_A_1 = Area(pos1, pos2, pos3);
					v_A = v_A + v_A_1;//SUM

					Vector3d v1 = pos2 - pos1;
					Vector3d v2 = pos3 - pos1;
					v1 /= v1.L2Norm();
					v2 /= v2.L2Norm();
					double temp = v1.Dot(v2);
					Angle_v_1 = acos(temp);
					Angle_v = Angle_v + Angle_v_1;
				}

				nex = nex->Prev()->Twin();

			}

			//process the last nex
			if (nex && !nex->IsBoundary())
			{
				const Vector3d & pos1 = vList[i]->Position();//p1
				const Vector3d & pos2 = nex->End()->Position();//p2
				const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
				if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
				{
					const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
					cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
					color_v = color_v + cot_sum*(pos3 - pos1);

				}

				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM

				//computing the sum of angles for Gaussian curvature
				Vector3d v1 = pos2 - pos1;
				Vector3d v2 = pos3 - pos1;
				v1 /= v1.L2Norm();
				v2 /= v2.L2Norm();
				double temp = v1.Dot(v2);
				Angle_v_1 = acos(temp);
				Angle_v = Angle_v + Angle_v_1;
			}//end if

//			double PI = 3.14159265358979323846;
//			double G = (2.0 / v_A)*(2 * PI - Angle_v);//Gaussian curvature
			Vector3d L = color_v / (2 * v_A); //mean curvature   
		//	double lamda = 0.0005;
			if (bheList.size() == 0)
				vList[i]->SetCur_pos(vList[i]->Position() + lamda*L);
			else
			vList[i]->SetPosition(vList[i]->Position() + lamda*L);
			//compute principle curvatures

		}
	}
	if (bheList.size() == 0)
		for (i = 0; i < vList.size(); i++)
			vList[i]->SetPosition(vList[i]->Cur_pos());
}

void Mesh::ImplicitUmbrellaSmooth(double lamda)
{
    cout<< "Implicit Umbrella Smooth starts..."<<endl;
	
	Matrix L(vList.size(), vList.size());
	for (size_t i = 0; i < vList.size(); i++)
	{
		vector<double> weight;
		vector<int> index;
		if (1)//(!vList[i]->IsBoundary()) //for interior vertices only
		{
			double  v_A_1 = 0.0; //initialize the area of any face incident to each vertex v
			double  v_A = 0.0;//the area of all face incident to vertex v
			double cot_sum = 0.0;//coefficient of (vj - vi) in the mean curvature function
			double weight_sum = 0.0;
			HEdge *cur = vList[i]->HalfEdge();
			HEdge *nex = cur;
			while (nex && nex->Prev()->Twin() != cur)
			{
				if (nex && !nex->IsBoundary())//exclude the holes formed by boundary loops
				{
					const Vector3d & pos1 = vList[i]->Position();//p1
					const Vector3d & pos2 = nex->End()->Position();//p2
					const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
					if (1)//(!vList[i]->IsBoundary())//if the current vertex is an interior vertex
					{
						const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
						cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
						weight.push_back(cot_sum);
						index.push_back(nex->End()->Index());
					//	L.AddElement(i,nex->End()->Index(), cot_sum);
						weight_sum += cot_sum;
					}
					v_A_1 = Area(pos1, pos2, pos3);
					v_A = v_A + v_A_1;//SUM
				}

				nex = nex->Prev()->Twin();

			}

			//process the last nex
			if (nex && !nex->IsBoundary())
			{
				const Vector3d & pos1 = vList[i]->Position();//p1
				const Vector3d & pos2 = nex->End()->Position();//p2
				const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
				if (1)//(!vList[i]->IsBoundary())//if the current vertex is an interior vertex
				{
					const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
					cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
					weight.push_back(cot_sum);
					index.push_back(nex->End()->Index());
				//	L.AddElement(i,nex->End()->Index(), cot_sum);
					weight_sum += cot_sum;
				}

				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM

			}//end if
			L.AddElement(i, i, 1+lamda*weight_sum / 2 / v_A);
			for (int j = 0; j < index.size(); j++){
				L.AddElement(i, index[j],-lamda* weight[j] / 2 / v_A);
			}
		}
	}

	L.SortMatrix();

	double *x, *y, *z, *x_new, *y_new, *z_new;
	x = new double[vList.size()];
	y = new double[vList.size()];
	z = new double[vList.size()];
	x_new = new double[vList.size()];
	y_new = new double[vList.size()];
	z_new = new double[vList.size()];
	for (size_t i = 0; i < vList.size(); i++){
		Vector3d pos = vList[i]->Position();
		x[i] = pos[0];
		y[i] = pos[1];
		z[i] = pos[2];
		x_new[i] = pos[0];
		y_new[i] = pos[1];
		z_new[i] = pos[2];
	}
	
	L.BCG(x, x_new, 3, 0.01);
	L.BCG(y, y_new, 3, 0.01);
	L.BCG(z, z_new, 3, 0.01);
	for (size_t i = 0; i < vList.size(); i++){
		Vector3d pos(x_new[i], y_new[i], z_new[i]);
		vList[i]->SetPosition(pos);
	}		
	delete x;
	delete y;
	delete z;
	delete x_new;
	delete y_new;
	delete z_new;
}

void Mesh::ComputeVertexCurvatures()
{
#define GAUSS_CURV_SCHEME 0
#define MEAN_CURV_SCHEME 0
#define MAX_PRINC_CURV_SCHEME 0
#define MIN_PRINC_CURV_SCHEME 1
	
	//Gaussian curvature
	size_t i;
	double min_c = 200000;//initialize the maximum value of curvature
	double max_c = -200000;//initialize the minimum value of curvature
	//	VertexList vList = mesh.vList;
	for (i = 0; i< vList.size(); i++)
	{
		if (!vList[i]->IsBoundary()) //for interior vertices only
		{
			double  v_A_1 = 0.0; //initialize the area of any face incident to each vertex v
			double  v_A = 0.0;//the area of all face incident to vertex v
			double  Angle_v_1 = 0.0; //initialize any angle incident on each vertex v
			double  Angle_v = 0.0;//the angles incident on vertex v

			double cot_sum = 0.0;//coefficient of (vj - vi) in the mean curvature function
			Vector3d color_v(0.0, 0.0, 0.0);

			HEdge *cur = vList[i]->HalfEdge();
			HEdge *nex = cur;
			while (nex && nex->Prev()->Twin() != cur)
			{
				if (nex && !nex->IsBoundary())//exclude the holes formed by boundary loops
				{
					const Vector3d & pos1 = vList[i]->Position();//p1
					const Vector3d & pos2 = nex->End()->Position();//p2
					const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
					if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
					{
						const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
						cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
						color_v = color_v + cot_sum*(pos3 - pos1);

					}
					v_A_1 = Area(pos1, pos2, pos3);
					v_A = v_A + v_A_1;//SUM

					Vector3d v1 = pos2 - pos1;
					Vector3d v2 = pos3 - pos1;
					v1 /= v1.L2Norm();
					v2 /= v2.L2Norm();
					double temp = v1.Dot(v2);
					Angle_v_1 = acos(temp);
					Angle_v = Angle_v + Angle_v_1;
				}

				nex = nex->Prev()->Twin();

			}

			//process the last nex
			if (nex && !nex->IsBoundary())
			{
				const Vector3d & pos1 = vList[i]->Position();//p1
				const Vector3d & pos2 = nex->End()->Position();//p2
				const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
				if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
				{
					const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
					cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
					color_v = color_v + cot_sum*(pos3 - pos1);

				}

				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM

				//computing the sum of angles for Gaussian curvature
				Vector3d v1 = pos2 - pos1;
				Vector3d v2 = pos3 - pos1;
				v1 /= v1.L2Norm();
				v2 /= v2.L2Norm();
				double temp = v1.Dot(v2);
				Angle_v_1 = acos(temp);
				Angle_v = Angle_v + Angle_v_1;
			}//end if

			double PI = 3.14159265358979323846;
			double G = (2.0 / v_A)*(2 * PI - Angle_v);//Gaussian curvature
			double H = -color_v.L2Norm() / (2 * v_A); //mean curvature   

			//compute principle curvatures
			double K1 = H + sqrt(fabs(H*H - G));//max principle curvature

			double K2 = H - sqrt(fabs(H*H - G));//min principle curvature
			//remember different types of curvatures for each 
			vList[i]->G = G; //record gaussian curvature at vList[i]
			vList[i]->H = H; //record mean curvature at vList[i]
			vList[i]->K1 = K1; //record max principle curvature at vList[i]
			vList[i]->K2 = K2; //record min principle curvature at vList[i]
#if MAX_PRINC_CURV_SCHEME
			//"max principal curvature" scheme
			if(K1 < min_c) // reset min curvatures
			min_c = K1;
			if(K1 > max_c) //reset max curvatures
			max_c = K1;

#elif MIN_PRINC_CURV_SCHEME			
			//"min principal curvature" scheme
			if (K2 < min_c) // reset min curvatures
				min_c = K2;
			if (K2 > max_c) //reset max curvatures
				max_c = K2;
#elif MEAN_CURV_SCHEME		
			//"mean curvature" scheme
			if(H < min_c) // reset min curvatures
			min_c = H;
			if(H > max_c) //reset max curvatures
			max_c = H;
#elif GAUSS_CURV_SCHEME		
			//"Gaussian curvature" scheme
			if(G < min_c) // reset min curvatures
			min_c = G;
			if(G > max_c) //reset max curvatures
			max_c = G;
#endif
		}//end 'if' condition for interior vertices

	}//end the first 'for' loop

	/**********interpolation based on the scheme of [0->1] equals [blue->red]**********/
#if MAX_PRINC_CURV_SCHEME
	//"max principal curvature" scheme

	for (i=0; i< vList.size(); i++)
	{
	if(!vList[i]->IsBoundary())//process interior vertices
	{
	Vector3d color_max((vList[i]->K1 - min_c)/(max_c - min_c),0.0,1-(vList[i]->K1 - min_c)/(max_c - min_c));
	vList[i]->SetColor(color_max);
	}
	else //process boundary vertices
	{
	Vector3d color_max(0.0,0.0,0.0);
	vList[i]->SetColor(color_max);
	}
	}//end this for

	
#elif MIN_PRINC_CURV_SCHEME		
	//"min principal curvature" scheme
	for (i = 0; i< vList.size(); i++)
	{
		if (!vList[i]->IsBoundary())//process interior vertices
		{
			Vector3d color_min;
			float r = (vList[i]->K2 - min_c) / (max_c - min_c);
			if (r<0.8)
				color_min = Vector3d(0, 1, 0);
			else color_min = Vector3d(r, 0, 1 - r);
			vList[i]->SetColor(color_min);
		}
		else //process boundary vertices
		{
			Vector3d color_max(0.0, 0.0, 0.0);
			vList[i]->SetColor(color_max);
		}
	}//end this for


#elif MEAN_CURV_SCHEME	
	//"mean curvature" scheme
	for (i=0; i< vList.size(); i++) 
	{ 
	  if(!vList[i]->IsBoundary())//process interior vertices
	  {
	    Vector3d color_mean((vList[i]->H - min_c)/(max_c - min_c),0.0,1-(vList[i]->H - min_c)/(max_c - min_c));
	    vList[i]->SetColor(color_mean); 
	  }
	  else //process boundary vertices
	  {
		Vector3d color_mean(0.0,0.0,0.0);
	    vList[i]->SetColor(color_mean); 
	  }
	}//end this for
	  
	
#elif GAUSS_CURV_SCHEME	
	 	//"Gaussian curvature" scheme
	 	for (i=0; i< vList.size(); i++) 
	 	{ 
	 	  if(!vList[i]->IsBoundary())//process interior vertices
	 	  {
	 	    Vector3d color_Gau((vList[i]->G - min_c)/(max_c - min_c),0.0,1-(vList[i]->G - min_c)/(max_c - min_c));
	 	    vList[i]->SetColor(color_Gau); 
	 	  }
	 	  else //process boundary vertices
	 	  {
	 		Vector3d color_max(0.0,0.0,0.0);
	 	    vList[i]->SetColor(color_max); 
	 	  }
	 
	 	}//end this for

#endif
}
void DeleteVertex(Vertex * v)
{

}
