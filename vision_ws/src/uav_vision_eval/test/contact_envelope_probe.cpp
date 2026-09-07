#ifdef USE_GAZEBO_ODE
#include <gazebo/ode/ode.h>
#else
#include <ode/ode.h>
#endif
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Hits { double nearest=10.; int contacts=0; };
void rayCallback(void* data,dGeomID a,dGeomID b) {
 if(dGeomIsSpace(a)||dGeomIsSpace(b)){dSpaceCollide2(a,b,data,rayCallback);return;}
 if(dGeomGetClass(a)!=dRayClass && dGeomGetClass(b)!=dRayClass)return;
 dContactGeom hit;
 if(dCollide(a,b,1,&hit,sizeof(hit)))static_cast<Hits*>(data)->nearest=std::min(static_cast<Hits*>(data)->nearest,double(hit.depth));
}
void contactCallback(void* data,dGeomID a,dGeomID b) {
 if(dGeomIsSpace(a)||dGeomIsSpace(b)){dSpaceCollide2(a,b,data,contactCallback);return;}
 dContactGeom hit;
 if(dCollide(a,b,1,&hit,sizeof(hit)))static_cast<Hits*>(data)->contacts++;
}
int main(int argc,char** argv) {
 if(argc!=2)return 2;
 std::ifstream file(argv[1]);std::vector<dReal> vertices;std::vector<dTriIndex> faces;std::string line;
 while(std::getline(file,line)) {std::istringstream in(line);std::string kind;in>>kind;
  if(kind=="v"){double x,y,z;in>>x>>y>>z;vertices.insert(vertices.end(),{x,y,z});}
  if(kind=="f"){std::vector<dTriIndex> polygon;int index;while(in>>index)polygon.push_back(index-1);for(size_t k=1;k+1<polygon.size();++k)faces.insert(faces.end(),{polygon[0],polygon[k],polygon[k+1]});}
 }
 dInitODE();dSpaceID world=dSimpleSpaceCreate(0),guardSpace=dSimpleSpaceCreate(world),raySpace=dSimpleSpaceCreate(0);
 dTriMeshDataID mesh=dGeomTriMeshDataCreate();dGeomTriMeshDataBuildDouble(mesh,vertices.data(),3*sizeof(dReal),vertices.size()/3,faces.data(),faces.size(),3*sizeof(dTriIndex));
 dGeomID guard=dCreateTriMesh(guardSpace,mesh,0,0,0);dGeomSetPosition(guard,0,0,.20);
 dGeomID wall=dCreateBox(world,2,.10,2);dGeomSetPosition(wall,0,1.5,1.0);
 dGeomSetCategoryBits(wall,1);dGeomSetCollideBits(wall,~1u);
 dGeomID ray=dCreateRay(raySpace,10.);double dx=.05776,dy=.77775,dz=.15599,n=std::sqrt(dx*dx+dy*dy+dz*dz);
 dGeomRaySet(ray,.011,.02329,.13,dx/n,dy/n,dz/n);dGeomRaySetClosestHit(ray,1);
 dGeomSetCategoryBits((dGeomID)raySpace,2);dGeomSetCollideBits((dGeomID)raySpace,~2u);
 dGeomSetCategoryBits(ray,2);dGeomSetCollideBits(ray,~2u);
 Hits before;dSpaceCollide2((dGeomID)raySpace,(dGeomID)world,&before,rayCallback);
 dGeomSetCategoryBits((dGeomID)guardSpace,2);dGeomSetCollideBits((dGeomID)guardSpace,~2u);
 dGeomSetCategoryBits(guard,2);dGeomSetCollideBits(guard,~2u);
 Hits after;dSpaceCollide2((dGeomID)raySpace,(dGeomID)world,&after,rayCallback);
 dGeomSetPosition(wall,0,.285,.20);dGeomBoxSetLengths(wall,.10,.10,.10);
 Hits physical;dSpaceCollide(world,&physical,contactCallback);
 bool ok=before.nearest>.2 && before.nearest<.4 && after.nearest>1.0 && physical.contacts>0;
 std::cout<<"before_ray_range="<<before.nearest<<" after_ray_range="<<after.nearest<<" physical_contacts_after_mask="<<physical.contacts<<" result="<<(ok?"PASS":"FAIL")<<std::endl;
 dSpaceDestroy(raySpace);dSpaceDestroy(world);dGeomTriMeshDataDestroy(mesh);dCloseODE();return ok?0:1;
}
