#include <plan_env/vertical_obstacle_support.h>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <array>
using namespace fast_planner;
int main(int argc,char** argv) {
  if(argc!=9) return 2;
  double height=atof(argv[1]),base=atof(argv[2]),radius=atof(argv[3]),
         lo=atof(argv[4]),hi=atof(argv[5]),xy=atof(argv[6]),up=atof(argv[7]),down=atof(argv[8]);
  const double res=.05; const int nx=81,ny=81,nz=81,c=40;
  std::vector<std::array<double,3>> cloud;
  for(double z=0.;z<=height+1e-6;z+=.025) {
    const double r=z<base?.06:radius*(height-z)/(height-base);
    for(int i=0;i<160;++i) {
      const double a=i*2*3.141592653589793/160.;
      cloud.push_back({r*cos(a),r*sin(a),z});
    }
  }
  auto begin=std::chrono::steady_clock::now();
  VerticalObstacleSupport s(nx,ny,.15/res,3,.2);
  for(auto p:cloud) if(p[2]>=.4) s.observe(int(floor(p[0]/res))+c,int(floor(p[1]/res))+c,p[2]);
  MiddleHeightColumnSelector m(nx,ny,.15/res,lo,hi);
  for(auto p:cloud) if(p[2]>=.4) {
    const int x=int(floor(p[0]/res))+c,y=int(floor(p[1]/res))+c;
    if(s.supported(x,y)) m.observe(x,y,p[2]);
  }
  m.build();
  for(auto p:cloud) if(p[2]>=.4) m.observeBand(int(floor(p[0]/res))+c,int(floor(p[1]/res))+c,p[2]);
  UpwardObstacleColumns col(nx,ny,nz);
  const int ir=ceil(xy/res-1e-9),iu=ceil(up/res-1e-9),id=ceil(down/res-1e-9);
  m.forEachFootprint([&](int x,int y,double z){
    for(int i=std::max(0,x-ir);i<=std::min(nx-1,x+ir);++i)
      for(int j=std::max(0,y-ir);j<=std::min(ny-1,y+ir);++j) col.mark(i,j,int(floor(z/res))-id);
  });
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::cerr<<"support_and_middle_ms="<<ms<<" points="<<cloud.size()<<"\n";
  std::vector<unsigned char> real(nx*ny*nz,0);
  auto address=[&](int x,int y,int z){return (x*ny+y)*nz+z;};
  for(auto p:cloud) {
    const int x=int(floor(p[0]/res))+c,y=int(floor(p[1]/res))+c,z=int(floor(p[2]/res));
    for(int i=std::max(0,x-ir);i<=std::min(nx-1,x+ir);++i)
      for(int j=std::max(0,y-ir);j<=std::min(ny-1,y+ir);++j)
        for(int k=std::max(0,z-id);k<=std::min(nz-1,z+iu);++k) real[address(i,j,k)]=1;
  }
  std::cout<<"x,y,z,kind\n";
  for(auto p:cloud)std::cout<<p[0]<<","<<p[1]<<","<<p[2]<<",0\n";
  for(int x=0;x<nx;++x) for(int y=0;y<ny;++y) for(int z=0;z<64;++z) {
    const bool physical=real[address(x,y,z)],column=col.occupied(x,y,z);
    if(physical||column) std::cout<<(x-c+.5)*res<<","<<(y-c+.5)*res<<","<<(z+.5)*res<<","<<(physical?1:2)<<"\n";
  }
}
