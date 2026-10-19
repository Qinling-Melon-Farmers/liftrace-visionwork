#include <fstream>
#include <iomanip>
#include <iostream>
#include "patrol_control/landing_high_alignment.h"
int main(int argc,char**argv){
 if(argc!=3)return 2;
 std::ifstream in(argv[1]);std::ofstream out(argv[2]);
 patrol_control::LandingHighAlignmentWindow gate;
 double now,odom,receipt,mark,mr,xy,speed;int valid;int n=0,ready=0;
 out<<"now,ready\n"<<std::setprecision(17);
 while(in>>now>>odom>>receipt>>mark>>mr>>xy>>speed>>valid){
  bool r=gate.update(now,odom,receipt,mark,mr,xy,speed,valid!=0);
  out<<now<<","<<r<<"\n";n++;ready+=r;
 }
 std::cout<<"samples="<<n<<" ready="<<ready<<"\n";
 return !in.eof();
}