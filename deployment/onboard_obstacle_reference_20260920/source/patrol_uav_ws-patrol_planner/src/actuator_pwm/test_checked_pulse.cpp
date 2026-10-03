#include "actuator_pwm/CheckedPulse.h"
#include <cassert>
struct Fake {
 int n=0,fail=0,waits=0,enables=0;
 bool step(){return ++n!=fail;}
 bool disable(){return step();}
 bool setDutyCycle(unsigned){return step();}
 bool setPeriod(unsigned){return step();}
 bool setPolarity(const char*){return step();}
 bool enable(){++enables;return step();}
};
int main(){
 Fake ok; assert(checkedPulse(ok,1700000,[&](){++ok.waits;})); assert(ok.waits==1);
 for(int f=1;f<=4;++f){Fake p;p.fail=f;assert(!checkedPulse(p,1700000,[&](){++p.waits;}));if(f<3)assert(p.enables==0);}
 Fake init;assert(checkedInitialize(init,700000,[](){}));
 for(int f=1;f<=8;++f){Fake p;p.fail=f;assert(!checkedInitialize(p,700000,[](){}));}
}
