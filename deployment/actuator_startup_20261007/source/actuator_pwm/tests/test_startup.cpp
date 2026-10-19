#include "actuator_pwm/CheckedPulse.h"
#include "actuator_pwm/PassiveStartup.h"
#include "actuator_pwm/PWMController.h"
#include "actuator_pwm/SlotCalibration.h"
#include <sys/inotify.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

void require(bool result, const char* what) {
    if (!result) throw std::runtime_error(what);
}
struct FakeReadChecks {
    std::map<std::string,std::string> values{
        {"/pwm/period","20000000"},{"/pwm/polarity","normal"},{"/pwm/enable","0"}};
    std::string denied;
    std::vector<std::string> reads, permissions;
    bool read(const std::string& path,std::string& value) {
        reads.push_back(path);
        if (!values.count(path)) return false;
        value=values[path]; return true;
    }
    bool writable(const std::string& path) {
        permissions.push_back(path); return path!=denied;
    }
};
struct FakePWM {
    FakeReadChecks checks;
    std::vector<std::string> writes;
    int failWrite=0;
    bool record(const std::string& event) {
        writes.push_back(event); return int(writes.size())!=failWrite;
    }
    bool validatePassiveStartup() {return checkedPassiveConfiguration(checks,"/pwm");}
    bool disable() {return record("disable");}
    bool enable() {return record("enable");}
    bool setDutyCycle(unsigned duty) {return record("duty="+std::to_string(duty));}
    bool setPeriod(unsigned period) {return record("period="+std::to_string(period));}
    bool setPolarity(const std::string& polarity) {return record("polarity="+polarity);}
};

// This fixture substitutes only a temporary filesystem root, never /sys or hardware.
// IN_MODIFY catches even writing the same enable/duty value during destruction.
class TempSysfs {
public:
    std::string root, path;
    int monitor=-1;
    TempSysfs() {
        char pattern[]="/tmp/actuator_passive_test_XXXXXX";
        char* made=mkdtemp(pattern); require(made,"mkdtemp"); root=made;
        require(mkdir((root+"/test.pwm").c_str(),0700)==0,"mkdir device");
        require(mkdir((root+"/test.pwm/pwmchip5").c_str(),0700)==0,"mkdir chip");
        path=root+"/test.pwm/pwmchip5/pwm0";
        require(mkdir(path.c_str(),0700)==0,"mkdir channel");
        require(symlink((root+"/test.pwm/pwmchip5").c_str(),(root+"/pwmchip5").c_str())==0,"symlink");
        set("period","20000000"); set("polarity","normal");
        set("enable","0"); set("duty_cycle","700000");
        monitor=inotify_init1(IN_NONBLOCK|IN_CLOEXEC); require(monitor>=0,"inotify");
        for(const char* field : {"period","polarity","enable","duty_cycle"})
            require(inotify_add_watch(monitor,(path+"/"+field).c_str(),IN_MODIFY|IN_ACCESS)>=0,"watch");
    }
    ~TempSysfs() {
        if(monitor>=0) close(monitor);
        for(const char* field : {"period","polarity","enable","duty_cycle"}) {
            chmod((path+"/"+field).c_str(),0600); unlink((path+"/"+field).c_str());
        }
        rmdir(path.c_str()); unlink((root+"/pwmchip5").c_str());
        rmdir((root+"/test.pwm/pwmchip5").c_str()); rmdir((root+"/test.pwm").c_str()); rmdir(root.c_str());
    }
    void set(const std::string& field,const std::string& value) {
        std::ofstream out(path+"/"+field); out<<value; out.close();
        require(bool(out),"fixture write");
    }
    std::pair<int,int> events() {
        alignas(inotify_event) char buffer[4096]; int writes=0,reads=0;
        for(;;) {
            const ssize_t size=read(monitor,buffer,sizeof(buffer));
            if(size<0) {require(errno==EAGAIN,"inotify read");break;}
            if(size==0) break;
            for(ssize_t pos=0;pos<size;) {
                const auto* event=reinterpret_cast<inotify_event*>(buffer+pos);
                if(event->mask&IN_MODIFY) ++writes;
                if(event->mask&IN_ACCESS) ++reads;
                pos+=sizeof(inotify_event)+event->len;
            }
        }
        return {writes,reads};
    }
};

int main() {
    try {
        require(actuator_pwm::kInitialDutyNs[0]==1700000 && actuator_pwm::kReleaseDutyNs[0]==2100000,"field requested rear release 2100us");
        require(actuator_pwm::kInitialDutyNs[1]==1000000 && actuator_pwm::kInitialDutyNs[2]==1100000 &&
                actuator_pwm::kReleaseDutyNs[1]==2100000 && actuator_pwm::kReleaseDutyNs[2]==2100000,"right/left unchanged");
        int cases=2;
        for(int slot=0;slot<3;++slot) {
            FakePWM pwm; int waits=0;
            require(checkedStartup(pwm,actuator_pwm::kInitialDutyNs[slot],[&](){++waits;}),"default passive");
            require(waits==0 && pwm.writes.empty(),"passive zero writes/no waits");
            require(pwm.checks.reads==std::vector<std::string>{"/pwm/period","/pwm/polarity","/pwm/enable"},"required readchecks");
            require(pwm.checks.permissions==std::vector<std::string>{"/pwm/period","/pwm/duty_cycle","/pwm/polarity","/pwm/enable"},"all write permissions");
            ++cases;
        }
        for(const auto& bad : std::vector<std::pair<std::string,std::string>>{
                {"period","0"},{"period","19999999"},{"polarity","inversed"},{"enable","1"}}) {
            FakePWM pwm; pwm.checks.values["/pwm/"+bad.first]=bad.second;
            require(!checkedStartup(pwm,1700000,[](){}),"bad passive configuration fails");
            require(pwm.writes.empty(),"configuration failure zero writes"); ++cases;
        }
        for(const char* field : {"period","polarity","enable"}) {
            FakePWM pwm; pwm.checks.values.erase(std::string("/pwm/")+field);
            require(!checkedStartup(pwm,1700000,[](){}),"missing read fails");
            require(pwm.writes.empty(),"missing read zero writes"); ++cases;
        }
        for(const char* field : {"period","duty_cycle","polarity","enable"}) {
            FakePWM pwm; pwm.checks.denied=std::string("/pwm/")+field;
            require(!checkedStartup(pwm,1700000,[](){}),"permission failure");
            require(pwm.writes.empty(),"permission failure zero writes"); ++cases;
        }
        for(int slot=0;slot<3;++slot) {
            FakePWM pwm; int waits=0;
            require(checkedStartup(pwm,actuator_pwm::kInitialDutyNs[slot],[&](){++waits;},true),"explicit reset succeeds");
            const std::vector<std::string> expected={"disable","duty=0","period=20000000","polarity=normal",
                "disable","duty="+std::to_string(actuator_pwm::kInitialDutyNs[slot]),"enable","disable"};
            require(pwm.writes==expected && waits==1 && pwm.checks.reads.empty(),"old checked reset transaction retained"); ++cases;
        }
        for(int failure=1;failure<=8;++failure) {
            FakePWM pwm; pwm.failWrite=failure; int waits=0;
            require(!checkedStartup(pwm,1700000,[&](){++waits;},true),"reset write fault fails");
            require(waits==(failure==8?1:0),"reset fault wait gating");
            require(int(pwm.writes.size())==(failure==7?8:failure),"reset fault stops/enable cleanup"); ++cases;
        }
        for(int slot=0;slot<3;++slot) {
            FakePWM pwm; int waits=0;
            require(checkedPulse(pwm,actuator_pwm::kReleaseDutyNs[slot],[&](){++waits;}),"mock release transaction");
            require(pwm.writes==std::vector<std::string>{"disable","duty="+std::to_string(actuator_pwm::kReleaseDutyNs[slot]),"enable","disable"} && waits==1,"release calibration/transaction"); ++cases;
        }
        for(int failure=1;failure<=4;++failure) {
            FakePWM pwm; pwm.failWrite=failure; int waits=0;
            require(!checkedPulse(pwm,700000,[&](){++waits;}),"release fault fails");
            require(waits==(failure==4?1:0) && int(pwm.writes.size())==(failure==3?4:failure),"release stop/cleanup"); ++cases;
        }
        {
            TempSysfs fs; fs.events();
            { PWMController pwm(5,0,"test.pwm",fs.root);
              require(checkedStartup(pwm,1700000,[](){throw std::runtime_error("unexpected wait");}),"real passive readchecks"); }
            const auto events=fs.events();
            require(events.first==0 && events.second>=3,"production constructor/startup/destructor zero writes, actual reads");
            std::ifstream input(fs.path+"/duty_cycle"); std::string duty;input>>duty;
            require(duty=="700000","preexisting duty unchanged"); ++cases;
        }
        for(const auto& bad : std::vector<std::pair<std::string,std::string>>{
                {"period","10"},{"period","20000000 extra"},{"polarity","inversed"},{"enable","1"}}) {
            TempSysfs fs; fs.set(bad.first,bad.second); fs.events();
            { PWMController pwm(5,0,"test.pwm",fs.root);
              require(!pwm.validatePassiveStartup(),"production invalid config fails"); }
            require(fs.events().first==0,"production failure/destructor zero writes"); ++cases;
        }
        require(geteuid()!=0,"permission tests require ordinary developer user");
        for(const char* field : {"period","duty_cycle","polarity","enable"}) {
            TempSysfs fs; require(chmod((fs.path+"/"+field).c_str(),0400)==0,"fixture chmod"); fs.events();
            bool failed=false;
            try { PWMController pwm(5,0,"test.pwm",fs.root); failed=!pwm.validatePassiveStartup(); }
            catch(const std::runtime_error&) {failed=true;}
            require(failed && fs.events().first==0,"production permission failure zero writes"); ++cases;
        }
        {
            TempSysfs fs; require(chmod((fs.path+"/period").c_str(),0000)==0,"unreadable"); fs.events();
            {PWMController pwm(5,0,"test.pwm",fs.root); require(!pwm.validatePassiveStartup(),"unreadable read fails");}
            require(fs.events().first==0,"unreadable failure zero writes"); ++cases;
        }
        {
            TempSysfs fs; fs.events(); bool failed=false;
            try {PWMController first(5,0,"test.pwm",fs.root); PWMController second(5,0,"wrong.pwm",fs.root);}
            catch(const std::runtime_error&) {failed=true;}
            require(failed && fs.events().first==0,"address mismatch/unwind zero writes"); ++cases;
        }
        {
            TempSysfs fs;
            {PWMController pwm(5,0,"test.pwm",fs.root); require(pwm.enable(),"fixture-only active write");fs.events();}
            std::ifstream input(fs.path+"/enable");std::string enabled;input>>enabled;
            require(enabled=="0" && fs.events().first>0,"active destructor still disables fixture output"); ++cases;
        }
        std::cout<<"PASS "<<cases<<" cases: passive zero writes/readchecks, invalid config/permissions, explicit reset and pulse faults; no hardware/ROS node run\n";
    } catch(const std::exception& error) {
        std::cerr<<"FAIL: "<<error.what()<<"\n";return 1;
    }
    return 0;
}
