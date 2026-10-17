#include "src/navigation/ReflectorDockingGate.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
using std::min;using std::max;using std::abs;
enum OperationType {OP_IDLE,OP_DOCK,OP_MOW,OP_CHARGE,OP_ERROR};
enum Sensor {SENS_NONE,SENS_BUMPER,SENS_OBSTACLE};
enum {EVT_USER_STOP,EVT_USER_DOCK,EVT_USER_START,EVT_CHARGER_CONNECTED,EVT_BUMPER_OBSTACLE};
struct {OperationType stateOp=OP_IDLE;Sensor stateSensor=SENS_NONE;ReflectorDockingGate reflectorDockingGate;bool dockReflectorStripsUndockCompleted=false;} stateEstimator;
struct {bool undock=false;bool isUndocking(){return undock;}} maps;
struct {int stops=0;void stopImmediately(bool all){(void)all;++stops;}} motor;
struct {void event(int){}} Logger;
struct {template<class T>void println(T){} } CONSOLE;
struct {bool connected=false;bool chargerConnected(){return connected;}void setIsDocked(bool){}} battery;
class Op {public: Op*nextOp=nullptr;Op*previousOp=nullptr;bool shouldStop=false;bool cameraEntryRequested=false;void changeOp(Op&,bool=false);void changeOperationTypeByOperator(OperationType);bool stopOnDockingObstacle(Sensor);void onChargerConnected();void setInitiatedByOperator(bool){};};
class ErrorOp:public Op {public:bool dockingObstacleStop=false;void run();void end();};
ErrorOp errorOp;Op dockOp,mowOp,idleOp,chargeOp;Op*activeOp=&idleOp;Op*newActiveOp=nullptr;
bool optionEnabled=true;
#define DOCK_STOP_ON_OBSTACLE optionEnabled
#include "obstacle-production.inc"
bool marker=true,front=true;float imagePosition=0;
bool cameraReflectorStripsFound(){return marker;}float cameraReflectorStripsHorizontalError(){return imagePosition;}float cameraReflectorStripsConfidence(){return .9f;}
struct {bool isPlaying(){return true;}void sound(int,bool){}} buzzer;
#define SND_WARNING 1
#define DOCK_REFLECTOR_STRIPS_ANGULAR_P .7f
#define DOCK_REFLECTOR_STRIPS_MAX_ANGULAR .35f
#define DOCK_REFLECTOR_STRIPS_MAX_ERROR_FOR_DRIVE .65f
#define DOCK_LINEAR_SPEED .1f
#define DOCK_FRONT_SIDE front
float linear=0,angular=0;bool mow=true;
void track(){
 bool runControl=true;
#include "tracking-production.inc"
}
#define UNDOCK_TEST_TIMEOUT_MS 300000UL
#define DOCK_REFLECTOR_STRIPS
unsigned long testNow=0,startTime=0;
unsigned long millis(){return testNow;}
void changeOp(Op& op,bool chain){mowOp.changeOp(op,chain);}
bool trackingReached=false;
void undockTestRun(){
#include "undock-test-production.inc"
 trackingReached=true;
}
int main(){
 maps.undock=true;newActiveOp=nullptr;activeOp=&mowOp;
 testNow=299999;undockTestRun();assert(trackingReached&&!newActiveOp);
 trackingReached=false;testNow=300000;undockTestRun();assert(!trackingReached&&newActiveOp==&idleOp);
 testNow=1;newActiveOp=nullptr;stateEstimator.dockReflectorStripsUndockCompleted=true;
 undockTestRun();assert(!trackingReached&&newActiveOp==&idleOp);
 stateEstimator.dockReflectorStripsUndockCompleted=false;maps.undock=false;newActiveOp=nullptr;
 undockTestRun();assert(!trackingReached&&newActiveOp==&idleOp);
 motor.stops=0;newActiveOp=nullptr;
 stateEstimator.stateOp=OP_DOCK;activeOp=&dockOp;
 assert(dockOp.stopOnDockingObstacle(SENS_BUMPER));assert(motor.stops==1&&newActiveOp==&errorOp&&errorOp.dockingObstacleStop);assert(stateEstimator.stateSensor==SENS_BUMPER&&!errorOp.nextOp);
 activeOp=&errorOp;newActiveOp=nullptr;battery.connected=true;
 errorOp.run();assert(!newActiveOp);errorOp.onChargerConnected();assert(!newActiveOp);errorOp.changeOp(mowOp,true);assert(!newActiveOp);
 errorOp.changeOperationTypeByOperator(OP_IDLE);assert(newActiveOp==&idleOp&&!errorOp.dockingObstacleStop);errorOp.end();
 stateEstimator.stateOp=OP_MOW;maps.undock=true;activeOp=&mowOp;newActiveOp=nullptr;
 assert(mowOp.stopOnDockingObstacle(SENS_OBSTACLE));assert(newActiveOp==&errorOp&&stateEstimator.stateSensor==SENS_OBSTACLE);
 errorOp.end();maps.undock=false;newActiveOp=nullptr;int stops=motor.stops;
 assert(!mowOp.stopOnDockingObstacle(SENS_BUMPER)&&motor.stops==stops&&!newActiveOp);
 stateEstimator.stateOp=OP_IDLE;maps.undock=true;assert(!idleOp.stopOnDockingObstacle(SENS_BUMPER));maps.undock=false;
 optionEnabled=false;stateEstimator.stateOp=OP_DOCK;assert(!dockOp.stopOnDockingObstacle(SENS_BUMPER));optionEnabled=true;
 // A bounced charger contact during Mow/Undock cannot enter Charge/retry.
 stateEstimator.stateOp=OP_MOW;activeOp=&mowOp;newActiveOp=nullptr;
 maps.undock=true;mowOp.onChargerConnected();assert(!newActiveOp);
 maps.undock=false;mowOp.onChargerConnected();assert(!newActiveOp);
 stateEstimator.stateOp=OP_IDLE;activeOp=&idleOp;
 idleOp.onChargerConnected();assert(newActiveOp==&chargeOp);newActiveOp=nullptr;
 auto&g=stateEstimator.reflectorDockingGate;
 for(auto phase:{g.AcquireDock,g.AcquireUndock,g.Blocked,g.Inactive}){g.phase=phase;linear=1;angular=1;track();assert(linear==0&&angular==0&&!mow);}
 for(bool reverse:{false,true}){maps.undock=reverse;g.phase=reverse?g.Undock:g.Dock;marker=false;track();assert(linear==0&&angular==0);marker=true;imagePosition=.1;track();assert(reverse?linear<0:linear>0);assert(angular<0);}
 front=false;maps.undock=false;track();assert(linear<0);maps.undock=true;track();assert(linear>0);
 imagePosition=.8;track();assert(linear==0&&angular<0);
 std::puts("PASS production camera motion/stop, both docking directions, bumper/obstacle latch, charger-event guard, explicit release, option disabled");
}
