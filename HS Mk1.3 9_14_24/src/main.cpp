/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       declan                                                    */
/*    Created:      6/20/2024, 3:56:54 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;

motor RB = motor(PORT2, ratio6_1, false);
motor RC = motor(PORT3, ratio6_1, true);
motor RF = motor(PORT1, ratio6_1, false);
motor LB = motor(PORT8, ratio6_1, true);
motor LC = motor(PORT7, ratio6_1, false);
motor LF = motor(PORT6, ratio6_1, true);

motor_group DriveL = motor_group(LB, LC, LF);
motor_group DriveR = motor_group(RB, RC, RF);


motor Intake = motor(PORT19, ratio6_1, false);

motor Lift = motor(PORT4,ratio36_1,false);

digital_out bMog = digital_out(Brain.ThreeWirePort.F);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.G);
digital_out fMog = digital_out(Brain.ThreeWirePort.H);
digital_out hang = digital_out(Brain.ThreeWirePort.D);

inertial Inertial = inertial(PORT20);
rotation RotationDrive = rotation(PORT17);
distance ringDetect = distance(PORT12);
optical optColor = optical(PORT14);
limit selectAuton = limit(Brain.ThreeWirePort.E);

bool homeColor = false; //false red, true blue
bool autoIntakeFWDOn = false;
bool autoIntakeREVOn = false;
bool autoIntakeBackout = false;

void spinDrive(double rSpeed,double lSpeed){
  RF.spin(fwd,rSpeed,pct);
  RC.spin(fwd,rSpeed,pct);
  RB.spin(fwd,rSpeed,pct);
  LF.spin(fwd,lSpeed,pct);
  LC.spin(fwd,lSpeed,pct);
  LB.spin(fwd,lSpeed,pct);
}

void spinDriveVolt(double rSpeed,double lSpeed){
  rSpeed = rSpeed*.12;
  lSpeed = lSpeed*.12;
  RF.spin(fwd,rSpeed,volt);
  RC.spin(fwd,rSpeed,volt);
  RB.spin(fwd,rSpeed,volt);
  LF.spin(fwd,lSpeed,volt);
  LC.spin(fwd,lSpeed,volt);
  LB.spin(fwd,lSpeed,volt);
}

void start(){
  Lift.setVelocity(100, percent);
  Lift.spinFor(forward, 0.2, seconds);
  Lift.spinFor(reverse, 0.2, seconds);
}

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

void drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

float get_right_position_in(){
  //return( (DriveR.position(deg)*0.8)*(M_PI/360)*2.75);
  return( (RotationDrive.position(deg))*(M_PI/360)*2.02);
}
float get_left_position_in(){
  //return( (DriveL.position(deg)*0.8)*(M_PI/360)*2.75);
  return( (RotationDrive.position(deg))*(M_PI/360)*2.02);
}

float DkP = 1;
float DkD = 0.2;
float DkI = 0;
float Derr = 0;
float pErr = 0;
float dDeriv = 0;
float DIntegral = 0;
float dpw = 0;


void drivePID(float setPoint, float multi){
  RB.resetPosition();
  LB.resetPosition();

  float DIstance =0;
  float inches = 0;
 while (inches<setPoint-0.1 or inches>setPoint+0.1){
  DIstance = (LB.position(degrees))+(RB.position(degrees))/2;
  inches = (DIstance*0.8)*(M_PI/360)*2.75;
 Derr = setPoint - inches;
 DIntegral = DIntegral + Derr;
 if (Derr == 0 or fabs(inches) > fabs(setPoint)){
 DIntegral = 0;
 }
 if (Derr>15){
 DIntegral = 0;
 }
 dDeriv = Derr - pErr;
 pErr = Derr;
 dpw = Derr*DkP+ dDeriv*DkD;
 //+ dDIntEgral*dDkI;
 LB.spin(forward, dpw*multi, percent);
 LC.spin(forward, dpw*multi, percent);
 LF.spin(forward, dpw*multi, percent);
 RB.spin(forward, dpw*multi, percent);
 RC.spin(forward, dpw*multi, percent);
 RF.spin(forward, dpw*multi, percent);
 wait (15, msec);
 }
 LB.stop(brake);
 LC.stop(brake);
 LF.stop(brake);
 RB.stop(brake);
 RC.stop(brake);
 RF.stop(brake);
}



void drivPID(float setPoint, float multi, float kp, float kD){
  RB.resetPosition();
  LB.resetPosition();

  float DIstance =0;
  float inches = 0;
 while (inches<setPoint-0.1 or inches>setPoint+0.1){
  DIstance = (LB.position(degrees))+(RB.position(degrees))/2;
  inches = (DIstance*0.8)*(M_PI/360)*2.75;
 Derr = setPoint - inches;
 DIntegral = DIntegral + Derr;
 if (Derr == 0 or fabs(inches) > fabs(setPoint)){
 DIntegral = 0;
 }
 if (Derr>15){
 DIntegral = 0;
 }
 dDeriv = Derr - pErr;
 pErr = Derr;
 dpw = Derr*kp+ dDeriv*kD;
 //+ dDIntEgral*dDkI;
 LB.spin(forward, dpw*multi, percent);
 LC.spin(forward, dpw*multi, percent);
 LF.spin(forward, dpw*multi, percent);
 RB.spin(forward, dpw*multi, percent);
 RC.spin(forward, dpw*multi, percent);
 RF.spin(forward, dpw*multi, percent);
 wait (15, msec);
 }
 LB.stop(brake);
 LC.stop(brake);
 LF.stop(brake);
 RB.stop(brake);
 RC.stop(brake);
 RF.stop(brake);
}

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle <= 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}



float get_absolute_heading(){ 
  return( reduce_0_to_360( Inertial.rotation()) ); 
}


void turnPID(float setPoint, float tim){
  float ttKP = 0.15;
  float ttKD = 1; 
  float ttKI = 0.0;
  float ttErr = 100;
  float ttPErr = 0;
  float ttDeriv = 0;
  float ttIntEgral = 0;
  float tpw;
  float texitcondition = 0;
  float head = 0;
  float ttoffset = 1;
  float ttTimeDoubleCheck = 20;
  float dTim = 0;
  float maxTVoltage = 12;
  
  reduce_negative_180_to_180(setPoint - get_absolute_heading());

  float offset = 0;

  while (texitcondition< tim and dTim<ttTimeDoubleCheck){
  //while ((ttErr<2 or ttErr>-2 and texitcondition< tim)){
    
    ttErr = reduce_negative_180_to_180(setPoint - get_absolute_heading());
    
    
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(setPoint);
    //Controller1.Screen.setCursor(2,1);
    //Controller1.Screen.print(ttErr);
    //Controller1.Screen.setCursor(3,1);
    //Controller1.Screen.print(texitcondition);

    
    ttIntEgral = ttIntEgral - ttErr;
    if (ttErr == 0 or fabs(-Inertial.rotation(degrees)) > fabs(setPoint)){
    ttIntEgral = 0;
    }
    if (ttErr>15){
    ttIntEgral = 0;
    }
    ttDeriv = ttErr - ttPErr;
    ttPErr = ttErr;
    tpw = ttErr*ttKP + ttDeriv*ttKD + ttIntEgral*ttKI + ttDeriv*ttKD;
    
    
    if (ttErr > 0){
      offset = ttoffset;
    }else{
      offset = ttoffset*-1;
    }
    if (!(ttErr>0.5 or ttErr<-0.5)){
      dTim = dTim+5;
    }else{
      dTim = 0;
    }
    DriveL.spin(forward, clamp(tpw+offset,-maxTVoltage,maxTVoltage), volt);
    DriveR.spin(reverse, clamp(tpw+offset,-maxTVoltage,maxTVoltage), volt);
    texitcondition =texitcondition+ 5;
    wait (5, msec);
 
 }
 DriveL.stop(brake);
 DriveR.stop(brake);
}




void drive(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_timeout, float drive_settle_error = 1.5, float drive_settle_time = 20, float drive_kp = 2.6, float drive_ki = 0, float drive_kd = 10, float drive_starti = 0, float heading_kp = 0.3, float heading_ki = 0 , float heading_kd = 1.5, float heading_starti = 0){
  PID drivePID(distance, 1, 0, 8, 0, drive_settle_error, 300, 3000);
  PID headingPID(reduce_negative_180_to_180(heading - Inertial.heading()), 0.3, 0, 1.2, 0);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  int timePassed = 0;

  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);
    
    float maxVoltStart = 6;
    float maxVoltLimit = 1;
    if (drive_error<0){
      maxVoltStart = 5;
      maxVoltLimit = 0.9;
    }
    
    
    float maxVolt = maxVoltStart+fabs(start_average_position-average_position)*maxVoltLimit;

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_output = clamp(drive_output, -maxVolt, maxVolt);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
    timePassed = timePassed+10;
    if (timePassed>=drive_timeout){break;}
  }
  if (drive_settle_error<5){
    DriveL.stop(brake);
    DriveR.stop(brake);
  }else{
    DriveL.stop(coast);
    DriveR.stop(coast);
  }
}

int current_auton_selection = 0;
bool auto_started = false;
int colorSelect = 0;

bool intakeRedirectToggle = false;
bool intakeButTog = false;
bool overrideColor = false;
bool colorDeflect = true;
bool colorDeflectButTog = false;

// void brainDisp(){
//   while(true){
//     if(colorDeflect == false){
//       Brain.Screen.setFillColor(green);
//       Brain.Screen.drawRectangle(0,0,100,100);
//     }
//     if(intakeTog == true){
//       Controller1.rumble(".");
//     }
//   }
// }

void pre_auton(void) { 
  // Initializing Robot Configuration. DO NOT REMOVE!
  bMog.set(false);
Inertial.calibrate();
while(Inertial.isCalibrating()){
  wait(50,msec);
}
Controller1.Screen.print("Calibrated");
  while(auto_started == false){  
    Brain.Screen.setCursor(1,1);          //Changing the names below will only change their names on the
    Brain.Screen.clearScreen();
    if (homeColor == false){
      Brain.Screen.setFillColor(13);
    }else{
      Brain.Screen.setFillColor(190);
    }
    Brain.Screen.setCursor(1,1);
    Brain.Screen.drawRectangle(0,0,500,250);
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFont(mono40);
    int xpos = 0;
    int ypos = 125;
    switch(current_auton_selection){
    case 0:
      Brain.Screen.printAt(xpos, ypos, "Test");
      homeColor = true;  // True is Blue                False is Red
      break;
    case 1:
      Brain.Screen.printAt(xpos, ypos, "Blue_Negative_Elims");
      homeColor = true;
      break;
    case 2:
      Brain.Screen.printAt(xpos, ypos, "Red_Negative_Elims");
      homeColor = false;
      break;
    case 3:
      Brain.Screen.printAt(xpos, ypos, "Blue_Positive_Elims");
      homeColor = true;
      break;
    case 4:
      Brain.Screen.printAt(xpos, ypos, "Red_Positive_Elims");
      homeColor = false;
      break;
    case 5:
      Brain.Screen.printAt(xpos, ypos, "Blue_Positive_GoalRush");
      homeColor = true;
      break;
    case 6:
      Brain.Screen.printAt(xpos, ypos, "Red_Positive_GoalRush");
      homeColor = false;
      break;
    case 7:
      Brain.Screen.printAt(xpos, ypos, "mogoSide_BLUE_3Ring");
      homeColor = false;
      break;
    case 8:
      Brain.Screen.printAt(xpos, ypos, "skillsMoa");
      homeColor = true;
      break;
    }
    if(selectAuton.pressing()){
      while(selectAuton.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
    task::sleep(100);
  }
}

void liftMov(){
  Lift.spinFor(forward, 0.75, seconds);
  Lift.setVelocity(65, pct);
  Lift.spinFor(reverse, 1.25, seconds);
}

void intakeDivert(){
  while(true){
  if (ringDetect.objectDistance(mm)>7){ 
          Intake.spin(forward, 100, percent);
          // once the distance sensor registers that a ring is there it will wait 30 msecs to get the ring into the correct spot 
          //and then reverse the intake to redirect them into the wall stake mechanism
        } else if(ringDetect.objectDistance(mm)<7){
          wait(20, msec);
          Intake.setVelocity(-100, percent);
          Intake.spinFor(1, seconds);
        }
  }
}

void intakeExpel(){
  while(true){
    bool detectCol = false;
    optColor.setLightPower(100,pct);
        //detect if the color that is detected is either red or blue
        if(homeColor == false){
          //if the homeColor is true that means that we are red and want to expel the blue rings
          //if the value is between 140 and 230 that is blue which sets detectCol to true to then expel them
          detectCol = 140<optColor.hue() and optColor.hue()<230;
        }else{
          //if the homeColor is false that means that we are blue and want to expel the red rings
          //if the value is between 0 and 40 that is red which sets detectCol to true to then expel them
          detectCol = 0<optColor.hue() and optColor.hue()<40;
        }
    if (ringDetect.objectDistance(mm)>7){
            Intake.spin(forward, 100, percent);
          } else if(ringDetect.objectDistance(mm)<7 and detectCol){
            wait(110, msec);
            Intake.setVelocity(-100, percent);
            Intake.spinFor(0.2, seconds);
          }
  }
}

void repTurn(float angle,int times){
  for(int i=0;i<times;i++){
    turnPID(angle,3000);
    turnPID(0,3000);
  }
}

double bMogwaitTime = 0;
bool bMogState = false;

void bMogoTime(){
  wait(bMogwaitTime,msec);
  bMog.set(bMogState);
}

double intakeLiTime = 0;
bool intakeLiState = false;

void intakeLiftTime(){
  wait(intakeLiTime,msec);
  intakeLift.set(intakeLiState);
}

double fMogwaitTime = 0;
bool fMogState = false;

void fMogoTime(){
  wait(fMogwaitTime,msec);
  fMog.set(fMogState);
}

double liftwaitTime = 0;
double liftState = 0;

void liftTime(){
  wait(liftwaitTime,msec);
  Lift.spinToPosition(liftState,deg,false);
}
void testFunctions(){
  /*
  repTurn(90,2);
  repTurn(45,2);
  repTurn(180,2);
  repTurn(30,2);
  drive(24,0,12,12,5000);
  turnPID(90,2000);
  drive(24,90,12,12,5000);
  turnPID(225,2000);
  drive(24*sqrt(2),225,12,12,5000);
  turnPID(0,2000);
  */
  drive(24,0,12,12,5000);
  wait(5,sec);
  drive(-24,0,12,12,5000);
  //drive(3,0,12,12,5000);
  //drive(-3,0,12,12,5000);
}

void Blue_Negative_4(){
  //Lift.spin(fwd,100,pct);
  Lift.setVelocity(100,pct);
  Lift.spinFor(260,deg,false);
  wait(0.5,sec);
  drive(7.5, 3, 6, 12, 2000);
  //Lift.spin(reverse,100,pct);
  Lift.spinFor(-140,deg,false);
  //Lift.spinFor(reverse,360,deg,100,pct,false);
  wait(0.275, sec);
  drive(-7.5, 0, 9, 12, 2000);
  Lift.spinFor(-120,deg,false);
  turnPID(-40,1000);
  drive(-20.8, -40, 12, 12, 2000);
  turnPID(-5,1000);
  bMogwaitTime = 400;
  bMogState = true;
  thread z(bMogoTime);
  //This is where it is fucking up about 1/3 of the time and the inertial goes crazy
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLift.set(true);
  drive(-9, -5, 6, 12, 2000);
  autoIntakeFWDOn = true;
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 600;
  intakeLiState = false;
  thread z1(intakeLiftTime);
  drive(20.5, 3, 9, 12, 2000);
  drive(-8, 16, 12, 12, 2000);
  turnPID(-155,1000);
  drive(35, -155, 12, 12, 2000);
  turnPID(-70,1000);
  Lift.setStopping(hold);
  Lift.spinToPosition(300,deg,false);
  drive(41, -70, 12, 12, 1200);
  wait(0.3,sec);
  drive(-12, -70, 12, 12, 2000);
  Lift.spinToPosition(0,deg,false);
  turnPID(75,1000);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  bMogwaitTime = 600;
  bMogState = false;
  thread x(bMogoTime);
  fMogwaitTime = 700;
  fMogState = true;
  thread x1(fMogoTime);
  drive(48,75,12,12,2000,20);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  autoIntakeFWDOn = false;
}

void Red_Negative_4(){
  //Lift.spin(fwd,100,pct);
  Lift.setVelocity(100,pct);
  Lift.spinFor(260,deg,false);
  wait(0.5,sec);
  drive(7.5, 1, 6, 12, 2000);
  //Lift.spin(reverse,100,pct);
  Lift.spinFor(-140,deg,false);
  //Lift.spinFor(reverse,360,deg,100,pct,false);
  wait(0.275, sec);
  drive(-7.5, 0, 9, 12, 2000);
  Lift.spinFor(-120,deg,false);
  turnPID(40,1000);
  drive(-20.8, 40, 12, 12, 2000);
  turnPID(5,1000);
  bMogwaitTime = 400;
  bMogState = true;
  thread z(bMogoTime);
  //This is where it is fucking up about 1/3 of the time and the inertial goes crazy
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLift.set(true);
  drive(-10, 5, 6, 12, 2000);
  autoIntakeFWDOn = true;
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 600;
  intakeLiState = false;
  thread z1(intakeLiftTime);
  drive(21.5, -3, 9, 12, 2000);
  drive(-8, -16, 12, 12, 2000);
  turnPID(155,1000);
  drive(35, 155, 12, 12, 2000);
  turnPID(70,1000);
  Lift.setStopping(hold);
  Lift.spinToPosition(300,deg,false);
  drive(41, 70, 12, 12, 1200);
  wait(0.3,sec);
  drive(-12, 70, 12, 12, 2000);
  Lift.spinToPosition(0,deg,false);
  turnPID(-65,1000);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  bMogwaitTime = 600;
  bMogState = false;
  thread x(bMogoTime);
  fMogwaitTime = 700;
  fMogState = true;
  thread x1(fMogoTime);
  drive(48,-65,12,12,2000,20);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  autoIntakeFWDOn = false;
}

void Red_Positive_GoalRush(){
  Lift.spinToPosition(200,deg,false);
  wait(0.1,sec);
  autoIntakeFWDOn = true;
  drive(5, 0, 12, 12, 800);
  wait(0.3,sec);
  autoIntakeFWDOn = false;
  turnPID(41,1000);
  Lift.spinToPosition(0,deg,false);
  drive(-38, 41, 12, 12, 2000);
  //turnPID(10,1000);
  bMogwaitTime = 750;
  bMogState = true;
  thread z(bMogoTime);
  drive(-15.5, 16, 12, 12, 2000);
  autoIntakeFWDOn = true;
  turnPID(55,1000);
  drive(22, 55, 12, 12, 2000);
  turnPID(93,1000);
  bMog.set(false);
  Lift.spinToPosition(260,deg,false);
  liftwaitTime = 1300;
  liftState = 0;
  thread y1(liftTime);
  drive(50, 93, 12, 12, 1700);
  Lift.spinToPosition(0,deg,false);
  wait(0.2,sec);
  drive(-7.5, 93, 9, 12, 2000);
  //Lift.spinFor(-120,deg,false);
  turnPID(59,1000);
  bMogwaitTime = 800;
  bMogState = true;
  thread z2(bMogoTime);
  intakeLift.set(true);
  drive(-29, 59, 12, 12, 2000);
  //autoIntakeFWDOn = true;
  turnPID(90,1000);
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 700;
  intakeLiState = false;
  thread z3(intakeLiftTime);
  drive(21, 90, 9, 12, 2000);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  //bMogwaitTime = 300;
  //bMogState = false;
  //thread x(bMogoTime);
  fMogwaitTime = 400;
  fMogState = true;
  thread x1(fMogoTime);
  drive(25,-132,12,12,2000,20);
  DriveL.stop(hold);
  DriveR.stop(hold);
  //autoIntakeFWDOn = false;

  
}

void Blue_Positive_GoalRush(){
  Lift.spinToPosition(200,deg,false);
  wait(0.1,sec);
  autoIntakeFWDOn = true;
  drive(5, 0, 12, 12, 800);
  wait(0.3,sec);
  autoIntakeFWDOn = false;
  turnPID(-41,1000);
  Lift.spinToPosition(0,deg,false);
  drive(-38, -41, 12, 12, 2000);
  //turnPID(10,1000);
  bMogwaitTime = 750;
  bMogState = true;
  thread z(bMogoTime);
  drive(-16, -17, 12, 12, 2000);
  autoIntakeFWDOn = true;
  turnPID(-55,1000);
  drive(22, -55, 12, 12, 2000);
  turnPID(-92.5,1000);
  bMog.set(false);
  Lift.spinToPosition(260,deg,false);
  liftwaitTime = 1300;
  liftState = 0;
  thread y1(liftTime);
  drive(49, -92.5, 12, 12, 1700);
  Lift.spinToPosition(0,deg,false);
  wait(0.2,sec);
  drive(-7.5, -93, 9, 12, 2000);
  //Lift.spinFor(-120,deg,false);
  turnPID(-63.5,1000);
  bMogwaitTime = 800;
  bMogState = true;
  thread z2(bMogoTime);
  intakeLift.set(true);
  drive(-29.5, -63.5, 12, 12, 2000);
  //autoIntakeFWDOn = true;
  turnPID(-90,1000);
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 700;
  intakeLiState = false;
  thread z3(intakeLiftTime);
  drive(21, -90, 9, 12, 2000);
  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  //bMogwaitTime = 300;
  //bMogState = false;
  //thread x(bMogoTime);
  fMogwaitTime = 400;
  fMogState = true;
  thread x1(fMogoTime);
  drive(25,142,12,12,2000,20);
  DriveL.stop(hold);
  DriveR.stop(hold);
  //autoIntakeFWDOn = false;

  
}



//---- ELIMS AUTOS -------- ELIMS AUTOS -------- ELIMS AUTOS -------- ELIMS AUTOS -------- ELIMS AUTOS -------- ELIMS AUTOS -------- ELIMS AUTOS ----




void Blue_Negative_Elims(){
  //Lift.spin(fwd,100,pct);
  Lift.setVelocity(100,pct);
  Lift.spinFor(260,deg,false);
  wait(0.5,sec);
  drive(7.5, 3, 6, 12, 2000);
  //Lift.spin(reverse,100,pct);
  Lift.spinFor(-140,deg,false);
  //Lift.spinFor(reverse,360,deg,100,pct,false);
  wait(0.275, sec);
  drive(-7.5, 0, 9, 12, 2000);
  Lift.spinFor(-120,deg,false);
  turnPID(-40,1000);
  drive(-20.8, -40, 12, 12, 2000);
  turnPID(-5,1000);
  bMogwaitTime = 400;
  bMogState = true;
  thread z(bMogoTime);
  //This is where it is fucking up about 1/3 of the time and the inertial goes crazy
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLift.set(true);
  drive(-9, -5, 6, 12, 2000);
  autoIntakeFWDOn = true;
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 600;
  intakeLiState = false;
  thread z1(intakeLiftTime);
  drive(20.5, 3, 9, 12, 2000);
  drive(-8, 16, 12, 12, 2000);
  turnPID(-155,1000);
  drive(35, -155, 12, 12, 2000);
  turnPID(-70,1000);
  Lift.setStopping(hold);
  Lift.spinToPosition(300,deg,false);
  drive(41, -70, 12, 12, 1200);
  wait(0.3,sec);
  drive(-12, -70, 12, 12, 2000);
  Lift.spinToPosition(0,deg,false);
  turnPID(75,1000);
  //bMogwaitTime = 600;
  //bMogState = false;
  //thread x(bMogoTime);
  //fMogwaitTime = 700;
  //fMogState = true;
  //thread x1(fMogoTime);

  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  drive(80,55,12,12,2000,20);

  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  //autoIntakeFWDOn = false;
}


void Red_Negative_Elims(){
  //Lift.spin(fwd,100,pct);
  Lift.setVelocity(100,pct);
  Lift.spinFor(260,deg,false);
  wait(0.5,sec);
  drive(7.5, 1, 6, 12, 2000);
  //Lift.spin(reverse,100,pct);
  Lift.spinFor(-140,deg,false);
  //Lift.spinFor(reverse,360,deg,100,pct,false);
  wait(0.275, sec);
  drive(-7.5, 0, 9, 12, 2000);
  Lift.spinFor(-120,deg,false);
  turnPID(40,1000);
  drive(-20.8, 40, 12, 12, 2000);
  turnPID(5,1000);
  bMogwaitTime = 400;
  bMogState = true;
  thread z(bMogoTime);
  //This is where it is fucking up about 1/3 of the time and the inertial goes crazy
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLift.set(true);
  drive(-10, 5, 6, 12, 2000);
  autoIntakeFWDOn = true;
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(Inertial.angle());
  intakeLiTime = 600;
  intakeLiState = false;
  thread z1(intakeLiftTime);
  drive(21.5, -3, 9, 12, 2000);
  drive(-8, -16, 12, 12, 2000);
  turnPID(155,1000);
  drive(35, 155, 12, 12, 2000);
  turnPID(70,1000);
  Lift.setStopping(hold);
  Lift.spinToPosition(300,deg,false);
  drive(41, 70, 12, 12, 1200);
  wait(0.3,sec);
  drive(-12, 70, 12, 12, 2000);
  Lift.spinToPosition(0,deg,false);
  turnPID(-45,1000);
  //bMogwaitTime = 600;
  //bMogState = false;
  //thread x(bMogoTime);
  //fMogwaitTime = 700;
  //fMogState = true;
  //thread x1(fMogoTime);

  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  drive(80,-55,12,12,2000,20);

  DriveL.setStopping(coast);
  DriveR.setStopping(coast);
  //bMog.set(false);
  //autoIntakeFWDOn = false;
}








void SoloAWP_Quad_RED(){
  Lift.setVelocity(75, pct);
  thread l(liftMov);
  drive(4.9, 0, 3, 12, 2000);
  wait(0.6, seconds); 
  drive(-19.5, 0, 5, 6, 5000);
  drive(0, 42, 12, 10, 3000);
  drive(-10, 42, 4, 12, 3000, 1.5, 300, 0.4);
  bMog.set(true);
  wait(.5, seconds);
  drive(2, 180, 10, 12, 3000);
  Intake.spin(forward, 100, percent);
  drive(20, 180, 10, 10, 5000);
  drive(4, 140, 12, 12, 5000);
  drive(6, 140, 4, 12, 5000);
  drive(0, 45, 12, 10, 5000);
  drive(15, 45, 12, 12, 5000);
  wait(0.5, seconds);
  drive(-32, 120, 6,12, 3000);
  bMog.set(false);
  Intake.stop();
  drive(-4, 100, 12, 12, 5000);
  // drivePID(20, 1);
  // turnPID(-70, 1000);
  // drivePID(36, 1);   
}

void SoloAWP_Quad_BLUE(){
  Lift.setVelocity(75, pct);
  thread l(liftMov);
  drive(5.5, 0, 3, 12, 2000);
  wait(0.6, seconds); 
  drive(-19.6, 0, 5, 6, 5000);
  drive(0, -42, 12, 12, 3000);
  drive(-10, -45, 4, 12, 3000, 1.5, 300, 0.4);
  bMog.set(true);
  wait(.5, seconds);
  drive(2, -180, 12, 12, 3000);
  Intake.spin(forward, 100, pct);
  drive(20, -180, 12, 12, 5000);
  drive(4, -140, 12, 12, 5000);
  drive(6, -140, 4, 12, 5000);
  drive(0, -45, 12, 10, 5000);
  drive(15, -45, 12, 12, 5000);
  wait(0.5, seconds);
  drive(-32, -120, 6,12, 3000);
  Intake.stop();
  drive(-3, -100, 12, 12, 5000);
  bMog.set(false);
}

void dropMogo(){
  wait(0.9, seconds);
  bMog.set(false);
}
void liftthingy(){
  Lift.spinFor(0.6, seconds);
  Lift.stop(hold);
}

void quad_4Ring_RED(){
  thread p(start);
  drive(-22, 0, 4, 12, 3000);
  bMog.set(true);
  wait(0.2, seconds);
  Intake.spin(forward, 100, percent);
  drive(4, 143, 3, 12, 3000);
  drive(9, 143, 10, 12, 3000);
  turnPID(100, 1500);
  drivePID(13, 0.8);
  turnPID( -10, 1000);
  drivePID(20, 1);
  Intake.stop();
  thread l(liftthingy);
  thread m(dropMogo);
  drive(38, -130, 12, 12, 3000);
  Lift.setVelocity(70, pct);
  Lift.spinFor(reverse, 0.2, seconds);
}

void quad_4Ring_BLUE(){
  thread p(start);
  drive(-22, 0, 4, 12, 3000);
  bMog.set(true);
  wait(0.2, seconds);
  thread i(intakeExpel);
  drive(4, -143, 6, 12, 3000);
  drive(15, -143, 10, 12, 3000);
  turnPID(-95, 1500);
  drivePID(15, 0.8);
  turnPID(10, 1000);
  drivePID(20, 1);
  Intake.stop();
  thread l(liftthingy);
  thread m(dropMogo);
  drive(38, 130, 12, 12, 3000);
  Lift.setVelocity(70, pct);
  Lift.spinFor(reverse, 0.2, seconds);
}

int armUP(){
  wait(0.2, seconds);
  
  Lift.spin(forward, 60, percent);
  wait(2, seconds);
  Lift.spinFor(reverse, 80, degrees);
  Lift.stop(hold);

  return 0;
}
int armDOWN(){
  Lift.spin(reverse, 60, pct);
  wait(1, seconds);
  Lift.stop(hold);
  return 0;
}

void stupidLiftForIntakeLift(){
  wait(0.2, seconds);
  Lift.spinFor(forward, 0.5, seconds);
}

void Rush_NotMogo_RED(){
  thread l(armUP);
  drivePID(40, 0.65);
  thread d(armDOWN);
  wait(0.3, seconds);
  drivePID(-4,1);
  drivePID(-28,1);
}

void mogoSide_BLUE_halfAWP(){
  thread p(start);
  drive(-22, 0, 3, 12, 3000);
  bMog.set(true);
  wait(0.2, seconds);
  Intake.spin(forward, 100, percent);
  turnPID(90, 1000);
  drive(24, 90, 12, 12, 3000);
  wait(1, seconds);
  drive(-31, 65, 12, 12, 3000);
  bMog.set(false);
}

void mogoSide_BLUE_3Ring(){
  thread s(start);
  drive(-22, 0, 3, 12, 5000);
  bMog.set(true);
  wait(0.2, seconds);
  Intake.spin(forward, 100, percent);
  turnPID(90, 1000);
  drivePID(20, 1);
  turnPID(-62, 1500);
  thread h(stupidLiftForIntakeLift);
  intakeLift.set(true);
  drive(42, -62, 12, 12, 5000);
  intakeLift.set(false);
  drivePID(-5, 1);
  turnPID(-135, 1500);
  bMog.set(false);
  drive(12, 170, 3, 12, 5000);
  drive(0, 180, 12, 3, 5000);
}

void mogoSide_RED_halfAWP(){
  thread p(start);
  drive(-22, 0, 3, 12, 3000);
  bMog.set(true);
  wait(0.2, seconds);
  Intake.spin(forward, 100, percent);
  turnPID(-90, 1000);
  drive(24, -90, 12, 12, 3000);
  wait(1, seconds);
  Intake.stop();
  drive(-31, -65, 12, 12, 5000);
  bMog.set(false);
}



void mogoSide_RED_3Ring(){
  thread s(start);
  drive(-22, 0, 3, 12, 5000);
  bMog.set(true);
  wait(0.2, seconds);
  Intake.spin(forward, 100, percent);
  turnPID(-90, 1000);
  drivePID(20, 1);
  turnPID(62, 1500);
  thread h(stupidLiftForIntakeLift);
  intakeLift.set(true);
  drive(42, 62, 8, 12, 5000);
  intakeLift.set(false);
  drivePID(-5, 1);
  turnPID(135, 1500);
  bMog.set(false);
  drive(12, -170, 3, 12, 5000);
  drive(0, 180, 6, 3, 5000);
}

void sTurn(float degs, float dist, float tim){

}
float tim;

int clampDrive(){
  wait(tim, seconds);
  bMog.set(true);
  return 0;
}



void liftDown(){
  Lift.spinFor(reverse, 1.5, seconds);
}

void skillsMoa(){
  tim = 0.65;
  Lift.setVelocity(75, pct);
  thread l(liftMov);
  wait(1.3, sec);
  //pickup mogo
  thread c(clampDrive);
  drive(-11, 0, 4, 12, 3000);
  Intake.spin(forward, 100, percent);
  wait(0.5, seconds);
  //drive to first ring
  //drive(42, 90, 12, 11.4, 5000); 
  drive(0, 45, 12, 9, 5000);
  drive(22, 45, 10, 9, 5000);
  drive(0, 90, 12, 9, 5000);
  drive(36, 90, 10, 9, 5000);
  // maybe go to neutral stake?? would go here

  //drive through next 3 rings
  //drive(18, -50, 8, 5.5, 5000);
  //drive(33, -50, 5, 12, 5000);
  drive(-1, 0, 9,6, 5000);
  drive(17, 0, 9, 5, 5000);
  drive(0, -45, 9,5, 5000);
  drive(27, -45, 7,9, 5000);
  wait(1.5, sec);
  //turn to 5th ring
  drive(-4, 50, 4, 8, 3000);
  drive(8, 50, 3, 8, 3000);
  //drop mogo in corner
  drive(-8, 170, 3, 12,3000);
  wait(0.5, sec);
  bMog.set(false);
  wait(0.5, seconds);
  drive(4, 170, 12, 12, 3000);
  drive(0, 40, 12, 12, 5000);
  tim = 1.65;
  thread h(clampDrive);
  drive(-24, 35, 4, 12, 5000);
  //drive(-5, -125, 12, 12, 5000);
  turnPID(-134, 1500);
  wait(0.5, sec);
  drive(69, -134, 5, 12, 5000);
  drive(-10, -45, 6, 12, 5000);
  drive(12, -45, 6, 12, 5000);
  drive(-5, 90, 6, 12, 5000);
  wait(0.5, sec);
  drive(-5, 90, 8, 12, 2000);
  bMog.set(false);
  thread i(intakeDivert);
  drive(15, 100, 7,7, 5000);
  drive(0, 135, 7,7, 3000);
  drive(35, 135, 7,7, 3000);
  drive(-2, 225, 7,7, 3000);
  wait(2, sec);
  Lift.setStopping(hold);
  Lift.spinFor(forward, 1.5, seconds);
  drive(13, 225, 4, 12, 3000);
  thread p(liftDown);
  wait(0.75, sec);
  drive(-12, 225, 4, 12, 3000);


}



void autonomous(void) {
  auto_started = true;
  colorDeflect = true;
  switch(current_auton_selection){  
    case 0:
      //Blue_Negative_4();
      
      //    ALL AUTOS Old
      // SoloAWP_Quad_RED(); //DONE 5/5     CHECK IF CROSSES LINE
      // SoloAWP_Quad_BLUE(); //DONE 4/5    CHECK IF CROSSES LINE
      // quad_4Ring_RED(); //DONE 4.5/5     CHECK IF CROSSES LINE
      // quad_4Ring_BLUE(); //DONE 4.9/5    CHECK IF CROSSES LINE
      // mogoSide_RED_halfAWP(); //DONE 5/5
      // mogoSide_BLUE_halfAWP(); //DONE 5/5
      // mogoSide_RED_3Ring(); //DONE 5/5
      // mogoSide_BLUE_3Ring(); // DONE 5/5
      // skillsMoa();

      //    New Autos
      //testFunctions();
      //Blue_Negative_4();
      //Red_Negative_4();
      Blue_Negative_Elims();
      //Red_Negative_Elims();
      //Blue_Positive_GoalRush();
      //Red_Positive_GoalRush();
      
      break;       
    case 1:
      Blue_Negative_Elims();       
      break;
    case 2:
      Red_Negative_Elims();
      break;
    case 3:
      Red_Negative_Elims();
      break;
    case 4:
      Blue_Negative_Elims();
      break;
    case 5:
      Blue_Positive_GoalRush();
      break;
    case 6:
      Red_Positive_GoalRush();
      break;
    case 7:
      mogoSide_BLUE_3Ring();      
      break;
    case 8:
      skillsMoa();
      break;
  }
}

int intakeManager(){
  bool detectCol = false;
  int preDeCol = 0;
  int stoppage = 0;
  while(1){
    bool stoppageCheck = false;
    if(colorDeflect == true){
        optColor.setLightPower(100,pct);
        //detect if the color that is detected is either red or blue
        if (140<optColor.hue() and optColor.hue()<230){
          preDeCol = 1;
        }else if(0<optColor.hue() and optColor.hue()<40){
          preDeCol = 2;
        }

        if(homeColor == true and preDeCol == 2){
          detectCol = true;
        }else if (homeColor == false and preDeCol == 1){
          detectCol = true;
        }else{
          detectCol = false;
        }
      }

    //Redirect Toggle

    if(Controller1.ButtonA.pressing() and !intakeButTog){
      intakeRedirectToggle = !intakeRedirectToggle;
      intakeButTog = true;
    
    }else if (!Controller1.ButtonA.pressing() and intakeButTog){
      intakeButTog = false;
    }


    //Color Expulsion Toggle

    if(Controller1.ButtonB.pressing() and !colorDeflectButTog){
      colorDeflect = !colorDeflect;
      colorDeflectButTog = true;
    
    }else if (!Controller1.ButtonB.pressing() and colorDeflectButTog){
      colorDeflectButTog = false;
    }
      
    
 
    if(intakeRedirectToggle == true){
  
      if (Controller1.ButtonR1.pressing() or autoIntakeFWDOn == true) // this is our intake button
      {
        //detect if detectCol is false to then go through our regular intake function with the redirect of our intake
        if(detectCol == false or colorDeflect == false){
           //7 mm is the distance in which our distance sensor registers the ring so while it is greater than 7 it will intake
          if (ringDetect.objectDistance(mm)>50){ 
          Intake.spin(forward, 100, percent);
          stoppageCheck = true;
          // once the distance sensor registers that a ring is there it will wait 30 msecs to get the ring into the correct spot 
          //and then reverse the intake to redirect them into the wall stake mechanism
          } else if(ringDetect.objectDistance(mm)<=50){
            Intake.setVelocity(-100, percent);
            Intake.spinFor(1, seconds);
          }
        }
        //check if the color of the ring is the color we want to expel and if it is then go through the code that will expel it
        else if(detectCol == true){
          if(ringDetect.objectDistance(mm)<20){
            //once the distance sensor registers the ring, because its the color we want to expel it will go through our expel code
            wait(40, msec); //this wait is the time we need to wait and then spin the intake reverse in order to expel the discs
            Intake.setVelocity(-100, percent);
            Intake.spinFor(0.2, seconds); //this was the time that we have tested to work the best to expel the rings
          }else{
            Intake.spin(forward,100,pct);
            stoppageCheck = true;
          }
        }
      } else if (Controller1.ButtonL1.pressing() or autoIntakeREVOn == true){
        Intake.spin(reverse, 100, percent);
      }else{
        Intake.stop();
      }
    }else if(intakeRedirectToggle == false){
      if(colorDeflect == true){

        if (Controller1.ButtonR1.pressing() or autoIntakeFWDOn == true)
        {
          if (ringDetect.objectDistance(mm)<=20 and detectCol){
            wait(25, msec);
            Intake.setVelocity(-100, percent);
            Intake.spinFor(0.2, seconds);
          }else {
            Intake.spin(forward, 100, percent);
            stoppageCheck = true;
          }
        } else if (Controller1.ButtonL1.pressing() or autoIntakeREVOn == true){
          Intake.spin(reverse, 100, percent);
        }else{
          Intake.stop();
        }
      }else{

        if (Controller1.ButtonR1.pressing() or autoIntakeFWDOn == true)
        { 
            Intake.spin(forward, 100, percent);
            stoppageCheck = true;
          }
        else if (Controller1.ButtonL1.pressing() or autoIntakeREVOn == true)
        {
            Intake.spin(reverse, 100, percent);
          }
        else{
          Intake.stop();
        }
      }
    }
    if (stoppageCheck == true and autoIntakeBackout == true){
      if (abs(Intake.velocity(rpm)) < 50){
        stoppage = stoppage + 1;
        if (stoppage >= 50){
          stoppage = 0;
          Intake.setVelocity(100,pct);
          Intake.spinFor(reverse,0.35,sec);
        }
      }else{
        stoppage = 0;
      }
    }else{
      stoppage = 0;
    }


    wait(2,msec);
  }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void displayColor(){
  while(true){
    if(homeColor == false){
      Controller1.Screen.setCursor(0,0);
      Controller1.Screen.print("red");
    }
    if(homeColor == true){
      Controller1.Screen.setCursor(0,0);
      Controller1.Screen.print("blue");
    }
    Controller1.Screen.clearLine();
    wait(10, msec);
  }
}

void usercontrol(void) {
  colorDeflect = true;
  fMog.set(false);
  autoIntakeFWDOn = false;
  autoIntakeREVOn = false;
  autoIntakeBackout = true;

  bool bMogToggle = false;
  bool bMogButTog = false;

  bool fMogToggle = false;
  bool fMogButTog = false;

  
  bool intakeLiftToggle = false;
  bool intakeLiftButTog = false;

  bool hangButTog = false;
  bool hangToggle = false;

  bool liftOverride = false;

  // User control code here, inside the loop
  thread c(displayColor);
  while (1) {

    //Main Drive Controls

    float tAdjust = 0;
    /*double contPos = Controller1.Axis1.position();
    if (contPos > -25 and contPos < 25){
      tAdjust = contPos/3;
    }else if (contPos > 25){
      tAdjust = pow(contPos,1.8)/40;
    }else{
      tAdjust = pow(fabs(contPos),1.8)/40*-1;
    }*/
    tAdjust = Controller1.Axis1.position();
    spinDriveVolt(Controller1.Axis3.position()-tAdjust,Controller1.Axis3.position()+tAdjust);
    
    
    
    // if (Controller1.ButtonR1.pressing()){
    //   Intake.spin(fwd,100,pct);
    // }else if (Controller1.ButtonR2.pressing()){
    //   Intake.spin(reverse,100,pct);
    // }else{
    //   Intake.stop();
    // }

    //Mogo Clamp Toggle

    if (Controller1.ButtonR1.pressing() and Controller1.ButtonL1.pressing() and bMogButTog == false){
      bMogButTog = true;
      if(bMogToggle == false){
        bMogToggle = true;
        bMog.set(true);
      }else{
        bMogToggle = false;
        bMog.set(false);
      }
    }else if (!(Controller1.ButtonR1.pressing() and Controller1.ButtonL1.pressing()) and bMogButTog == true){
      bMogButTog = false;
    }

    //Front Manipulator Toggle

    if ((Controller1.ButtonR2.pressing() and Controller1.ButtonL2.pressing()) and fMogButTog == false){
      fMogButTog = true;
      liftOverride = true;
      if(fMogToggle == false){
        fMogToggle = true;
        fMog.set(true);
      }else{
        fMogToggle = false;
        fMog.set(false);
      }
    }else if (!(Controller1.ButtonR2.pressing() and Controller1.ButtonL2.pressing()) and fMogButTog == true){
      fMogButTog = false;
      liftOverride = false;
    }

    //Lift Controls
      
    if(liftOverride == false){
      if (Controller1.ButtonR2.pressing() and !Controller1.ButtonL2.pressing()){
        Lift.spin(fwd,100,pct);
      }else if (Controller1.ButtonL2.pressing() and !Controller1.ButtonR2.pressing()){
        Lift.spin(reverse,100,pct);
      }else if(Controller1.ButtonR2.pressing() and Controller1.ButtonL2.pressing()){
        Lift.stop(hold);
      }
      else{
        Lift.stop(hold);
      }
    }else{
      Lift.stop(hold);
    }

    //Intake Lifter

    if (Controller1.ButtonX.pressing() and intakeLiftButTog == false){
      intakeLiftButTog = true;
      if(intakeLiftToggle == false){
        intakeLiftToggle = true;
        intakeLift.set(true);
      }else{
        intakeLiftToggle = false;
        intakeLift.set(false);
      }
    }else if (!Controller1.ButtonX.pressing() and intakeLiftButTog == true){
      intakeLiftButTog = false;
    }
    
    // Hang Toggle

    if (Controller1.ButtonY.pressing() and !hangButTog){
      hangToggle = !hangToggle;
      hang.set(hangToggle);
      hangButTog = true;
    }
    if (!Controller1.ButtonY.pressing() and hangButTog){
      hangButTog = false;
    }


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  thread i(intakeManager);
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
    //if(!Competition.isAutonomous()){
    //  bMog.set(false);
    //}
  }
}

/*
//Check if we are in the intake redirect mode or in the regular intake mode. if intakeTog is true that means we are in the redirect mode
    if(intakeTog == true){
      bool detectCol = false;
      if(colorDeflect == true){
        optColor.setLightPower(100,pct);
        //detect if the color that is detected is either red or blue
        if(homeColor == true){
          //if the homeColor is true that means that we are red and want to expel the blue rings
          //if the value is between 140 and 230 that is blue which sets detectCol to true to then expel them
          detectCol = 140<optColor.hue() and optColor.hue()<230;
        }else{
          //if the homeColor is false that means that we are blue and want to expel the red rings
          //if the value is between 0 and 40 that is red which sets detectCol to true to then expel them
          detectCol = 0<optColor.hue() and optColor.hue()<40;
        }
      }
      if (Controller1.ButtonR1.pressing()) // this is our intake button
      {
        //detect if detectCol is false to then go through our regular intake function with the redirect of our intake
        if(detectCol == false){
           //7 mm is the distance in which our distance sensor registers the ring so while it is greater than 7 it will intake
          if (ringDetect.objectDistance(mm)>50){ 
          Intake.spin(forward, 100, percent);
          // once the distance sensor registers that a ring is there it will wait 30 msecs to get the ring into the correct spot 
          //and then reverse the intake to redirect them into the wall stake mechanism
        } else if(ringDetect.objectDistance(mm)<=50){
          Intake.setVelocity(-100, percent);
          Intake.spinFor(1, seconds);
        }
        }
        //check if the color of the ring is the color we want to expel and if it is then go through the code that will expel it
        else if(detectCol == true){
          if(ringDetect.objectDistance(mm)<7){
            //once the distance sensor registers the ring, because its the color we want to expel it will go through our expel code
            wait(105, msec); //this wait is the time we need to wait and then spin the intake reverse in order to expel the discs
            Intake.setVelocity(-100, percent);
            Intake.spinFor(0.2, seconds); //this was the time that we have tested to work the best to expel the rings
          }
        }
        } else if (Controller1.ButtonL1.pressing()){
          Intake.spin(reverse, 100, percent);
        }else{
          Intake.stop();
        }
    }
    */