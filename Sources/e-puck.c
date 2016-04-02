/*Sandeep Manandhar & Omid Moradtalab
University of Burgundy, France
Autonomous Robotics
Bug2.0 algorithm implementation on Epuck
email: manandhar.sandeep@gmail.com
*/
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <math.h>
//#include <stdio.h>

#define TIME_STEP 64
double WHEEL_RADIUS = 2.05;
double AXLE_LENGTH = 5.2;
double WALL = 300;
double ENCODER_RESOLUTION = 160;
#define RANGE (1024 / 2)
#define squareLength 0.25
double PI = 3.141582653;
double sum ;
double avgF;
int obsFlag = 0;

double irVal;
double irerr;
int state = 1;

double totalDistance = 0;
double totalAng = 0;
double ang = 0, d;
double rotAng = 0;
double err = 0;
double l = 0 ;
double r = 0 ;
int rotateFlag = 1;
int moveFlag = 0;
double speed = 400;
double dev_step = 64/4;
double obs[8];
double x = 0, y = 0;
double gx = 60, gy = 60, goalD;
double delta;
double dline;
int bias;
int flagRot=0, flagMove=0;
int obstacle = 0;
int wallEncounter = 0;
double slope;
bool foundIntersection = 0;
bool blink = false;
void see(){
  printf("x: %f, y: %f, ang: %f\n", x, y, ang*180.0/PI);
  printf("d: %f, cos: %f, sin: %f\n", d, cos(ang), sin(ang));
  printf("goalx: %f, goaly: %f, goalang: %f\n", gx, gy, atan2(gy - y, gx - x)*180.0/PI);
  printf("disErr: %f, angErr: %f\n", goalD, delta*180/PI);


}

//calculate distance to d-line and goal and angle to goal
void getDistances(){
    delta = atan2(gy - y, gx - x) - ang;
    dline = fabs(gy*x - gx*y)/sqrt(gy*gy + gx*gx);
    goalD = sqrt(pow(gx - x, 2) +(pow(gy - y,2)));
}

//find the intersection with d-line if strayed away by obstruction
bool findIntersection(){
    if(fabs(y - slope*x) < 0.1){  return true;}
    printf("finding %f\n", fabs(y - slope*x));
    return false;
}

int move(double distance){
    err = distance - d;
    if(err > 0){
        wb_differential_wheels_set_speed(400, 400);
        return -1; //still not there
    }
    return 1; //goal point reached!!!
}

int rotate(double rotZ){
    rotZ = rotZ/180.0*PI;
    if(ang < rotZ){
        printf("ang: %f\n", ang);
        wb_differential_wheels_set_speed(-200, 200);
        return -1; //rotation incomplete
    }
    return 1; //rotation complete

}


int main(int argc, char *argv[]) {


    /* define variables */

    int i,j;
   // double speed[2];


    double dl;
    double dr;
    double sl, sr;
    double ps_values[8];
    /* initialize Webots */
    wb_robot_init();
    
    slope = gy/(gx+0.000001);
    
    WbDeviceTag ds[8];
    WbDeviceTag led[10];
    
    char led_names[10][5] = { "led0", "led1", "led2", "led3",
    "led4", "led5", "led5", "led6", "led7", "led8", "led9"};
    char ps_names[8][4] = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };

    for (i=0; i<8; i++) {
      ds[i] = wb_robot_get_device(ps_names[i]);
      wb_distance_sensor_enable(ds[i], TIME_STEP);
    }
    
     for (i=0; i<10; i++) {
      led[i] = wb_robot_get_device(led_names[i]);
      wb_led_set(led[i],0);
    }

    


    wb_differential_wheels_enable_encoders(TIME_STEP);
    wb_differential_wheels_set_encoders(0,0);


    /* main loop */
    for (;;) {
        wb_robot_step(TIME_STEP);
        l =  wb_differential_wheels_get_left_encoder();
        r =  wb_differential_wheels_get_right_encoder();
        dl = l/ENCODER_RESOLUTION*WHEEL_RADIUS; // distance covered by left wheel in meter
        dr = r/ENCODER_RESOLUTION*WHEEL_RADIUS; // distance covered by right wheel in meter
        d = (dr + dl)/2;
        ang +=  (dr - dl)/(AXLE_LENGTH);
        x += d*cos(ang); y += d*sin(ang);
        getDistances();

        for (i=0; i<8 ; i++)
          ps_values[i] = wb_distance_sensor_get_value(ds[i]);
        
           bool left_obstacle =
              ps_values[0] > 400.0 ||
              ps_values[1] > 400.0 ||
              ps_values[2] > 400.0;
            bool right_obstacle =
              ps_values[5] > 400.0 ||
              ps_values[6] > 400.0 ||
              ps_values[7] > 400.0;
        
            // init speeds
            if((left_obstacle || right_obstacle) & !wallEncounter){
              wallEncounter = 1; foundIntersection  =0;
              }
            // modify speeds according to obstacles
            if(wallEncounter == 1 && !foundIntersection)
            {

            double ps5, ps6, ps7;
            double psErr = 850 - (ps_values[2]+(ps_values[1] + ps_values[0] + ps_values[7])); 
            sl = psErr/6 + 200; 
            sr = -psErr/6 + 200;
            if(sl > 800) sl = 800; else if(sl < -800) sl = -800;
            if(sr > 800) sr = 800; else if(sr < -800) sr = -800;
         //   wb_robot_step(128);
               
               
       
            }
            else{
                obstacle = 0;
                wallEncounter = 0;
                 if(fabs(delta) > 0.035){
                if(delta > 0) bias = 100;
                else bias = -10;
                sr = delta*300 + bias;//p controller with 50 for bias
             //   printf("sr: %f\n", sr);
              if(sr > 800) sr = 800;
              else if(sr < -800) sr = -800;
              sl = -sr;
              flagRot = 1;
              }
              else{
              sl = sr = 0;
                  flagRot = 0;
                    if(goalD > 0.005) flagMove = 1;
            //      wb_robot_step(128);
              }
        
        
        if(flagRot == 0 && flagMove == 1) //no rotation required
        {
          if(fabs(goalD) > 0.008){
            sr = goalD*10 + 50;
             if(sr > 800) sr = 800;
             else if(sr < -800) sr = -800; //not required though
             sl = sr;
          }
          else{
            sl = sr = 0;
            flagMove = 0;
            printf("MISSION COMPLETE!!!\n");
           
             for (i=0; i<10; i++) {
             wb_led_set(led[i],1);
              }
          }
        }
            }
        foundIntersection  = findIntersection();
      

        see();
         wb_differential_wheels_set_speed(sl, sr);
         wb_differential_wheels_set_encoders(0,0);
        }
        return 0;
    }
