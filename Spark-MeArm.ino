/*
 * Spark core implementation of MeArm controller.
 * Motion planning and S-Curve implementation borrowed with permission from the excellent TinyG firmware by Alden Hart and Chris Riley.
 * Original Inverse Kinematics implementation based on https://github.com/phenoptix/MeArm
 *  
 * https://github.com/synthetos/TinyG/blob/b620a304f6b5c4b8787c551afdaef804b672537e/firmware/tinyg/plan_line.c
 *
 * Implementation on SparkCore by Ben Delarre.
 *  
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
// This #include statement was automatically added by the Spark IDE.
#include "SparkIntervalTimer/SparkIntervalTimer.h"

// This #include statement was automatically added by the Spark IDE.
#include "move.h"

IntervalTimer myTimer; // timer for segment moves

// arm dimensions
const int bicep = 81; // bicep length in mm
const int forearm = 81; // forearm length in mm
const int base = 60; // base height in mm

// we can precalculate these to save runtime later
float for_sq = forearm*forearm;
float bic_sq = bicep*bicep;

#define NUM_SERVOS 4

// a little structure to store our servo info
struct servoData {
    int position; // position at end of all planned moves
    int target; // current target position for current move
    int pin; // pin on which servo is connected
    int bounds[3]; // min max bounds and home position
};

// allocate our servos
Servo servos[NUM_SERVOS];

// a pointer to the current move
move *m = NULL;

// global position values for planning moves
float global_current_position[AXES] = {0,0,0,0};

// initialise the servo info objects
servoData servo_obj[NUM_SERVOS] = {
    { 0, 0, A7, { 580, 2300, 1380 } },
    { 0, 0, A6, { 580, 2380, 1380 } },
    { 0, 0, A5, { 580, 2380, 1000 } },
    { 0, 0, A4, { 580, 1180, 680 } } // 1060
};

// sets a servo to a specific position, clamps to bounds
int setServoBounded(int servo, int us) {
    
    if (!servos[servo].attached()) {
        servos[servo].attach(servo_obj[servo].pin);
    }
    
    us = max(servo_obj[servo].bounds[0],us);// clamp lower bound
    us = min(servo_obj[servo].bounds[1],us);// clamp upper boundtru
    
    servos[servo].writeMicroseconds(us);
    
    // update current pos immediately since this is just an immediate move
    servo_obj[servo].position = us;
    servo_obj[servo].target = us;
    
    /*Serial.print("Servo ");
    Serial.print(servo,DEC);
    Serial.print(" = ");
    Serial.println(us,DEC);*/
    
    return us;
}

// detaches the servo specified using the command string of the form "<SERVO NUMBER>"
int detachServoCmd(String servoString) {
    int servoNumber = servoString.toInt();
    servoNumber = min(NUM_SERVOS-1,servoNumber);
    servoNumber = max(0,servoNumber);
    
    if (servos[servoNumber].attached()) {
        servos[servoNumber].detach();
        return 1;
    }
    return 0;
}

// set position of servos using Inverse Kinematics
// initial implementation borrowed from https://github.com/phenoptix/MeArm/blob/master/MeArmIK/MeArmIK.ino
void setPosition(float x, float y, float z, float g) {
    // clamp input values to range
    /*x = max(20, min(240, x));
    y = max(0, min(179, y));
    z = max(24, min(310, z));
    g = max(40, min(90, g));*/
    
    /* Serial.print("IK input : [");
    Serial.print(x,5);
    Serial.print(", ");
    Serial.print(y,5);
    Serial.print(", ");
    Serial.print(z,5);
    Serial.print(", ");
    Serial.print(g,5);
    Serial.println("]");*/
    
    global_current_position[AXIS_X] = x;
    global_current_position[AXIS_Y] = y;
    global_current_position[AXIS_Z] = z;
    global_current_position[AXIS_G] = g;
    
    // do IK math....
    int b = sqrt ((x * x) + (z * z));        // b = distance from the origin to the start of the gripper
    float q1 = atan2( x, z );                // q1 = angle between the horizontal and the line b
    float c = (bic_sq - for_sq + (b * b))/(2 * bicep * b);
    c = min(1.0, max(-1.0, c));
    float q2 = acos(c); // q1 = angle between line b and the bicep
    float abi = q1 + q2;                     // abi = angle between horizontal and the bicep
    c = (bic_sq + for_sq - (b * b))/(2 * bicep * forearm);
    c = min(1.0, max(-1.0, c));
    float afo = acos(c); // afo = angle between bicep and forearm
    
    // convert to degrees from radians
    abi = 180 - (abi * 57.29);
    afo = (afo * 57.29);

    
    y = floor(servo_obj[0].bounds[0] +  (y/180.0)*(servo_obj[0].bounds[1]-servo_obj[0].bounds[0]));
    abi = floor(servo_obj[1].bounds[0] +  (abi/180.0)*(servo_obj[1].bounds[1]-servo_obj[1].bounds[0]));
    afo = floor(servo_obj[2].bounds[0] +  (afo/180.0)*(servo_obj[2].bounds[1]-servo_obj[2].bounds[0]));
    g = floor(servo_obj[3].bounds[0] +  (g/90.0)*(servo_obj[3].bounds[1]-servo_obj[3].bounds[0]));

    /*Serial.print("Servo positions : [");
    Serial.print(y,5);
    Serial.print(", ");
    Serial.print(afo,5);
    Serial.print(", ");
    Serial.print(abi,5);
    Serial.print(", ");
    Serial.print(g,5);
    Serial.println("]");*/
    
    setServoBounded(0, y);
    setServoBounded(1, afo);
    setServoBounded(2, abi);
    setServoBounded(3, g);
    
}

// takes the command string from the spark core api of the form "<X>,<Y>,<Z>,<G>" specified in degrees as integers
int setPositionCmd(String command) {
    if (command.length() < 5) {
        return -1; // has to be at least 3 characters
    }
  
    // x parameter
    int comma1 = command.indexOf(',');
    int x = command.substring(0,comma1).toInt();
    int comma2 = command.indexOf(',',comma1+1);
    int y = command.substring(comma1+1, comma2).toInt();
    comma1 = command.indexOf(',', comma2+1);
    int z = command.substring(comma2+1, comma1).toInt();
    int g = command.substring(comma1+1).toInt();
    
    float target[AXES] = {(float)x,(float)y,(float)z,(float)g};
    
    Serial.print("Set X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.print(z);
    Serial.print(" G=");
    Serial.println(g);
    
    float len = get_axis_vector_length(global_current_position, target);
    float duration = len / 10000;
    Serial.print("Length = ");
    Serial.print(len,5);
    Serial.print(" Duration = ");
    Serial.print(duration,5);
    
    m = new_move(global_current_position, target, duration, 0.0001);
    return 0;
}

// sets a specific servo to a specific position from command string in the form "<SERVO NUMBER>,<MICROSECONDS>"
int setServo(String command) {
    if (command.length() < 3) {
        return -1; // has to be at least 3 characters
    }
    
    String servoString = command.substring(0,1);
    if (servoString==NULL) {
        return -1;
    }
    
    int servoNumber = servoString.toInt();
    
    if (servoNumber < 0 || servoNumber > 4) return -1;
    
    String usString = command.substring(2);
    if (usString==NULL) {
        return -1;
    }
    
    int us = usString.toInt();
    
    us = setServoBounded(servoNumber,us);
    
    return us;
}
// timeout handler to process our moves
void onServoTimeout() {
    // do we have a move?
    if (m!=NULL) {
        // execute it
        stat_t state = exec_move(m);
        // is it complete?
        if (state==STAT_COMPLETE) {
            // clear the move!
            m = NULL;
        }
    }
}

void setup() {
    Serial.begin(9600);
    
    // attach all the servos at startup
    for (int i=0; i < NUM_SERVOS; i++) {
        servos[i].attach(servo_obj[i].pin);
    }
    
    // move servos to intial state
    setServoBounded(0, 1440);
    setServoBounded(1, 1425);
    setServoBounded(2, 1660);
    setServoBounded(3, 880);
    
    // set global initial state position
    global_current_position[AXIS_X] = 45;
    global_current_position[AXIS_Y] = 90;
    global_current_position[AXIS_Z] = 100;
    global_current_position[AXIS_G] = 45;
    delay(30);
    
    // initialise the move planner with current global position and pass it the callback for processing move segments
    init_planner(global_current_position, setPosition);
    
    
    Spark.function("setServo", setServo);
    Spark.function("setPosition", setPositionCmd);
    Spark.function("detachServo", detachServoCmd);
    
    myTimer.begin(onServoTimeout, INTERRUPT_DELAY, uSec, TIMER4);
}

void loop() {
    // nothing in the loop, all interrupt driven!
}