/*
 * Motion planning and S-Curve implementation borrowed with permission from the excellent TinyG firmware by Alden Hart and Chris Riley.
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
 
#ifndef _MOVE_H
#define _MOVE_H

#include <math.h>
#include "application.h"


enum moveState {
	MOVE_STATE_OFF = 0,		// move inactive (MUST BE ZERO)
	MOVE_STATE_NEW,			// general value if you need an initialization
	MOVE_STATE_RUN,			// general run state (for non-acceleration moves) 
	MOVE_STATE_RUN2,		// used for sub-states
	MOVE_STATE_HEAD,		// aline() acceleration portions
	MOVE_STATE_BODY,		// aline() cruise portions
	MOVE_STATE_TAIL,		// aline() deceleration portions
	MOVE_STATE_SKIP,		// mark a skipped block
	MOVE_STATE_END			// move is marked as done (used by dwells)
};

#define MOVE_STATE_RUN1 MOVE_STATE_RUN

#define MIN_LINE_LENGTH 		((float)0.08)		// Smallest line the system can plan (mm) (0.02)
#define MIN_SEGMENT_LENGTH 		((float)0.05)		// Smallest accel/decel segment (mm). Set to produce ~10 ms segments (0.01)
#define MIN_LENGTH_MOVE 		((float)0.001)		// millimeters

const int INTERRUPT_DELAY = 2500;

#define NOM_SEGMENT_USEC        ((float)5000)
#define MIN_SEGMENT_USEC 		((float)2500)		// minimum segment time
#define MICROSECONDS_PER_MINUTE ((float)60000000)
#define uSec(a) ((float)(a * MICROSECONDS_PER_MINUTE))
#define MIN_SEGMENT_TIME 		(MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_TIME_MOVE  			((float)0.0000001)

#define JERK_MAX 50000000

#ifndef EPSILON
#define EPSILON 	0.00001					// rounding error for floats
//#define EPSILON 	0.000001				// rounding error for floats
#endif

#ifndef fp_EQ
#define fp_EQ(a,b) (fabs(a-b) < EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef fp_NE
#define fp_NE(a,b) (fabs(a-b) > EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef fp_ZERO
#define fp_ZERO(a) (fabs(a) < EPSILON)		// requires math.h to be included in each file used
#endif
#ifndef fp_NOT_ZERO
#define fp_NOT_ZERO(a) (fabs(a) > EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef fp_FALSE
#define fp_FALSE(a) (a < EPSILON)			// float is interpreted as FALSE (equals zero)
#endif
#ifndef fp_TRUE
#define fp_TRUE(a) (a > EPSILON)			// float is interpreted as TRUE (not equal to zero)
#endif

#define TRAPEZOID_ITERATION_MAX 10
#define TRAPEZOID_ITERATION_ERROR_PERCENT 0.10
#define TRAPEZOID_LENGTH_FIT_TOLERANCE (0.0001)	// allowable mm of error in planning phase
#define TRAPEZOID_VELOCITY_TOLERANCE (max(2,bf->entry_velocity/100))

#define stat_t uint8_t

#define STAT_OK 0
#define STAT_ERROR 1
#define STAT_EAGAIN 2
#define STAT_NOOP 3
#define STAT_SKIPPED 4
#define STAT_COMPLETE 5
#define STAT_MINIMUM_LENGTH_MOVE_ERROR 100
#define STAT_MINIMUM_TIME_MOVE_ERROR 101


#define AXES 4
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define AXIS_G 3

#define square(a) pow(a,2)

struct move {
    float target[AXES];
    float unit[AXES];
    
    float time;
    float min_time;
    
    float cruise_velocity;
    float entry_velocity;
    float exit_velocity;
    
    float length;
    float jerk;
    float cbrt_jerk;
    float recip_jerk;
    
    float cruise_vmax;
    float entry_vmax;
    float delta_vmax;
    float exit_vmax;
    float braking_velocity;
    
    float head_length;
    float body_length;
    float tail_length;
    
    int move_state;
};

struct moveRuntime {

    float midpoint_velocity;
    float move_time;
    float segments;
    float segment_move_time;
    uint32_t segment_count;
    
    int move_state;
    int section_state;
    
    float forward_diff_1;
    float forward_diff_2;
    float segment_velocity;
    float cruise_velocity;
    float exit_velocity;
    float entry_velocity;
    
    float target[AXES];
    float position[AXES];
    
    float unit[AXES];
    float endpoint[AXES];
    
    float head_length;
    float body_length;
    float tail_length;
    
    float microseconds;
    
    float jerk;
    
    
};


move* new_move(const float from[], const float target[], const float minutes, const float min_time);
void free_move(move *m);
stat_t exec_move(move *m);
void init_planner(const float from[], void (*callback)(float x, float y, float z, float g) );
float get_axis_vector_length(const float a[], const float b[]);

#endif
