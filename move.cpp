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

#include "move.h"
#include <math.h>

moveRuntime mr;
void (*callback_func)(float x, float y, float z, float g);

void log_array(float a[], int len) {
    Serial.print("[");
    for (int i=0; i < len; i++) {
        Serial.print(a[i],5);
        if (i < len-1) {
            Serial.print(",");
        }
    }
    Serial.print("]");
}
void log_move(move *m)  {
    Serial.println("Move:");
    Serial.println("---------------------------------------");
    Serial.print("\ttarget = "); log_array(m->target,AXES); Serial.println(".");
    Serial.print("\tunit = "); log_array(m->unit,AXES); Serial.println(".");
    
    Serial.print("\ttime = "); Serial.println(m->time,5);
    Serial.print("\tmin_time = "); Serial.println(m->min_time,5);
    
    Serial.print("\tcruise_velocity = "); Serial.println(m->cruise_velocity,5);
    Serial.print("\texit_velocity = "); Serial.println(m->exit_velocity,5);
    Serial.print("\tentry_velocity = "); Serial.println(m->entry_velocity,5);
    
    Serial.print("\tlength = "); Serial.println(m->length,5);
    Serial.print("\tjerk = "); Serial.println(m->jerk,5);
    Serial.print("\tcbrt_jerk = "); Serial.println(m->cbrt_jerk,5);
    Serial.print("\trecip_jerk = "); Serial.println(m->recip_jerk,5);
    
    Serial.print("\tcruise_vmax = "); Serial.println(m->cruise_vmax,5);
    Serial.print("\tentry_vmax = "); Serial.println(m->entry_vmax,5);
    Serial.print("\tdelta_vmax = "); Serial.println(m->delta_vmax,5);
    Serial.print("\texit_vmax = "); Serial.println(m->exit_vmax,5);
    Serial.print("\tbraking_velocity = "); Serial.println(m->braking_velocity,5);
    
    Serial.print("\thead_length = "); Serial.println(m->head_length,5);
    Serial.print("\tbody_length = "); Serial.println(m->body_length,5);
    Serial.print("\ttail_length = "); Serial.println(m->tail_length,5);
    
    Serial.print("\tmove_state = "); Serial.println(m->move_state,DEC);
    Serial.println("---------------------------------------");
}
void log_runtime()  {
    Serial.println("Move Runtime:");
    Serial.println("---------------------------------------");
    Serial.print("\tmidpoint_velocity = "); Serial.println(mr.midpoint_velocity,5);
    Serial.print("\tmove_time = "); Serial.println(mr.move_time,5);
    Serial.print("\tsegments = "); Serial.println(mr.segments,DEC);
    Serial.print("\tsegment_move_time = "); Serial.println(mr.segment_move_time,5);
    Serial.print("\tsegment_count = "); Serial.println(mr.segment_count,DEC);
    Serial.print("\tmove_state = "); Serial.println(mr.move_state,DEC);
    Serial.print("\tsection_state = "); Serial.println(mr.section_state,DEC);
    
    Serial.print("\tforward_diff_1 = "); Serial.println(mr.forward_diff_1,5);
    Serial.print("\tforward_diff_2 = "); Serial.println(mr.forward_diff_2,5);
    Serial.print("\tsegment_velocity = "); Serial.println(mr.segment_velocity,5);
    Serial.print("\tcruise_velocity = "); Serial.println(mr.cruise_velocity,5);
    Serial.print("\texit_velocity = "); Serial.println(mr.exit_velocity,5);
    Serial.print("\tentry_velocity = "); Serial.println(mr.entry_velocity,5);
    
    Serial.print("\ttarget = "); log_array(mr.target,AXES); Serial.println(".");
    Serial.print("\tposition = "); log_array(mr.position,AXES); Serial.println(".");
    
    Serial.print("\tunit = "); log_array(mr.unit,AXES); Serial.println(".");
    Serial.print("\tendpoint = "); log_array(mr.endpoint,AXES); Serial.println(".");
    
    Serial.print("\thead_length = "); Serial.println(mr.head_length,5);
    Serial.print("\tbody_length = "); Serial.println(mr.body_length,5);
    Serial.print("\ttail_length = "); Serial.println(mr.tail_length,5);
    
    Serial.print("\tmicroseconds = "); Serial.println(mr.microseconds,DEC);
    
    Serial.print("\tjerk = "); Serial.println(mr.jerk,5);
    Serial.println("---------------------------------------");
}
float get_target_length(float entry, float exit, move *m) {
    return fabs(entry - exit) * sqrt(fabs(entry - exit) * m->recip_jerk);
}

float get_target_velocity(float Vi, float L, move *m) {
    return pow(L, 0.66666666) * m->cbrt_jerk + Vi;
}


void copy_axis_vector(float dst[], const float src[]) 
{
	memcpy(dst, src, sizeof(float)*AXES);
}

float get_axis_vector_length(const float a[], const float b[]) 
{
    float sum = 0;
    for (int i=0; i < AXES; i++) {
        sum += square(a[i] - b[i]);
    }
	return (sqrt(sum));
}

static void init_forward_diffs(float t0, float t2) {
	float H_squared = (1/mr.segments);
	
	//Serial.print("H_squared = ");
	//printFloat(H_squared,5);
	//Serial.print(" t0 = ");
	//printFloat(H_squared,5);
	//Serial.print(" t2 = ");
	//printFloat(H_squared,5);
    
    H_squared *= H_squared;
	// A = T[0] - 2*T[1] + T[2], if T[0] == T[1], then it becomes - T[0] + T[2]
	float AH_squared = (t2 - t0) * H_squared;

	// Ah²+Bh, and B=2 * (T[1] - T[0]), if T[0] == T[1], then it becomes simply Ah^2
	mr.forward_diff_1 = AH_squared;
	mr.forward_diff_2 = 2*AH_squared;
	mr.segment_velocity = t0;
	
	
	//Serial.print("Forward diffs : ");
	//printFloat(mr.forward_diff_1,5);
	//Serial.print(" ");
	//printFloat(mr.forward_diff_2,5);
	//Serial.println(".");
}


#define MIN_HEAD_LENGTH MIN_SEGMENT_TIME * (bf->cruise_velocity + bf->entry_velocity)
#define MIN_TAIL_LENGTH MIN_SEGMENT_TIME * (bf->cruise_velocity + bf->exit_velocity)
#define MIN_BODY_LENGTH MIN_SEGMENT_TIME * (bf->cruise_velocity)

void calculate_trapezoid(move *bf) {
    
	bf->head_length = 0;		// inialize the lengths
	bf->body_length = 0;
	bf->tail_length = 0;

	// Combined short cases:
	//	- H and T requested-fit cases (exact fit cases, to within TRAPEZOID_LENGTH_FIT_TOLERANCE)
	//	- H" and T" degraded-fit cases
	//	- H' and T' requested-fit cases where the body residual is less than MIN_BODY_LENGTH
	//	- no-fit case
	// Also converts 2 segment heads and tails that would be too short to a body-only move (1 segment)
	float minimum_length = get_target_length(bf->entry_velocity, bf->exit_velocity, bf);
	if (bf->length <= (minimum_length + MIN_BODY_LENGTH)) {	// Head & tail cases
		if (bf->entry_velocity > bf->exit_velocity)	{		// Tail cases
			if (bf->length < (minimum_length - TRAPEZOID_LENGTH_FIT_TOLERANCE)) { 	// T" (degraded case)
				bf->entry_velocity = get_target_velocity(bf->exit_velocity, bf->length, bf);
			}
			bf->cruise_velocity = bf->entry_velocity;
			if (bf->length >= MIN_TAIL_LENGTH) {			// run this as a 2+ segment tail
				bf->tail_length = bf->length;
			} else if (bf->length > MIN_BODY_LENGTH) {		// run this as a 1 segment body
				bf->body_length = bf->length;
			} else {
				bf->move_state = MOVE_STATE_SKIP;			// tell runtime to skip the block
			}
			return;
		}
		if (bf->entry_velocity < bf->exit_velocity)	{		// Head cases
			if (bf->length < (minimum_length - TRAPEZOID_LENGTH_FIT_TOLERANCE)) { 	// H" (degraded case)
				bf->exit_velocity = get_target_velocity(bf->entry_velocity, bf->length, bf);
			}
			bf->cruise_velocity = bf->exit_velocity;
			if (bf->length >= MIN_HEAD_LENGTH) {			// run this as a 2+ segment head
				bf->head_length = bf->length;
			} else if (bf->length > MIN_BODY_LENGTH) {		// run this as a 1 segment body
				bf->body_length = bf->length;
			} else {
				bf->move_state = MOVE_STATE_SKIP;			// tell runtime to skip the block
			}
			return;
		}
	}
	// Set head and tail lengths
	bf->head_length = get_target_length(bf->entry_velocity, bf->cruise_velocity, bf);
	bf->tail_length = get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
	if (bf->head_length < MIN_HEAD_LENGTH) { bf->head_length = 0;}
	if (bf->tail_length < MIN_TAIL_LENGTH) { bf->tail_length = 0;}

	// Rate-limited HT and HT' cases
	if (bf->length < (bf->head_length + bf->tail_length)) { // it's rate limited

		// Rate-limited HT case (symmetric case)
		if (fabs(bf->entry_velocity - bf->exit_velocity) < TRAPEZOID_VELOCITY_TOLERANCE) {
			bf->head_length = bf->length/2;
			bf->tail_length = bf->head_length;
			bf->cruise_velocity = min(bf->cruise_vmax, get_target_velocity(bf->entry_velocity, bf->head_length, bf));
			return;
		}

		// Rate-limited HT' case (asymmetric) - this is relatively expensive but it's not called very often
		float computed_velocity = bf->cruise_vmax;
		uint8_t i=0;
		do {
			bf->cruise_velocity = computed_velocity;	// initialize from previous iteration 
			bf->head_length = get_target_length(bf->entry_velocity, bf->cruise_velocity, bf);
			bf->tail_length = get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
			if (bf->head_length > bf->tail_length) {
				bf->head_length = (bf->head_length / (bf->head_length + bf->tail_length)) * bf->length;
				computed_velocity = get_target_velocity(bf->entry_velocity, bf->head_length, bf);
			} else {
				bf->tail_length = (bf->tail_length / (bf->head_length + bf->tail_length)) * bf->length;
				computed_velocity = get_target_velocity(bf->exit_velocity, bf->tail_length, bf);
			}
			if (++i > TRAPEZOID_ITERATION_MAX) { Serial.println("_calculate_trapezoid() failed to converge"); }
		} while ((fabs(bf->cruise_velocity - computed_velocity) / computed_velocity) > TRAPEZOID_ITERATION_ERROR_PERCENT);
		bf->cruise_velocity = computed_velocity;
		bf->head_length = get_target_length(bf->entry_velocity, bf->cruise_velocity, bf);
		bf->tail_length = bf->length - bf->head_length;
		if (bf->head_length < MIN_HEAD_LENGTH) {
			bf->tail_length = bf->length;			// adjust the move to be all tail...
			bf->head_length = 0;					// adjust the jerk to fit to the adjusted length
		}
		if (bf->tail_length < MIN_TAIL_LENGTH) {
			bf->head_length = bf->length;			//...or all head
			bf->tail_length = 0;
		}
		return;
	}

	// Requested-fit cases: remaining of: HBT, HB, BT, BT, H, T, B, cases
	bf->body_length = bf->length - bf->head_length - bf->tail_length;

	// If a non-zero body is < minimum length distribute it to the head and/or tail
	// This will generate small (acceptable) velocity errors in runtime execution
	// but preserve correct distance, which is more important.
	if ((bf->body_length < MIN_BODY_LENGTH) && (fp_NOT_ZERO(bf->body_length))) {
		if (fp_NOT_ZERO(bf->head_length)) {
			if (fp_NOT_ZERO(bf->tail_length)) {			// HBT reduces to HT
				bf->head_length += bf->body_length/2;
				bf->tail_length += bf->body_length/2;
			} else {									// HB reduces to H
				bf->head_length += bf->body_length;
			}
		} else {										// BT reduces to T
			bf->tail_length += bf->body_length;
		}
		bf->body_length = 0;

	// If the body is a standalone make the cruise velocity match the entry velocity 
	// This removes a potential velocity discontinuity at the expense of top speed
	} else if ((fp_ZERO(bf->head_length)) && (fp_ZERO(bf->tail_length))) {
		bf->cruise_velocity = bf->entry_velocity;
	}
}

move* new_move(const float from[], const float target[], const float minutes, const float min_time) {
    move *m = (move*)malloc(sizeof(move));
    
    float length = get_axis_vector_length(target, from);
    // TODO: check length is minimum or more
    if (length < MIN_LENGTH_MOVE) { return NULL; }
	if (minutes < MIN_TIME_MOVE) { return NULL; }
	
    m->move_state = MOVE_STATE_NEW;
    
    m->time = minutes;
    m->min_time = min_time;
    m->length = length;
    
    copy_axis_vector(m->target, target);
    
    float jerk_squared = 0;
    float diff = 0;
    for (int i=0; i < AXES; i++) {
        diff = target[i] - from[i];
        m->unit[i] = 0;
        if (fp_NOT_ZERO(diff)) {
            m->unit[i] = diff / length;
            jerk_squared += square(m->unit[i] * JERK_MAX);
        }
    }
    m->jerk = sqrt(jerk_squared);
    
    m->cbrt_jerk = cbrt(m->jerk);
    m->recip_jerk = 1/m->jerk;
    
    m->cruise_vmax = (m->length / m->time);
    m->entry_vmax = 0;
    m->exit_vmax = 0;
    m->delta_vmax = get_target_velocity(0, (m->length), m);
    
    
    m->entry_velocity = 0;
    m->exit_velocity = 0;
    m->cruise_velocity = m->cruise_vmax;
    
    calculate_trapezoid(m);
    
    
    // TODO: copy m->target to global position value for planning moves
    log_move(m);
    //delay(500);
    
    return m;
}

void free_move(move *m) {
    free(m);
}

stat_t exec_segment(uint8_t correction_flag)
{
    float travel[AXES];
    
	if ((correction_flag == true) && (mr.segment_count == 1)) {
		mr.target[AXIS_X] = mr.endpoint[AXIS_X];
		mr.target[AXIS_Y] = mr.endpoint[AXIS_Y];
		mr.target[AXIS_Z] = mr.endpoint[AXIS_Z];
		mr.target[AXIS_G] = mr.endpoint[AXIS_G];
	} else {
		float intermediate = mr.segment_velocity * mr.segment_move_time;
		mr.target[AXIS_X] = mr.position[AXIS_X] + (mr.unit[AXIS_X] * intermediate);
		mr.target[AXIS_Y] = mr.position[AXIS_Y] + (mr.unit[AXIS_Y] * intermediate);
		mr.target[AXIS_Z] = mr.position[AXIS_Z] + (mr.unit[AXIS_Z] * intermediate);
		mr.target[AXIS_G] = mr.position[AXIS_G] + (mr.unit[AXIS_G] * intermediate);
	}
	
	travel[AXIS_X] = mr.target[AXIS_X] - mr.position[AXIS_X];
	travel[AXIS_Y] = mr.target[AXIS_Y] - mr.position[AXIS_Y];
	travel[AXIS_Z] = mr.target[AXIS_Z] - mr.position[AXIS_Z];
	travel[AXIS_G] = mr.target[AXIS_G] - mr.position[AXIS_G];
	
	// IK here?
	//ik_kinematics(travel, mr.microseconds);
	callback_func(mr.target[AXIS_X],mr.target[AXIS_Y],mr.target[AXIS_Z],mr.target[AXIS_G]);
	
	copy_axis_vector(mr.position, mr.target);
	
	if (--mr.segment_count == 0) {
	    return STAT_COMPLETE;
	}
	return STAT_EAGAIN;
}
stat_t exec_tail()
{
	if (mr.section_state == MOVE_STATE_NEW) {
		if (fp_ZERO(mr.tail_length)) { return(STAT_OK);}		// end the move
		mr.midpoint_velocity = (mr.cruise_velocity + mr.exit_velocity) / 2;
		mr.move_time = mr.tail_length / mr.midpoint_velocity;
		mr.segments = ceil(uSec(mr.move_time) / (2 * NOM_SEGMENT_USEC));// # of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);// time to advance for each segment
		mr.segment_count = (uint32_t)mr.segments;
		
		Serial.print("Tail : move_time = ");
		Serial.print(mr.move_time,5);
		Serial.print(" segments = ");
		Serial.print(mr.segments,DEC);
		Serial.print(" segment_move_time = ");
		Serial.print(mr.segment_move_time,5);
		Serial.print(" cruise_velocity = ");
		Serial.print(mr.cruise_velocity);
		Serial.print(" midpoint_velocity = ");
		Serial.println(mr.midpoint_velocity,5);
		
		if ((mr.microseconds = uSec(mr.segment_move_time)) < MIN_SEGMENT_USEC) {
			return(STAT_SKIPPED);					// exit without advancing position
		}
		
		init_forward_diffs(mr.cruise_velocity, mr.midpoint_velocity);
		
		Serial.print("Forward diffs : ");
		Serial.print(mr.forward_diff_1,10);
		Serial.print(" ");
		Serial.println(mr.forward_diff_2,10);
		
		mr.section_state = MOVE_STATE_RUN1;
	}
	if (mr.section_state == MOVE_STATE_RUN1) {				// convex part (period 4)
		mr.segment_velocity += mr.forward_diff_1;
		if (exec_segment(false) == STAT_COMPLETE) { 	  	// set up for second half
			mr.segment_count = (uint32_t)mr.segments;
			mr.section_state = MOVE_STATE_RUN2;

			// Here's a trick: The second half of the S starts at the end of the first,
			//  And the only thing that changes is the sign of mr.forward_diff_2
			mr.forward_diff_2 = -mr.forward_diff_2;
		} else {
			mr.forward_diff_1 += mr.forward_diff_2;
		}
		return(STAT_EAGAIN);
	}
	if (mr.section_state == MOVE_STATE_RUN2) {				// concave part (period 5)
		mr.segment_velocity += mr.forward_diff_1;
		mr.forward_diff_1 += mr.forward_diff_2;
		if (exec_segment(true) == STAT_COMPLETE) { return (STAT_OK);}	// end the move
	}
	return(STAT_EAGAIN);
}


stat_t exec_body()
{
	if (mr.section_state == MOVE_STATE_NEW) {
		if (fp_ZERO(mr.body_length)) {
			mr.move_state = MOVE_STATE_TAIL;
			return(exec_tail());			// skip ahead to tail periods
		}
		mr.move_time = mr.body_length / mr.cruise_velocity;
		mr.segments = ceil(uSec(mr.move_time) / NOM_SEGMENT_USEC);
		mr.segment_move_time = mr.move_time / mr.segments;
		mr.segment_velocity = mr.cruise_velocity;
		mr.segment_count = (uint32_t)mr.segments;
		Serial.print("Body : move_time = ");
		Serial.print(mr.move_time,5);
		Serial.print(" segments = ");
		Serial.print(mr.segments,DEC);
		Serial.print(" segment_move_time = ");
		Serial.println(mr.segment_move_time,5);
		if ((mr.microseconds = uSec(mr.segment_move_time)) < MIN_SEGMENT_USEC) {
			return(STAT_SKIPPED);		// exit without advancing position
		}
		
		mr.section_state = MOVE_STATE_RUN;
	}
	if (mr.section_state == MOVE_STATE_RUN) {				// stright part (period 3)
		if (exec_segment(false) == STAT_COMPLETE) {
			if (fp_ZERO(mr.tail_length)) { return(STAT_OK);}	// end the move
			mr.move_state = MOVE_STATE_TAIL;
			mr.section_state = MOVE_STATE_NEW;
		}
	}
	return(STAT_EAGAIN);
}
stat_t exec_head() {
    if (mr.section_state == MOVE_STATE_NEW) {
        if (fp_ZERO(mr.head_length)) {
            Serial.println("Head length is zero!");
            mr.move_state = MOVE_STATE_BODY;
            return exec_body();
        }
        
        mr.midpoint_velocity = (mr.entry_velocity + mr.cruise_velocity) / 2;
		mr.move_time = mr.head_length / mr.midpoint_velocity;	// time for entire accel region
		mr.segments = ceil(uSec(mr.move_time) / (2 * NOM_SEGMENT_USEC)); // # of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);
		mr.segment_count = (uint32_t)mr.segments;
		
		Serial.print("Head : move_time = ");
		Serial.print(mr.move_time,5);
		Serial.print(" segments = ");
		Serial.print(mr.segments,DEC);
		Serial.print(" segment_move_time = ");
		Serial.print(mr.segment_move_time,5);
		Serial.print(" entry_velocity = ");
		Serial.print(mr.entry_velocity);
		Serial.print(" midpoint_velocity = ");
		Serial.println(mr.midpoint_velocity,5);
		
		if ((mr.microseconds = uSec(mr.segment_move_time)) < MIN_SEGMENT_USEC) {
		    Serial.println("skipped head");
			return(STAT_SKIPPED);		// exit without advancing position
		}
		init_forward_diffs(mr.entry_velocity, mr.midpoint_velocity);
		
		Serial.print("Forward diffs : ");
		Serial.print(mr.forward_diff_1,5);
		Serial.print(" ");
		Serial.println(mr.forward_diff_2,5);
		
		mr.section_state = MOVE_STATE_RUN1;
    }
    if (mr.section_state == MOVE_STATE_RUN1) {	// concave part of accel curve (period 1)
		mr.segment_velocity += mr.forward_diff_1;
		if (exec_segment(false) == STAT_COMPLETE) { // set up for second half
			mr.segment_count = (uint32_t)mr.segments;
			mr.section_state = MOVE_STATE_RUN2;

			// Here's a trick: The second half of the S starts at the end of the first,
			//  And the only thing that changes is the sign of mr.forward_diff_2
			mr.forward_diff_2 = -mr.forward_diff_2;
		} else {
			mr.forward_diff_1 += mr.forward_diff_2;
		}
		return(STAT_EAGAIN);
	}
	if (mr.section_state == MOVE_STATE_RUN2) {	// convex part of accel curve (period 2)
		mr.segment_velocity += mr.forward_diff_1;
		mr.forward_diff_1 += mr.forward_diff_2;
		if (exec_segment(false) == STAT_COMPLETE) {
			if ((fp_ZERO(mr.body_length)) && (fp_ZERO(mr.tail_length))) { return(STAT_OK);}	// end the move
			mr.move_state = MOVE_STATE_BODY;
			mr.section_state = MOVE_STATE_NEW;
		}
	}
	return(STAT_EAGAIN);
}

stat_t exec_move(move *m) {
    stat_t status = STAT_OK;
    
    if (m->move_state == MOVE_STATE_OFF) {
        Serial.println("Move inactive.");
        return STAT_NOOP;
    }
    
    if (mr.move_state == MOVE_STATE_OFF) {
        Serial.println("New move");
        
        if (fp_ZERO(m->length)) {
            mr.move_state = MOVE_STATE_OFF;
            mr.section_state = MOVE_STATE_OFF;
            return STAT_NOOP;
        }
        // new move
        m->move_state = MOVE_STATE_RUN;
        mr.move_state = MOVE_STATE_HEAD;
        mr.section_state = MOVE_STATE_NEW;
        
        mr.jerk = m->jerk;
        mr.head_length = m->head_length;
        mr.body_length = m->body_length;
        mr.tail_length = m->tail_length;
		mr.entry_velocity = m->entry_velocity;
        mr.cruise_velocity = m->cruise_velocity;
		mr.exit_velocity = m->exit_velocity;
		
		copy_axis_vector(mr.unit, m->unit);
		copy_axis_vector(mr.endpoint, m->target);	// save the final target of the move
    }
    
    switch (mr.move_state) {
        case (MOVE_STATE_HEAD): {
            //Serial.println("exec_head()");
            status = exec_head(); break;
        }
		case (MOVE_STATE_BODY): {
		    //Serial.println("exec_body()");
		    status = exec_body(); break;
		}
		case (MOVE_STATE_TAIL): {
		    //Serial.println("exec_tail()");
		    status = exec_tail(); break;
		}
		case (MOVE_STATE_SKIP): {
		    Serial.println("skip state");
		    status = STAT_OK; break;
		}
	}
	//	log_runtime();
    //Serial.print("internal move status = ");
    //Serial.println(status);
	
	if (status == STAT_EAGAIN) {
	    // we'll carry on with this move
	    return status;
	} else {
	    mr.move_state = MOVE_STATE_OFF;
	    mr.section_state = MOVE_STATE_OFF;
	    if (m->move_state == MOVE_STATE_RUN) {
	        free_move(m);
	        return STAT_COMPLETE;
	    }
	}
	return STAT_OK;
}
void init_planner(const float from[], void (*callback)(float x, float y, float z, float g) ) {
    memset(&mr, 0, sizeof(mr));
    
    mr.move_state = MOVE_STATE_OFF;
    mr.position[AXIS_X] = from[AXIS_X];
    mr.position[AXIS_Y] = from[AXIS_Y];
    mr.position[AXIS_Z] = from[AXIS_Z];
    mr.position[AXIS_G] = from[AXIS_G];
    
    callback_func = callback;
}