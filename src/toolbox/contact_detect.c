/**
 *  Functions and auxiliary tools for contact detection
 **/

#include "contact_detect.h"

double contactDetectionFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

/* step function */
double reflectBack(double dmax, double dmin, double time)
{
    /* function parameters */
    double t0 = 2.0; // time to begin cycles
    double dtw = 0.0; // wait time
    double dto = 1.0; // open time
    double dth = 0.5; // hold time

    double T = 0.0;
    T = dtw + dto + dth;

    double local_time = 0.0;

    if (time < t0) {
        return dmax;
    }
    else {
        local_time = fmod(time - t0, T);
        if(local_time < dtw) {
            return dmax;
        }
        else if(local_time < (dtw + dto)) {
            return (dmax - dmin) / 2.0 * cos((local_time - dtw) * M_PI / dto) + (dmax + dmin) / 2.0;
        }
        else {
            return dmin;
        }
    }
}

void contactDetectionInit(ContactDetectionParams *contactDetectionParams, ContactDetectionLowPassFilterParams *contactDetectionLowPassFilterParams ,ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    /* parameters definition */
    contactDetectionParams->cutoff_frequency_LPF[0] = 100.0; // Hz
    contactDetectionParams->cutoff_frequency_LPF[1] = 100.0;
    contactDetectionParams->cutoff_frequency_HPF[0] = 5.0; // Hz
    contactDetectionParams->cutoff_frequency_HPF[1] = 5.0;
    contactDetectionParams->sample_time = 0.5e-3; // second
    contactDetectionParams->pressure_change_threshold[0] = 0.02; //2.5e-3; // Nm
    contactDetectionParams->pressure_change_threshold[1] = 0.02; //2.5e-3;
    contactDetectionParams->reflect_stiffness[0] = 10.0; // Nm/rad
    contactDetectionParams->reflect_stiffness[1] = 10.0;
    contactDetectionParams->reflect_distance[0] = 0.4;//0.4; // rad
    contactDetectionParams->reflect_distance[1] = 0.4;//0.4;
    contactDetectionParams->contact_detection_time[0] = 4000;
    contactDetectionParams->contact_detection_time[1] = 4000;
    contactDetectionParams->pos_buffer_size[0] = 1000;
    contactDetectionParams->pos_buffer_size[1] = 1000;
    contactDetectionParams->pos_deviation_threshold[0] = 1.0e-2;
    contactDetectionParams->pos_deviation_threshold[1] = 1.0e-2;
    contactDetectionParams->location[0] = 0;
    contactDetectionParams->location[1] = 0;
    contactDetectionParams->pos_deviation[0] = 5.0e-2;
    contactDetectionParams->pos_deviation[1] = 5.0e-2;
    contactDetectionParams->contact_flag_local[0] = 0;
    contactDetectionParams->contact_flag_local[1] = 0;
    contactDetectionParams->reflect_flag[0] = 0;
    contactDetectionParams->reflect_flag[1] = 0;
    contactDetectionParams->reflect_back_flag[0] = 0;
    contactDetectionParams->reflect_back_flag[1] = 0;
    contactDetectionParams->time[0] = 0.0;
    contactDetectionParams->time[1] = 0.0;
    contactDetectionParams->vel_threshold[0] = 1.0e-4;
    contactDetectionParams->vel_threshold[1] = 1.0e-4;
    contactDetectionParams->contact_detection_trigger = 0;

    /* high-pass filter parameters initialization */
    for (int i = 0; i < 2; i ++) {
        contactDetectionHighPassFilterParams->lambda[i] = 2.0 * M_PI * contactDetectionParams->cutoff_frequency_HPF[i];
        contactDetectionHighPassFilterParams->a1[i] = -1.0 * (contactDetectionHighPassFilterParams->lambda[i] * contactDetectionParams->sample_time - 2.0) / (contactDetectionHighPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
        contactDetectionHighPassFilterParams->b0[i] = 2.0 / (contactDetectionHighPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
        contactDetectionHighPassFilterParams->b1[i] = -2.0 / (contactDetectionHighPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
    }

    for (int i = 0; i < 2; i ++) {
        contactDetectionLowPassFilterParams->lambda[i] = 2.0 * M_PI * contactDetectionParams->cutoff_frequency_LPF[i];
        contactDetectionLowPassFilterParams->a1[i] = -1.0 * (contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time - 2.0) / (contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
        contactDetectionLowPassFilterParams->b0[i] = contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time / (contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
        contactDetectionLowPassFilterParams->b1[i] = contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time / (contactDetectionLowPassFilterParams->lambda[i] * contactDetectionParams->sample_time + 2.0);
    }


    /* previous variable initialization */
    contactDetectionPreviousVariable->pressure[0] = rdda->psensor.analogIn.val1;
    contactDetectionPreviousVariable->pressure[1] = rdda->psensor.analogIn.val2;
    contactDetectionPreviousVariable->filtered_pressure_LPF[0] = rdda->psensor.analogIn.val1;
    contactDetectionPreviousVariable->filtered_pressure_LPF[1] = rdda->psensor.analogIn.val2;
    contactDetectionPreviousVariable->filtered_pressure_HPF[0] = rdda->psensor.analogIn.val1;
    contactDetectionPreviousVariable->filtered_pressure_HPF[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < 2; i ++) {
        contactDetectionPreviousVariable->pressure_change_slop[i] = 0.0;
        contactDetectionPreviousVariable->filtered_pressure_change_slop[i] = 0.0;
    }


    /* pressure buffer initialization */
    for (int i = 0; i < 2; i ++) {
        for (int j = 0; j < contactDetectionParams->pos_buffer_size[i]; j ++) {
            contactDetectionPreviousVariable->pos_buffer[i][j] = rdda->motor[i].motorIn.act_pos;
        }
        contactDetectionPreviousVariable->pos_variance_sum[i] = 0.0;
        contactDetectionPreviousVariable->pos_average[i] = rdda->motor[i].motorIn.act_pos;
        contactDetectionPreviousVariable->pos_deviation[i] = 0.0;
        contactDetectionPreviousVariable->contact_detection_count[i] = contactDetectionParams->contact_detection_time[i];
    }
}

void contactDetection(ContactDetectionParams *contactDetectionParams, ContactDetectionLowPassFilterParams *contactDetectionLowPassFilterParams ,ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    int num = 2; // only reflect finger 1
    double pressure[num];
    double pos[num];
    double filtered_pressure_LPF[num];
    double filtered_pressure_HPF[num];
    double pressure_change_slop[num];
    double filtered_pressure_change_slop[num];
    //int intersection = 20;
    double previous_pos_average[num];
    int pos_flag = 0;
    int vel_flag = 0;

    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos;
        rdda->motor[i].stiffness = contactDetectionParams->reflect_stiffness[i];
    }

    /* pressure filtering */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;

    for (int i = 0; i < num; i ++) {
        filtered_pressure_LPF[i] = contactDetectionFirstOrderIIRFilter(pressure[i], contactDetectionPreviousVariable->pressure[i], contactDetectionPreviousVariable->filtered_pressure_LPF[i], contactDetectionLowPassFilterParams->b0[i], contactDetectionLowPassFilterParams->b1[i], contactDetectionLowPassFilterParams->a1[i]);
        // filtered_pressure_HPF[i] = contactDetectionFirstOrderIIRFilter(pressure[i], contactDetectionPreviousVariable->pressure[i], contactDetectionPreviousVariable->filtered_pressure_HPF[i], contactDetectionHighPassFilterParams->b0[i], contactDetectionHighPassFilterParams->b1[0], contactDetectionHighPassFilterParams->a1[i]);
        filtered_pressure_HPF[i] = contactDetectionFirstOrderIIRFilter(filtered_pressure_LPF[i], contactDetectionPreviousVariable->filtered_pressure_LPF[i], contactDetectionPreviousVariable->filtered_pressure_HPF[i], contactDetectionHighPassFilterParams->b0[i], contactDetectionHighPassFilterParams->b1[0], contactDetectionHighPassFilterParams->a1[i]);
        pressure_change_slop[i] = (filtered_pressure_LPF[i] - contactDetectionPreviousVariable->filtered_pressure_LPF[i]) / contactDetectionParams->sample_time;
        filtered_pressure_change_slop[i] = contactDetectionFirstOrderIIRFilter(pressure_change_slop[i], contactDetectionPreviousVariable->pressure_change_slop[i], contactDetectionPreviousVariable->filtered_pressure_change_slop[i], contactDetectionLowPassFilterParams->b0[i], contactDetectionLowPassFilterParams->b1[i], contactDetectionLowPassFilterParams->a1[i]);
    }

    /* pre-contact pos & vel requirement */
    // if ((rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos < -0.6) && (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos > -0.2) && (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos < 0.4)) {
        pos_flag = 1;
    // }
    // if ((fabs(rdda->motor[0].motorIn.act_vel) < 2.0e-2) && (fabs(rdda->motor[1].motorIn.act_vel) < 2.0e-2)) {
        vel_flag = 1;
    // }
    if (pos_flag && vel_flag) {
        contactDetectionParams->contact_detection_trigger = 1;
    }
    else contactDetectionParams->contact_detection_trigger = 0;

    rdda->motor[0].rddaPacket.test = pos[1] - contactDetectionPreviousVariable->pos_average[1] - contactDetectionParams->pos_deviation_threshold[1];
    rdda->motor[1].rddaPacket.test = pos[1] - contactDetectionPreviousVariable->pos_average[1] + contactDetectionParams->pos_deviation_threshold[1];

    /* contact detection */
    for (int i = 0; i < num; i ++) {
        if ((contactDetectionParams->reflect_flag[i] == 0) && (contactDetectionParams->reflect_back_flag[i] == 0) && (contactDetectionParams->contact_detection_time[i] == contactDetectionPreviousVariable->contact_detection_count[i]) && contactDetectionParams->contact_detection_trigger) {
            // if ((filtered_pressure_change_slop[i] > contactDetectionParams->pressure_change_threshold[i]) && (fabs(rdda->motor[i].motorIn.act_vel) > 0.0)) {    
            if (pos[i] - contactDetectionPreviousVariable->pos_average[i] > contactDetectionParams->pos_deviation_threshold[i]) {    
                contactDetectionParams->contact_flag_local[i] = 1;
                // rdda->motor[i].rddaPacket.contact_flag = 1;
                // rdda->motor[i].stiffness = contactDetectionParams->reflect_stiffness[i];
            }
            // else if ((filtered_pressure_change_slop[i] < - contactDetectionParams->pressure_change_threshold[i]) && (fabs(rdda->motor[i].motorIn.act_vel) > 0.0)) {
            else if (pos[i] - contactDetectionPreviousVariable->pos_average[i] < -1.0*contactDetectionParams->pos_deviation_threshold[i]) {    
                contactDetectionParams->contact_flag_local[i] = -1;
                rdda->motor[i].rddaPacket.contact_flag = 1;
                // rdda->motor[i].stiffness = contactDetectionParams->reflect_stiffness[i];
            }
            else contactDetectionParams->contact_flag_local[i] = 0;
        }
        else {
            contactDetectionParams->contact_flag_local[i] = 0;
            contactDetectionPreviousVariable->contact_detection_count[i]--;
            if (contactDetectionPreviousVariable->contact_detection_count[i] == 0) {
                rdda->motor[i].rddaPacket.contact_flag = 0;
                contactDetectionPreviousVariable->contact_detection_count[i] = contactDetectionParams->contact_detection_time[i];
            }
        }
    }

    /* finger reflection */
    for (int i = 0; i < num; i ++) {
        if (contactDetectionParams->contact_flag_local[i] != 0) {
            contactDetectionPreviousVariable->contact_detection_count[i]--;
            contactDetectionPreviousVariable->pos_collision[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
            rdda->motor[i].rddaPacket.pos_ref = contactDetectionPreviousVariable->pos_collision[i] + contactDetectionParams->contact_flag_local[i] * contactDetectionParams->reflect_distance[i];
            contactDetectionParams->reflect_flag[i] = 1;
        }
    }

    /* back to collision position */
    for (int i = 0; i < num; i ++) {
        if ((contactDetectionParams->reflect_flag[i] == 1) && (fabs(rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos - rdda->motor[i].rddaPacket.pos_ref) <= contactDetectionParams->pos_deviation[i])) {
            contactDetectionParams->reflect_flag[i] = 0;
            contactDetectionParams->reflect_back_flag[i] = 1;
            contactDetectionPreviousVariable->pos_reflection[i] = rdda->motor[i].rddaPacket.pos_ref;
        }
        if ((contactDetectionParams->reflect_back_flag[i] == 1) && (fabs(rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos - contactDetectionPreviousVariable->pos_collision[i]) > contactDetectionParams->pos_deviation[i]) && (contactDetectionParams->time[i] <= 3.5)) {
            rdda->motor[i].rddaPacket.pos_ref = reflectBack(contactDetectionPreviousVariable->pos_reflection[i], contactDetectionPreviousVariable->pos_collision[i], contactDetectionParams->time[i]);
            contactDetectionParams->time[i] += contactDetectionParams->sample_time;
        }
        else if ((contactDetectionParams->reflect_back_flag[i] == 1) && (fabs(rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos - contactDetectionPreviousVariable->pos_collision[i]) <= contactDetectionParams->pos_deviation[i]) && (fabs(rdda->motor[i].motorIn.act_vel) <= contactDetectionParams->vel_threshold[i])) {
            contactDetectionParams->reflect_back_flag[i] = 0;
            rdda->motor[i].rddaPacket.pos_ref = contactDetectionPreviousVariable->pos_collision[i];
            // rdda->motor[i].stiffness = 0.0;
            contactDetectionParams->time[i] = 0.0;
        }
    }

    /* pressure moving average data */
    for (int i = 0; i < num; i ++) {
        previous_pos_average[i] = contactDetectionPreviousVariable->pos_average[i];
        contactDetectionPreviousVariable->pos_average[i] += (pos[i] - contactDetectionPreviousVariable->pos_buffer[i][contactDetectionParams->location[i]]) / contactDetectionParams->pos_buffer_size[i];
        contactDetectionPreviousVariable->pos_variance_sum[i] += (pos[i] - contactDetectionPreviousVariable->pos_buffer[i][contactDetectionParams->location[i]]) * ((pos[i] + contactDetectionPreviousVariable->pos_buffer[i][contactDetectionParams->location[i]]) - previous_pos_average[i] - contactDetectionPreviousVariable->pos_average[i]);
        contactDetectionPreviousVariable->pos_buffer[i][contactDetectionParams->location[i]] = pos[i];
        contactDetectionPreviousVariable->pos_deviation[i] = sqrt(contactDetectionPreviousVariable->pos_variance_sum[i] / (contactDetectionParams->pos_buffer_size[i] - 1));
        if (contactDetectionParams->location[i] == contactDetectionParams->pos_buffer_size[i] - 1) contactDetectionParams->location[i] = 0;
        else contactDetectionParams->location[i]++;
        contactDetectionParams->pos_deviation_threshold[i] = 9.0 * contactDetectionPreviousVariable->pos_deviation[i];
    }

    /* previous variables update*/
    for (int i = 0; i < num; i ++) {
        contactDetectionPreviousVariable->pressure[i] = pressure[i];
        contactDetectionPreviousVariable->pressure_change_slop[i] = pressure_change_slop[i];
        contactDetectionPreviousVariable->filtered_pressure_change_slop[i] = filtered_pressure_change_slop[i];
        contactDetectionPreviousVariable->filtered_pressure_LPF[i] = filtered_pressure_LPF[i];
        contactDetectionPreviousVariable->filtered_pressure_HPF[i] = filtered_pressure_HPF[i];
    }

}