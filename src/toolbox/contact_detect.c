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

void contactDetectionInit(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    /* parameters definition */
    contactDetectionParams->cutoff_frequency_HPF[0] = 1; // Hz
    contactDetectionParams->cutoff_frequency_HPF[1] = 1;
    contactDetectionParams->sample_time = 0.5e-3; // second
    contactDetectionParams->pressure_threshold[0] = 5.0e-3; //2.5e-3; // Nm
    contactDetectionParams->pressure_threshold[1] = 5.0e-3; //2.5e-3;
    contactDetectionParams->reflect_stiffness[0] = 2.0; // Nm/rad
    contactDetectionParams->reflect_stiffness[1] = 2.0;
    contactDetectionParams->reflect_distance[0] = 0.2;//0.4; // rad
    contactDetectionParams->reflect_distance[1] = 0.2;//0.4;
    contactDetectionParams->contact_detection_time[0] = 2000;
    contactDetectionParams->contact_detection_time[1] = 2000;
    contactDetectionParams->pressure_buffer_size[0] = 80;
    contactDetectionParams->pressure_buffer_size[1] = 80;
    contactDetectionParams->pressure_deviation_threshold[0] = 1.5e-3;
    contactDetectionParams->pressure_deviation_threshold[1] = 1.5e-3;
    contactDetectionParams->location[0] = 0;
    contactDetectionParams->location[1] = 0;
    contactDetectionParams->pos_deviation[0] = 4.0e-2;
    contactDetectionParams->pos_deviation[1] = 4.0e-2;
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

    /* previous variable initialization */
    contactDetectionPreviousVariable->pressure[0] = rdda->psensor.analogIn.val1;
    contactDetectionPreviousVariable->pressure[1] = rdda->psensor.analogIn.val2;
    contactDetectionPreviousVariable->filtered_pressure_HPF[0] = rdda->psensor.analogIn.val1;
    contactDetectionPreviousVariable->filtered_pressure_HPF[1] = rdda->psensor.analogIn.val2;

    /* pressure buffer initialization */
    for (int i = 0; i < 2; i ++) {
        for (int j = 0; j < contactDetectionParams->pressure_buffer_size[i]; j ++) {
            contactDetectionPreviousVariable->pressure_buffer[i][j] = contactDetectionPreviousVariable->pressure[i];
        }
        contactDetectionPreviousVariable->pressure_summation[i] = contactDetectionPreviousVariable->pressure[i] * contactDetectionParams->pressure_buffer_size[i];
        contactDetectionPreviousVariable->pressure_average[i] = contactDetectionPreviousVariable->pressure[i];
        contactDetectionPreviousVariable->pressure_deviation[i] = 0.0;
        contactDetectionPreviousVariable->contact_detection_count[i] = contactDetectionParams->contact_detection_time[i];
    }
}

void contactDetection(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    int num = 1; // only reflect finger 1
    double pressure[num];
    double filtered_pressure_HPF[num];
    //int intersection = 20;
    double pressure_deviation[2];
    int pos_flag = 0;
    int vel_flag = 0;

    /* pressure filtering */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;

    for (int i = 0; i < num; i ++) {
        filtered_pressure_HPF[i] = contactDetectionFirstOrderIIRFilter(pressure[i], contactDetectionPreviousVariable->pressure[i], contactDetectionPreviousVariable->filtered_pressure_HPF[i], contactDetectionHighPassFilterParams->b0[0], contactDetectionHighPassFilterParams->b1[0], contactDetectionHighPassFilterParams->a1[0]);
    }

    /* pressure moving average data */
    for (int i = 0; i < num; i ++) {
        contactDetectionPreviousVariable->pressure_summation[i] +=  pressure[i] - contactDetectionPreviousVariable->pressure_buffer[i][contactDetectionParams->location[i]];
        contactDetectionPreviousVariable->pressure_buffer[i][contactDetectionParams->location[i]] = pressure[i];
        contactDetectionPreviousVariable->pressure_average[i] = contactDetectionPreviousVariable->pressure_summation[i] / contactDetectionParams->pressure_buffer_size[i];
        for (int j = 0; j < contactDetectionParams->pressure_buffer_size[i]; j ++) {
            pressure_deviation[i] += pow(contactDetectionPreviousVariable->pressure_buffer[i][j] - contactDetectionPreviousVariable->pressure_average[i], 2);
        }
        contactDetectionPreviousVariable->pressure_deviation[i] = sqrt(pressure_deviation[i] / (contactDetectionParams->pressure_buffer_size[i] - 1));
        if (contactDetectionParams->location[i] == contactDetectionParams->pressure_buffer_size[i] - 1) contactDetectionParams->location[i] = 0;
        else contactDetectionParams->location[i]++;
    }

    /* pre-contact pos & vel requirement */
    if ((rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos < -0.7) && (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos > 0.0) && (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos < 0.4)) {
        pos_flag = 1;
    }
    if ((fabs(rdda->motor[0].motorIn.act_vel) < 1.0e-2) && (fabs(rdda->motor[1].motorIn.act_vel) < 1.0e-2)) {
        vel_flag = 1;
    }
    if (pos_flag && vel_flag) {
        contactDetectionParams->contact_detection_trigger = 1;
    }
    else contactDetectionParams->contact_detection_trigger = 0;

    printf("%+2.8lf, %+2.8lf, %+2.8lf, %+2.8lf, %d\r", rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos, rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos, rdda->motor[0].motorIn.act_vel, rdda->motor[1].motorIn.act_vel, contactDetectionParams->contact_detection_trigger);

    /* contact detection */
    for (int i = 0; i < num; i ++) {
        if ((contactDetectionPreviousVariable->pressure_deviation[i] <= contactDetectionParams->pressure_deviation_threshold[i]) && (contactDetectionParams->reflect_flag[i] == 0) && (contactDetectionParams->reflect_back_flag[i] == 0) && (contactDetectionParams->contact_detection_time[i] == contactDetectionPreviousVariable->contact_detection_count[i]) && contactDetectionParams->contact_detection_trigger) {
            if (pressure[i] - contactDetectionPreviousVariable->pressure_average[i] > contactDetectionParams->pressure_threshold[i]) {
                contactDetectionParams->contact_flag_local[i] = 1;
                rdda->motor[i].rddaPacket.contact_flag = 1;
                rdda->motor[i].stiffness = 2.0;
            }
            else if (pressure[i] - contactDetectionPreviousVariable->pressure_average[i] < -contactDetectionParams->pressure_threshold[i]) {
                contactDetectionParams->contact_flag_local[i] = -1;
                rdda->motor[i].rddaPacket.contact_flag = 1;
                rdda->motor[i].stiffness = 2.0;
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
            rdda->motor[i].stiffness = 0.0;
            contactDetectionParams->time[i] = 0.0;
        }
    }

    /* previous variables update*/
    for (int i = 0; i < num; i ++) {
        contactDetectionPreviousVariable->pressure[i] = pressure[i];
        contactDetectionPreviousVariable->filtered_pressure_HPF[i] = filtered_pressure_HPF[i];
    }

}