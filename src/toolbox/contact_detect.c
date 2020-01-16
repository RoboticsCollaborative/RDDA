/**
 *  Functions and auxiliary tools for contact detection
 **/

#include "contact_detect.h"

double contactDetectionFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void contactDetectionInit(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    /* parameters definition */
    contactDetectionParams->cutoff_frequency_HPF[0] = 1; // Hz
    contactDetectionParams->cutoff_frequency_HPF[1] = 1;
    contactDetectionParams->sample_time = 0.5e-3; // second
    contactDetectionParams->pressure_threshold[0] = 2.0e-2; // Nm
    contactDetectionParams->pressure_threshold[1] = 2.0e-2;
    contactDetectionParams->reflect_stiffness[0] = 0.3; // Nm/rad
    contactDetectionParams->reflect_stiffness[1] = 0.3;
    contactDetectionParams->reflect_distance[0] = 0.2; // rad
    contactDetectionParams->reflect_distance[1] = 0.2;
    contactDetectionParams->intersection[0] = 200;
    contactDetectionParams->intersection[1] = 200;

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
}

void contactDetection(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda) {
    int num = 2;
    double pressure[num];
    double filtered_pressure_HPF[num];
    int intersection = 20;

    /* pressure filtering */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        filtered_pressure_HPF[i] = contactDetectionFirstOrderIIRFilter(pressure[i], contactDetectionPreviousVariable->pressure[i], contactDetectionPreviousVariable->filtered_pressure_HPF[i], contactDetectionHighPassFilterParams->b0[0], contactDetectionHighPassFilterParams->b1[0], contactDetectionHighPassFilterParams->a1[0]);
    }

    /* contact detection */
    for (int i = 0; i < num; i ++) {
        if (contactDetectionParams->intersection[i] == intersection) {
            if (filtered_pressure_HPF[i] > contactDetectionParams->pressure_threshold[i]) {
                rdda->motor[i].rosIn.contact_flag = 1;
                contactDetectionParams->intersection[i]--;
            }
            else if (filtered_pressure_HPF[i] < -contactDetectionParams->pressure_threshold[i]) {
                rdda->motor[i].rosIn.contact_flag = -1;
                contactDetectionParams->intersection[i]--;
            }
            else rdda->motor[i].rosIn.contact_flag = 0;
        }
        else if (contactDetectionParams->intersection[i] == 0) contactDetectionParams->intersection[i] = intersection;
        else {
            rdda->motor[i].rosIn.contact_flag = 0;
            contactDetectionParams->intersection[i]--;
        }

    }

    /* finger reflection */
    for (int i = 0; i < num; i ++) {
        if (rdda->motor[i].rosIn.contact_flag != 0) {
            rdda->motor[i].rosOut.pos_ref = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos + rdda->motor[i].rosIn.contact_flag * contactDetectionParams->reflect_distance[i];
        }
    }

    /* previous variables update*/
    for (int i = 0; i < num; i ++) {
        contactDetectionPreviousVariable->pressure[i] = pressure[i];
        contactDetectionPreviousVariable->filtered_pressure_HPF[i] = filtered_pressure_HPF[i];
    }

}