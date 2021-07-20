#ifndef CONTACT_DETECT_H
#define CONTACT_DETECT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "shm_data.h"
#include "rdda_base.h"

typedef struct
{
    double cutoff_frequency_HPF[2];
    double sample_time;
    double pressure_threshold[2];
    double reflect_stiffness[2];
    double reflect_distance[2];
    int contact_detection_time[2];
    int pressure_buffer_size[2];
    double pressure_deviation_threshold[2];
    int location[2];
    double pos_deviation[2];
    int contact_flag_local[2];
    int reflect_flag[2];
    int reflect_back_flag[2];
    double time[2];
    double vel_threshold[2];
    double finger_inertia;
    double finger_damping;
    double finger_stiffness;
    double series_stiffness;
    double series_damping;
    double finger_est_pos[2];
    double ext_force_est_gain;
    double ext_force_est[2];
    double ext_force_int[2];
} ContactDetectionParams;

typedef struct
{
    double lambda[2];
    double a1[2];
    double b0[2];
    double b1[2];
} ContactDetectionHighPassFilterParams;

typedef struct
{
    double pressure[2];
    double filtered_pressure_HPF[2];
    double pos_collision[2];
    double pos_reflection[2];
    double pressure_buffer[2][80];
    double pressure_summation[2];
    double pressure_average[2];
    double pressure_deviation[2];
    int contact_detection_count[2];
} ContactDetectionPreviousVariable;

void contactDetectionInit(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda);
void contactDetection(ContactDetectionParams *contactDetectionParams, ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda);

#endif // CONTACT_DETECT_H