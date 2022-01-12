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
    double cutoff_frequency_LPF[2];
    double cutoff_frequency_HPF[2];
    double sample_time;
    double pressure_change_threshold[2];
    double reflect_stiffness[2];
    double reflect_distance[2];
    int contact_detection_time[2];
    int pos_buffer_size[2];
    double hard_contact_pos_deviation_threshold[2];
    double hard_contact_pos_deviation_multiplier[2];
    double soft_contact_pos_deviation_threshold[2];
    double soft_contact_pos_deviation_multiplier[2];
    int soft_contact_count[2];
    int soft_contact_count_threshold[2];
    int location[2];
    double pos_deviation[2];
    int contact_flag_local[2];
    int reflect_flag[2];
    int reflect_back_flag[2];
    double time[2];
    double vel_threshold[2];
    int contact_detection_trigger;
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
    double lambda[2];
    double a1[2];
    double b0[2];
    double b1[2];
} ContactDetectionLowPassFilterParams;

typedef struct
{
    double pressure[2];
    double pressure_change_slop[2];
    double filtered_pressure_change_slop[2];
    double filtered_pressure_LPF[2];
    double filtered_pressure_HPF[2];
    double pos_collision[2];
    double pos_reflection[2];
    double pos_buffer[2][1000];
    double pos_average[2];
    double pos_deviation[2];
    double pos_variance_sum[2];
    int contact_detection_count[2];
} ContactDetectionPreviousVariable;

void contactDetectionInit(ContactDetectionParams *contactDetectionParams, ContactDetectionLowPassFilterParams *contactDetectionLowPassFilterParams ,ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda);
void contactDetection(ContactDetectionParams *contactDetectionParams, ContactDetectionLowPassFilterParams *contactDetectionLowPassFilterParams ,ContactDetectionHighPassFilterParams *contactDetectionHighPassFilterParams, ContactDetectionPreviousVariable *contactDetectionPreviousVariable, Rdda *rdda);

#endif // CONTACT_DETECT_H