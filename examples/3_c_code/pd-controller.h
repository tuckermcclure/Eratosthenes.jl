/*
 * See pd-controller.c.
 */

#pragma once

void pd_controller(const double gains[2], const double q_TI[4], const double q_BI[4], const double w_BI_B[3], double f_B[3], double tau_B[3]);
