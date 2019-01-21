/*
 * Copyright (C) 2018 Guido de Croon
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file RANSAC.h
 * @brief Perform Random Sample Consensus (RANSAC), a robust fitting method.
 *
 * The concept is to select (minimal) subsets of the samples for learning
 * a model, then determining the error on the entire data set, and selecting the best fit. In determining the error, the individual sample errors
 * are capped, so that outliers can be identified and have only a modest influence on the results.
 *
 * Read: Fischler, M. A., & Bolles, R. C. (1981). Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography.
 * Communications of the ACM, 24(6), 381-395.
 *
 * This file depends on the function fit_linear_model in math/pprz_matrix_decomp_float.h/c
 */


#include "RANSAC.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit, of size D + 1 (accounting for a constant 1 being added to the samples to represent a potential bias)
 * @param[out] fit_error* Total error of the fit
 *
 */
void RANSAC_linear_model(int n_samples, int n_iterations, float error_threshold, float targets[MAX_SAMPLES], int Dimension,
                         float samples[MAX_SAMPLES][VECTOR_DIMENSION], uint16_t count, float *params, float *fit_error)
{
  printf("[RANSAC] 0\n");
  int i,j,k,d;
  float err_sum;
  float prediction;
  float min_err;
  int min_ind;

  uint8_t D_1 = Dimension + 1;
  float err;
  float errors[MAX_ITERATIONS];
  int indices_subset[MAX_SAMPLES];
  float subset_targets[MAX_SAMPLES];
  float subset_samples[MAX_SAMPLES][VECTOR_DIMENSION];
  float subset_params[MAX_ITERATIONS][VECTOR_DIMENSION+1];
  bool use_bias = true;

  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  printf("[RANSAC] 1\n");
  // do the RANSAC iterations:
  for ( i = 0; i < n_iterations; i++) {

    // get a subset of indices
    get_indices_without_replacement(indices_subset, n_samples, count);

  printf("[RANSAC] 2\n");
    // get the corresponding samples and targets:
    for ( j = 0; j < n_samples; j++) {
      subset_targets[j] = targets[indices_subset[j]];
      for ( k = 0; k < Dimension; k++) {
        subset_samples[j][k] = samples[indices_subset[j]][k];
      }
    }

    // fit a linear model on the small system:
    fit_linear_model(subset_targets, Dimension, subset_samples, n_samples, use_bias, subset_params[i], &err);

    // determine the error on the whole set:
    err_sum = 0.0f;
    
  printf("[RANSAC] 3\n");
    for ( j = 0; j < count; j++) {
      // predict the sample's value and determine the absolute error:
      prediction = predict_value(samples[j], subset_params[i], Dimension, use_bias);
      err = fabsf(prediction - targets[j]);
      // cap the error with the threshold:
      err = (err > error_threshold) ? error_threshold : err;
      err_sum += err;
    }
    errors[i] = err_sum;
  }

  printf("[RANSAC] 4\n");
  // determine the minimal error:
  min_err = errors[0];
  min_ind = 0;
  for ( i = 1; i < n_iterations; i++) {
    if (errors[i] < min_err) {
      min_err = errors[i];
      min_ind = i;
    }
  }

  // copy the parameters:
  for ( d = 0; d < D_1; d++) {
    params[d] = subset_params[min_ind][d];
  }
  printf("[RANSAC] 5\n");

}

/** Predict the value of a sample with linear weights.
 *
 * @param[in] sample The sample vector of size D
 * @param[in] weights The weight vector of size D+1
 * @param[in] D The dimension of the sample.
 * @return The predicted value
 */
float predict_value(float *sample, float *weights, int Dimension, bool use_bias)
{
  int w;

  float sum = 0.0f;

  for ( w = 0; w < Dimension; w++) {
    sum += weights[w] * sample[w];
  }
  if (use_bias) {
    sum += weights[Dimension];
  }

  // printf("Prediction = %f\n", sum);

  return sum;
}

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */

void get_indices_without_replacement(int *indices_subset, int n_samples, int count)
{
  int j, k;
  bool new_val;

  int index;

  for ( j = 0; j < n_samples; j++) {
    bool picked_number = false;
    while (!picked_number) {
      index = rand() % count;
      new_val = true;
      for ( k = 0; k < j; k++) {
        if (indices_subset[k] == index) {
          new_val = false;
          break;
        }
      }
      if (new_val) {
        indices_subset[j] = index;
        picked_number = true;
      }
    }
  }
}
