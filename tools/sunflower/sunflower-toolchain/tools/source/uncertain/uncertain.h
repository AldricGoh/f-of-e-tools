#ifndef  UNCERTAIN_H_
#define  UNCERTAIN_H_

typedef float approximate_float;

float unf_getcov(approximate_float, approximate_float);
float unf_getvar(approximate_float);


/* Calculate the covariance between two approximate floats. */
float unf_covar(approximate_float, approximate_float);

/* Calculate the variance of an approximate floats. */
float unf_var(approximate_float);

/* Calculate the best guess of an approximate float. */
float unf_best_guess(approximate_float);

/* Create an independent approximate float with given best guess
 * and variance.
 */
approximate_float unf_create(float best_guess, float variance);

/* Create an approximate float with no uncertainty. */
approximate_float unf_create_exact(float);

approximate_float sqrtunf(approximate_float);
approximate_float asinunf(approximate_float);
approximate_float atan2unf(approximate_float y, approximate_float x);

#endif
