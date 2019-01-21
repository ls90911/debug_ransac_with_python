

#include "filter.h"
#include "fifo.h"
#include "ransac_run.h"
#include "flightplan.h"

#include "std.h"



struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;



void filter_reset()
{
  // Time
  dr_state.time = 0.0f;

  // Position
  dr_state.x = -3.0f;
  dr_state.y = -1.2f;

  // Speed
  dr_state.vx = 0.0f;
  dr_state.vy = 0.0f;

  // Heading
  dr_state.psi = 0.0f;

  // Vision latency
  fifo_reset();
  ransac_reset();
}




// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81f
#define DR_FILTER_DRAG  0.5f
#define DR_FILTER_THRUSTCORR  0.8f

void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  float az = DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  float abx =  sinf(-theta) * az;
  float aby =  sinf( phi)   * az;

  // Earth accelerations
  float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
  float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


  // Velocity and Position
  dr_state.vx += ax * dt;
  dr_state.vy += ay * dt;
  dr_state.x += dr_state.vx * dt;
  dr_state.y += dr_state.vy * dt;

  // Time
  dr_state.time += dt;

  // Store psi for local corrections
  dr_state.psi = psi;

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  ransac_propagate(ax,ay,dt);
}

float log_mx, log_my;

void filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement)
  float sx, sy, sz;
  float rotx, roty;
  float mx, my;

  float vision_scale = 0.6f;

  fifo_pop(&sx, &sy, &sz);



  // Compute current absolute position
  // TODO: check: this is wrong!
  rotx =  cosf(dr_fp.gate_psi) * dr_vision.dx + sinf(dr_fp.gate_psi) * dr_vision.dy;
  roty = -sinf(dr_fp.gate_psi) * dr_vision.dx + cosf(dr_fp.gate_psi) * dr_vision.dy;

  mx = dr_fp.gate_x + rotx * vision_scale;
  my = dr_fp.gate_y + roty * vision_scale;

  log_mx = dr_fp.gate_x;
  log_my = dr_fp.gate_y;

  // Push to RANSAC
  ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my);
}
