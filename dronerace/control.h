

struct dronerace_control_struct
{
  // States
  float psi_ref;    ///< maintain a rate limited speed set-point to smooth heading changes

  // Outputs to inner loop
  float phi_cmd;
  float theta_cmd;
  float psi_cmd;
  float alt_cmd;
};

extern struct dronerace_control_struct dr_control;

extern void control_reset(void);
extern void control_run(float dt);

extern float test_add(float a,float b);
extern void test_test();
extern void vector_add(float *a,float *b,float *c,int length);
