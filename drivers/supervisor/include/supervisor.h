#ifndef SUPERVISOR_H_
#define SUPERVISOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define M_PI       3.14159265358979323846f
#define DEG_TO_RAD (M_PI / 180.0f)

void
safety_supervisor(float current_angle);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISOR_H_ */
