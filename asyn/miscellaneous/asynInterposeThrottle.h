#ifndef asynInterposeThrottle_H
#define asynInterposeThrottle_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

ASYN_API int asynInterposeThrottleConfig(const char *portName, int addr, double min_delay);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* asynInterposeThrottle_H */
