#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void SensorTaskFunc(void *argument);
void ControlTaskFunc(void *argument);
void CommTaskFunc(void *argument);

#ifdef __cplusplus
}
#endif
