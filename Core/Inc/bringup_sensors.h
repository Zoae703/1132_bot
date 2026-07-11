#ifndef BRINGUP_SENSORS_H
#define BRINGUP_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

void SensorBringup_Init(void);
void SensorBringup_PrintHelp(void);
void SensorBringup_PrintImu(void);
void SensorBringup_PrintDepth(void);
void SensorBringup_PrintAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BRINGUP_SENSORS_H */
