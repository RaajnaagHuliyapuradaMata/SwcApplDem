

#ifndef DEM_MANAGER_H
#define DEM_MANAGER_H

#include "DemManagerX.hpp"
#include "Ara_Dem_Type.hpp"

#define DEMMGR__BCM_PERIPHERAL_MASTER_CLOCK_TIMEOUT_MS              ((uint32) 1000U)
#define DEMMGR__ESP_WHEEL_PULSE_STAMPED_TIMEOUT_MS                  ((uint32) 400U)
#define DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT_MS          ((uint32) 3000U)
#define DEMMGR__TMM_STATUS_BODY_LCAN_TIMEOUT_MS                     ((uint32) 30000U)
#define DEMMGR__VEH_STS_BODY_LCAN_TIMEOUT_MS                        ((uint32) 3000U)
#define DEMMGR__VMS_STS_REQ_BODY_LCAN_TIMEOUT_MS                    ((uint32) 30000U)

#define DEMMGR__TASK_CYCLICITY                                      ((uint32) 200U)
#define DEMMGR__TIMEOUT_COUNTER(timeout_ms)                         ((uint16) ((timeout_ms) / DEMMGR__TASK_CYCLICITY))

#define DEMMGR__BCM_PERIPHERAL_MASTER_CLOCK_TIMEOUT                 ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__BCM_PERIPHERAL_MASTER_CLOCK_TIMEOUT_MS))
#define DEMMGR__ESP_WHEEL_PULSE_STAMPED_TIMEOUT                     ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__ESP_WHEEL_PULSE_STAMPED_TIMEOUT_MS))
#define DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT             ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT_MS))
#define DEMMGR__TMM_STATUS_BODY_LCAN_TIMEOUT                        ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__TMM_STATUS_BODY_LCAN_TIMEOUT_MS))
#define DEMMGR__VEH_STS_BODY_LCAN_TIMEOUT                           ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__VEH_STS_BODY_LCAN_TIMEOUT_MS))
#define DEMMGR__VMS_STS_REQ_BODY_LCAN_TIMEOUT                       ((uint16) DEMMGR__TIMEOUT_COUNTER(DEMMGR__VMS_STS_REQ_BODY_LCAN_TIMEOUT_MS))

#define DEMMGR__MISSING_RX_COUNTER_INIT                             ((uint16) 0U)

#define DEMMGR__ESP_WHEEL_PULSE_STAMPED_MIN_SPEED                   ((uint16) 0U)

#define cECU_UBAT_TIMEOUT 15

#define cDEMMGR_DTC_FIFO_SIZE 20
#define cDEMMGR_DTC_FIFO_EMPTY 0xffu

#define cWsDefect   0x01
#define cWsMissing  0x10

typedef struct{
  Dem_EventIdType ucEvent;
  Dem_EventStatusType ucStatus;
}sDtcEventFifoBuffer;

static void DemMGR_CheckEcuIdentificationAndApplMemory(void);
static boolean DemMGR__CheckSignalTimeout(DemMGR_Enum_RxMessages EN_RxMessage);
static void DemMGR__BcmPeripheralMasterClockMonitor(void);
static void DemMGR__EspWheelPulseStampedMonitor(void);
static void DemMGR__EspWspeedFrontRearBodyDirectionLCANMonitor(void);
static void DemMGR__EspWspeedFrontRearBodySpeedLCANMonitor(void);
static void DemMGR__TmmStatusBodyLCANMonitor(void);
static void DemMGR__VehStsBodyLCANMonitor(void);
static void DemMGR__VmsStsBodyLCANMonitor(void);

static void DEMMGR_FifoPut(Dem_EventIdType ucDtc, Dem_EventStatusType ucStatus);
static void DEMMGR_FiFoGet(Dem_EventIdType* ucError, Dem_EventStatusType* ucStatus);

#endif
