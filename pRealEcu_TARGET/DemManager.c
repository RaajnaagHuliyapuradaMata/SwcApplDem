/******************************************************************************/
/* File   : DemManager.c                                                      */
/*                                                                            */
/* Author : Raajnaag HULIYAPURADA MATA                                        */
/*                                                                            */
/* License / Warranty / Terms and Conditions                                  */
/*                                                                            */
/* Everyone is permitted to copy and distribute verbatim copies of this lice- */
/* nse document, but changing it is not allowed. This is a free, copyright l- */
/* icense for software and other kinds of works. By contrast, this license is */
/* intended to guarantee your freedom to share and change all versions of a   */
/* program, to make sure it remains free software for all its users. You have */
/* certain responsibilities, if you distribute copies of the software, or if  */
/* you modify it: responsibilities to respect the freedom of others.          */
/*                                                                            */
/* All rights reserved. Copyright � 1982 Raajnaag HULIYAPURADA MATA           */
/*                                                                            */
/* Always refer latest software version from:                                 */
/* https://github.com/RaajnaagHuliyapuradaMata?tab=repositories               */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "Std_Types.hpp"

#include "DemManager.hpp"
#include "EnvManagerX.hpp"
#include "CanManagerX.hpp"
#include "SwcApplDcm.hpp"
#include "ProductionFlashInterfaceX.hpp"
#include "FeeFblBlockInterfaceX.hpp"
#include "EcuDiag.hpp"
#include "Dem.hpp"
#include "Fee_30_SmallSector.hpp"
#include "AdcX.hpp"
#include "rba_DiagLib.hpp"

#include "iTpms_Interface.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
typedef void (*DemMGR_SignalMonitorFunction) (void);

typedef struct{
   uint16                       U16_SignalTimeout;
   DemMGR_SignalMonitorFunction PF_SignalMonitorFunction;
   Dem_EventIdType              U16_EventId;
}DemMGR__Struct_SignalMonitoringData;

typedef struct{
   Type_SwcApplTpms_eErrors E_TPMSError;
   Dem_EventIdType    U16_EventId;
}DemMGR__Struct_DTCMapping;

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/
static const DemMGR__Struct_DTCMapping DemMGR__CAS_DTCMappings[SwcApplTpms_eError_MAX_NUMBER] = {
      {SwcApplTpms_eError_WS_LIFE_TIME_FL_CNT_LOW,                    DemConf_DemEventParameter_DemEventParameter_SensorLifeTimeCounterLowFL}
   ,  {SwcApplTpms_eError_WS_LIFE_TIME_FR_CNT_LOW,                    DemConf_DemEventParameter_DemEventParameter_SensorLifeTimeCounterLowFR}
   ,  {SwcApplTpms_eError_WS_LIFE_TIME_RR_CNT_LOW,                    DemConf_DemEventParameter_DemEventParameter_SensorLifeTimeCounterLowRR}
   ,  {SwcApplTpms_eError_WS_LIFE_TIME_RL_CNT_LOW,                    DemConf_DemEventParameter_DemEventParameter_SensorLifeTimeCounterLowRL}
   ,  {SwcApplTpms_eError_WS_DEFECT_FL,                               DemConf_DemEventParameter_DemEventParameter_SensorDefectFL}
   ,  {SwcApplTpms_eError_WS_DEFECT_FR,                               DemConf_DemEventParameter_DemEventParameter_SensorDefectFR}
   ,  {SwcApplTpms_eError_WS_DEFECT_RR,                               DemConf_DemEventParameter_DemEventParameter_SensorDefectRR}
   ,  {SwcApplTpms_eError_WS_DEFECT_RL,                               DemConf_DemEventParameter_DemEventParameter_SensorDefectRL}
   ,  {SwcApplTpms_eError_WS_DEFECT_UNKNOWN_POS,                      DemConf_DemEventParameter_DemEventParameter_SensorDefectUnknownPosition}
   ,  {SwcApplTpms_eError_WS_MISSING_FL,                              DemConf_DemEventParameter_DemEventParameter_SensorMissingFL}
   ,  {SwcApplTpms_eError_WS_MISSING_FR,                              DemConf_DemEventParameter_DemEventParameter_SensorMissingFR}
   ,  {SwcApplTpms_eError_WS_MISSING_RR,                              DemConf_DemEventParameter_DemEventParameter_SensorMissingRR}
   ,  {SwcApplTpms_eError_WS_MISSING_RL,                              DemConf_DemEventParameter_DemEventParameter_SensorMissingRL}
   ,  {SwcApplTpms_eError_WS_MISSING_UNKNOWN_POS,                     DemConf_DemEventParameter_DemEventParameter_SensorMissingUnknownPosition}
   ,  {SwcApplTpms_eError_WS_OVER_TEMP_FL,                            DemConf_DemEventParameter_DemEventParameter_SensorOvertempFL}
   ,  {SwcApplTpms_eError_WS_OVER_TEMP_FR,                            DemConf_DemEventParameter_DemEventParameter_SensorOvertempFR}
   ,  {SwcApplTpms_eError_WS_OVER_TEMP_RR,                            DemConf_DemEventParameter_DemEventParameter_SensorOvertempRR}
   ,  {SwcApplTpms_eError_WS_OVER_TEMP_RL,                            DemConf_DemEventParameter_DemEventParameter_SensorOvertempRL}
   ,  {SwcApplTpms_eError_AXLE_LOCATE_FAIL,                           DemConf_DemEventParameter_DemEventParameter_AutolocationFail}
   ,  {SwcApplTpms_eError_NO_QUALIFIED_SENSOR_SIGNALS,                DemConf_DemEventParameter_DemEventParameter_NoQualifiedSensorSignals}
   ,  {SwcApplTpms_eError_DETECTED_TOO_MANY_SENSORS,                  DemConf_DemEventParameter_DemEventParameter_DetectedTooManySensors}
   ,  {SwcApplTpms_eError_NOT_FUNCTIONING_DUE_TO_ECU_INTERNAL_FAULT,  DemConf_DemEventParameter_DemEventParameter_TpmsNotFunctional}
   ,  {SwcApplTpms_eError_WS_MISSING_DURING_CONTINUOUS_INVALID_SPEED, DemConf_DemEventParameter_DemEventParameter_CanSpeedSignalMissing}
   ,  {SwcApplTpms_eError_RF_INTERFERENCE,                            DemConf_DemEventParameter_DemEventParameter_EcuRfInterference}
   ,  {SwcApplTpms_eError_AUTOLOCATION_FAILED,                        DemConf_DemEventParameter_DemEventParameter_AutolocationFail}
};

static const DemMGR__Struct_SignalMonitoringData DemMGR__CS_SignalMonitoringData[DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED] = {
      {DEMMGR__BCM_PERIPHERAL_MASTER_CLOCK_TIMEOUT,     &DemMGR__BcmPeripheralMasterClockMonitor,            DemConf_DemEventParameter_DemEventParameter_CanClockSyncSignalMissing}
   ,  {DEMMGR__ESP_WHEEL_PULSE_STAMPED_TIMEOUT,         &DemMGR__EspWheelPulseStampedMonitor,                DemConf_DemEventParameter_DemEventParameter_CanAbsWheelPulseSignalMissing}
   ,  {DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT, &DemMGR__EspWspeedFrontRearBodyDirectionLCANMonitor, DemConf_DemEventParameter_DemEventParameter_CanWheelDirectionSignalMissing}
   ,  {DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT, (DemMGR_SignalMonitorFunction)0,                     DemConf_DemEventParameter_DemEventParameter_CanWheelDirectionSignalMissing}
   ,  {DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT, &DemMGR__EspWspeedFrontRearBodySpeedLCANMonitor,     DemConf_DemEventParameter_DemEventParameter_CanSpeedSignalMissing}
   ,  {DEMMGR__ESP_WSPEED_FRONT_REAR_BODY_LCAN_TIMEOUT, (DemMGR_SignalMonitorFunction)0,                     DemConf_DemEventParameter_DemEventParameter_CanSpeedSignalMissing}
   ,  {DEMMGR__TMM_STATUS_BODY_LCAN_TIMEOUT,            &DemMGR__TmmStatusBodyLCANMonitor,                   DemConf_DemEventParameter_DemEventParameter_CanAtmosphericTemperatureSignalMissing}
   ,  {DEMMGR__VEH_STS_BODY_LCAN_TIMEOUT,               &DemMGR__VehStsBodyLCANMonitor,                      DemConf_DemEventParameter_DemEventParameter_CanVehicleModeSignalMissing}
   ,  {DEMMGR__VMS_STS_REQ_BODY_LCAN_TIMEOUT,           &DemMGR__VmsStsBodyLCANMonitor,                      DemConf_DemEventParameter_DemEventParameter_CanOffroadSignalMissing}
};

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
static uint16 DemMGR__AU16_MissingRxCounter[DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED];
static boolean DemMGR__BO_IsMonitoringActive;
static sDtcEventFifoBuffer sDtcFifo[cDEMMGR_DTC_FIFO_SIZE];
static uint8 ucEcuBatHiCounter = 0xff;
static uint8 ucEcuBatLoCounter = 0xff;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
void DemMGR_Init(void){
   uint8 U8_Counter;
   for(
      U8_Counter = 0;
      U8_Counter < DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED;
      U8_Counter ++
   ){
      DemMGR__AU16_MissingRxCounter[U8_Counter] = DEMMGR__MISSING_RX_COUNTER_INIT;
   }
   DemMGR__BO_IsMonitoringActive = FALSE;
   DemMGR_CheckEcuIdentificationAndApplMemory();
   for(
      U8_Counter = 0;
      U8_Counter<cDEMMGR_DTC_FIFO_SIZE;
      U8_Counter ++
   ){
      sDtcFifo[U8_Counter].ucEvent  = (Dem_EventIdType)cDEMMGR_DTC_FIFO_EMPTY;
      sDtcFifo[U8_Counter].ucStatus = (Dem_EventStatusType)cDEMMGR_DTC_FIFO_EMPTY;
   }
}

void DemMGR_StartMonitoring(void){
   if(
         FALSE
      == DemMGR__BO_IsMonitoringActive
   ){
      uint8 U8_Counter;
      for(
         U8_Counter = 0;
         U8_Counter < DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED;
         U8_Counter ++
      ){
         DemMGR__AU16_MissingRxCounter[U8_Counter] = DEMMGR__MISSING_RX_COUNTER_INIT;
      }
      Dem_SetOperationCycleState(
            DemConf_DemOperationCycle_DemOperationCycle_BUSACTIVE
         ,  DEM_CYCLE_STATE_START
      );
      DemMGR__BO_IsMonitoringActive = TRUE;
   }
}

void DemMGR_StopMonitoring(void){
   Dem_SetOperationCycleState(
         DemConf_DemOperationCycle_DemOperationCycle_BUSACTIVE
      ,  DEM_CYCLE_STATE_END
   );
   DemMGR__BO_IsMonitoringActive = FALSE;
}

void DemMGR_MainFunction(void){
   sDtcEventFifoBuffer sLatestDtc;
   static              uint8 ucDelay = 0;
   if(
         FALSE
      != DemMGR__BO_IsMonitoringActive
   ){
      uint8 U8_Counter;
      for(
         U8_Counter = 0;
         U8_Counter < DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED;
         U8_Counter ++
      ){
         if(
               DemMGR__AU16_MissingRxCounter[U8_Counter]
            <  0xFFFFU
         ){
            DemMGR__AU16_MissingRxCounter[U8_Counter]++;
         }
         if(
               DemMGR__CS_SignalMonitoringData[U8_Counter].PF_SignalMonitorFunction
            != 0
         ){
            DemMGR__CS_SignalMonitoringData[U8_Counter].PF_SignalMonitorFunction();
         }
         if(
               ucDelay
            == 0
         ){
            DEMMGR_FiFoGet(
                  &sLatestDtc.ucEvent
               ,  &sLatestDtc.ucStatus
            );
            if(
                  sLatestDtc.ucEvent
               != (Dem_EventIdType)cDEMMGR_DTC_FIFO_EMPTY
            ){
               Dem_SetEventStatus(
                     sLatestDtc.ucEvent
                  ,  sLatestDtc.ucStatus
               );
               ucDelay = 8;
            }
         }
         else{
            ucDelay--;
         }
      }
   }
}

void DemMGR_RxReceived(
   DemMGR_Enum_RxMessages EN_RxMessage
){
   DemMGR__AU16_MissingRxCounter[EN_RxMessage] = DEMMGR__MISSING_RX_COUNTER_INIT;
}

void DemMGR_ClearMonitor(
   DemMGR_Enum_RxMessages EN_RxMessage
){
   DemMGR__AU16_MissingRxCounter[EN_RxMessage] = DEMMGR__MISSING_RX_COUNTER_INIT;
}

void DemMGR_EcuVoltageMonitorMainFunction(void){
   Dem_UdsStatusByteType ucStatus;
   uint8                 ucVoltage = ADC_GetKl30Voltage();
   if(
         ucVoltage
      != 0xff
   ){
      Dem_GetEventStatus(
            DemConf_DemEventParameter_Event_BatteryVoltageLow
         ,  &ucStatus
      );
      if(
            (
                  ucStatus
               &  DEM_UDS_STATUS_TF
            )
         == DEM_UDS_STATUS_TF
      ){
         if(
               ucEcuBatLoCounter
            == 0xff
         ){
            ucEcuBatLoCounter = 15;
         }
         if(
               ucVoltage
            >= cECU_UBAT_MIN
         ){
            ucEcuBatLoCounter--;
         }
      }
      else{
         if(
               ucEcuBatLoCounter
            == 0xff
         ){
            ucEcuBatLoCounter = 0;
         }
         if(
               ucVoltage
            >= cECU_UBAT_MIN
         ){
            ucEcuBatLoCounter = 0;
         }
         else{
            ucEcuBatLoCounter++;
         }
      }
      Dem_GetEventStatus(
            DemConf_DemEventParameter_Event_BatteryVoltageHigh
         ,  &ucStatus
      );
      if(
            (
                  ucStatus
               &  DEM_UDS_STATUS_TF
            )
         == DEM_UDS_STATUS_TF
      ){
         if(
               ucEcuBatHiCounter
            == 0xff
         ){
            ucEcuBatHiCounter = 15;
         }
         if(
               ucVoltage
            <= cECU_UBAT_MAX
         ){
            ucEcuBatHiCounter--;
         }
      }
      else{
         if(
               ucEcuBatHiCounter
            == 0xff
         ){
            ucEcuBatHiCounter = 0;
         }
         if(
               ucVoltage
            <= cECU_UBAT_MAX
         ){
            ucEcuBatHiCounter = 0;
         }
         else{
            ucEcuBatHiCounter++;
         }
      }
      if(
            ucEcuBatLoCounter
         == cECU_UBAT_TIMEOUT
      ){
         DEMMGR_FifoPut(
               DemConf_DemEventParameter_Event_BatteryVoltageLow
            ,  DEM_EVENT_STATUS_FAILED
         );
      }
      else if(
            ucEcuBatLoCounter
         == 0
      ){
         DEMMGR_FifoPut(
               DemConf_DemEventParameter_Event_BatteryVoltageLow
            ,  DEM_EVENT_STATUS_PASSED
         );
      }
      else{
      }
      if(
            ucEcuBatHiCounter
         == cECU_UBAT_TIMEOUT
      ){
         DEMMGR_FifoPut(
               DemConf_DemEventParameter_Event_BatteryVoltageHigh
            ,  DEM_EVENT_STATUS_FAILED
         );
      }
      else if(
            ucEcuBatHiCounter
         == 0
      ){
         DEMMGR_FifoPut(
               DemConf_DemEventParameter_Event_BatteryVoltageHigh
            ,  DEM_EVENT_STATUS_PASSED
         );
      }
      else{
      }
   }
}

#include "CfgSwcServiceStartUp.hpp"
static void DemMGR_CheckEcuIdentificationAndApplMemory(void){ //TBD: Move to StartUp/MemoryServices
   uint8  ucBuffer[40];
   uint8  ucSize;
   uint32 i, j;
   uint16 ushChecksumCalculated;
   uint16 ushChecksumRead;
   PRODFLASH_GetEolCheckerByte(
      ucBuffer
   );
   if(
         ucBuffer[0]
      == 0xff
   ){
      ushChecksumCalculated = 0;
      ushChecksumRead = *(uint16*)(CfgSwcServiceStartUp_dAddressApplicationEnd-1);
      for(
         i = CfgSwcServiceStartUp_dAddressApplicationStart;
         i < (CfgSwcServiceStartUp_dAddressApplicationEnd-1);
         i ++
      ){
         ushChecksumCalculated += *(uint8*)i;
      }
      ushChecksumCalculated =
            ((ushChecksumCalculated & 0x00FFu) << 8)
         |  ((ushChecksumCalculated & 0xFF00u) >> 8)
      ;
      if(
            ushChecksumCalculated
         != ushChecksumRead
      ){
         DEMMGR_FifoPut(
               DemConf_DemEventParameter_Event_FailedMemory
            ,  DEM_EVENT_STATUS_FAILED
         );
      }
      PRODFLASH_PutApplicationChecksum(
         (const uint8*)&ushChecksumCalculated
      );
   }
   else{
      PRODFLASH_GetManufacturingSupportMode(
         ucBuffer
      );
      if(
            ucBuffer[0]
         == cMANUFSUPPMODE_RIVIAN
      ){
         ucSize = PRODFLASH_GetVin(
            ucBuffer
         );
         j = 0;
         for(
            i = 0;
            i < ucSize;
            i ++
         ){
            if(
                  ucBuffer[i]
               == 0xff
            ){
               j++;
            }
         }
         if(
            j > 0
         ){
            DEMMGR_FifoPut(
                  DemConf_DemEventParameter_Event_NoVin
               ,  DEM_EVENT_STATUS_FAILED
            );
         }
         else{
            DEMMGR_FifoPut(
                  DemConf_DemEventParameter_Event_NoVin
               ,  DEM_EVENT_STATUS_PASSED
            );
         }
         ucSize = PRODFLASH_GetRivianEcuSerialNumber(
            ucBuffer
         );
         j = 0;
         for(
            i = 0;
            i < ucSize;
            i ++
         ){
            if(
                  ucBuffer[i]
               == 0xff
            ){
               j++;
            }
         }
         if(
            j > 0
         ){
            DEMMGR_FifoPut(
                  DemConf_DemEventParameter_Event_NoEcuSerialNumber
               ,  DEM_EVENT_STATUS_FAILED
            );
         }
         else{
            DEMMGR_FifoPut(
                  DemConf_DemEventParameter_Event_NoEcuSerialNumber
               ,  DEM_EVENT_STATUS_PASSED
            );
         }
      }
   }
}

static boolean DemMGR__CheckSignalTimeout(
   DemMGR_Enum_RxMessages EN_RxMessage
){
   boolean BO_SignalTimeoutOccured = FALSE;
   if(
         EN_RxMessage
      <  DEMMGR_E_NUMBER_OF_RX_MESSAGES_MONITED
   ){
      if(
            DemMGR__AU16_MissingRxCounter[EN_RxMessage]
         >= DemMGR__CS_SignalMonitoringData[EN_RxMessage].U16_SignalTimeout
      ){
         BO_SignalTimeoutOccured = true;
      }
   }
   return BO_SignalTimeoutOccured;
}

static void DemMGR__BcmPeripheralMasterClockMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_BCM_PERIPHERAL_MASTER_CLOCK_INDEX].U16_EventId
      ,  &U8_UdsStatus
   );
   if(
         DemMGR__CheckSignalTimeout(
            DEMMGR_E_BCM_PERIPHERAL_MASTER_CLOCK_INDEX
         )
      != FALSE
   ){    if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_BCM_PERIPHERAL_MASTER_CLOCK_INDEX].U16_EventId, DEM_EVENT_STATUS_FAILED);}}
   else{ if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_BCM_PERIPHERAL_MASTER_CLOCK_INDEX].U16_EventId, DEM_EVENT_STATUS_PASSED);}}
}

static void DemMGR__EspWheelPulseStampedMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WHEEL_PULSE_STAMPED_INDEX].U16_EventId
      ,  &U8_UdsStatus
   );
   if(
         DemMGR__CheckSignalTimeout(
            DEMMGR_E_ESP_WHEEL_PULSE_STAMPED_INDEX
         )
      != FALSE
   ){
      Type_SwcApplTpms_stStatusBody* PS_EnvData = Env_GetEnvironmentData();
      if(
            PS_EnvData->uiVehSpeed
         >  DEMMGR__ESP_WHEEL_PULSE_STAMPED_MIN_SPEED
      ){
         if(
               FALSE
            == infSwcApplTpmsSwcServiceDem_bIsFinishedAutolocation()
         ){ if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WHEEL_PULSE_STAMPED_INDEX].U16_EventId, DEM_EVENT_STATUS_FAILED);}}
      }
   }
   else{
            if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WHEEL_PULSE_STAMPED_INDEX].U16_EventId, DEM_EVENT_STATUS_PASSED);}
   }
}

static void DemMGR__EspWspeedFrontRearBodyDirectionLCANMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   boolean               bTimeout = FALSE;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX1].U16_EventId
      ,  &U8_UdsStatus
   );
   if(DemMGR__CheckSignalTimeout(DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX1) == TRUE){bTimeout = TRUE; Env_SetWheelInfo(ENV_E_FRONT_LEFT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN); Env_SetWheelInfo(ENV_E_FRONT_RIGHT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);}
   if(DemMGR__CheckSignalTimeout(DEMMGR_E_ESP_WSPEED_REAR_BODY_LCAN_INDEX1)  == TRUE){bTimeout = TRUE; Env_SetWheelInfo(ENV_E_REAR_LEFT_WHEEL_INDEX,  CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN); Env_SetWheelInfo(ENV_E_REAR_RIGHT_WHEEL_INDEX,  CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);}
   if(
         bTimeout
      == TRUE
   ){    if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX1].U16_EventId, DEM_EVENT_STATUS_FAILED);}}
   else{ if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX1].U16_EventId, DEM_EVENT_STATUS_PASSED);}}
}

static void DemMGR__EspWspeedFrontRearBodySpeedLCANMonitor(void){
  Dem_UdsStatusByteType U8_UdsStatus;
  boolean bTimeout = FALSE;
  Dem_GetEventStatus(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX2].U16_EventId, &U8_UdsStatus);
  if(DemMGR__CheckSignalTimeout(DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX2) == TRUE){
   bTimeout = TRUE;
    Env_SetWheelInfo(ENV_E_FRONT_LEFT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);
    Env_SetWheelInfo(ENV_E_FRONT_RIGHT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);
  }
  if(DemMGR__CheckSignalTimeout(DEMMGR_E_ESP_WSPEED_REAR_BODY_LCAN_INDEX2) == TRUE){
   bTimeout = TRUE;
    Env_SetWheelInfo(ENV_E_REAR_LEFT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);
    Env_SetWheelInfo(ENV_E_REAR_RIGHT_WHEEL_INDEX, CANMGR__WSPEED_SPEED_INVALID, (Env_Enum_VehicleDirection) ENV_E_VEHICLE_DIRECTION_UNKNOWN);
  }
  if(bTimeout == TRUE){
   if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){
      DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX2].U16_EventId, DEM_EVENT_STATUS_FAILED);
   }
  }
  else{
   if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){
      DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX2].U16_EventId, DEM_EVENT_STATUS_PASSED);
   }
  }
}

static void DemMGR__TmmStatusBodyLCANMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_TMM_STATUS_BODY_LCAN_INDEX].U16_EventId
      ,  &U8_UdsStatus
   );
   if(
         DemMGR__CheckSignalTimeout(
            DEMMGR_E_TMM_STATUS_BODY_LCAN_INDEX
         )
      != FALSE
   ){
      Env_SetAmbientTemperature(
         ATMOSPHERIC_TEMPERATURE_SIGNAL_MISSING_VALUE
      );
      if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_TMM_STATUS_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_FAILED);}
   }
   else{
      if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_TMM_STATUS_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_PASSED);}
   }
}

static void DemMGR__VehStsBodyLCANMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_VEH_STS_BODY_LCAN_INDEX].U16_EventId
      ,  &U8_UdsStatus
   );
   if(DemMGR__CheckSignalTimeout(DEMMGR_E_VEH_STS_BODY_LCAN_INDEX) != FALSE){
      Env_SetIgnitionStateFromBus(ENV_E_IGNITION_UNKNOWN);
      if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_VEH_STS_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_FAILED);}
   }
   else{
      if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_VEH_STS_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_PASSED);}
   }
}

static void DemMGR__VmsStsBodyLCANMonitor(void){
   Dem_UdsStatusByteType U8_UdsStatus;
   Dem_GetEventStatus(
         DemMGR__CS_SignalMonitoringData[DEMMGR_E_VMS_STS_REQ_BODY_LCAN_INDEX].U16_EventId
      ,  &U8_UdsStatus
   );
   if(
         DemMGR__CheckSignalTimeout(
            DEMMGR_E_VMS_STS_REQ_BODY_LCAN_INDEX
         )
      != FALSE
   ){    if((U8_UdsStatus & DEM_UDS_STATUS_TF) != DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_VMS_STS_REQ_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_FAILED);}}
   else{ if((U8_UdsStatus & DEM_UDS_STATUS_TF) == DEM_UDS_STATUS_TF){DEMMGR_FifoPut(DemMGR__CS_SignalMonitoringData[DEMMGR_E_VMS_STS_REQ_BODY_LCAN_INDEX].U16_EventId, DEM_EVENT_STATUS_PASSED);}}
}

static void DEMMGR_FiFoGet(
      Dem_EventIdType*     ucError
   ,  Dem_EventStatusType* ucStatus
){
   uint8 i;
   *ucError  = sDtcFifo[0].ucEvent;
   *ucStatus = sDtcFifo[0].ucStatus;
   for(
      i = 0;
      i < (cDEMMGR_DTC_FIFO_SIZE - 1);
      i ++
   ){
      sDtcFifo[i].ucEvent  = sDtcFifo[i+1].ucEvent;
      sDtcFifo[i].ucStatus = sDtcFifo[i+1].ucStatus;
   }
   sDtcFifo[cDEMMGR_DTC_FIFO_SIZE - 1].ucEvent  = (Dem_EventIdType)cDEMMGR_DTC_FIFO_EMPTY;
   sDtcFifo[cDEMMGR_DTC_FIFO_SIZE - 1].ucStatus = (Dem_EventStatusType)cDEMMGR_DTC_FIFO_EMPTY;
}

static void DEMMGR_FifoPut(Dem_EventIdType ucDtc, Dem_EventStatusType ucStatus){
   uint8 i = 0;
   while(
         (
               sDtcFifo[i].ucEvent
            != cDEMMGR_DTC_FIFO_EMPTY
         )
      && (
               i
            <  cDEMMGR_DTC_FIFO_SIZE
         )
   ){
      if(
            sDtcFifo[i].ucEvent
         == ucDtc
      ){
         i = cDEMMGR_DTC_FIFO_SIZE;
      }
      else{
         i++;
      }
   }
   if(
         i
      <  cDEMMGR_DTC_FIFO_SIZE
   ){
      sDtcFifo[i].ucEvent  = ucDtc;
      sDtcFifo[i].ucStatus = ucStatus;
   }
}

void ClientIf_SetCurrentErrorERR(
   Type_SwcApplTpms_eErrors eErrorType
){
   uint8 U8_Counter;
   for(
      U8_Counter = 0;
      U8_Counter < SwcApplTpms_eError_MAX_NUMBER;
      U8_Counter ++
   ){
      if(
            DemMGR__CAS_DTCMappings[U8_Counter].E_TPMSError
         == eErrorType
      ){
         Dem_UdsStatusByteType U8_UdsStatus;
         Dem_GetEventStatus(
               DemMGR__CAS_DTCMappings[U8_Counter].U16_EventId
            ,  &U8_UdsStatus
         );
         if(
               (
                     U8_UdsStatus
                  &  DEM_UDS_STATUS_TF
               )
            != DEM_UDS_STATUS_TF
         ){
            DEMMGR_FifoPut(
                  DemMGR__CAS_DTCMappings[U8_Counter].U16_EventId
               ,  DEM_EVENT_STATUS_FAILED
            );
         }
      }
   }
}

void ClientIf_DeleteCurrentErrorERR(
   Type_SwcApplTpms_eErrors eErrorType
){
   uint8 U8_Counter;
   for(
      U8_Counter = 0;
      U8_Counter < SwcApplTpms_eError_MAX_NUMBER;
      U8_Counter ++
   ){
      if(
            DemMGR__CAS_DTCMappings[U8_Counter].E_TPMSError
         == eErrorType
      ){
         Dem_UdsStatusByteType U8_UdsStatus;
         Dem_GetEventStatus(
               DemMGR__CAS_DTCMappings[U8_Counter].U16_EventId
            ,  &U8_UdsStatus
         );
         if(
               (
                     U8_UdsStatus
                  &  DEM_UDS_STATUS_TF
               )
            == DEM_UDS_STATUS_TF
         ){
            DEMMGR_FifoPut(
                  DemMGR__CAS_DTCMappings[U8_Counter].U16_EventId
               ,  DEM_EVENT_STATUS_PASSED
            );
         }
      }
   }
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

