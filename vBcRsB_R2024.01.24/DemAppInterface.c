#include "Std_Types.hpp"

#include <stdio.h>
#include <time.h>

#include "DemAppInterfaceX.hpp"
#include "EnvManagerX.hpp"
#include "DemManagerX.hpp"
#include "RdcManagerX.hpp"
#include "SysManagerX.hpp"
#include "AdcX.hpp"

static uint8 ucCurTirePress[4];
static uint8 ucCurTireTemp[4];
static uint8 ucCurLearnLocateStatus;

void Dem_SetCurrentTirePressures(uint8 ucFL, uint8 ucFR, uint8 ucRL, uint8 ucRR){
   ucCurTirePress[0] = ucFL;
   ucCurTirePress[1] = ucFR;
   ucCurTirePress[2] = ucRL;
   ucCurTirePress[3] = ucRR;
}

void Dem_SetCurrentTireTemperatures(uint8 ucFL, uint8 ucFR, uint8 ucRL, uint8 ucRR){
   ucCurTireTemp[0] = ucFL;
   ucCurTireTemp[1] = ucFR;
   ucCurTireTemp[2] = ucRL;
   ucCurTireTemp[3] = ucRR;
}

void Dem_SetCurrentLearnLocateStatus(uint8 ucLLStatus){ucCurLearnLocateStatus = ucLLStatus;}
Dem_ReadExternalDataElementFnc RTE_EngineSpeed_Data(uint8* ucBuffer){return E_OK;}

Dem_ReadExternalDataElementFnc Dem_GetData_VehicleSpeed(uint8* ucBuffer){
   tsEnv_Data* envData = Env_GetEnvironmentData();
   ucBuffer[1] = (envData->uiVehSpeed & 0xff);
   ucBuffer[0] = (envData->uiVehSpeed >> 8) & 0xff;
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_HVSoc(uint8* ucBuffer){
   ucBuffer[0] = 0xff;
   ucBuffer[1] = 0xff;
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_LVBatteryVoltage(uint8* ucBuffer){
   ucBuffer[0] = ADC_GetKl30Voltage();
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_VehicleMode(uint8* ucBuffer){
   ucBuffer[0] = Env_GetVehicleMode();
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_AmbientTemperature(uint8* ucBuffer){
   tsEnv_Data* tEnvData = Env_GetEnvironmentData();
   ucBuffer[0] = tEnvData->ucAmbTemperature;
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_OccDate(uint8* ucBuffer){
   uint64 ullCurrentTimeDate;
   struct tm sTimeDate;
   Rdc_StoreLatestCGMTimestamp(&ullCurrentTimeDate);
   ullCurrentTimeDate /= 1000;
   sTimeDate = *localtime((const time_t*)&ullCurrentTimeDate);
   sprintf((char*)ucBuffer, "%02d.%02d.%02d", sTimeDate.tm_mday, sTimeDate.tm_mon + 1, sTimeDate.tm_year - 100);
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_OccTime(uint8* ucBuffer){
   uint64 ullCurrentTimeDate;
   struct tm sTimeDate;
   Rdc_StoreLatestCGMTimestamp(&ullCurrentTimeDate);
   ullCurrentTimeDate /= 1000;
   sTimeDate = *localtime((const time_t*)&ullCurrentTimeDate);
   sprintf((char*)ucBuffer, "%02d:%02d:%02d", sTimeDate.tm_hour, sTimeDate.tm_min, sTimeDate.tm_sec);
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_TirePressures(uint8* ucBuffer){
   DEM_MEMCPY(ucBuffer, (const uint8*)ucCurTirePress, 4);
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_TireTemperatures(uint8* ucBuffer){
   DEM_MEMCPY(ucBuffer, (const uint8*)ucCurTireTemp, 4);
   return E_OK;
}

Dem_ReadExternalDataElementFnc Dem_GetData_EcuMode_RoadMode_LLStatus(uint8* ucBuffer){
   uint8 ucValues;
   tsEnv_Data* envData;
   boolean bCoastMode;
   envData = Env_GetEnvironmentData();
   bCoastMode = bGetCoastModeActive();
   ucValues = (SYSMGR_GetEcuMode() & 0x07) << 5;
   ucValues |= (envData->bRoadMode & 0x01) << 4;
   ucValues |= (envData->ucIgnition & 0x01) << 3;
   ucValues |= (bCoastMode << 2);
   ucValues |= (ucCurLearnLocateStatus & 0x03);
   *ucBuffer = ucValues;
   return E_OK;
}

Std_ReturnType DemEvent_BatteryVoltageLow_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_BatteryVoltageHigh_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}

Std_ReturnType DemEvent_AutolocationFail_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_AUTOLOCATION_FAILED); break;
      default:                                                                                break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanAbsWheelPulseSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: DemMGR_ClearMonitor(DEMMGR_E_ESP_WHEEL_PULSE_STAMPED_INDEX); break;
      default:                                                                                  break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanVehicleModeSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: DemMGR_ClearMonitor(DEMMGR_E_VEH_STS_BODY_LCAN_INDEX); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanAtmosphericTemperatureSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: DemMGR_ClearMonitor(DEMMGR_E_TMM_STATUS_BODY_LCAN_INDEX); break;
      default:                                                                               break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanBusoffError_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_CommunicationError_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}

Std_ReturnType DemEvent_DetectedTooManySensors_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_DETECTED_TOO_MANY_SENSORS); break;
      default:                                                                                      break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanWheelDirectionSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR:
         DemMGR_ClearMonitor(DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX1);
         DemMGR_ClearMonitor(DEMMGR_E_ESP_WSPEED_REAR_BODY_LCAN_INDEX1);
         break;

      default:
         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanClockSyncSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: DemMGR_ClearMonitor(DEMMGR_E_BCM_PERIPHERAL_MASTER_CLOCK_INDEX); break;
      default:                                                                                      break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanSpeedSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR:
         DemMGR_ClearMonitor(DEMMGR_E_ESP_WSPEED_FRONT_BODY_LCAN_INDEX2);
         DemMGR_ClearMonitor(DEMMGR_E_ESP_WSPEED_REAR_BODY_LCAN_INDEX2);
         break;

      default:
         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_CanOffroadSignalMissing_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: DemMGR_ClearMonitor(DEMMGR_E_VMS_STS_REQ_BODY_LCAN_INDEX); break;
      default:                                                                                break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_EcuRfInterference_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_RF_INTERFERENCE); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_NoQualifiedSensorSignals_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_NO_QUALIFIED_SENSOR_SIGNALS); break;
      default:                                                                                        break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensMissWhileInvalidSpeedSignal_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_DURING_CONTINUOUS_INVALID_SPEED); break;
      default:                                                                                                       break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorDefectFL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_DEFECT_FL); break;
      default:                                                                         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorDefectFR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_DEFECT_FR); break;
      default:                                                                         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorDefectRL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_DEFECT_RL); break;
      default:                                                                         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorDefectRR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_DEFECT_RR); break;
      default:                                                                         break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorDefectUnknownPosition_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_DEFECT_UNKNOWN_POS); break;
      default:                                                                                  break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorLifeTimeCounterLowFL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_LIFE_TIME_FL_CNT_LOW); break;
      default:                                                                                    break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorLifeTimeCounterLowFR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_LIFE_TIME_FR_CNT_LOW); break;
      default:                                                                                    break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorLifeTimeCounterLowRL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_LIFE_TIME_RL_CNT_LOW); break;
      default:                                                                                    break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorLifeTimeCounterLowRR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_LIFE_TIME_RR_CNT_LOW); break;
      default:                                                                                    break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorMissingFL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_FL); break;
      default:                                                                          break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorMissingFR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_FR); break;
      default:                                                                          break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorMissingRL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_RL); break;
      default:                                                                          break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorMissingRR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_RR); break;
      default:                                                                          break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorMissingUnknownPosition_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_MISSING_UNKNOWN_POS); break;
      default:                                                                                   break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorOvertempFL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_OVER_TEMP_FL); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorOvertempFR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_OVER_TEMP_FR); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorOvertempRL_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_OVER_TEMP_RL); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_SensorOvertempRR_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_WS_OVER_TEMP_RR); break;
      default:                                                                            break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_TpmsNotFunctional_InitMonForEvent(Dem_InitMonitorReasonType InitMonitorReason){
   switch(InitMonitorReason){
      case DEM_INIT_MONITOR_CLEAR: HufIf_GetStatusErrorERR(E_TPMS_ERROR_NOT_FUNCTIONING_DUE_TO_ECU_INTERNAL_FAULT); break;
      default:                                                                                                      break;
   }
   return E_OK;
}

Std_ReturnType DemEvent_FailedMemory_InitMonForEvent          (Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_NoEcuSerialNumber_InitMonForEvent     (Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_NoFingerprint_InitMonForEvent         (Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_NoVin_InitMonForEvent                 (Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_xxx1_InitMonForEvent                  (Dem_InitMonitorReasonType InitMonitorReason){return E_OK;}
Std_ReturnType DemEvent_LowBatteryVoltageLow_EventDataChanged (Dem_EventIdType EventId){return E_OK;}
Std_ReturnType Dem_CallbackDtcStatusChanged                   (uint32 DTC, Dem_UdsStatusByteType DTCStatusOld, Dem_UdsStatusByteType DTCStatusNew){return E_OK;}
