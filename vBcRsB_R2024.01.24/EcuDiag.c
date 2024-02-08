#include "Std_Types.hpp"

#define EcuNoInternalError           ((uint8) 0x00u)
#define EcuUnknownError              ((uint8) 0x01u)
#define EcuTpmsCyclicFunctionDelayed ((uint8) 0x02u)
#define EcuRamTestFailed             ((uint8) 0x04u)
#define EcuRomTestFailed             ((uint8) 0x08u)
#define EcuRfReceiverDefect          ((uint8) 0x20u)

#define cDiagRunning        ((uint8) 0)
#define cDiagError          ((uint8) 1)
#define cDiagFinishNoError  ((uint8) 2)

#define RAM_START_ADDRESS  0xFEDF8000u
#define RAM_END_ADDRESS    0xFEE00000u
#define RAM_TESTENDE_ID    0xFEE55555u

#define START_FLASHADD4APPL     0x00020000u
#define END_FLASHADD4APPL       0x00077BFDu
#define CHECKSUMM_ADD_IN_APPL   0x00077BFEu
#define READ_BYTES4CHKInTask    0x40u

#define ushDefDemodSameLevelCtc 1200
#define ushDefDemodDifLevelGoodCtc 1200
#define ui16MaxRfRecInactiveTimeIn50msec 6000
#define ui16RecActiveTimeIn50msec 2400

static uint8  ECUD_DiagRFRec(void);
static uint8  ECUD_PerformRamCheck(void);
static uint8  ECUD_PerformRomCheck(void);
static uint32 ECUD_RAMCheckUi32(uint32 ulRamStartAddress);

#include "SysManagerX.hpp"
#include "EnvManagerX.hpp"
#include "DemManagerX.hpp"
#include "adcX.hpp"

static uint32 ulCheckRamAddress;
static uint32 ulRamStartAddress = RAM_START_ADDRESS;

static uint8*  piPDI                 = (uint8*)  START_FLASHADD4APPL;
static uint16* piP2ApplChecksum      = (uint16*) CHECKSUMM_ADD_IN_APPL;
static uint8   ucByteCnt4MaxInTask   = 0;
static uint8   ucApplCheckSumLowByte = 0;
static uint16  uiCheckSum            = 0;
static uint16  uiApplCheckSum        = 0;

#include "infMcalAtaRx.hpp"
static boolean ECUD_DiagRFRec(void){
   static uint8         ucLastDemodLevel      = (uint8) 0;
   static uint16        ushDemodSameLevelCt   = 0;
   static uint16        ui16RfRecInactiveTime = 0;
          unsigned char ucCurLevl             = Env_GetNoiseLevel();
          boolean       bErrorActive          = FALSE;
   if(ucLastDemodLevel == ucCurLevl){
      if(ushDefDemodSameLevelCtc <= ushDemodSameLevelCt){
         bErrorActive = TRUE;
      }
      else{
         ushDemodSameLevelCt++;
      }
   }
   else{
      if(ushDefDemodSameLevelCtc <= ushDemodSameLevelCt){
         if((ushDefDemodSameLevelCtc + ushDefDemodDifLevelGoodCtc) > ushDemodSameLevelCt){
            ushDemodSameLevelCt++;
            bErrorActive = TRUE;
            ushDemodSameLevelCt = 0;
         }
      }
      else{
         ushDemodSameLevelCt = 0;
      }
   }
   ucLastDemodLevel = ucCurLevl;
   if(FALSE != infMcalAtaRx_bIsStateProcess()){
      if(ui16MaxRfRecInactiveTimeIn50msec <= ui16RfRecInactiveTime){
         if((ui16MaxRfRecInactiveTimeIn50msec + ui16RecActiveTimeIn50msec) > ui16RfRecInactiveTime){
            ui16RfRecInactiveTime++;
            bErrorActive = TRUE;
         }
         else{
            ui16RfRecInactiveTime = 0;
         }
      }
      else{
         ui16RfRecInactiveTime = 0;
      }
   }
   else{
      if(ui16MaxRfRecInactiveTimeIn50msec <= ui16RfRecInactiveTime){
         bErrorActive = TRUE;
      }
      else{
         ui16RfRecInactiveTime++;
      }
   }
   return bErrorActive;
}

static uint8 ECUD_PerformRamCheck(void){
    uint8 ucRetVal;
    if(ulRamStartAddress != 0x00){
      if(ulRamStartAddress < RAM_END_ADDRESS){
         ulCheckRamAddress = ECUD_RAMCheckUi32(ulRamStartAddress);
         ulRamStartAddress = ulCheckRamAddress;
         ucRetVal = cDiagRunning;
      }
      else{
         ucRetVal = cDiagFinishNoError;
      }
   }
   else{
      ucRetVal = cDiagError;
   }
   return ucRetVal;
}

#pragma ghs ZO
static uint32 ECUD_RAMCheckUi32(uint32 ulRamAddress){
   uint32 ul32Tmp;
   ul32Tmp = ulRamAddress;
   asm(" di  ");
   asm(" pushsp r20-r23 ");
   asm(" mov r6, r20");
   asm(" ldl.w [r20],r21 ");
   asm(" mov 0x55AA55AA, r23 ");
   asm(" stc.w r23,[r20] ");
   asm(" ldl.w [r20],r22 ");
   asm(" cmp r22,r23 ");
   asm(" bne _RAM_ERROR ");
   asm(" mov 0xAA55AA55, r23 ");
   asm(" stc.w r23,[r20] ");
   asm(" ldl.w [r20],r22 ");
   asm(" cmp r22,r23 ");
   asm(" bne _RAM_ERROR ");
   asm(" br _RAM_CHK_OKAY ");
   asm(" _RAM_ERROR:  ");
   asm(" mov 0x0, r6 ");
   ul32Tmp = 0;
   asm(" br _RAM_CHK_ENDE ");
   asm(" _RAM_CHK_OKAY:  ");
   asm(" stc.w r21,[r20] ");
   asm(" mov 0x0, r20  ");
   asm(" mov 0x0, r21  ");
   asm(" mov 0x0, r22  ");
   asm(" mov 0x0, r23  ");
   asm(" add 4, r6 ");
   ul32Tmp = ul32Tmp + 4;
   asm(" _RAM_CHK_ENDE:  ");
   asm(" popsp r20-r23 ");
   asm(" ei  ");
   return(ul32Tmp);
}

static uint8 ECUD_PerformRomCheck(void){
   uint8 ucRetVal = cDiagRunning;
   for(ucByteCnt4MaxInTask = 0; ucByteCnt4MaxInTask < READ_BYTES4CHKInTask; ucByteCnt4MaxInTask++){
      if((uint8*)END_FLASHADD4APPL != piPDI){
         uiCheckSum += (uint16)*piPDI;
         piPDI++;
      }
      else{
         uiApplCheckSum        = (uint16)*piP2ApplChecksum;
         ucApplCheckSumLowByte = ((uint8)((uiApplCheckSum >> 8) & 0x00FF));
         uiApplCheckSum        = ((uint16)(uiApplCheckSum << 8));
         uiApplCheckSum        = uiApplCheckSum + ((uint16)ucApplCheckSumLowByte);
         ucByteCnt4MaxInTask   = READ_BYTES4CHKInTask;
         if(uiApplCheckSum != uiCheckSum){
            ucRetVal = cDiagError;
         }
         else{
            ucRetVal = cDiagFinishNoError;
         }
      }
   }
   return(ucRetVal);
}

