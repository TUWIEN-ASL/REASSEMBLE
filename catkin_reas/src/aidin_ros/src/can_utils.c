///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2013 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo common helper functions

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file EciDemoCommon.c
*/


//////////////////////////////////////////////////////////////////////////
// compiler directives

//////////////////////////////////////////////////////////////////////////
// include files
#include "aidin_ros/can_utils.h"

//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables

/** Look up table to convert the CAN FD DLC code into the real CAN data length */
static const BYTE ECI_CAN_FD_DLC_LUT[16] = { 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64 };

//////////////////////////////////////////////////////////////////////////
// global variables

//////////////////////////////////////////////////////////////////////////
// static function prototypes

//////////////////////////////////////////////////////////////////////////
// global functions

//////////////////////////////////////////////////////////////////////////
// static functions

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif

int max(int a, int b) { (((a) > (b)) ? (a) : (b));}
int min(int a, int b) { (((a) < (b)) ? (a) : (b));}

///////////////////////////////////////////////////////////////////////////////
/**
  Prints the given hardware information on the screen

  @param pstcHwInfo
    Pointer to ECI_HW_INFO to print

  @ingroup EciDemo
*/
void EciPrintHwInfo(const ECI_HW_INFO* pstcHwInfo)
{
  //*** Check Argument
  if(NULL == pstcHwInfo)
  {
    OS_Printf("ERROR: invalid pointer: pstcHwInfo\n");
    return;
  }

  OS_Printf( "HwInfoVer: \"%u\"\n",
    pstcHwInfo->dwVer);

  //*** Struct Version 0
  if(pstcHwInfo->dwVer == ECI_STRUCT_VERSION_V0 ||
     pstcHwInfo->dwVer == ECI_STRUCT_VERSION_V1)
  {
    DWORD dwCtrlIndex = 0;

    OS_Printf( "BoardType  : \"%s\"\n",
      pstcHwInfo->u.V1.szHwBoardType);
    OS_Printf( "HwSerial   : \"%s\"\n",
      pstcHwInfo->u.V1.szHwSerial);
    OS_Printf( "HwVersion  : \"%u.%u.%u.%u\"\n",
      pstcHwInfo->u.V1.abHwVersion[0],
      pstcHwInfo->u.V1.abHwVersion[1],
      pstcHwInfo->u.V1.abHwVersion[2],
      pstcHwInfo->u.V1.abHwVersion[3]);
    OS_Printf( "BmVersion  : \"%u.%u.%u.%u\"\n",
      pstcHwInfo->u.V1.abBmVersion[0],
      pstcHwInfo->u.V1.abBmVersion[1],
      pstcHwInfo->u.V1.abBmVersion[2],
      pstcHwInfo->u.V1.abBmVersion[3]);
    OS_Printf( "FwIdentif  : \"%s\"\n",
      pstcHwInfo->u.V1.szFwIdentification);
    OS_Printf( "FwVersion  : \"%u.%u.%u.%u\"\n",
      pstcHwInfo->u.V1.abFwVersion[0],
      pstcHwInfo->u.V1.abFwVersion[1],
      pstcHwInfo->u.V1.abFwVersion[2],
      pstcHwInfo->u.V1.abFwVersion[3]);
    OS_Printf( "EciApiVer  : \"%u.%u.%u.%u\"\n",
      pstcHwInfo->u.V1.adwApiVersion[0],
      pstcHwInfo->u.V1.adwApiVersion[1],
      pstcHwInfo->u.V1.adwApiVersion[2],
      pstcHwInfo->u.V1.adwApiVersion[3]);
    OS_Printf( "DevId      : \"0x%X\"\n",
      pstcHwInfo->u.V1.dwDevId);

    OS_Printf( "CtrlCount  : \"%u\"\n",
      pstcHwInfo->u.V1.dwCtrlCount);
    for(dwCtrlIndex=0; dwCtrlIndex < pstcHwInfo->u.V1.dwCtrlCount; dwCtrlIndex++)
    {
      OS_Printf( "CtrlIndex  : \"%u\"\n",
        dwCtrlIndex);
      OS_Printf( "CtrlClass  : \"0x%04x\"\n",
        pstcHwInfo->u.V1.sCtrlInfo[dwCtrlIndex].wCtrlClass);
      OS_Printf( "CtrlState  : \"0x%04x\"\n",
        pstcHwInfo->u.V1.sCtrlInfo[dwCtrlIndex].wCtrlState);
    }//end for
  }//endif
}


///////////////////////////////////////////////////////////////////////////////
/**
  Prints the given controller capabilities on the screen

  @param pstcCtrlCaps
    Pointer to ECI_CTRL_CAPABILITIES to print

  @ingroup EciDemo
*/
void EciPrintCtrlCapabilities(const ECI_CTRL_CAPABILITIES* pstcCtrlCaps)
{
  //*** Check Argument
  if(NULL == pstcCtrlCaps)
  {
    OS_Printf("ERROR: invalid pointer: pstcCtrlCaps\n");
    return;
  }

  //*** Switch on controller class
  switch(pstcCtrlCaps->wCtrlClass)
  {
    //*** CAN
    case ECI_CTRL_CAN:
    {
      OS_Printf( "CapsVer    : \"%u\"\n",
        pstcCtrlCaps->u.sCanCaps.dwVer);

      //*** Struct Version 0 or Struct Version 1
      if(pstcCtrlCaps->u.sCanCaps.dwVer <= ECI_STRUCT_VERSION_V1)
      {
        OS_Printf( "CanType    : \"0x%04x\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.wCanType);
        OS_Printf( "BusCoupl   : \"0x%04x\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.wBusCoupling);
        OS_Printf( "Feature    : \"0x%04x\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwFeatures);
        OS_Printf( "PriClkFreq : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwClockFreq);
        OS_Printf( "TscDiv     : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwTscDivisor);
        OS_Printf( "DtxDiv     : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwDtxDivisor);
        OS_Printf( "DtxMaxTic  : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwDtxMaxTicks);
        OS_Printf( "PrioQueue  : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V0.dwNoOfPrioQueues);
      }

      //*** Struct Version 1
      if(pstcCtrlCaps->u.sCanCaps.dwVer == ECI_STRUCT_VERSION_V1)
      {
        OS_Printf( "CANClkFreq : \"%u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V1.dwCanClkFreq);
        OS_Printf( "SdrRangeMin: \"Mode 0x%04X, BPS %4u, TS1 %2u, TS2 %2u, SJW %2u, TDO %3u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.dwMode,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.dwBPS,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.wTS1,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.wTS2,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.wSJW,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMin.wTDO);
        OS_Printf( "SdrRangeMax: \"Mode 0x%04X, BPS %4u, TS1 %2u, TS2 %2u, SJW %2u, TDO %3u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.dwMode,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.dwBPS,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.wTS1,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.wTS2,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.wSJW,
          pstcCtrlCaps->u.sCanCaps.u.V1.sSdrRangeMax.wTDO);
        OS_Printf( "FdrRangeMin: \"Mode 0x%04X, BPS %4u, TS1 %2u, TS2 %2u, SJW %2u, TDO %3u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.dwMode,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.dwBPS,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.wTS1,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.wTS2,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.wSJW,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMin.wTDO);
        OS_Printf( "FdrRangeMax: \"Mode 0x%04X, BPS %4u, TS1 %2u, TS2 %2u, SJW %2u, TDO %3u\"\n",
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.dwMode,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.dwBPS,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.wTS1,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.wTS2,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.wSJW,
          pstcCtrlCaps->u.sCanCaps.u.V1.sFdrRangeMax.wTDO);
      }
      break;
    }//end case ECI_CTRL_CAN


    //*** LIN
    case ECI_CTRL_LIN:
    {
      OS_Printf( "CapsVer    : \"%u\"\n",
        pstcCtrlCaps->u.sCanCaps.dwVer);

      //*** Struct Version 0
      if(pstcCtrlCaps->u.sLinCaps.dwVer == ECI_STRUCT_VERSION_V0)
      {
        OS_Printf( "LinType    : \"0x%04x\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.wLinType);
        OS_Printf( "BusCoupl   : \"0x%04x\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.wBusCoupling);
        OS_Printf( "Feature    : \"0x%04x\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwFeatures);
        OS_Printf( "ClockFreq  : \"%u\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwClockFreq);
        OS_Printf( "TscDiv     : \"%u\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwTscDivisor);
        OS_Printf( "DtxDiv     : \"%u\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwDtxDivisor);
        OS_Printf( "DtxMaxTic  : \"%u\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwDtxMaxTicks);
        OS_Printf( "PrioQueue  : \"%u\"\n",
          pstcCtrlCaps->u.sLinCaps.u.V0.dwNoOfPrioQueues);
      }
      break;
    }// end case ECI_CTRL_LIN


    //*** Unknown
    default:
      break;
  }//end switch
}


///////////////////////////////////////////////////////////////////////////////
/**
  Prints the given controller message on the screen

  @param pstcCtrlMsg
    Pointer to ECI_CTRL_MESSAGE to print

  @ingroup EciDemo
*/
void EciPrintCtrlMessage(const ECI_CTRL_MESSAGE* pstcCtrlMsg)
{
  //*** Check Argument
  if(NULL == pstcCtrlMsg)
  {
    OS_Printf("ERROR: invalid pointer: pstcCtrlMsg\n");
    return;
  }

  //*** Switch on controller class
  switch(pstcCtrlMsg->wCtrlClass)
  {
    //*** CAN
    case ECI_CTRL_CAN:
    {
      //*** Struct Version 0 and Struct Version 1
      if((ECI_STRUCT_VERSION_V0 == pstcCtrlMsg->u.sCanMessage.dwVer) ||
         (ECI_STRUCT_VERSION_V1 == pstcCtrlMsg->u.sCanMessage.dwVer) )
      {
        OS_Printf( "Time: %10u, Type: %2u, ID: %08X, Flags: %c%c%c%c%c%c ",
                   pstcCtrlMsg->u.sCanMessage.u.V1.dwTime,
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.type,
                   pstcCtrlMsg->u.sCanMessage.u.V1.dwMsgId,
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.ext ? 'E' : ' ',
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.rtr ? 'R' : ' ',
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.srr ? 'S' : ' ',
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.edl ? 'L' : ' ',
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.brs ? 'B' : ' ',
                   pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.ovr ? 'O' : ' ');

        //*** Print Data if not RTR
        if(!pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.rtr)
        {
          BYTE bRealLen = ECI_CAN_FD_DLC_LUT[pstcCtrlMsg->u.sCanMessage.u.V1.uMsgInfo.Bits.dlc];

          if(ECI_STRUCT_VERSION_V0 == pstcCtrlMsg->u.sCanMessage.dwVer)
            { bRealLen = min(bRealLen, _countof(pstcCtrlMsg->u.sCanMessage.u.V0.abData)); }
          else
          if(ECI_STRUCT_VERSION_V1 == pstcCtrlMsg->u.sCanMessage.dwVer)
            { bRealLen = min(bRealLen, _countof(pstcCtrlMsg->u.sCanMessage.u.V1.abData)); }

          //*** Data available
          if(bRealLen)
          {
            BYTE bIndex = 0;
            OS_Printf("Data: ");
            for(bIndex=0; bIndex < bRealLen; bIndex++)
              { OS_Printf("%02X ", pstcCtrlMsg->u.sCanMessage.u.V1.abData[bIndex]);}
          }
        }
        else
        {
          OS_Printf("DLC: %u", pstcCtrlMsg->u.sCanMessage.u.V0.uMsgInfo.Bits.dlc);
        }
      }//endif Struct Version 0 and Struct Version 1
      break;
    }//end case ECI_CTRL_CAN


    //*** LIN
    case ECI_CTRL_LIN:
    {
      //*** Struct Version 0
      if(ECI_STRUCT_VERSION_V0 == pstcCtrlMsg->u.sLinMessage.dwVer)
      {
        OS_Printf( "Time: %10u, Type: %2u, ID: %08X, Flags: %c%c%c%c ",
                   pstcCtrlMsg->u.sLinMessage.u.V0.dwTime,
                   pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.type,
                   pstcCtrlMsg->u.sLinMessage.u.V0.dwMsgId,
                   pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.ecs ? 'E' : ' ',
                   pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.ido ? 'I' : ' ',
                   pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.sor ? 'S' : ' ',
                   pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.ovr ? 'O' : ' ');

        //*** Print Data if not ID only
        if(!pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.ido)
        {
          //*** Data available
          if(pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.dlc)
          {
            BYTE bIndex = 0;
            OS_Printf("Data: ");
            for(bIndex=0; bIndex < pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.dlc; bIndex++)
              { OS_Printf("%02X ", pstcCtrlMsg->u.sLinMessage.u.V0.abData[bIndex]);}
          }
        }
        else
        {
          OS_Printf("DLC: %u", pstcCtrlMsg->u.sLinMessage.u.V0.uMsgInfo.Bits.dlc);
        }
      }//endif Struct Version 0
      break;
    }//end case ECI_CTRL_LIN


    //*** Unknown
    default:
      break;
  }//end switch
  OS_Printf("\n");
  OS_Fflush(stdout);
}


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the nth controller of given controller class.

  @param pstcHwInfo
    Reference to ECI_HW_INFO to find controller from.

  @param eCtrlClass
    Class of controller to find.

  @param dwRelCtrlIndex
    Relative controller index of given controller class.

  @param pdwCtrIndex
    Variable which receives the controller index.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciDemo
*/
ECI_RESULT EciGetNthCtrlOfClass( const ECI_HW_INFO* pstcHwInfo,
                                 e_CTRLCLASS        eCtrlClass,
                                 DWORD              dwRelCtrlIndex,
                                 DWORD*             pdwCtrIndex)
{
  ECI_RESULT hResult = ECI_ERR_RESOURCE_NOT_FOUND;

  //*** Check Arguments
  if((NULL != pstcHwInfo) && (NULL != pdwCtrIndex))
  {
    //*** Struct Version 0
    if(pstcHwInfo->dwVer == ECI_STRUCT_VERSION_V0 ||
       pstcHwInfo->dwVer == ECI_STRUCT_VERSION_V1)
    {
      DWORD dwIndex = 0;
      //*** Iterate through all controllers
      for(dwIndex=0; dwIndex < pstcHwInfo->u.V1.dwCtrlCount; dwIndex++)
      {
        if(pstcHwInfo->u.V1.sCtrlInfo[dwIndex].wCtrlClass == eCtrlClass)
        {
          //*** Controller not found yet
          if(hResult != ECI_OK)
          {
            if(dwRelCtrlIndex == 0)
            {
              //*** Controller found
              *pdwCtrIndex = dwIndex;
              hResult    = ECI_OK;
            }
            else
            {
              dwRelCtrlIndex--;
            }
          }
        }//endif
      }//end for
    }//endif
  }
  else
  {
    hResult = ECI_ERR_INVALID_POINTER;
  }

  return hResult;
}

#ifdef __cplusplus
}
#endif
