///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  OS dependent function hiding

  @file OsEci.h
*/

#ifndef __OSECI_H__
#define __OSECI_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <stdio.h>
#include <memory.h> // for memset (softer than including string.h)
#include <errno.h>  // for ENOMEM
#include <poll.h>   // for poll
#include <stddef.h> // for offsetof
#include <stdlib.h> // for malloc / free
#include <stdint.h> // for integers

#include <ECI_error.h> // for error definition

//////////////////////////////////////////////////////////////////////////
// constants and macros

#define OS_uSleep                   OSECI_uSleep
#define OS_Sleep                    OSECI_Sleep
#define OS_InterlockedExchange      OSECI_InterlockedExchange
#define OS_InterlockedExchangeAdd   OSECI_InterlockedExchangeAdd
#define OS_InterlockedOr            OSECI_InterlockedOr
#define OS_InterlockedIncrement     OSECI_InterlockedIncrement
#define OS_InterlockedDecrement     OSECI_InterlockedDecrement
#define OS_GetTimeInUs              OSECI_GetTimeInUs
#define OS_GetTimeInMs              OSECI_GetTimeInMs
#define OS_CreateEvent              OSECI_CreateEvent
#define OS_SetEvent                 OSECI_SetEvent
#define OS_WaitForSingleObject      OSECI_WaitForSingleObject
#define OS_CreateMutex              OSECI_CreateMutex
#define OS_LockMutex                OSECI_LockMutex
#define OS_UnlockMutex              OSECI_UnlockMutex
#define OS_CloseHandle              OSECI_CloseHandle
#define OS_GetLastError             OSECI_GetLastError

/** @def OS_uSleep
  @brief @copybrief OSECI_uSleep @n <b> See function:</b> @ref OSECI_uSleep */
/** @def OS_Sleep
  @brief @copybrief OSECI_Sleep @n <b> See function:</b> @ref OSECI_Sleep */
/** @def OS_InterlockedExchange
  @brief @copybrief OSECI_InterlockedExchange @n <b> See function:</b> @ref OSECI_InterlockedExchange */
/** @def OS_InterlockedExchangeAdd
  @brief @copybrief OSECI_InterlockedExchangeAdd @n <b> See function:</b> @ref OSECI_InterlockedExchangeAdd */
/** @def OS_InterlockedOr
  @brief @copybrief OSECI_InterlockedOr @n <b> See function:</b> @ref OSECI_InterlockedOr */
/** @def OS_InterlockedIncrement
  @brief @copybrief OSECI_InterlockedIncrement @n <b> See function:</b> @ref OSECI_InterlockedIncrement */
/** @def OS_InterlockedDecrement
  @brief @copybrief OSECI_InterlockedDecrement @n <b> See function:</b> @ref OSECI_InterlockedDecrement */
/** @def OS_GetTimeInUs
  @brief @copybrief OSECI_GetTimeInUs @n <b> See function:</b> @ref OSECI_GetTimeInUs */
/** @def OS_GetTimeInMs
  @brief @copybrief OSECI_GetTimeInMs @n <b> See function:</b> @ref OSECI_GetTimeInMs */
/** @def OS_CreateEvent
  @brief @copybrief OSECI_CreateEvent @n <b> See function:</b> @ref OSECI_CreateEvent */
/** @def OS_SetEvent
  @brief @copybrief OSECI_SetEvent @n <b> See function:</b> @ref OSECI_SetEvent */
/** @def OS_WaitForSingleObject
  @brief @copybrief OSECI_WaitForSingleObject @n <b> See function:</b> @ref OSECI_WaitForSingleObject */
/** @def OS_CreateMutex
  @brief @copybrief OSECI_CreateMutex @n <b> See function:</b> @ref OSECI_CreateMutex */
/** @def OS_LockMutex
  @brief @copybrief OSECI_LockMutex @n <b> See function:</b> @ref OSECI_LockMutex */
/** @def OS_UnlockMutex
  @brief @copybrief OSECI_UnlockMutex @n <b> See function:</b> @ref OSECI_UnlockMutex */
/** @def OS_CloseHandle
  @brief @copybrief OSECI_CloseHandle @n <b> See function:</b> @ref OSECI_CloseHandle */
/** @def OS_GetLastError
  @brief @copybrief OSECI_GetLastError @n <b> See function:</b> @ref OSECI_GetLastError */

/** DLL EXPORT definition */
#ifdef __DLL_EXPORT__
  #define ECI_DLLEXPORT __attribute__ ((visibility("default")))
#else
  #define ECI_DLLEXPORT
#endif

/** Calling convention */
#if defined(__i386__)
  #define ECI_APICALL __attribute__ ((stdcall))
#elif defined(__arm__)
  #define ECI_APICALL
#elif defined(__amd64__)
  #define ECI_APICALL
#elif defined(__aarch64__)
  #define ECI_APICALL
#endif

typedef void          VOID;         ///< Define used datatype to easier porting the ECI to another OS
typedef char          CHAR;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint8_t     BYTE;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint8_t     UINT8;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int32_t     BOOL;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint16_t    WORD;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint16_t    UINT16;       ///< Define used datatype to easier porting the ECI to another OS
typedef __uint32_t    DWORD;        ///< Define used datatype to easier porting the ECI to another OS
typedef __uint32_t    UINT32;       ///< Define used datatype to easier porting the ECI to another OS
typedef __uint64_t    QWORD;        ///< Define used datatype to easier porting the ECI to another OS
typedef long          LONG;         ///< Define used datatype to easier porting the ECI to another OS
typedef unsigned long ULONG;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int32_t     INT32;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int64_t     INT64;        ///< Define used datatype to easier porting the ECI to another OS
typedef __uint32_t    HRESULT;      ///< Define used datatype to easier porting the ECI to another OS
typedef float         FLOAT;        ///< Define used datatype to easier porting the ECI to another OS
typedef __uint64_t    UINT64;       ///< Define used datatype to easier porting the ECI to another OS

typedef VOID          *PVOID;       ///< Define used datatype to easier porting the ECI to another OS
typedef CHAR          *PCHAR;       ///< Define used datatype to easier porting the ECI to another OS
typedef BYTE          *PBYTE;       ///< Define used datatype to easier porting the ECI to another OS
typedef BOOL          *PBOOL;       ///< Define used datatype to easier porting the ECI to another OS
typedef WORD          *PWORD;       ///< Define used datatype to easier porting the ECI to another OS
typedef DWORD         *PDWORD;      ///< Define used datatype to easier porting the ECI to another OS
typedef LONG          *PLONG;       ///< Define used datatype to easier porting the ECI to another OS
typedef FLOAT         *PFLOAT;      ///< Define used datatype to easier porting the ECI to another OS
typedef void          *ECI_HANDLE;  ///< Define used datatype to easier porting the ECI to another OS
typedef ECI_HANDLE    *PECI_HANDLE; ///< Define used datatype to easier porting the ECI to another OS


#define LOBYTE(wVal)  ((BYTE) wVal)           ///< Macro for accessing low byte @ingroup OsEci
#define HIBYTE(wVal)  ((BYTE) ( wVal >> 8))   ///< Macro for accessing high byte @ingroup OsEci
#define LOWORD(dwVal) ((WORD) dwVal)          ///< Macro for accessing low word @ingroup OsEci
#define HIWORD(dwVal) ((WORD) ( dwVal >> 16)) ///< Macro for accessing high word @ingroup OsEci

#define OSECI_WAIT_FOREVER  ((DWORD)0xFFFFFFFF)  ///< Blocking function call @ingroup OsEci

// return values of OSECI_WaitForSingleObject
#define OSECI_WAIT_OBJECT_0 0UL                  ///< Wait succeeded @ingroup OsEci
#define OS_WAIT_OBJECT_0    OSECI_WAIT_OBJECT_0
/** @def OS_WAIT_OBJECT_0
  @brief @copybrief OSECI_WAIT_OBJECT_0 @n <b> See function:</b> @ref OSECI_WAIT_OBJECT_0 */
#define OSECI_WAIT_TIMEOUT  1UL                  ///< Wait timed out @ingroup OsEci
#define OS_WAIT_TIMEOUT     OSECI_WAIT_TIMEOUT
/** @def OS_WAIT_TIMEOUT
  @brief @copybrief OSECI_WAIT_TIMEOUT @n <b> See function:</b> @ref OSECI_WAIT_TIMEOUT */
#define OSECI_WAIT_FAILED   ((DWORD)0xFFFFFFFF)  ///< Wait failed @ingroup OsEci
#define OS_WAIT_FAILED      OSECI_WAIT_FAILED
/** @def OS_WAIT_FAILED
  @brief @copybrief OSECI_WAIT_FAILED @n <b> See function:</b> @ref OSECI_WAIT_FAILED */
  
// device id macros
#define OS_DOMAIN_NUM(devid)    OSECI_DOMAIN_NUM(devid)
/** @def OS_DOMAIN_NUM
  @brief @copybrief OSECI_DOMAIN_NUM @n <b> See function:</b> @ref OSECI_DOMAIN_NUM */
#define OS_BUS_NUM(devid)       OSECI_BUS_NUM(devid)
/** @def OS_BUS_NUM
  @brief @copybrief OSECI_BUS_NUM @n <b> See function:</b> @ref OSECI_BUS_NUM */
#define OS_SLOT_NUM(devid)      OSECI_SLOT_NUM(devid)
/** @def OS_SLOT_NUM
  @brief @copybrief OSECI_SLOT_NUM @n <b> See function:</b> @ref OSECI_SLOT_NUM */
#define OS_FUNC_NUM(devid)      OSECI_FUNC_NUM(devid)
/** @def OS_FUNC_NUM
  @brief @copybrief OSECI_FUNC_NUM @n <b> See function:</b> @ref OSECI_FUNC_NUM */

#define OSECI_DOMAIN_NUM(devid)    (devid  >> 16)          ///< Macro for accessing domain number
#define OSECI_BUS_NUM(devid)       (((devid) >> 8) & 0xff) ///< Macro for accessing bus number
#define OSECI_SLOT_NUM(devid)      (((devid) >> 3) & 0x1f) ///< Macro for accessing slot number
#define OSECI_FUNC_NUM(devid)      ((devid) & 0x07)        ///< Macro for accessing func number

// Definition of TRUE and FALSE
#ifndef FALSE
  #define FALSE 0 ///< FALSE @ingroup OsEci
#endif
#ifndef TRUE
  #define TRUE 1  ///< TRUE @ingroup OsEci
#endif

/** Macro to find max value */
// #ifndef max
//   #define max(a,b)   (((a) > (b)) ? (a) : (b))
// #endif

// /** Macro to find min value */
// #ifndef min
//   #define min(a,b)   (((a) < (b)) ? (a) : (b))
// #endif

/** Macro to get element count of an array  @ingroup OsEci */
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))

/** Macro to swap the bytes of a WORD (ab -> ba)  @ingroup OsEci */
#define SWAP16(w)  ( (WORD)  ((LOBYTE(w) << 8) | HIBYTE (w)))

/** Macro to swap the bytes of a DWORD (abcd -> dcba)  @ingroup OsEci */
#define SWAP32(dw) ( (DWORD) (((SWAP16 (LOWORD (dw))) << 16) | (SWAP16 (HIWORD (dw)))))

/** Macro for getchar redirection  @ingroup OsEci */
#define OSECI_Fgets fgets
#define OS_Fgets OSECI_Fgets
/** @def OS_Fgets
  @brief @copybrief OSECI_Fgets @n <b> See function:</b> @ref OSECI_Fgets */

/** Macro for printf redirection  @ingroup OsEci */
#define OSECI_Printf printf
#define OS_Printf    OSECI_Printf
/** @def OS_Printf
  @brief @copybrief OSECI_Printf @n <b> See function:</b> @ref OSECI_Printf */

/** Macro for fflush redirection  @ingroup OsEci */
#define OSECI_Fflush(stream) fflush(stream)
#define OS_Fflush(stream)    OSECI_Fflush(stream)
/** @def OS_Fflush
  @brief @copybrief OSECI_Fflush @n <b> See function:</b> @ref OSECI_Fflush */

///////////////////////////////////////////////////////////////////////////////
/**
  This macro retrieve a reference to the host object.

  @param t
    Type of the host object
  @param m
    Member variable which represents the inner object

  @return
   Reference to the host object.

  @note
    Set -Wno-invalid-offsetof flag in compiler to omit warning messages
    generated by this macro
*/
#ifndef HOSTOBJECT
  #define HOSTOBJECT(t,m) ((t&) *((char*) this - offsetof(t,m)))
#endif


//////////////////////////////////////////////////////////////////////////
// exported functions

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMicroseconds
    Number of microseconds [us] to suspend thread

  @ingroup OsEci
*/
ECI_DLLEXPORT void  ECI_APICALL OSECI_uSleep ( DWORD dwMicroseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMilliseconds
    Number of milliseconds [ms] to suspend thread

  @ingroup OsEci
*/
ECI_DLLEXPORT void ECI_APICALL OSECI_Sleep ( DWORD dwMilliseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Sets a LONG variable to the specified value as an atomic operation.

  @param plDest
    A pointer to the value to be exchanged. The function sets this variable
    to Value, and returns its prior value.
  @param lValue
    The value to be exchanged with the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OSECI_InterlockedExchange ( PLONG plDest,
                                                        LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic addition of two LONG values.

  @param plDest
    A pointer to the variable. The value of this variable will be replaced
    with the result of the operation
  @param lValue
    The value to be added to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OSECI_InterlockedExchangeAdd( PLONG plDest,
                                                          LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic OR operation on the specified LONG values. The function
  prevents more than one thread from using the same variable simultaneously.

  @param plDest
    A pointer to the first operand. This value will be ORed with lValue.
  @param lValue
    The value to be ORed to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OSECI_InterlockedOr( PLONG plDest,
                                                 LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Increments the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be incremented.

  @retval LONG
    The function returns the resulting incremented value.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OSECI_InterlockedIncrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Decrements the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be decremented.

  @retval LONG
    The function returns the resulting decremented value.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OSECI_InterlockedDecrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in microseconds [us].

  @retval DWORD
    Current time in microseconds [us].

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_GetTimeInUs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in milliseconds [ms].

  @retval DWORD
    Current time in milliseconds [ms].

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_GetTimeInMs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed event. The event is initially not signaled and auto
  reset.

  @retval ECI_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT ECI_HANDLE ECI_APICALL OSECI_CreateEvent ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Set the state of the specified event object to signaled

  @param hEvent
    Event handle to set

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_SetEvent ( ECI_HANDLE hEvent );


///////////////////////////////////////////////////////////////////////////////
/**
  Wait until either the specified object is in the signaled state, or until
  the time-out interval elapses.

  @param hEvent
    Handle to wait for get signaled
  @param dwTimeout
    Specifies the time-out interval, in milliseconds [ms]. If the interval
    elapses, the function returns, even if the object's state is non signaled.

  @retval DWORD
    @ref OSECI_WAIT_OBJECT_0
      Success. The specified object�s state is signaled. @n
    @ref OSECI_WAIT_TIMEOUT
      Failure. The time-out interval elapsed, and the object's state is
      non signaled. @n
    @ref OSECI_WAIT_FAILED
      Failure. Waiting failed maybe due to an invalid handle. To get extended
      error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_WaitForSingleObject ( ECI_HANDLE hEvent,
                                                         DWORD  dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed mutex which can be uses for mutual exclusion

  @retval ECI_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT ECI_HANDLE ECI_APICALL OSECI_CreateMutex ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Acquires a mutual lock

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSECI_GetLastError.

  @note
    This mutex is not re-entrant. Trying to acquire the mutex from the
    same thread a second time will lead to a dead-lock situation!

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_LockMutex ( ECI_HANDLE hMutex );


///////////////////////////////////////////////////////////////////////////////
/**
  Release a mutual lock.

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_UnlockMutex ( ECI_HANDLE hMutex );


///////////////////////////////////////////////////////////////////////////////
/**
  Closes a handle and deletes the object.

  @param hHandle
    Handle to close

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSECI_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_CloseHandle ( ECI_HANDLE hHandle );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current thread's last exception code.

  @retval DWORD
    The calling thread�s last-error code value.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OSECI_GetLastError ( void );


#ifdef __cplusplus
}
#endif // __cplusplus


#endif //__OSECI_H__
