// ===========================================================================
/*!
 *   This file is part of the ATRACSYS fusiontrack library.
 *   Copyright (C) 2018-2018 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkEvent.h
 *  \brief    Definition of the class handling fTk events.
 *
 */
// ===========================================================================
#ifndef FTKEVENT_H
#define FTKEVENT_H

#ifdef GLOBAL_DOXYGEN
/**
 * \addtogroup sdk
 * \{
 */
#else
/** \defgroup eventSDK Event Functions
 * \brief Function to manage events
 * \{
 */
#endif

#include <ftkErrors.h>
#include <ftkPlatform.h>

/** \cond PEDANTIC
 */
#ifdef __cplusplus
struct ftkLibraryImp;
#else
typedef struct ftkLibraryImp ftkLibraryImp;
#endif
typedef ftkLibraryImp* ftkLibrary;
/** \endcond
 */


/** \brief Definitions of the event types.
 *
 * The event type indicates both the type \e and the version. For instance,
 * sending the 8 temperatures is type 0, in the future, if only 6 of them are
 * sent, this could be \c detTempV4 \c = \c 42.
 */
TYPED_ENUM( uint32, FtkEventType )
{
    /** \brief Temperature sending, version 1.
     *
     * \deprecated This event is not anymore forwarded to the user.
     * \see EvtTemperatureV1Payload.
     */
    fetTempV1 = 0,
    /** \brief Event indicating the current device temperature is too low.
     */
    fetLowTemp = 1,
    /** \brief Event indicating the current device temperature is too high.
     */
    fetHighTemp = 2,
    /** \brief Event indicating the device has received at least one shock
     * above the threshold, leading to a possible decalibration.
     */
    fetShockDetected = 3,
    /** \brief Event indicating the watchdog timer deactivated the data
     * sending.
     *
     * \deprecated This event is not anymore forwarded to the user.
     */
    fetWatchdogTimeout = 4,
    /** \brief Temperature sending, version 2.
     *
     * \deprecated This event is not anymore forwarded to the user.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * and then two byte for the fan `frequency':
     *   -# fan 0;
     *   -# fan 1.
     */
    fetTempV2 = 5,
    /** \brief Temperature sending, version 3.
     *
     * \deprecated This event is not anymore forwarded to the user.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * followed by:
     *   -# one byte indicating the status of the fan readings;
     *   -# 1 byte for the fan 0 input in percent;
     *   -# 2 bytes for the fan 0 speed;
     *   -# 1 byte for the fan 1 input in percent;
     *   -# 2 bytes for the fan 1 speed;
     *   -# 1 reserved byte (unused).
     */
    fetTempV3 = 6,
    /** \brief Event related to presence and state of wireless markers.
     *
     * \deprecated This event is not anymore forwarded to the user.
     *
     * The payload contains 16 mandatory bits, indicating which marker was
     * detected (one bit per marker). For each \e present marker, 16 additional
     * bits are present, currently not used.
     */
    fetWirelessMarkerV1 = 7,
    /** \brief Event related to availability of the calibration for a wireless
     * marker.
     *
     * The payload contains a ftkGeometry instance.
     *
     * \deprecated This event is not anymore forwarded to the user.
     *
     * \see ftkGeometry
     */
    fetWirelessMarkerCalibV1 = 8,
    /** \brief Accelerometer data
     */
    fetAccelerometerV1 = 10,
    /** \brief Temperatures values with sensor index.
     */
    fetTempV4 = 11,
    /** \brief Fans data.
     */
    fetFansV1 = 12,
    /** \brief Active markers mask.
     */
    fetActiveMarkersMaskV1 = 13,
    /** \brief Active button state.
     */
    fetActiveMarkersButtonStatusV1 = 14,
    /** \brief Active battery state.
     */
    fetActiveMarkersBatteryStateV1 = 15,
    /** \brief Synthetic temperatures data.
     */
    fetSyntheticTemperaturesV1 = 16,
    /** \brief Temperature compensation debug (internal use).
     */
    fetTemperatureCompensationDbg = 255,
    fetLastEvent = 0xFFFFFFFFu
};

/** \brief Structure holding an event as sent by the tracker.
 *
 * This structure implements the device-oriented data structure of an event.
 *
 * Events are generated and sent by the tracker itself. Some are handled
 * directly by the driver itself, others are forwarded to the user. In order to
 * get those event in a custom made program, the ftkReadEvent() function is
 * used.
 *
 * \cond STK
 * \warning When using a spryTrack, events are only available as part of a
 *          ftkFrameQuery object retrieved through ftkGetLastFrame(). The
 *          ftkReadEvent() function will return nothing.
 * \endcond
 */
PACK1_STRUCT_BEGIN( ftkEvent )
{
    /** \brief Type of the event.
     */
    FtkEventType Type;
    /** \brief Timestamp of the event
     *
     * This is created by the fusionTrack, it is the \f$ \mu{}s \f$ counter
     * value at the generation of the event.
     */
    uint64 Timestamp;
    /** \brief Serial number of the sending device.
     */
    uint64 SerialNumber;
    /** \brief Size of the data contained in the payload.
     */
    uint32 Payload;
    /** \brief Pointer on the payload data.
     */
    uint8* Data;
}
PACK1_STRUCT_END( ftkEvent );

/** \brief Structure holding the fan status.
*
* This structure holds the various status bits related to fan control.
*/
PACK1_STRUCT_BEGIN( ftkFanStatus )
{
    /** \brief Contains 1 if the module is enabled.
    */
    uint8 FanModuleEnabled : 1;
    /** \brief Contains 1 if the fan 0 is on.
    */
    uint8 Fan0PWMEnabled : 1;
    /** \brief Contains 1 if the fan 1 is on.
    */
    uint8 Fan1PWMEnabled : 1;
    /** \brief Contains 1 if the speed reading for fan 0 is valid.
    */
    uint8 Fan0SpeedValid : 1;
    /** \brief Contains 1 if the speed reading for fan 1 is valid.
    */
    uint8 Fan1SpeedValid : 1;
#ifdef __cplusplus
    /** \brief Conversion operator.
     *
     * This operator converts a ftkFanStatus to a uint8.
     *
     * \retval the content of the instance, as a uint8.
     */
    operator uint8() const
    {
        union { uint8 nbr; ftkFanStatus status; };
        status = *this;
        return nbr;
    }
private:
    /** \brief Reserved unused bits.
    */
    uint8 Reserved : 3;
#endif
}
PACK1_STRUCT_END( ftkFanStatus );

#if !defined(__cplusplus) || (defined(ATR_MSVC) && _MSC_VER < 1900)
    #define FTK_MEASURE_ACCELEROMETER 3u
    #define FTK_NUM_FANS_PER_EVENT 2u
    #define FTK_MAX_NUM_TEMP_PER_EVENT 20u
    #define FTK_MAX_ACC_PER_EVENT 2u
    #define FTK_MAX_ACTIVE_MARKERS 16u
    #define MINIMUM_EVENTS_NBR 5u
#else
/** \brief Number of components in an accelerometer measurement (3, i.e. \f$
 * x, y, z\f$.
 */
constexpr uint32 FTK_MEASURE_ACCELEROMETER = 3u;
/** \brief Number of fans in the fusionTrack.
 */
constexpr uint32 FTK_NUM_FANS_PER_EVENT = 2u;
/** \brief Maximum number of temperature measurements in an event.
 */
constexpr uint32 FTK_MAX_NUM_TEMP_PER_EVENT = 20u;
/** \brief Maximum number of accelerometer measurements in an event.
*/
constexpr uint32 FTK_MAX_ACC_PER_EVENT = 2u;
/** \brief Maximum number of paired active markers.
 */
constexpr uint32 FTK_MAX_ACTIVE_MARKERS = 16u;
/** \brief Minimum number of events allocated when ftkCreateFrame is called.
 */
constexpr uint32 MINIMUM_EVENTS_NBR = 5u;
#endif

/** \brief Structure holding one accelerometer measurement.
 */
PACK1_STRUCT_BEGIN( EvtAccelerometerV1Payload )
{
    /** \brief Value of the measured acceleration.
     *
     * This member stores the x, < and z component of the acceleration measured
     * by \e one accelerometer, in SI units, i.e. \f$m\cdots^{-2}\f$.
     */
    float32 Measure[ FTK_MEASURE_ACCELEROMETER ];
}
PACK1_STRUCT_END( EvtAccelerometerV1Payload );

/** \brief Structure discribing the payload for a FtkEventType::fetTempV1 event.
 *
 * \deprecated This event is not anymore forwarded to the user.
 */
PACK1_STRUCT_BEGIN_DEPRECATED( EvtTemperatureV1Payload )
{
    /** \brief Temperature values.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board.
     */
    float32 Temperatures[ 8u ];
}
PACK1_STRUCT_END_DEPRECATED( EvtTemperatureV1Payload );

/** \brief Structure holding the parameter of a fan.
 */
PACK1_STRUCT_BEGIN( ftkFanState )
{
    /** \brief Percentage of the voltage applied to the fan.
     */
    uint8 PwmDuty;
    /** \brief Fan rotation speed, in rpm.
     */
    uint16 Speed;
}
PACK1_STRUCT_END( ftkFanState );

/** \brief Structure holding the payload for events of type
 * FtkEventType::fetFansV1.
 */
PACK1_STRUCT_BEGIN( EvtFansV1Payload )
{
    /** \brief Status of the fans.
     */
    ftkFanStatus FansStatus;
    /** \brief Fan parameters.
     */
    ftkFanState Fans[ FTK_NUM_FANS_PER_EVENT ];
}
PACK1_STRUCT_END( EvtFansV1Payload );

/** \brief Structure holding \e one temperature measurement.
 */
PACK1_STRUCT_BEGIN( EvtTemperatureV4Payload )
{
    /** \brief ID of the sensor.
     */
    uint32 SensorId;
    /** \brief Temperature value in degree Celsius.
     */
    float32 SensorValue;
}
PACK1_STRUCT_END( EvtTemperatureV4Payload );

/** \brief Structure holding the payload for events of type
 * FtkEventType::fetActiveMarkersMaskV1.
 */
PACK1_STRUCT_BEGIN( EvtActiveMarkersMaskV1Payload )
{
    /** \brief Active marker mask.
     *
     * Bit \f$ i \f$ is set to \c 1 if short id \f$ i \f$ is currently paired.
     */
    uint16 ActiveMarkersMask;
}
PACK1_STRUCT_END( EvtActiveMarkersMaskV1Payload );

#ifdef ATR_MSVC
__declspec( deprecated )
#endif
/** \brief Typedef for backward compatibility.
 *
 * \deprecated This typedef will be removed in further releases.
 */
typedef EvtActiveMarkersMaskV1Payload EventActiveMarkersMaskV1Payload
#if defined( ATR_GCC ) || defined ( ATR_CLANG )
__attribute__( ( deprecated ) )
#endif
;

/** \brief Structure discribing the payload for a FtkEventType::fetTempV2 event.
 *
 * \deprecated This event is not anymore forwarded to the user.
 */
PACK1_STRUCT_BEGIN_DEPRECATED( EvtTemperatureV2Payload )
{
    /** \brief Temperature values.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * followed by the fan order (in percent):
     *   -# fan 0;
     *   -# fan 1.
     */
    float32 Temperatures[ 8u ];
    /** \brief Percentage of the voltage applied to the fan.
     */
    uint8 FanOrder[ 2u ];
}
PACK1_STRUCT_END_DEPRECATED( EvtTemperatureV2Payload );

/** \brief Structure discribing the payload for a FtkEventType::fetTempV3 event.
 *
 * \deprecated This event is not anymore forwarded to the user.
 */
PACK1_STRUCT_BEGIN_DEPRECATED( EvtTemperatureV3Payload )
{
    /** \brief Temperature values.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * followed by:
     *   -# fan module status (indicates whether the fans are on and if the
     * speed could be read successfully);
     *   -# fan 0 order (in percent);
     *   -# fan 0 speed (in rmp);
     *   -# fan 0 order (in percent);
     *   -# fan 0 speed (in rmp);
     *   -# a reserved field (not used).
     */
    float32 Temperatures[ 8u ];
    /** \brief Status of the fans.
     */
    ftkFanStatus FansStatus;
    /** \brief Percentage of the voltage applied to fan 0.
     */
    uint8 Fan0Order;
    /** \brief Fan 0 rotation speed, in rpm.
     */
    uint16 Fan0Speed;
    /** \brief Percentage of the voltage applied to fan 1.
     */
    uint8 Fan1Order;
    /** \brief Fan 1 rotation speed, in rpm.
     */
    uint16 Fan1Speed;
    /** \brief Unused data.
     */
    uint8 Reserved;
}
PACK1_STRUCT_END_DEPRECATED( EvtTemperatureV3Payload );

/**
 * \brief Structure describing the payload for a FtkEventType::fetActiveMarkersButtonStatusV1
 * event.
 */
PACK1_STRUCT_BEGIN( EvtActiveMarkersButtonStatusesV1Payload )
{
    uint32 ImageCount; ///< Imagecounter at the time the button state was retrieved.
    uint8  DeviceID; ///<  The Active Marker short ID
    uint8  ButtonStatus; ///<  The state of the button (mask)
}
PACK1_STRUCT_END( EvtActiveMarkersButtonStatusesV1Payload );

#ifdef ATR_MSVC
__declspec( deprecated )
#endif
/** \brief Typedef for backward compatibility.
*
* \deprecated This typedef will be removed in further releases.
*/
typedef EvtActiveMarkersButtonStatusesV1Payload EventActiveMarkersButtonStatusesV1Payload
#if defined( ATR_GCC ) || defined ( ATR_CLANG )
__attribute__( ( deprecated ) )
#endif
;

/**
 * \brief Structure describing the payload for a FtkEventType::fetActiveMarkersBatteryStateV1
 * event.
 */
PACK1_STRUCT_BEGIN( EvtActiveMarkersBatteryStateV1Payload )
{
    uint32 ImageCount; ///< Imagecounter at the time the battery state was retrieved.
    uint8 DeviceID; ///< The Active Marker short ID
    /** \brief The state of the battery.
     *
     *  This number can be converted to Volts using the following formula:
     *
     *  18.5 * 10^-3 * BatteryState.
     *
     *  For all Atracsys Active Markers, the maximum is 3.6 [V].
     *
     *  A Marker will stop functioning when this value reaches 2.2 [V].
     */
    uint8 BatteryState;
}
PACK1_STRUCT_END( EvtActiveMarkersBatteryStateV1Payload );

/** \brief Structore describing the payload for a
 * FtkEventType::fetSyntheticTemperaturesV1 event.
 */
PACK1_STRUCT_BEGIN( EvtSyntheticTemperaturesV1Payload )
{
    /** \brief Current value of the synthetic temperature.
     */
    float CurrentValue;
    /** \brief Value of the synthetic temperature during geometrical
     * calibration (in a 20°C environment).
     */
    float ReferenceValue;
}
PACK1_STRUCT_END( EvtSyntheticTemperaturesV1Payload );

#ifdef ATR_MSVC
__declspec( deprecated )
#endif
/** \brief Typedef for backward compatibility.
*
* \deprecated This typedef will be removed in further releases.
*/
typedef EvtActiveMarkersBatteryStateV1Payload EventActiveMarkersBatteryStateV1Payload
#if defined( ATR_GCC ) || defined ( ATR_CLANG )
__attribute__( ( deprecated ) )
#endif
;

/** \brief Function creating a ftkEvent instance.
 *
 * This function allows to create an instance of an Event. If needed, the
 * Event::Data pointer is allocated, but has to be manually set.
 *
 * \param[in] type type of the received (or sent) event.
 * \param[in] timestamp fTk timestamp corresponding to the creation of the
 * event.
 * \param[in] serial serial number of the sending device.
 * \param[in] payload size of the additional data.
 *
 * \return a pointer on the allocated instance, or \c 0 if an error occurred.
 *
 * \critical This function is involved in device event management.
 */
ATR_EXPORT ftkEvent* ftkCreateEvent( FtkEventType type, uint64 timestamp,
                                     uint64 serial, uint32 payload );

/** \brief Function deleting a ftkEvent instance.
 *
 * This function allows to free the allocated memory for an Event.
 *
 * \param[in] evt instance to delete.
 *
 * \retval ftkError::FTK_OK if the deletion could be successfully performed,
 * \retval ftkError::FTK_ERR_INV_PTR if \c evt is null,
 * \retval ftkError::FTK_ERR_INIT if \c evt was not created with ftkCreateEvent.
 *
 * \critical This function is involved in device event management.
 */
ATR_EXPORT ftkError ftkDeleteEvent( ftkEvent* evt );

ATR_EXPORT
#ifdef ATR_MSVC
__declspec( deprecated )
#endif
/** \brief Function accessing the generated events.
*
* This function allows to access the events generated by the device. The
* function reads the oldest event stored from the FIFO, the ownership of the
* ftkEvent instance is transferred to the caller of the function (i.e. the
* instance \e must be destroyed by calling ftkDeleteEvent()). If no event are
* currently in the FIFO, no error is triggered, the \c stackedEvts number
* value will be set to \c 0u.
*
* \warning The ownership of the memory allocated for the ftkEvent instance is
* transferred to caller of the function.
*
* \deprecated This function will be removed from the SDK as all events are now
* sent in the frame.
*
* \code
* ftkEvent* event( 0 );
* err = ftkReadEvent( lib, sn, &event, &stacked, &lost );
*
* if ( err != ftkError::FTK_OK )
* {
*     // error handling
* }
* else if ( event != 0 )
* {
*     switch( event->Type )
*     {
*         // ...
*     }
*     ftkDeleteEvent( event );
* }
* \endcode
*
* \param[in] lib initialised library handle.
* \param[in] sn serial number of the device from which the event was
* generated.
* \param[in,out] event pointer on an unitialised ftkEvent pointer, is
* overwritten by the function.
* \param[out] stackedEvts allows to retrieve the number of currently stacked
* events.
* \param[out] lostEvts allows to retrieve the number of currently lost events.
*
* \retval ftkError::FTK_OK if the event could be successfully retrieved,
* \retval ftkError::FTK_ERR_INV_PTR if \c lib or \c event is \c 0,
* \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved.
*
*
*
* \cond STK
* \warning Not available when using a spryTrack. Events are only available
*          as part of a ftkFrameQuery object retrieved through
*          ftkGetLastFrame().
* \endcond
*/
ftkError ftkReadEvent( ftkLibrary lib, uint64 sn, ftkEvent** event,
                                  uint32* stackedEvts,
                                  uint32* lostEvts)
#if defined( ATR_GCC ) || defined ( ATR_CLANG )
    __attribute__( ( deprecated ) )
#endif
;

/**
 * \}
 */

#endif // FTKEVENT_H
