// ===========================================================================
/**
 *   This file is part of the ATRACSYS fusionTrack library.
 *   Copyright (C) 2003-2018 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkInterface.h
 *  \brief    Main interface of Atracsys Passive Tracking SDK
 *
 */
// ===========================================================================

#ifndef ftkInterface_h
#define ftkInterface_h

#ifdef GLOBAL_DOXYGEN
/**
 * \addtogroup sdk
 * \{
 */
#endif

#include "ftkTypes.h"
#include "ftkPlatform.h"
#include "ftkErrors.h"
#include "ftkEvent.h"
#include "ftkOptions.h"

#ifdef __cplusplus
    #include <cstring>
#endif


#if !defined(__cplusplus) || (defined(ATR_MSVC) && _MSC_VER < 1900)
    #ifndef BUFFER_MAX_SIZE
    /** \brief Maximal size of ftkBuffer.
    *
    * The value is 10 KiB.
    */
    #define BUFFER_MAX_SIZE 10u * 1024u
#endif // BUFFER_MAX_SIZE
#else
constexpr uint32 BUFFER_MAX_SIZE( 10u * 1024u );
#endif

struct ftkLibraryImp;

/** \brief Library abstract handle
 */
typedef ftkLibraryImp* ftkLibrary;

/** \brief Available pixel formats
 */
TYPED_ENUM( uint8, ftkPixelFormat )
{
    /** \brief Pixels are represented as grayscale values ranging from 0
     * (black) to 255 (white).
     */
    GRAY8 = 0,
    /** \brief Pixels are represented as grayscale values ranging from 0
     * (black) to 255 (white). This images are AR.
     */
    GRAY8_AR=3,
    /** \brief Pixels are represented as grayscale values ranging from 0
     * (black) to 4096 (white).
     */
    GRAY16,
    /** \brief Pixels are represented as grayscale values ranging from 0
     * (black) to 4096 (white). This images are AR.
     */
    GRAY16_AR
};

#ifdef ATR_MSVC
// Disable warning because Visual C++ bug when using signed enums
    #pragma warning( push )
    #pragma warning(disable: 4341)
#endif
/** \brief Frame members status
 * \see ftkFrameQuery
 */
TYPED_ENUM( int8, ftkQueryStatus )
{
    /** \brief This field was not written.
     **/
    QS_WAR_SKIPPED = -1,
    /** \brief This field is initialised correctly and contains valid data.
     */
    QS_OK = 0,
    /** \brief This field is initialised correctly, but some data are missing
     * because buffer size is too small.
     */
    QS_ERR_OVERFLOW = 1,
    /** \brief The reserved size is not a multiple of the type's size.
     */
    QS_ERR_INVALID_RESERVED_SIZE = 2,
    /** \brief This field is requested to be reprocessed.
     */
    QS_REPROCESS = 10
};
#ifdef ATR_MSVC
// Disable warning because Visual C++ bug when using signed enums
    #pragma warning( pop )
#endif

/** \brief Type of device connected
 * \see ftkDeviceEnumCallback
 */
TYPED_ENUM( int8, ftkDeviceType )
{
    /** \brief The `device' is a simulator.
      */
    DEV_SIMULATOR = 0,
    /** \brief Device is an infiniTrack.
     */
    DEV_INFINITRACK = 1,
    /** \brief The device is a fusionTrack 500.
     */
    DEV_FUSIONTRACK_500 = 2,
    /** \brief The device is a fusionTrack 250.
     */
    DEV_FUSIONTRACK_250 = 3,
    /** \brief The device is a spryTrack 180.
     */
    DEV_SPRYTRACK_180 = 4,
    /** \brief Unknown device type.
     */
    DEV_UNKNOWN_DEVICE = 127
};

/** \brief Enumeration for the calibration type.
 *
 * This enum allows to assign a `tag' to a calibration file.
 */
TYPED_ENUM( uint32, CalibrationType )
{
    /** \brief Only use for tests, the dummy calibration file could
     * also use this.
     */
    DevOnly = 0u,
    /** \brief This calibration file contains a \e valid
     * precalibration.
     */
    Precalibration,
    /** \brief This calibration file contains an \e invalid
     * precalibration.
     */
    InvalidPrecalibration,
    /** \brief This calibration file contains a \e valid
     * calibration from CMM.
     */
    GeometricCalibration,
    /** \brief This calibration file contains an \e invalid
     * calibration from CMM.
     */
    InvalidGeometricCalibration,
    /** \brief This calibration file contains a \e valid
     * calibration with temperature compensation.
     */
    TemperatureCompensation,
    /** \brief This calibration file contains an \e invalid
     * calibration with temperature compensation.
     */
    InvalidTemperatureCompensation,
    /** \brief This calibration is in a unknown state (i.e. prior
     * to version 3).
     */
    Unknown = 0xffffffffu
};

/** \brief Enumeration for the data type.
 *
 * This enumeration is used to tell which information is gotten with the
 * ftkGetTotalObjectNumber function.
 */
TYPED_ENUM( uint32, ftkDataType )
{
    /** \brief Left raw data.
     */
    LeftRawData = 0x1,
    /** \brief Right raw data.
     */
    RightRawData = 0x2,
    /** \brief 3D fiducial data.
     */
    FiducialData = 0x4,
    /** \brief Marker data.
     */
    MarkerData = 0x8,
    /** \brief Events.
     */
    EventData = 0x10
};

/** \brief Structure carrying binary data in/out of library functions.
 *
 * This structure is used to get / set binary data. It helps reducing memory
 * leaks by taking care of the destruction of the memory.
 */
PACK1_STRUCT_BEGIN( ftkBuffer )
{
    /** \brief Data buffer.
     *
     *   This is a simple array of bytes.
     */
    union
    {
        char  data[ BUFFER_MAX_SIZE ];
        uint8 uData[ BUFFER_MAX_SIZE ];
        int8  sData[ BUFFER_MAX_SIZE ];
    };
    /** \brief Actual size of the data.
     *
     *   This member stores the real size of the data stored in the
     *   buffer.
     */
    uint32 size;
    /** \brief Method resetting the structure.
     *
     * This method erases the data buffer and sets the size to 0.
     */
#ifdef __cplusplus
    void reset();
#endif
}
PACK1_STRUCT_END( ftkBuffer );

#ifdef __cplusplus
inline void ftkBuffer::reset()
{
    memset( sData, 0, BUFFER_MAX_SIZE );
    size = 0u;
}
#endif

/** \brief Structure holding the status of a piece of data.
 *
 * This structure stores the status of the data. The structure is used across
 * ::ftkRawData, ::ftk3DFiducial and ::ftkMarker, which is for instance
 * allowing to propagate the status of the ::ftkRawData left instance to the
 * ::ftkMarker instance.
 */
PACK1_STRUCT_BEGIN( ftkStatus )
{
    /** \brief Contains 1 if the blob touches the right edge of the picture.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob touches the right edge of the
     * sensor. If set in a ::ftk3DFiducial instance, it means at least one of
     * two underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance touches the right edge of the sensor.
     */
    uint32 RightEdge  : 1;
    /** \brief Contains 1 if the blob touches the bottom edge of the picture.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob touches the bottom edge of
     * the sensor. If set in a ::ftk3DFiducial instance, it means at least one
     * of two underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance touches the bottom edge of the sensor.
     */
    uint32 BottomEdge : 1;
    /** \brief Contains 1 if the blob touches the left edge of the picture.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob touches the left edge of
     * the sensor. If set in a ::ftk3DFiducial instance, it means at least one
     * of two underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance touches the left edge of the sensor.
     */
    uint32 LeftEdge   : 1;
    /** \brief Contains 1 if the blob touches the top edge of the picture.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob touches the top edge of
     * the sensor. If set in a ::ftk3DFiducial instance, it means at least one
     * of two underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance touches the top edge of the sensor.
     */
    uint32 TopEdge    : 1;
    /** \brief Contains 1 if the blob lies in the part of the sensor where the
     * intensity descreases because of the lens cropping.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob lies in the area of the
     * sensor where the collected intensity drops due to lens cropping.
     * If set in a ::ftk3DFiducial instance, it means at least one of the two
     * underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance lies in the cropped area.
     *
     * \warning This is currently not implemented, i.e. the bit is presently
     * always \c 0.
     */
    uint32 SensorCrop : 1;
    /** \brief Contains 1 if the blob lies in the part of the sensor not
     * included in the accuracy measurement volume.
     *
     * This status is related to a ::ftkRawData instance. When set to \c 1 in
     * a ::ftkRawData instance, it means the blob lies outside of the
     * accuracy measurement volume .
     * If set in a ::ftk3DFiducial instance, it means at least one of the two
     * underlying ::ftkRawData (i.e. the left or the right one) has this
     * status. Finally, when set in a ::ftkMarker instance, at least one
     * ::ftk3DFiducial instance has it, meaning at least one of the used
     * ::ftkRawData instance lies outside the accuracy measurement volume.
     *
     * \warning This is currently not implemented, i.e. the bit is presently
     * always \c 0.
     */
    uint32 WorkingVolume : 1;
    /** \brief Contains 1 if the 3D fiducial has a z value outside the allowed
     * range.
     *
     * This status is related to a ::ftk3DFiducial instance. When set to \c 1,
     * it means the 3D pint lies outside the allowed \f$ z \f$ range (as
     * given in the device specifications). It set in a ::ftkMarker instance,
     * it means at least one of the used ::ftk3DFiducial instances lies outside
     * the allowed \f$ z \f$ range.
     *
     * \warning This is currently not implemented, i.e. the bit is presently
     * always \c 0.
     */
    uint32 ThreeDcrop    : 1;
#ifdef __cplusplus
    private:
#endif
    uint32 Reserved  : 25;
#ifdef __cplusplus
    public:
#endif

#ifdef __cplusplus
    /** \brief Conversion operator.
     *
     * This operator converts the structure into a unsigned integer.
     *
     * \return the value of the bit fields, seen as a 32-bits unsigned
     * integer.
     */
    operator uint32() const;
    /** \brief Affectation operator.
     *
     * This operator allows to promote a 32-bits unsigned integer into
     * a working instance.
     *
     * \param[in] val value to "dump" in the instance.
     *
     * \return a reference on the current instance.
     */
    ftkStatus& operator=( uint32 val );
#endif
}
PACK1_STRUCT_END( ftkStatus );

#ifdef __cplusplus
inline ftkStatus::operator uint32() const
{
    static_assert( sizeof( ftkStatus ) == 4u, "Problem with bitfield" );
    union
    {
        uint32 nbr; ftkStatus status;
    };
    status = *this;
    return nbr;
}

inline ftkStatus& ftkStatus::operator=( uint32 val )
{
    union
    {
        uint32 nbr; ftkStatus status;
    };
    nbr = val;
    *this = status;
    return *this;
}

#endif

#ifdef GLOBAL_DOXYGEN
/**
 * \}
 *
 * \addtogroup frame
 * \{
 */
#endif

/** \brief Structure to hold version and size
 *
 * This structure stores a version number and the reserved size for
 * an array in ftkFrameQuery.
 */
PACK1_STRUCT_BEGIN( ftkVersionSize )
{
    /** \brief Version number.
     */
    uint32 Version;
    /** \brief Size of the array in bytes.
     */
    uint32 ReservedSize;
}
PACK1_STRUCT_END( ftkVersionSize );

/** \brief Image header
 *
 * The image header contains basic information on the picture dimensions and
 * timing information.
 */
PACK1_STRUCT_BEGIN( ftkImageHeader )
{
    /** \brief Timestamp of the image in micro-seconds.
     */
    uint64 timestampUS;
    /** \brief Desynchronisation between left and right frames (infiniTrack
     * only, 0 otherwise).
     */
    uint64 desynchroUS;
    /** \brief Image counter.
     */
    uint32 counter;
    /** \brief Pixel format.
     */
    ftkPixelFormat format;
    /** \brief Image width (in pixels).
     */
    uint16 width;
    /** \brief Image height (in pixels).
     */
    uint16 height;
    /** \brief Image width * size of pixel + padding in bytes.
     */
    int32 imageStrideInBytes;
}
PACK1_STRUCT_END( ftkImageHeader );

/** \brief Fiducial raw data
 *
 * Fiducial raw data are low-level detection results from left and right
 * images.
 */
PACK1_STRUCT_BEGIN( ftkRawData )
{
    /** \brief Horizontal position of the center of the fiducial in image
     * reference (unit pixels).
     */
    floatXX centerXPixels;
    /** \brief Vertical position of the center of the fiducial in image
     * reference (unit pixels).
     */
    floatXX centerYPixels;
    /** \brief Status of the blob.
     */
    ftkStatus status;
    /** \brief Contain the surface of pixels composing the fiducial (unit
     * pixels).
     */
    uint32 pixelsCount;
    /** \brief Width of the bounding rectangle in pixels.
     */
    uint16 width;
    /** \brief Height of the bounding rectangle in pixels.
     */
    uint16 height;

}
PACK1_STRUCT_END( ftkRawData );

/** \brief 3D point structure
 *
 * This stores the coordinates of a 3D point.
 */
PACK1_STRUCT_BEGIN( ftk3DPoint )
{
    floatXX x, /**< 3D position, x component (unit mm) */
            y, /**< 3D position, y component (unit mm) */
            z; /**< 3D position, z component (unit mm) */
}
PACK1_STRUCT_END( ftk3DPoint );

/** \brief Fiducial 3D data after left-right triangulation
 *
 * 3D fiducials are retrieved after triangulation and matching of raw data.
 *
 * Errors description:
 *
 * `Epipolar geometry is the geometry of stereo vision. When two cameras view a
 * 3D scene from two distinct positions, there are a number of geometric
 * relations between the 3D points and their projections onto the 2D images
 * that lead to constraints between the image points.
 *
 * These relations are derived based on the assumption that the cameras can be
 * approximated by the pinhole camera model.' [source: <a
 * href="https://en.wikipedia.org/wiki/Epipolar_geometry">wikipedia</a>]
 *
 * - epipolarErrorPixels represents the signed distance between the right
 * epipolar line (of the left fiducial) and its matching right fiducial. Units
 * are in pixels.
 * - triangulationErrorMM represents the minimum distance of the 3D lines
 * defined by the left optical center and the left projection and the right
 * optical center and the right position. Units are in millimeters.
 *
 * Probability:
 *
 * The probability is defined by the number of potential matches. Basically
 * defined by number of potential matched points that are at a specified
 * distance from the epipolar lines. This ambiguity is usually disambiguated
 * once the 3D point is matched with a marker geometry.
 *
 * \see ftkRawData
 * \see http://en.wikipedia.org/wiki/Epipolar_geometry
 */
PACK1_STRUCT_BEGIN( ftk3DFiducial )
{
    /** \brief Status of the 3D fiducial.
     */
    ftkStatus status;
    /** \brief Index of the corresponding ftkRawData in the left image.
     */
    uint32 leftIndex;
    /** \brief Index of the corresponding ftkRawData in the right image.
     */
    uint32 rightIndex;
    /** \brief 3D position of the center of the fiducial (unit mm).
     */
    ftk3DPoint positionMM;
    /** \brief Epipolar matching error (unit pixel) (see introduction).
     */
    floatXX epipolarErrorPixels;
    /** \brief Triangulation error (unit mm) (see introduction).
     */
    floatXX triangulationErrorMM;
    /** \brief Probability (range 0..1) (see introduction).
     */
    floatXX probability;
}
PACK1_STRUCT_END( ftk3DFiducial );

/** \brief Structure containing calibration parameter for augmented reality
 *
 *  This structure contains all the parameters needed to do the math for
 *  augmented reality (a mix of 3d graphics computed using the positions
 *  returned by the SDK overlayed on top of the raw images).
 *
 */
PACK1_STRUCT_BEGIN( ftkARCalibrationParameters )
{
    /** \brief Principal Point X.
     */
    double principalPointX;
    /** \brief Principal Point Y.
     */
    double principalPointY;
    /** \brief Focal Length X.
     */
    double focalLenghtX;
    /** \brief Focal Length Y.
     */
    double focalLenghtY;
}
PACK1_STRUCT_END( ftkARCalibrationParameters );

#ifdef GLOBAL_DOXYGEN
/**
 * \}
 *
 * \addtogroup geometries
 * \{
 */
#endif

#if !defined(__cplusplus) || (defined(ATR_MSVC) && _MSC_VER < 1900)
    #define FTK_MAX_FIDUCIALS 6
#else
/** Maximum number of fiducials that define a geometry
 */
constexpr uint32 FTK_MAX_FIDUCIALS = 6;
#endif

/** \brief Geometric description of a marker
 *
 * The geometry can be defined in any referential. It will only influence to
 * output of the pose (which is the transformation between the geometry
 * referential and the device referential).
 */
PACK1_STRUCT_BEGIN( ftkGeometry )
{
    /** \brief Unique Id defining the geometry.
     */
    uint32 geometryId;
    /** \brief Version of the geometry structure.
     */
    uint32 version     : 8;
    /** \brief Number of points defining the geometry.
     */
    uint32 pointsCount : 24;
    /** \brief 3D position of points defining the geometry (unit mm).
     */
    ftk3DPoint positions[ FTK_MAX_FIDUCIALS ];
}
PACK1_STRUCT_END( ftkGeometry );

#ifdef GLOBAL_DOXYGEN
/**
 * \}
 *
 * \addtogroup frame
 * \{
 */
#endif

#if !defined(__cplusplus) || (defined(ATR_MSVC) && _MSC_VER < 1900)
    #define INVALID_ID 0xffffffffu
#else
/** Define an invalid fiducial correspondence
 */
constexpr uint32 INVALID_ID( 0xffffffffu );
#endif

/** \brief Marker data after left-right triangulation and marker retrieval
 *
 * Marker are retrieved within the 3D fiducials data based on their unique
 * geometry. When several markers are used, it is recommended to use specific
 * geometries in order to provide a unique tracking.
 *
 * Tracking id is provided to differentiate between several markers of the same
 * geometry. The id is reset when less than 3 spheres composing the marker are
 * visible or if the marker cannot be found again from its last known position.
 *
 * The geometryPresenceMask allows to get the correspondence between the
 * indices of fiducials specified in the geometry and 3D fiducial
 * (ftk3DFiducial) indexes of the current measure. When a match is invalid, it
 * is set to \c INVALID_ID. Alternatively, valid matches can be retrieved via
 * the geometryPresenceMask.
 *
 * Marker rigid transformation (rotation and translation) is the transformation
 * from the geometry referential to the measures of the sensor.
 *
 * Registration error is the mean distance of geometry and measured fiducials,
 * expressed in the same referential.
 *
 * \see ftkGeometry
 * \see ftk3DFiducial
 */
PACK1_STRUCT_BEGIN( ftkMarker )
{
    /** \brief Status of the marker.
     */
    ftkStatus status;
    /** \brief Tracking id.
     */
    uint32 id;
    /** \brief Geometric id, i.e. the unique id of the used geometry.
     */
    uint32 geometryId;
    /** \brief Presence mask of fiducials expressed as their geometrical
     * indices.
     */
    uint32 geometryPresenceMask;
    /** \brief Correspondence between geometry index and 3D fiducials indices
     * or \c INVALID_ID.
     */
    uint32 fiducialCorresp[ FTK_MAX_FIDUCIALS ];
    /** \brief Rotation matrix: format [row][column].
     */
    floatXX rotation[ 3 ][ 3 ];
    /** \brief Translation vector (unit mm).
     */
    floatXX translationMM[ 3 ];
    /** \brief Registration mean error (unit mm).
     */
    floatXX registrationErrorMM;
}
PACK1_STRUCT_END( ftkMarker );

/** \brief Store all data from a pair of images
 *
 * This structure stores all the buffers that can be retrieved from a pair of
 * images.
 *
 * The control of the fields to be retrieved can be specified during
 * initialization.
 *
 * Fields can be grouped in different sections:
 *   - imageHeader: get current image information;
 *   - imageLeftPixels: get left image pixels;
 *   - imageRightPixels: get right image pixels;
 *   - rawDataLeft: retrieve the raw data from left image;
 *   - rawDataRight: retrieve the raw data from right image;
 *   - threeDFiducials: retrieve 3D positions of left-right matches;
 *   - markers: retrieve marker poses (rotation + translation);
 *   - events: retrieve events.
 *
 * Every group of fields must be initialised to retrieve corresponding
 * information. The following example presents such an initialisation:
 * \code
 * ftkFrameQuery* fq( ftkCreateFrame );
 *
 * // Initialize a buffer of 100 raw data coming from the left image
 * ftkSetFrameOptions( false, 0u, 100u, 0u, 0u, 0u, fq );
 * // fq->rawDataLeftVersionSize.Version will be set after calling ftkGetLastFrame
 * // fq->rawDataLeftCount will be set after calling ftkGetLastFrame
 * // fq->rawDataLeftStat will be set after calling ftkGetLastFrame
 * \endcode
 *
 * Notes:
 *   - imageHeader, imageLeftPixels, imageRightPixels have only one occurrence;
 *   - imageLeftVersionSize and imageRightVersionSize present the allocated
 *     size for the image;
 *   - the different status inform if the operation is successful or not.
 */
PACK1_STRUCT_BEGIN( ftkFrameQuery )
{
    /** \brief Address where the image header will be written (input).
     */
    ftkImageHeader* imageHeader;
    /** \brief Version and \c sizeof( ftkImageHeader \c ) (input).
     */
    ftkVersionSize imageHeaderVersionSize;
    /** \brief Status of the image header, written by ftkGetLastFrame (output).
     */
    ftkQueryStatus imageHeaderStat;

    /** \brief Internal usage.
     */
    void* internalData;

    /** \brief Address of a pointer array of retrieved events (input).
     */
    ftkEvent** events;
    /** \brief Version and \c sizeof( \c events \c ) (input).
     */
    ftkVersionSize eventsVersionSize;
    /** \brief Number of events written by the SDK (output).
     */
    uint32 eventsCount;
    /** \brief Status of the events container (output).
     */
    ftkQueryStatus eventsStat;

    /** \brief Address where the left pixels will be written (input).
     */
    uint8* imageLeftPixels;
    /** \brief Version and \c sizeof( \c imageLeftPixels \c ), which must be
     * at least \f$ imageStrideInBytes \cdot height \f$ (input).
     */
    ftkVersionSize imageLeftVersionSize;
    /** \brief Status of the imageLeftPixels container (output).
     */
    ftkQueryStatus imageLeftStat;


    /** \brief Address where the right pixels will be written (input).
     */
    uint8* imageRightPixels;
    /** \brief Version and \c sizeof( \c imageLeftPixels \c ), which must be
     * at least \f$ imageStrideInBytes \cdot height \f$ (input).
     */
    ftkVersionSize imageRightVersionSize;
    /** \brief Status of the imageLeftPixels container (output).
     */
    ftkQueryStatus imageRightStat;

    /** \brief Address where the left raw data will be written (input).
     */
    ftkRawData* rawDataLeft;
    /** \brief Version and \c sizeof( \c rawDataLeft \c ) (input).
     */
    ftkVersionSize rawDataLeftVersionSize;
    /** \brief Number of left raw data written by the SDK (output).
     */
    uint32 rawDataLeftCount;
    /** \brief Status of the rawDataLeft container (output).
     */
    ftkQueryStatus rawDataLeftStat;

    /** \brief Address where the right raw data will be written (input).
     */
    ftkRawData* rawDataRight;
    /** \brief Version and \c sizeof( \c rawDataRight \c ) (input).
     */
    ftkVersionSize rawDataRightVersionSize;
    /** \brief Number of right raw data written by the SDK (output).
     */
    uint32 rawDataRightCount;
    /** \brief Status of the rawDataRight container (output).
     */
    ftkQueryStatus rawDataRightStat;

    /** \brief Address where the 3D fiducial data will be written (input).
     */
    ftk3DFiducial* threeDFiducials;
    /** \brief Version and \c sizeof( \c threeDFiducials \c ) (input).
     */
    ftkVersionSize threeDFiducialsVersionSize;
    /** \brief Number of 3D fiducial data written by the SDK (output).
     */
    uint32 threeDFiducialsCount;
    /** \brief Status of the threeDFiducials container (output).
     */
    ftkQueryStatus threeDFiducialsStat;

    /** \brief Address where the marker data will be written (input).
     */
    ftkMarker* markers;
    /** \brief Version and \c sizeof( \c markers \c ) (input).
     */
    ftkVersionSize markersVersionSize;
    /** \brief Number of marker data written by the SDK (output).
     */
    uint32 markersCount;
    /** \brief Status of the markers container (output).
     */
    ftkQueryStatus markersStat;
}
PACK1_STRUCT_END( ftkFrameQuery );

#ifdef GLOBAL_DOXYGEN
/**
 * \}
 */
#endif


/** System dependant function prefixes for DLLs
 */
#ifdef ATR_WIN
    #   define _CDECL_ __cdecl
#else
    #   define _CDECL_
#endif

#ifdef GLOBAL_DOXYGEN
/**
 * \addtogroup deviceSDK
 * \{
 */
#endif
/** \brief Callback for device enumeration.
 *
 * This is the signature for the device enumeration callback.
 *
 * \param[in] sn serial number of the enumerated device.
 * \param[in,out] user pointer on user data.
 * \param[in] type type of the device.
 */
typedef void ( _CDECL_ * ftkDeviceEnumCallback )( uint64 sn, void* user,
                                                  ftkDeviceType type );
#ifdef GLOBAL_DOXYGEN
/**
 * \}
 */
#endif

#ifdef GLOBAL_DOXYGEN
/**
 * \addtogroup options
 * \{
 */
#endif
/** \brief Callback for options enumeration.
 *
 * This is the signature for the device option enumeration callback.
 *
 * \param[in] sn serial number of the enumerated device.
 * \param[in,out] user pointer on user data.
 * \param[in] oi pointer on the enumerated option information.
 */
typedef void ( _CDECL_ * ftkOptionsEnumCallback )( uint64 sn, void* user,
                                                   ftkOptionsInfo* oi );
#ifdef GLOBAL_DOXYGEN
/**
 * \}
 */
#endif

#ifdef GLOBAL_DOXYGEN
/**
 * \addtogroup geometries
 * \{
 */
#endif
/** \brief Callback for geometry enumeration.
 *
 * This is the signature for the geometry enumeration callback.
 *
 * \param[in] sn serial number of the enumerated device.
 * \param[in,out] user pointer on user data.
 * \param[in] in pointer on the enumerated geometry.
 */
typedef void ( _CDECL_ * ftkGeometryEnumCallback )( uint64 sn, void* user,
                                                    ftkGeometry* in );
#ifdef GLOBAL_DOXYGEN
/**
 * \}
 */
#endif
// ----------------------------------------------------------------
#ifdef GLOBAL_DOXYGEN
/** \addtogroup library
 * \{
 */
#else
/** \defgroup library General Library Functions
 * \brief Functions to initialize and close the library.
 * \{
 */
#endif


/** \brief Function initialising the library.
 *
 * This function initialises the library and creates the handle needed by the
 * other library functions. It must therefore be called before any other
 * function from the library.
 *
 * \return the library handle or \c nullptr if an error occurred.
 *
 * \code
 * // Initialize and close library
 *
 * ftkLibrary handle( ftkInit() );
 * if ( handle == nullptr )
 * {
 *     ERROR( "Cannot open library" );
 * }
 *
 * // ...
 *
 * if ( ftkClose( &handle ) != ftkError::FTK_OK )
 * {
 *     ERROR( "Cannot close library" );
 * }
 * \endcode
 *
 * \see ftkClose
 */
ATR_EXPORT ftkLibrary ftkInit();

/** \brief `Extended' initialisation of the library.
 *
 * This function allows to customise the library default initialisation.
 * \if STK
 * It currently provides no customisation.
 * \else
 * It currently provides support for changing the IP address of the device it
 * connects to. Please note the device port default value cannot be changed.
 * The configuration file uses
 * <a href="http://www.json.org/" target="_blank">JSON</a> format, with the
 * following syntax:
 * \verbatim
 {
   "network" :
   {
     "version": 1,
     "interfaces": [
       {
         "address": "172.17.10.7",
         "port": 3509
       }
     ]
   }
 }
 \endverbatim
 * If several interfaces are defined, then the SDK will allow to connect to
 * multiple fusionTrack devices. This means the ftkEnumerateDevices will be
 * able to detect up to one device per defined interface. Each function
 * taking a serial number as input argument will behave correctly, i.e. will
 * interact with the device with the given serial number. When using multiple
 * fusionTrack with the same SDK, it is recommended to set them to use distinct
 * subnetworks (i.e. only the two first bytes of the IP address match).
 * \endif
 * The function also allows the caller to get information about initialisation
 * errors, as shown on the following snippet:
 * \code
 * ftkBuffer buffer;
 * ftkLibrary lib( ftkInitExt( "configuration.json", &buffer ) );
 *
 * if ( lib == nullptr )
 * {
 *     cerr << buffer.data << endl;
 *     cerr << "Cannot initialise library" << endl;
 *     return 1;
 * }
 * \endcode
 *
 * \param[in] fileName name of the JSON file from which the configuration will
 * be read.
 * \param[out] errs pointer on the buffer containing potential errors.
 *
 * \return the library handle or \c nullptr if an error occurred.
 */
ATR_EXPORT ftkLibrary ftkInitExt( const char* fileName, ftkBuffer* errs );

/** \brief Close the library.
 *
 * This function destroys the library handle and frees the resources. Any
 * call to a library function after this will fail.
 *
 * \param[in] ptr pointer an initialised library handle
 *
 * \retval ftkError::FTK_OK if the library could be closed successfully,
 * \retval ftkError::FTK_ERR_INV_PTR if the \c ptr is \c 0 or if the library handle
 * pointed by \c ptr has already been closed.
 *
 * \see ftkInit
 */
ATR_EXPORT ftkError ftkClose( ftkLibrary* ptr );


/** \brief Getter for the library version
 *
 * This function allows to access the current SDK version as a string. The
 * format is Major.Minor.Revision.Build (Unix timestamp).
 *
 * \param[out] bufferOut pointer on the output buffer, contains the data and
 * the actual data size.
 */
ATR_EXPORT void ftkVersion( ftkBuffer* bufferOut );

/** \brief Getter for the last error.
 *
 * This function allows to retrieve the last error (i.e. the error from the
 * previous call to another SDK function). The error is formatted as an XML
 * string:
 * \verbatim
 <ftkError>
   <errors> ... </errors>
   <warnings> ... </warnings>
   <messages> ... </messages>
 </ftkError>
 \endverbatim
 * The errors tag contains the error codes and strings, the warning tag
 * contains the warning codes and string. The messages tag contains optional
 * extra messages.
 *
 * \param[in] lib initialised library handle.
 * \param[in] strSize size of the string passed as argument.
 * \param[in,out] str allocated character array containing the output XML.
 *
 * \retval ftkError::FTK_OK if the retrieving could be done successfully,
 * \retval ftkError::FTK_ERR_INV_PTR if lib or \c str is \c 0.
 */
ATR_EXPORT ftkError ftkGetLastErrorString( ftkLibrary lib, size_t strSize,
                                           char str[] );

/** \brief Getter for the `explanation' of a status code.
 *
 * This function allows to get the message associated to a status code.
 *
 * \param[in] status code for which the string must be retrieved.
 * \param[out] buffer pointer on the output buffer, contains the data
 * and the actual data size.
 *
 * \retval ftkError::FTK_OK if the message could be retrieved successfully,
 * \retval ftkError::FTK_ERR_INV_PTR if \c buffer is \c nullptr,
 * \retval ftkError::FTK_ERR_INTERNAL if an unexpected error occurred (i.e. the
 * \c status value is unknown).
 */
ATR_EXPORT ftkError ftkStatusToString( ftkError status, ftkBuffer* buffer );

/* \} */

// ----------------------------------------------------------------
#ifdef GLOBAL_DOXYGEN
/** \addtogroup deviceSDK
 * \{
 */
#else
/** \defgroup deviceSDK Device Functions
 * \brief Function to enumerate devices
 * \{
 */
#endif

/** \brief Enumerate available tracker devices.
 *
 * This function allows to scan all the connected devices and to call the
 * user-defined callback function for any found device.
 *
 * \param[in] lib an initialised library handle
 * \param[in] cb the device enumeration callback
 * \param[in] user parameter of the callback
 *
 * \code
 * void deviceEnumCallback( uint64 sn, void* user, ftkDeviceType type )
 * {
 *     uint64* lastDevice = (uint64*) user;
 *     if ( lastDevice != 0 )
 *     {
 *         lastDevice = sn;
 *     }
 * }
 *
 * main()
 * {
 *     // Initialize library (see ftkInit example)
 *
 *     uint64 sn( 0uLL );
 *     if ( ftkEnumerateDevices( lib, deviceEnumCallback, &sn ) != ftkError::FTK_OK )
 *     {
 *         ERROR( "Cannot enumerate devices" );
 *     }
 *     if ( sn == 0uLL )
 *     {
 *         ERROR( "No device connected" );
 *     }
 *
 *     // ...
 * }
 * \endcode
 *
 * \see ftkInit
 * \see ftkClose
 *
 * \retval ftkError::FTK_OK if the enumeration could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised,
 * \retval ftkError::FTK_WAR_CALIB_AUTHENTICATION if the calibration file
 * authentication could not be checked.
 *
 * \critical This function is involved in the device detection chain.
 */
ATR_EXPORT ftkError ftkEnumerateDevices( ftkLibrary lib,
                                         ftkDeviceEnumCallback cb, void* user );


/** \brief Opens the XML file for the dump.
*
* This function opens a XML file in which the data will be dumped. If a file
* is already opened, an error is returned.
*
* \param[in] lib initialised library handle.
* \param[in] serialNbr serial number of the device for which the file is
* created.
* \param[in] fileName name of the XML file to create (if not provided, then a
* file Dump_serialNumber.xml is created).
*
* \retval ftkError::FTK_OK if the file could be successfully opened and
* written,
* \retval ftkError::FTK_ERR_INV_PTR if \c lib is \c nullptr,
* \retval ftkError::FTK_ERR_INV_SN if \c serialNbr is not a valid serial
* number,
* \retval ftkError::FTK_ERR_WRITE if a file is already opened or if if the file
* cannot be opened.
*/
ATR_EXPORT ftkError ftkOpenDumpFile( ftkLibrary lib, uint64 serialNbr,
                                     const char* fileName );

/** \brief Closes the XML dump file.
*
* This function closes the XML file used for dump. Any call to
* stereoDumpDeviceInfo or stereoDumpFrame will fail. If no files are opened an
* error is returned.
*
* \param[in] lib initialised library handle,
* \param[in] serialNbr device serial number.
*
* \retval ftkError::FTK_OK if the file could be successfully written and
* closed,
* \retval ftkError::FTK_ERR_INV_PTR if \c lib is \c nullptr,
* \retval ftkError::FTK_ERR_INV_SN if \c serialNbr is not a valid serial
* number,
* \retval ftkError::FTK_ERR_WRITE if a file is not opened.
*/
ATR_EXPORT ftkError ftkCloseDumpFile( ftkLibrary lib, uint64 serialNbr );



/** \brief Function dumping the device information.
 *
 * The device information consist of the option and internal register values.
 * If no files are opened an error is returned.
 *
 * \param[in] lib initialised library handle,
 * \param[in] sn device serial number.
 *
 * \retval ftkError::FTK_OK if the file could be written.
 * \retval ftkError::FTK_ERR_INV_PTR if \c lib is \c nullptr,
 * \retval ftkError::FTK_ERR_INV_SN if \c sn is not valid,
 * \retval ftkError::FTK_ERR_WRITE if the file could not be written or was not
 * opened,.
 */
ATR_EXPORT ftkError ftkDumpInfo( ftkLibrary lib, uint64 sn );

/* \} */

// ----------------------------------------------------------------
#ifdef GLOBAL_DOXYGEN
/** \addtogroup frame
 * \{
 */
#else
/** \defgroup frame Frame Functions
 * \brief Functions to acquire frames (data from the tracking system)
 * \{
 */
#endif

/** \brief Creates a frame instance.
 *
 * This function allows to initialise a frame instance. The following
 * example is valid.
 *
 * \code
 * ftkFrameQuery* frame( ftkCreateFrame() );
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return;
 * }
 * \endcode
 *
 * \warning Not using this function may lead to problems getting the data
 * when using ftkGetLastFrame.
 *
 * \warning The user must call ftkDeleteFrame to correctly deallocate the
 * memory.
 *
 * \return a ftkFrameQuery pointer, or \c nullptr if an error occurred.
 *
 */
ATR_EXPORT ftkFrameQuery* ftkCreateFrame();

/** \brief Initialises a frame.
 *
 * This function allows to initialise a frame instance, which content can
 * be parametrised. This function can also be used to reinitialise an
 * instance, e.g. to increase the number of retrieved markers. The
 * following example is valid.
 *
 * \code
 * ftkFrameQuery* frame( ftkCreateFrame() );
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return;
 * }
 *
 * if ( ftkSetFrameOptions( false, 10u, 128u, 128u, 100u, 10u, frame ) != ftkError::FTK_OK )
 * {
 *   // error management
 *   return;
 * }
 *
 * if ( ftkSetFrameOptions( false, 10u, 64u, 64u, 42u, 10u, frame ) != ftkError::FTK_OK )
 * {
 *   // error management
 *   return;
 * }
 * \endcode
 *
 * \warning Not using this function will lead to problems getting the data
 * when using ftkGetLastFrame().
 *
 * \warning The user must call ftkDeleteFrame() to correctly deallocate the
 * memory.
 *
 * \param[in] pixels should be \c true for the left and right pictures to be
 * retrieved.
 * \param[in] eventsSize maximal number of retrieved ftkEvents
 * instances for the left camera.
 * \param[in] leftRawDataSize maximal number of retrieved ftkRawData
 * instances for the left camera.
 * \param[in] rightRawDataSize maximal number of retrieved ftkRawData
 * instances for the right camera.
 * \param[in] threeDFiducialsSize maximal number of retrieved ftk3DFiducial
 * instances.
 * \param[in] markersSize maximal number of retrieved ftkMarker instances.
 * \param[in,out] frame pointer on an initialised ftkFrameQuery instance.
 *
 * \retval ftkError::FTK_OK if the option setting could be successfully performed,
 * \retval ftkError::FTK_ERR_INIT if the \c frame pointer was not initialised using
 * ftkCreateFrame,
 * \retval ftkError::FTK_ERR_INV_PTR if an allocation failed.
 *
 * \critical This function is involved in ftkFrameQuery management.
 */
ATR_EXPORT ftkError ftkSetFrameOptions( bool pixels,
                                        uint32 eventsSize,
                                        uint32 leftRawDataSize,
                                        uint32 rightRawDataSize,
                                        uint32 threeDFiducialsSize,
                                        uint32 markersSize,
                                        ftkFrameQuery* frame );

/** \brief Frees the memory from the allocated fields.
 *
 * This function deallocates the used memory for the \e members of the
 * instance, \c not the instance itself!. The
 * following example is valid.
 *
 * \code
 * ftkFrameQuery* frame = ftkCreateFrame();
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return;
 * }
 *
 * if ( ftkSetFrameOptions( false, 10u, 128u, 128u, 100u, 10u, frame ) !=
 *FTK_OK )
 * {
 *   // error management
 *   return;
 * }
 *
 * if ( ftkSetFrameOptions( false, 10u, 64u, 64u, 42u, 10u, frame ) != ftkError::FTK
 *)
 * {
 *   // error management
 *   return;
 * }
 *
 * err = ftkDeleteFrame( frame );
 * if ( err != ftkError::FTK_OK )
 * {
 *     // error management
 *     return;
 * }
 *
 * // Using frame will crash the API, as it was deleted!
 * \endcode
 *
 * \param[in] frame pointer on the instance which will be deleted.
 *
 * \retval ftkError::FTK_OK if the cleaning performed successfully,
 * \retval ftkError::FTK_ERR_INIT if the \c frame pointer was not initialised using
 * ftkCreateFrame.
 *
 * \critical This function is involved in ftkFrameQuery management.
 */
ATR_EXPORT ftkError ftkDeleteFrame( ftkFrameQuery* frame );

/** \brief Retrieves the number of data of each type for the \e previous frame.
 *
 * This function allows to know how many elements of each type were available
 * in the \e previous frame. This allows to know how many data were lost when
 * a container status is ftkQueryStatus::QS_ERR_OVERFLOW.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[out] what type of info to retrieve.
 * \param[out] value pointer on the retrieved value.
 *
 * \retval ftkError::FTK_OK if the information could be successfully retrieved,
 * ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised or if the \c value is null or if the internal
 * data for the picture could not be allocated,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_WAR_NOT_SUPPORTED if the desired information is not
 * handled.
 */
ATR_EXPORT ftkError ftkGetTotalObjectNumber( ftkLibrary lib, uint64 sn,
                                             ftkDataType what, uint32* value );

/** \brief Retrieve the latest available frame.
 *
 * A frame contains all the data related to a pair of image. The frame query
 * structure must be initialised prior to this function call in order to
 * specify what type of information should be retrieved.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in,out] frameQueryInOut an initialised ftkFrameQuery structure.
 * \param[in] timeoutMS returns FTK_WAR_NO_FRAME if no frame is available
 * within X ms.
 *
 * \code
 * main ()
 * {
 *     // Initialize library (see ftkInit() example)
 *     // Get an attached device (see ftkEnumerateDevices() example)
 *
 *     ftkFrameQuery* fq( ftkCreateFrame() );
 *     if ( fq == 0 )
 *     {
 *         ERROR( "Error allocating frame" );
 *     {
 *     if ( ftkSetFrameOptions( false, 0u, 0u, 0u, 0u, 16u, fq ) != ftkError::FTK_OK )
 *     {
 *         ERROR( "Error initialising frame" );
 *     }
 *
 *     // Wait until the next frame is available
 *     if ( ftkGetLastFrame( lib, sn, fq, 10u ) != ftkError::FTK_OK )
 *     {
 *         ERROR ("Error acquiring frame");
 *     }
 *
 *     if ( fq->markersStat == ftkQueryStatus::QS_OK )
 *     {
 *         for ( uint32 u( 0u ); u < fq->markersCount; ++u )
 *         {
 *             // Access marker data here (fq->markers[ u ] ...)
 *         }
 *     }
 * }
 * \endcode
 *
 * \see ftkInit
 * \see ftkEnumerateDevices
 *
 * \retval ftkError::FTK_OK if the frame could be retrieved correctly,
 * \retval ftkError::FTK_ERR_INIT if the \c frame pointer was not initialised
 * using ftkCreateFrame,
 * \retval ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised or if the \c frameQueryInOut is null or if the internal
 * data for the picture could not be allocated,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INTERNAL if the triangulation or the marker
 * matcher class are not properly initialised  or if no image are retrieved or
 * if the image size is invalid or if a compressed image is corrupted,
 * \retval ftkError::FTK_ERR_COMP_ALGO if the temperature compensation
 * algorithm is undefined,
 * \retval ftkError::FTK_ERR_SYNC if the retrieved pictures are not
 * synchronised,
 * \retval ftkError::FTK_WAR_NO_FRAME if no frame are available,
 * \retval ftkError::FTK_ERR_SEG_OVERFLOW if an overflow occurred during image
 * segmentation,
 * \retval ftkError::FTK_ERR_IMG_DEC if a picture cannot be decompressed,
 * \retval ftkError::FTK_ERR_IMG_FMT if the gotten picture data are not
 * compatible with the SDK,
 * \retval ftkError::FTK_ERR_IMPAIRING_CALIB if the calibration stored in the
 * device prevents triangulation,
 * \retval ftkError::FTK_WAR_REJECTED_PIC of the pictures were too big and not
 * sent by the device,
 * \retval ftkError::FTK_WAR_SHOCK_DETECTED if a shock which potentially
 * decalibrated the device has been detected,
 * \retval ftkError::FTK_WAR_SHOCK_SENSOR_OFFLINE if the shock sensor is
 * currently offline,
 * \retval ftkError::FTK_WAR_TEMP_LOW if the current temperature is too low for
 * compensation,
 * \retval ftkError::FTK_WAR_TEMP_HIGH if the current temperature of the device
 * is too high for compensation,
 * \retval ftkError::FTK_WAR_TEMP_INVALID if the last temperature reading was
 * invalid.
 */
ATR_EXPORT ftkError ftkGetLastFrame( ftkLibrary lib, uint64 sn,
                                     ftkFrameQuery* frameQueryInOut,
                                     uint32 timeoutMS );

/** \brief Reprocess the given frame.
 *
 * Frame reprocessing allows the user to reprocess only a part of the contained
 *  data, i.e. the markers, or 3D and markers. The current implementation does
 * \e not support a built-in reprocessing of the pictures, meaning that pixel
 * reprocessing must be done by a user defined function.
 *
 * The user must specify which data must be reprocessed by setting the
 * corresponding status flag to ftkQueryStatus::QS_REPROCESS before calling the
 * function. This flag is recursive, in the sense that asking for a
 * reprocessing of the raw data will trigger a reprocessing of the 3D fiducials
 * and the markers.
 *
 * \warning Does \e not supply pixel reprocessing!
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in,out] frameQueryInOut an initialised ftkFrameQuery structure.
 *
 * \retval ftkError::FTK_OK if the frame could be reprocessed correctly (which
 * may indicate that no reprocessing was actually asked as well),
 * \retval ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised or if the \c frameQueryInOut is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INTERNAL if the triangulation or marker matcher
 * class are not initialised correctly or image reprocessing is requested,
 * \retval ftkError::FTK_ERR_IMPAIRING_CALIB if the calibration contained in
 * the device prevents triangulation,
 * \retval ftkError::FTK_WAR_FRAME if there are no images provided,
 * \retval ftkError::FTK_ERR_SEG_OVERFLOW if an overflow occurred during image
 * segmentation,
 * \retval ftkError::FTK_WAR_TEMP_LOW if the current temperature is too low for
 * compensation,
 * \retval ftkError::FTK_WAR_TEMP_HIGH if the current temperature of the device
 * is too high for compensation.
 */
ATR_EXPORT ftkError ftkReprocessFrame( ftkLibrary lib, uint64 sn,
                                       ftkFrameQuery* frameQueryInOut );

/** \brief Function dumping frame.
 *
 * This function allows to dump the current frame in XML format. It is meant
 * for debugging and diagnostics purpose.
 *
 * \warning This function is currently not implemented, i.e. it can be called
 * but no file will be produced.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 *
 * \retval ftkError::FTK_OK if the frame could be dumped successfully,
 * \retval ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_WRITE if the file could not be written,
 * \retval ftkError::FTK_WAR_NOT_SUPPORTED as long as the feature is not
 * implemented.
 */
ATR_EXPORT ftkError ftkDumpFrame( ftkLibrary lib, uint64 sn );

/* \} */

// ----------------------------------------------------------------

#if defined(ATR_FTK) || defined(ATR_STK)
    #ifdef GLOBAL_DOXYGEN
/** \addtogroup data
 * \{
 */
    #else
/** \defgroup data Data Functions
 * \brief Functions read data from the device sensors.
 * \{
 */
    #endif

/** \brief Getter for the accelerometer data.
 *
 * This function allows to access the current accelerometer data. The
 * acceleration are returned as a 3D vector in standard units.
 *
 * \warning In releases prior to 2.0.3.150, the acceleration was given in units
 * of the Earth gravitational constant (\f$ 9.81 m \, s^{-2}\f$).
 *
 * \deprecated This function should be replaced with ::ftkGetAccelerometerData.
 *
 * \param[in] lib an initialised library handle
 * \param[in] sn a valid serial number of the device
 * \param[out] firstValue pointer on the output acceleration read by the
 * first accelerometer in \f$m \, s^{-2}\f$ units.
 * \param[out] secondValue pointer on the output acceleration read by the
 * second accelerometer in \f$m \, s^{-2}\f$ units.
 *
 * \retval ftkError::FTK_OK if the data could be retrieved successfully.
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised, or the \c firstValue pointer is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT if the fudsonTrack does not support that
 * feature,
 * \retval ftkError::FTK_ERR_INV_OPT_VAL if the accelerometer status indicates
 * the reading is invalid,
 * \retval ftkError::FTK_ERR_READ if the wanted register cannot be read.
 */
ATR_EXPORT
#ifdef ATR_MSVC
__declspec( deprecated )
#endif
ftkError ftkGetAcceleration( ftkLibrary lib, uint64 sn,
                                        ftk3DPoint* firstValue,
                                        ftk3DPoint* secondValue )
#if defined( ATR_GCC ) || defined ( ATR_CLANG )
    __attribute__( ( deprecated ) )
#endif
;

/** \brief Getter for the accelerometer data.
 *
 * This function allows to access the current accelerometer data. The
 * acceleration are returned as a 3D vector in standard units.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in] index index of the accelerometer to read.
 * \param[out] value pointer on the output acceleration read by the
 * first accelerometer in \f$m \, s^{-2}\f$ units.
 *
 *
 * \retval ftkError::FTK_OK if the data could be retrieved successfully,
 * \retval ftkError::FTK_WAR_NOT_SUPPORTED if the fusionTrack does not support
 * that feature,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised, or the \c value pointer is \c nullptr,
 * \retval ftkError::FTK_ERR_INV_INDEX if the given index is invalid,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_VAL if the accelerometer status indicates
 * the reading is invalid,
 * \retval ftkError::FTK_ERR_READ if the wanted register cannot be read.
 */
ATR_EXPORT ftkError ftkGetAccelerometerData( ftkLibrary lib, uint64 sn,
                                             uint32 index, ftk3DPoint* value );

/** \brief Getter for the real time clock timestamp.
 *
 * This function allows to get the current timestamp of the fusionTrack, given
 * by the on board real time clock module. The value is the linux timestamp:
 * the number of seconds since January 1st 1970 at 00:00:00.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[out] timestamp pointer on the written timestamp.
 *
 * \retval ftkError::FTK_OK if the data could be retrieved successfully.
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised, or the \c firstValue pointer is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_PAR if the wanted register cannot be read,
 * \retval ftkError::FTK_WAR_NOT_SUPPORTED if the RTC component cannot be
 * accessed (occurs for firmware versions prior to 1.1.6.6B).
 */
ATR_EXPORT ftkError ftkGetRealTimeClock( ftkLibrary lib, uint64 sn,
                                         uint64* timestamp );

/** \}
 */
#endif


// ----------------------------------------------------------------
#ifdef GLOBAL_DOXYGEN
/** \addtogroup geometries
 * \{
 */
#else
/** \defgroup geometries Geometries Functions
 * \brief Functions to set, clear or enumerate marker geometries
 * \{
 */
#endif

/** \brief Register a new marker geometry to be detect.
 *
 * This function tells the driver to look for the given geometry in the data.
 *
 * The system will try to match the registered geometry with the raw data.
 * Adding a geometry is immediate. You can remove a geometry with
 * ftkClearGeometry() or enumerate them with ftkEnumerateGeometries().
 *
 * \if STK
 * When using a spryTrack, the geometry will be sent to the devices.
 * This way, the marker detection will operate whether onboard processing
 * is enable or not.
 * \endif
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in] geometryIn a valid geometry to be detected.
 *
 * \code
 * // Initialize library (see ftkInit example)
 * // Get an attached device (see ftkEnumerateDevices example)
 *
 * // Attach a new geometry (id=52) composed of four fiducials
 * // Note that you can define the geometry in any referential.
 * // It will only change the pose of the marker
 *
 * ftkGeometry markerGeometry;
 * markerGeometry.geometryId = 52;
 * markerGeometry.pointsCount = 4;
 * markerGeometry.positions[0].x = 0.000000f;
 * markerGeometry.positions[0].y = 0.000000f;
 * markerGeometry.positions[0].z = 0.000000f;
 *
 * markerGeometry.positions[1].x = 78.736931f;
 * markerGeometry.positions[1].y = 0.000000f;
 * markerGeometry.positions[1].z = 0.000000f;
 *
 * markerGeometry.positions[2].x = 21.860918f;
 * markerGeometry.positions[2].y = 47.757847f;
 * markerGeometry.positions[2].z = 0.000000f;
 *
 * markerGeometry.positions[3].x = 111.273277f;
 * markerGeometry.positions[3].y = 51.558617f;
 * markerGeometry.positions[3].z = -2.107444f;
 *
 * if ( ftkSetGeometry( lib, sn, &markerGeometry ) != ftkError::FTK_OK )
 * {
 *     ERROR( "Cannot set geometry" );
 * }
 * \endcode
 *
 * \see ftkInit
 * \see ftkEnumerateDevices
 * \see ftkClearGeometry
 * \see ftkEnumerateGeometries
 *
 * \retval ftkError::FTK_OK if the geometry could be set correctly,
 * \retval ftkError::FTK_ERR_INV_PTR if the \c lib handle was not correctly
 * initialised or if the \c geometryIn pointer is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_GEOM_PTS if the number of points in the geometry
 * is strictly lower than 3 or larger than the maximum number of points.
 */
ATR_EXPORT ftkError ftkSetGeometry( ftkLibrary lib, uint64 sn,
                                    ftkGeometry* geometryIn );

/** \brief  Clear a registered geometry (giving its geometry id).
 *
 * This function tells the driver to stop looking for the given geometry in the
 * data.
 *
 * \param[in] lib an initialised library handle
 * \param[in] sn a valid serial number of the device
 * \param[in] geometryId a valid geometry to be unregistered
 *
 * \retval ftkError::FTK_OK if the geometry could be set correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_WAR_GEOM_ID if the wanted geometry is not registered.
 */
ATR_EXPORT ftkError ftkClearGeometry( ftkLibrary lib, uint64 sn,
                                      uint32 geometryId );

/** \brief Enumerate the registered geometries.
 *
 * This function enumerates all the registered geometries and allows to
 * apply a user-defined function on each of them.
 *
 * \see ftkEnumerateDevices for an example of enumeration).
 *
 * \retval ftkError::FTK_OK if the geometry could be set correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly initialised,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_WAR_NOT_EXISTING if the callback is \c 0.
 */
ATR_EXPORT ftkError ftkEnumerateGeometries( ftkLibrary lib, uint64 sn,
                                            ftkGeometryEnumCallback cb,
                                            void* user );

/* \} */

// ----------------------------------------------------------------


#ifdef GLOBAL_DOXYGEN
/** \addtogroup options
 * \{
 */
#else
/** \defgroup options Options Functions
 * \brief Functions to get, set or enumerate options
 *
 * Options enable getting or setting configuration information from/to the
 * driver module.
 *
 * Options may be global or specific to a type of device.
 *
 * Most options can be tested and are accessible in the demo program.
 *
 * See the different options structures and enumerators for more detailed
 * information.
 *
 * \see ftkComponent
 * \see ftkOptionType
 * \see ftkOptionGetter
 * \see ftkOptionStatus
 * \see ftkOptionsInfo
 *
 * \{
 */
#endif


/** \brief Enumerate available options.
 *
 * This function enumerates all the available options and allows to apply a
 * user-defined function on each of them.
 *
 * \param[in] lib an initialised library handle
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, sn=0LL for general purpose options)
 * \param[in] cb the option enumeration callback
 * \param[in] user parameter of the callback
 *
 * Example of code to display available option and their respective ids.
 *
 * \code
 * void optionEnumerator( uint64 sn, void* user, ftkOptionsInfo* oi )
 * {
 *     cout << "Option (" << oi->id << ") " oi->name << endl;
 * }
 *
 * main()
 * {
 *     // Initialize library (see ftkInit() example)
 *     // Get an attached device (see ftkEnumerateDevices() example)
 *
 *     if ( ftkEnumerateOptions( lib, sn, optionEnumerator, 0 ) != ftkError::FTK_OK )
 *     {
 *         ERROR( "Cannot enumerate options" );
 *     }
 * }
 * \endcode
 *
 * \see ftkInit
 * \see ftkEnumerateDevices
 * \see ftkOptionsInfo
 *
 * \retval ftkError::FTK_OK if the enumeration could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved.
 */
ATR_EXPORT ftkError ftkEnumerateOptions( ftkLibrary lib, uint64 sn,
                                         ftkOptionsEnumCallback cb,
                                         void* user );

/** \brief Get an integer option.
 *
 * This function allows to get an integer option value. Different values can be
 * retrieved:
 *   - the minimum value for the option;
 *   - the maximum value for the option;
 *   - the default value of the option;
 *   - the current value of the option.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[out] out output value.
 * \param[in] what define what to retrieve minimum, maximum, default or actual
 * value.
 *
 * \see ftkSetInt32
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c out is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_PAR if the option does not exist, or is
 * not of type \c int32,
 * \retval ftkError::FTK_ERR_READ if the option cannot be read from the device.
 */
ATR_EXPORT ftkError ftkGetInt32( ftkLibrary lib, uint64 sn, uint32 optID,
                                 int32* out, ftkOptionGetter what );

/** \brief Set an integer option.
 *
 * This function allows to set an integer option.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[in] val value to set.
 *
 * \see ftkGetInt32
 *
 * \retval ftkError::FTK_OK if the value setting could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c out is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_VAL if the value is invalid,
 * \retval ftkError::FTK_ERR_INV_OPT if the option does not exist, or is not of
 * type \c int32,
 * \retval ftkError::FTK_ERR_READ if the option cannot be written to the
 * device,
 * \retval ftkError::FTK_WAR_OPT_VAL_RANGE if the set value is outside the
 * allowed range and has therefore been cropped,
 * \retval ftkError::FTK_WAR_OPT_NO_OP if the set option has currently no
 * effect.
 */
ATR_EXPORT ftkError ftkSetInt32( ftkLibrary lib, uint64 sn, uint32 optID,
                                 int32 val );

/** \brief Get a float option.
 *
 * This function allows to get a floating-point option value. Different values
 * can be retrieved:
 *   - the minimum value for the option;
 *   - the maximum value for the option;
 *   - the default value of the option;
 *   - the current value of the option.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[out] out output value.
 * \param[in] what define what to retrieve minimum, maximum, default or actual
 * value.
 *
 * \see ftkSetFloat32
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c out is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_PAR if the option does not exist, or is
 * not of type \c float32,
 * \retval ftkError::FTK_ERR_READ if the option cannot be read from the device,
 * \retval ftkError::FTK_ERR_VERSION if the calibration file does not contain
 * temperature information.
 */
ATR_EXPORT ftkError ftkGetFloat32( ftkLibrary lib, uint64 sn, uint32 optID,
                                   float32* out, ftkOptionGetter what );

/** \brief Set a float option.
 *
 * This function allows to set a floating-point option.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[in] val value to set.
 *
 * \see ftkGetFloat32
 *
 * \retval ftkError::FTK_OK if the value setting could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c out is nullptr,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INV_OPT_VAL if the value is invalid,
 * \retval ftkError::FTK_ERR_INV_OPT if the option does not exist, or is not of
 * type \c float32,
 * \retval ftkError::FTK_ERR_READ if the option cannot be written to the
 * device,
 * \retval ftkError::FTK_WAR_OPT_VAL_RANGE if the set value is outside the
 * allowed range and has therefore been cropped,
 * \retval ftkError::FTK_WAR_OPT_NO_OP if the set option has currently no
 * effect.
 */
ATR_EXPORT ftkError ftkSetFloat32( ftkLibrary lib, uint64 sn, uint32 optID,
                                   float32 val );

/** \brief Get a binary option.
 *
 * This function allows to get the value of a binary option. Note that only the
 * current value can be retrieved, there are no min/max or default values for
 * binary options.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[out] bufferOut pointer on the output buffer, contains the data and
 * the actual data size.
 *
 * \see ftkSetData
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c dataOut or \c dataSizeInBytes is \c nullptr,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INTERNAL if the needed file cannot be read,
 * \retval ftkError::FTK_INV_OPT_PAR if the wanted register cannot be read or
 * does not exist (invalid address).
 * \retval ftkError::FTK_ERR_READ if the option cannot be read from the device.
 */
ATR_EXPORT ftkError ftkGetData( ftkLibrary lib, uint64 sn, uint32 optID,
                                ftkBuffer* bufferOut );

/** \brief Set a binary option.
 *
 * This function allows to set an binary option.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, \c 0uLL for general purpose options).
 * \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions).
 * \param[in] bufferIn  pointer on the input buffer, contains the data and the
 * actual data size.
 *
 * \see ftkGetData
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INV_PTR the \c lib handle was not correctly
 * initialised or if \c dataOut or \c dataSizeInBytes is null,
 * \retval ftkError::FTK_ERR_INV_SN if the device could not be retrieved,
 * \retval ftkError::FTK_ERR_INTERNAL if a memory allocation error occurred or
 * if the wanted file cannot be written,
 * \retval ftkError::FTK_INV_OPT_PAR if the wanted register cannot be read or
 * does not exits (invalid address),
 * \retval ftkError::FTK_ERR_INV_OPT_VAL if the value cannot be set,
 * \retval ftkError::FTK_ERR_READ if the option cannot be written to the
 * device,
 * \retval ftkError::FTK_ERR_WRITE if the environment cannot be saved because
 * the file cannot be opened,
 * \retval ftkError::FTK_ERR_VERSION when loading an environment, if the SDK or
 * firwmare version used when saving the environment is different from the SDk
 * or firmware version used when loading the environment,
 * \retval ftkError::FTK_WAR_OPT_NO_OP if the set option has currently no
 * effect.
 */
ATR_EXPORT ftkError ftkSetData( ftkLibrary lib, uint64 sn, uint32 optID,
                                ftkBuffer* bufferIn );

/** \brief Function combining two 2D points into a 3D point.
 *
 * From This function compute the x-y-z coordinates of 2 known 2D points.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in] leftPixel a ftk3DPoint from the left camera with z equals to 0.
 * \param[in] rightPixel a ftk3DPoint from the right camera with z equals to 0.
 * \param[out] outPoint pointer on the triangulated point, with x-y-z
 * coordinates.
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INTERNAL if this function can not triangulate.
 */
ATR_EXPORT ftkError ftkTriangulate( ftkLibrary lib, uint64 sn,
                                    const ftk3DPoint* leftPixel,
                                    const ftk3DPoint* rightPixel,
                                    ftk3DPoint* outPoint );

/** \brief Fcuntion projecting a 3D point to two 2D points.
 *
 * This function projects 1 3D point to the 2 camera planes.
 *
 * \param[in] lib an initialised library handle.
 * \param[in] sn a valid serial number of the device.
 * \param[in] 1 ftk3DPoint with x-y-z coordinates.
 * \param[out] outLeftData pointer on the left projected point, with x-y-0
 * coordinates.
 * \param[out] outRightData pointer on the right projected point, with x-y-0
 * coordinates.
 *
 * \retval ftkError::FTK_OK if the value retrieval could be done correctly,
 * \retval ftkError::FTK_ERR_INTERNAL if this function can not project on the
 * left or right cam.
 */
ATR_EXPORT ftkError ftkReprojectPoint( ftkLibrary lib, uint64 sn,
                                       const ftk3DPoint* inPoint,
                                       ftk3DPoint* outLeftData,
                                       ftk3DPoint* outRightData );

/* \} */

#ifdef GLOBAL_DOXYGEN
/**
 * \}
 */
#endif

#endif
