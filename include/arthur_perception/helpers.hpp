// ============================================================================
/*!
*
*   This file is part of the ATRACSYS fusionTrack library.
*   Copyright (C) 2003-2018 by Atracsys LLC. All rights reserved.
*
*   \file helpers.hpp
*   \brief Helping functions used by sample applications
*
*/
// ============================================================================

#include <ftkInterface.h>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

/** \addtogroup Platform dependent functions
* \{
*/

/** \brief is the software having its own console?
*
* This function is used to know if the program was launch from an explorer.
* It needs to be called in main() before printing to stdout
* See http://support.microsoft.com/kb/99115
*
* \return true if program is in its own console (cursor at 0,0) or
*         false if it was launched from an existing console
*/
bool isLaunchedFromExplorer();

/** \brief Function waiting for a keyboard hit.
*
* This function allows to detect a hit on (almost) any key. Known non-detected
* keys are:
*  - shift;
*  - Caps lock;
*  - Ctrl;
*  - Alt;
*  - Alt Gr.
* The function blocks and only returns when a `valid' key is stroke.
*/
void waitForKeyboardHit();

/** \brief Function pausing the current execution process / thread.
*
* This function stops the current execution thread / process for at least
* the given amount of time.
*
* \param[in] ms amount of time to wait, in milliseconds.
*/
inline void sleep( long ms )
{
    std::this_thread::sleep_for( std::chrono::milliseconds( ms ) );
}


/**
* \}
*/

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

/** \brief Function displaying error messages on the standard error output.
*
* This function displays an error message and exits the current process. The
* value returned by the program is 1. On windows, the user is asked to press
* a key to exit.
*
* \param[in] message error message to print.
* \param[in] dontWaitForKeyboard, setting to true to avoid being prompted before exiting program
*/
inline void error( const char* message, bool dontWaitForKeyboard = false )
{
    std::cerr << "ERROR: " << message << std::endl;

#ifdef ATR_WIN
    if ( ! dontWaitForKeyboard )
    {
        std::cout << "Press the \"ANY\" key to quit" << std::endl;
        waitForKeyboardHit();
    }
#endif

    exit( 1 );
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

/** \brief Helper struct holding a device serial number and its type.
*/
struct DeviceData
{
    uint64 SerialNumber;
    ftkDeviceType Type;
};

/** \brief Callback function for devices.
*
* This function assigns the serial number to the \c user argument, so that the
* device serial number is retrieved.
*
* \param[in] sn serial number of the discovered device.
* \param[out] user pointer on a DeviceData instance where the information will
* be copied.
*/
inline void deviceEnumerator( uint64 sn, void* user, ftkDeviceType type )
{
    if ( user != 0 )
    {
        DeviceData* ptr = reinterpret_cast< DeviceData* >( user );
        ptr->SerialNumber = sn;
        ptr->Type = type;
    }
}

/** \brief Callback function for fusionTrack devices.
*
* This function assigns the serial number to the \c user argument, so that the
* device serial number is retrieved. If the enumerated device is a simulator,
* the device is considered as not detected.
*
* \param[in] sn serial number of the discovered device.
* \param[out] user pointer on the location where the serial will be copied.
* \param[in] devType type of the device.
*/
inline void fusionTrackEnumerator( uint64 sn, void* user,
    ftkDeviceType devType )
{
    if ( user != 0 )
    {
        if ( devType != ftkDeviceType::DEV_SIMULATOR )
        {
            DeviceData* ptr = reinterpret_cast< DeviceData* >( user );
            ptr->SerialNumber = sn;
            ptr->Type = devType;
        }
        else
        {
            std::cerr
                << "ERROR: This sample cannot be used with the simulator"
                << std::endl;
            DeviceData* ptr = reinterpret_cast< DeviceData* >( user );
            ptr->SerialNumber = 0u;
            ptr->Type = ftkDeviceType::DEV_UNKNOWN_DEVICE;
        }
    }
}

/** \brief Function exiting the program after displaying the error.
*
* This function retrieves the last error and displays the corresponding
* string in the terminal.
*
* The typical usage is:
* \code
* ftkError err( ... );
* if ( err != FTK_OK )
* {
*     checkError( lib );
* }
* \endcode
*
* \param[in] lib library handle.
*/
inline void checkError( ftkLibrary lib, bool dontWaitForKeyboard = false,  bool quit = true )
{
    char message[ 1024u ];
    ftkError err( ftkGetLastErrorString( lib, 1024u, message ) );
    if ( err == ftkError::FTK_OK )
    {
        std::cerr << message << std::endl;
    }
    else
    {
        std::cerr << "Uninitialised library handle provided" << std::endl;
    }

    if ( quit )
    {
#ifdef ATR_WIN
    if ( ! dontWaitForKeyboard )
    {
        std::cout << "Press the \"ANY\" key to exit" << std::endl;
        waitForKeyboardHit();
    }
#endif
        exit( 1 );
    }
}

/** \brief Function enumerating the devices and keeping the last one.
*
* This function uses the ftkEnumerateDevices library function and the
* deviceEnumerator callback so that the last discovered device is kept.
*
* If no device is discovered, the execution is stopped by a call to error();
*
* \param[in] lib initialised library handle.
* \param[in] allowSimulator setting to \c false requires to discover only real
* \param[in] quiet setting to \c true to disactivate printouts
* \param[in] dontWaitForKeyboard, setting to true to avoid being prompted before exiting program
*
* devices (i.e. the simulator device is not retrieved).
*
* \return the serial number of the discovered device.
*/
inline DeviceData retrieveLastDevice( ftkLibrary lib, bool allowSimulator = true, bool quiet = false, bool dontWaitForKeyboard = false )
{
    DeviceData device;
    device.SerialNumber = 0uLL;
    // Scan for devices
    ftkError err( ftkError::FTK_OK );
    if ( allowSimulator )
    {
        err = ftkEnumerateDevices( lib, deviceEnumerator, &device );
    }
    else
    {
        err = ftkEnumerateDevices( lib, fusionTrackEnumerator, &device );
    }

    if ( err > ftkError::FTK_OK )
    {
        checkError( lib, dontWaitForKeyboard );
    }
    else if ( err < ftkError::FTK_OK )
    {
        if ( ! quiet )
            checkError( lib, dontWaitForKeyboard, false );
    }

    if ( device.SerialNumber == 0uLL )
    {
        error( "No device connected", dontWaitForKeyboard );
    }
    std::string text;
    switch ( device.Type )
    {
    case ftkDeviceType::DEV_SPRYTRACK_180:
        text = "sTk 180";
        break;
    case ftkDeviceType::DEV_FUSIONTRACK_500:
        text = "fTk 500";
        break;
    case ftkDeviceType::DEV_FUSIONTRACK_250:
        text = "fTk 250";
        break;
    case ftkDeviceType::DEV_SIMULATOR:
        text = "fTk simulator";
        break;
    default:
        text = " UNKNOWN";
        error( "Unknown type", dontWaitForKeyboard );
    }
    if ( ! quiet )
    {
        std::cout << "Detected one " << text;

        std::cout << " with serial number 0x" << std::setw( 16u )
            << std::setfill( '0' ) << std::hex << device.SerialNumber << std::dec
            << std::endl << std::setfill( '\0' );
    }

    return device;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

/** \brief Helper class reading the ftkError string.
*
* This class provides an API to interpret the error string gotten from
* ftkGetLastErrorString.
*
* \code
* char tmp[ 1024u ];
* if ( ftkGetLastErrorString( lib, 1024u, tmp ) == FTK_OK )
* {
*     ErrorReader reader;
*     if ( ! reader.parse( tmp ) )
*     {
*         cerr << "Cannot interpret received error:" << endl << tmp << endl;
*     }
*     else
*     {
*         if ( reader.hasError( FTK_ERR_INTERNAL ) )
*         {
*             cout << "Internal error" << endl;
*         }
*         // ...
*     }
* }
* \endcode
*/
class ErrorReader
{
public:
    /** \brief Default constructor.
    */
    ErrorReader();
    /** \brief Destructor, does nothing fancy.
    */
    ~ErrorReader();
    /** \brief Parsing method.
    *
    * This method parses the error string. It is not a XML parser, as the
    * error syntax is very simple. It extracts the errors, warnings and other
    * messages from the provided string.
    *
    * Syntax errors are reported on std::cerr.
    *
    * \param[in] str string to be parsed.
    *
    * \retval true if the parsing could be done successfully,
    * \retval false if an error occurred,
    */
    bool parseErrorString( const std::string& str );
    /** \brief Getter for a given error.
    *
    * This method checks whether is the given error was flagged.
    *
    * \param[in] err error code to be checked.
    *
    * \retval true if the given error was flagged in the XML string,
    * \retval false if \c err is a warning, if no errors were flagged or if
    * \c err is not found in the flagged errors.
    */
    bool hasError( ftkError err ) const;
    /** \brief Getter for a given warning.
    *
    * This method checks whether is the given warning was flagged.
    *
    * \param[in] err warning code to be checked.
    *
    * \retval true if the given warning was flagged in the XML string,
    * \retval false if \c war is an error, if no warning were flagged or if
    * \c war is not found in the flagged errors.
    */
    bool hasWarning( ftkError war ) const;
    /** \brief Getter for OK status.
    *
    * This method checks if no errors or warnings are flagged.
    *
    * \retval true if no errors and no warnings are flagged,
    * \retval false if at least one error or one warning was flagged.
    */
    bool isOk() const;
private:
    std::string _ErrorString;
    std::string _WarningString;
    std::string _StackMessage;
};

inline ErrorReader::ErrorReader()
    : _ErrorString( "" )
    , _WarningString( "" )
    , _StackMessage( "" )
{}

inline ErrorReader::~ErrorReader()
{}

inline bool ErrorReader::parseErrorString( const std::string& str )
{
    if ( str.find( "<ftkError>" ) == std::string::npos ||
        str.find( "</ftkError>" ) == std::string::npos )
    {
        std::cerr << "Cannot find root element <ftkError>" << std::endl;
        return false;
    }

    size_t locBegin, locEnd;

    std::vector< std::string > what;
    what.push_back( "errors" );
    what.push_back( "warnings" );
    what.push_back( "messages" );

    std::string* pseudoRef( 0 );

    for ( std::vector< std::string >::const_iterator tagIt( what.begin() );
        tagIt != what.end(); ++tagIt )
    {
        if ( *tagIt == "errors" )
        {
            pseudoRef = &_ErrorString;
        }
        else if ( *tagIt == "warnings" )
        {
            pseudoRef = &_WarningString;
        }
        else if ( *tagIt == "messages" )
        {
            pseudoRef = &_StackMessage;
        }
        if ( str.find( "<" + *tagIt + " />" ) == std::string::npos )
        {
            locBegin = str.find( "<" + *tagIt + ">" );
            locEnd = str.find( "</" + *tagIt + ">" );
            if ( locBegin != std::string::npos &&
                locEnd != std::string::npos )
            {
                *pseudoRef = str.substr( locBegin + std::string( "<" + *tagIt + ">" ).size(),
                    locEnd - locBegin - std::string( "<" + *tagIt +  ">" ).size() );
            }
            else
            {
                std::cerr << "Cannot interpret <" << *tagIt << ">" << std::endl;
                return false;
            }
        }
        else
        {
            *pseudoRef = "";
        }

        locBegin = pseudoRef->find( "No errors" );
        if ( locBegin != std::string::npos )
        {
            *pseudoRef = pseudoRef->replace( locBegin, locBegin + strlen( "No errors" ), "" );
        }
    }

    return true;
}

inline bool ErrorReader::hasError( ftkError err ) const
{
    if ( err <= ftkError::FTK_OK )
    {
        return false;
    }
    else if ( _ErrorString.empty() )
    {
        return false;
    }

    std::stringstream convert;
    convert << int32( err ) << ":";

    return ( _ErrorString.find( convert.str() ) != std::string::npos );
}

inline bool ErrorReader::hasWarning( ftkError war ) const
{
    if ( war >= ftkError::FTK_OK )
    {
        return false;
    }
    else if ( _WarningString.empty() )
    {
        return false;
    }

    std::stringstream convert;
    convert << int32( war ) << ":";

    return ( _WarningString.find( convert.str() ) != std::string::npos );
}

inline bool ErrorReader::isOk() const
{
    return _ErrorString.empty() && _WarningString.empty();
}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------