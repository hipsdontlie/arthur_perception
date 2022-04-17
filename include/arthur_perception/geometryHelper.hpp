// ============================================================================
/*!
 *
 *   This file is part of the ATRACSYS fusionTrack library.
 *   Copyright (C) 2003-2018 by Atracsys LLC. All rights reserved.
 *
 *   \file geometryHelper.hpp
 *   \brief Helping functions for geometry manipulation.
 *
 */
// ============================================================================

#ifndef GEOMETRYHELPER_HPP
#define GEOMETRYHELPER_HPP

#include <ftkInterface.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <new>
#include <string>

bool loadFile( std::ifstream& is, ftkGeometry& geometry );
/** \brief Helper function loading a geometry.
 *
 * \param[in] fileName name of the file to load (file name only, \e no
 * directory information!).
 * \param[out] geometry instance of ftkGeometry holding the parameters.
 *
 * \retval 0 if everything went fine,
 * \retval 1 if the data were loaded from the system directory (windows only),
 * \retval 2 if the data could not be loaded.
 */
inline int loadGeometry( ftkLibrary lib, const uint64& sn,
                         const std::string& fileName, ftkGeometry& geometry )
{
    std::ifstream input;

    input.open( fileName.c_str() );

    if ( ! input.fail() && loadFile( input, geometry ) )
    {
        return 0;
    }
    else
    {
        ftkBuffer buffer;
        buffer.reset();
        if ( ftkGetData( lib, sn, FTK_OPT_DATA_DIR,
                         &buffer ) != ftkError::FTK_OK ||
             buffer.size < 1u )
        {
            return 2;
        }

        std::string fullFile( buffer.data, buffer.size );
        fullFile += "\\" + fileName;

        input.open( fullFile.c_str() );

        if ( ! input.fail() && loadFile( input, geometry ) )
        {
            return 1;
        }
    }

    return 2;
}

// ----------------------------------------------------------------------------

class IniFile
{
protected:
    long findEOL( char& c, char* addr, size_t size )
    {
        for ( size_t u( 0 ); u < size; ++u )
        {
            c = addr[ u ];
            if ( c == 0 || c == '\n' ) // note that MAC may only have a '\r'
            {
                return long( u );
            }
        }
        return -1;
    }
    bool parseLine( std::string& line )
    {
        size_t first_bracket = line.find_first_of( "[" ),
               last_bracket = line.find_last_of( "]" ),
               equal = line.find_first_of( "=" );

        if ( first_bracket != std::string::npos &&
             last_bracket != std::string::npos )
        {
            // Found section
            _currentSection = line.substr( first_bracket + 1,
                                           last_bracket - first_bracket - 1 );
            sections[ _currentSection ] = KeyValues();
        }
        else if ( equal != std::string::npos && _currentSection != "" )
        {
            // Found property in a section
            std::string key = line.substr( 0, equal ),
                        val = line.substr( equal + 1 );
            sections[ _currentSection ][ key ] = val;
        }
        else
        {
            // If the line is empty, just skip it, if not and is a comment, just
            // skip it
            // as well, otherwise the parsing cannot be done.
            line.erase( remove_if( line.begin(),
                                   line.end(), isspace ), line.end() );
            if ( ! line.empty() && line.substr( 0, 1 ) != ";" )
            {
                return false;
            }
        }
        return true;
    }

    std::string _currentSection;

public:

    typedef std::map< std::string, std::string > KeyValues;
    typedef std::map< std::string, KeyValues > Sections;

    Sections sections;
    bool parse( char* addr, size_t size )
    {
        sections.clear();
        _currentSection = "";

        std::string strLine;

        while ( size )
        {
            char c;
            long lineSize = findEOL( c, addr, size );

            if ( lineSize != 0 )
            {
                if ( lineSize > 0 )
                {
                    strLine = std::string( addr, lineSize );
                }
                else
                {
                    strLine = std::string( addr );
                }

                strLine.erase( remove( strLine.begin(),
                                       strLine.end(), '\r' ), strLine.end() );
                if ( ! parseLine( strLine ) )
                {
                    return false;
                }

                if ( lineSize < 0 )
                {
                    return true; // EOF at the end of the line
                }
            }
            if ( c == 0 || size == size_t( lineSize ) )
            {
                return true; // !!! eof not reached
            }
            addr += lineSize + 1;
            size -= lineSize + 1;
        }
        return true;
    } // Return false in case of syntax error
    bool save( std::string str )
    {
        FILE* file = fopen( str.c_str(), "wb" );
        if ( ! file )
        {
            return false;
        }

        Sections::iterator iterS = sections.begin();

        while ( iterS != sections.end() )
        {
            fprintf( file, "[%s]\n", iterS->first.c_str() );

            KeyValues& kw = iterS->second;
            KeyValues::iterator iterK = kw.begin();

            while ( iterK != kw.end() )
            {
                fprintf( file, "%s=%s\n",
                         iterK->first.c_str(), iterK->second.c_str() );
                iterK++;
            }
            iterS++;
        }
        fclose( file );
        return true;
    }
    bool toBuffer( char** out, size_t& outSize )
    {
        if ( ! out )
        {
            return false;
        }

        std::string buffer;
        char temp[ 1024 ];

        Sections::iterator iterS = sections.begin();

        while ( iterS != sections.end() )
        {
            sprintf( temp, "[%s]\n", iterS->first.c_str() );
            buffer += temp;

            KeyValues& kw = iterS->second;
            KeyValues::iterator iterK = kw.begin();

            while ( iterK != kw.end() )
            {
                sprintf( temp, "%s=%s\n",
                         iterK->first.c_str(), iterK->second.c_str() );
                buffer += temp;
                iterK++;
            }
            iterS++;
        }

        outSize = buffer.length() + 1;
        char* str = new ( std::nothrow ) char[ outSize ];

        if ( ! str )
        {
            return false;
        }

        strncpy( str, buffer.c_str(), outSize - 1 );
        str[ outSize - 1 ] = 0;
        *out = str;

        return true;
    } // Buffer must be unallocated by user
};

// ----------------------------------------------------------------------------
inline bool checkSection( IniFile& p, const std::string& section )
{
    if ( p.sections.find( section ) == p.sections.end() )
    {
        std::cerr << "Cannot find section \"" << section << "\"" << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
inline bool checkKey( IniFile&           p,
                      const std::string& section,
                      const std::string& key )
{
    if ( p.sections[ section ].find( key ) == p.sections[ section ].end() )
    {
        std::cerr << "Cannot find key \"" << key << "\" in section \""
                  << section << "\"" << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
inline bool assignUint32( IniFile& p, const std::string& section,
                          const std::string& key,
                          uint32* variable )
{
    if ( ! checkKey( p, section, key ) )
    {
        return false;
    }

    char* pEnd;
    std::string val( p.sections[ section ][ key ] );

    *variable = uint32( strtol( val.c_str(), &pEnd, 10 ) );

    return true;
}

// ----------------------------------------------------------------------------
inline bool assignFloatXX( IniFile& p, const std::string& section,
                           const std::string& key,
                           floatXX* variable )
{
    if ( ! checkKey( p, section, key ) )
    {
        return false;
    }

    char* pEnd;

    *variable =
        floatXX( strtod( p.sections[ section ][ key ].c_str(), &pEnd ) );

    return true;
}

// ----------------------------------------------------------------------------

#define CHECK_SECTION( p, section )     \
    if ( ! checkSection( p, section ) ) \
    {                                   \
        return false;                   \
    }

// ----------------------------------------------------------------------------
inline bool loadFile( std::ifstream& is, ftkGeometry& geometry )
{
    std::string line, fileContent( "" );

    while ( ! is.eof() )
    {
        getline( is, line );
        fileContent += line + "\n";
    }

    IniFile parser;

    if ( ! parser.parse( const_cast< char* >( fileContent.c_str() ),
                         fileContent.size() ) )
    {
        return false;
    }

    CHECK_SECTION( parser, "geometry" );

    uint32 tmp;

    if ( ! assignUint32( parser, "geometry", "count", &tmp ) )
    {
        return false;
    }
    geometry.version = 0u;
    geometry.pointsCount = tmp;
    if ( ! assignUint32( parser, "geometry", "id", &geometry.geometryId ) )
    {
        return false;
    }

    std::cout << "Loading geometry " << geometry.geometryId << ", composed of "
              << geometry.pointsCount << " fiducials" << std::endl;

    char sectionName[ 10u ];

    for ( uint32 i( 0u ); i < geometry.pointsCount; ++i )
    {
        sprintf( sectionName, "fiducial%u", i );

        if ( ! checkSection( parser, sectionName ) )
        {
            return false;
        }

        if ( ! assignFloatXX( parser, sectionName, "x",
                              &geometry.positions[ i ].x ) )
        {
            return false;
        }
        if ( ! assignFloatXX( parser, sectionName, "y",
                              &geometry.positions[ i ].y ) )
        {
            return false;
        }
        if ( ! assignFloatXX( parser, sectionName, "z",
                              &geometry.positions[ i ].z ) )
        {
            return false;
        }

        std::cout << "Loaded fiducial " << i << " ("
                  << geometry.positions[ i ].x << ", "
                  << geometry.positions[ i ].y << ", "
                  << geometry.positions[ i ].z << ")" << std::endl;
    }

    return true;
}

#undef CHECK_SECTION

#endif // GEOMETRYHELPER_HPP
