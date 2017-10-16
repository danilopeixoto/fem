// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_CONFIGURATION_H
#define OPENTISSUE_CONFIGURATION_H

#if (_MSC_VER >= 1200)
#pragma once
#pragma warning(default : 56 61 62 191 263 264 265 287 289 296 347 529 686)
#pragma warning(disable : 503)
#endif

#ifdef WIN32
#define _USE_MATH_DEFINES
#define NOMINMAX
#include <Windows.h>
#endif

#define OPENTISSUE_VERSION 0.994
#define OPENTISSUE_VERSION_MAJOR 0
#define OPENTISSUE_VERSION_MINOR 994

#endif