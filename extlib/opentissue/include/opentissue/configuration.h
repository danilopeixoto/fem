// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_CONFIGURATIOM_H
#define OPENTISSUE_CONFIGURATIOM_H

#if (_MSC_VER >= 1200)
#pragma once
#pragma warning(default : 56 61 62 191 263 264 265 287 289 296 347 529 686)
#pragma warning(disable : 503)
#endif

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#define _USE_MATH_DEFINES
#define NOMINMAX
#include <windows.h>
#undef WIN32_LEAN_AND_MEAN
#undef NOMINMAX
#endif

#define OPENTISSUE_VERSION 0.994
#define OPENTISSUE_VERSION_MAJOR 0
#define OPENTISSUE_VERSION_MINOR 994

#endif