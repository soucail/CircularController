#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CircularController_DLLIMPORT __declspec(dllimport)
#  define CircularController_DLLEXPORT __declspec(dllexport)
#  define CircularController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CircularController_DLLIMPORT __attribute__((visibility("default")))
#    define CircularController_DLLEXPORT __attribute__((visibility("default")))
#    define CircularController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CircularController_DLLIMPORT
#    define CircularController_DLLEXPORT
#    define CircularController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CircularController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CircularController_DLLAPI
#  define CircularController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CircularController_EXPORTS
#    define CircularController_DLLAPI CircularController_DLLEXPORT
#  else
#    define CircularController_DLLAPI CircularController_DLLIMPORT
#  endif // CircularController_EXPORTS
#  define CircularController_LOCAL CircularController_DLLLOCAL
#endif // CircularController_STATIC