#ifndef XBOWCOMPONENT_H
#define XBOWCOMPONENT_H

// Export macro for XbowComponent DLL for Windows only
#ifdef WIN32
#   ifdef XBOWCOMPONENT_EXPORTS
        // make DLL
#       define XBOWCOMPONENT_API __declspec(dllexport)
#   else
        // use DLL
#       define XBOWCOMPONENT_API __declspec(dllimport)
#   endif
#else
    // On other platforms, simply ignore this 
#   define XBOWCOMPONENT_API /* nothing */
#endif

#endif // XBOWCOMPONENT_H
