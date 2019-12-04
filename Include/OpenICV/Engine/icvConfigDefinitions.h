#ifndef icvConfigDefinitions_h
#define icvConfigDefinitions_h

#include "OpenICV/Engine/icvConfigIniParser.h"
#include "OpenICV/Engine/icvConfigJsonParser.h"

namespace icv { namespace engine
{
    /********** Preserved keys **********/
    namespace _keys
    {
        ICV_CONSTEXPR char typeKey[] = "type";
        ICV_CONSTEXPR char functionKey[] = "function";
        ICV_CONSTEXPR char nameKey[] = "name";
        ICV_CONSTEXPR char inputKey[] = "input";
        ICV_CONSTEXPR char outputKey[] = "output";
        ICV_CONSTEXPR char connectKey[] = "connect";
        ICV_CONSTEXPR char portSplitChar = '@';
        ICV_CONSTEXPR char portSplitpoint = '.';

    }
}}

#endif // icvConfigDefinitions_h
