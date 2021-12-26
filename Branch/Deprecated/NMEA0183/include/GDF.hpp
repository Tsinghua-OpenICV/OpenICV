#if ! defined( GDF_CLASS_HEADER )

#define GDF_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gdf.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:39p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/

class GDF : public WAYPOINT_LOCATION
{
   public:

      GDF();
      virtual ~GDF();
};

#endif // GDF_CLASS_HEADER
