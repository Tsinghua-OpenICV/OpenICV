#if ! defined( GDP_CLASS_HEADER )

#define GDP_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: gdp.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:49p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/

class GDP : public WAYPOINT_LOCATION
{
   public:

      GDP();
      virtual ~GDP();
};

#endif // GDP_CLASS_HEADER
