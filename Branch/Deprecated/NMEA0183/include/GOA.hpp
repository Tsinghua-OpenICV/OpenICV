#if ! defined( GOA_CLASS_HEADER )

#define GOA_CLASS_HEADER

/*
** Author: Samuel R. Blackburn
** Internet: sam_blackburn@pobox.com
**
** You can use it any way you like as long as you don't try to sell it.
**
** Copyright, 1996, Samuel R. Blackburn
**
** $Workfile: goa.hpp $
** $Revision: 4 $
** $Modtime: 10/10/98 4:46p $
*/

/*
** This Sentence Not Recommended For New Designs
** A combination of WPL, GLL, ZDA and ZTG is recommended.
*/

class GOA : public WAYPOINT_LOCATION
{
   public:

      GOA();
      virtual ~GOA();
};

#endif // GOA_CLASS_HEADER
