#if ! defined( GST_CLASS_HEADER )

#define GST_CLASS_HEADER

/*!
** Auteur : Maria Alwan 
** date : mai 2005
**
** modification : Gérald Dherbomez, octobre 2005
** ajout du temps dans la structure
*/

class GST : public RESPONSE
{
public:
  
  GST();
  virtual ~GST();
  
  /*
  ** Data
  */
  
  QString UTCTime;
  QDateTime Time;
  double RMSvalue;
  double ErrorEllipseMajor;
  double ErrorEllipseMinor;
  double ErrorEllipseOrientation;
  double LatitudeError;
  double LongitudeError;
  double HeightError;
  
  /*
  ** Methods
  */
  
  virtual void Empty( void );
  virtual BOOL Parse( const SENTENCE& sentence );
  virtual BOOL Write( SENTENCE& sentence );
  
  /*
  ** Operators
  */
  
  virtual const GST& operator = ( const GST& source );
};

#endif // GST_CLASS_HEADER
