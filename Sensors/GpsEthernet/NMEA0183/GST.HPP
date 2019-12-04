#if ! defined( GST_CLASS_HEADER )

#define GST_CLASS_HEADER

/*!
** Auteur : Maria Alwan 
** date : mai 2005
**
** modification : G�rald Dherbomez, octobre 2005
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
  
  std::string UTCTime;
  time_t Time;
  int msec;
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
  virtual bool Parse( const SENTENCE& sentence );
  virtual bool Write( SENTENCE& sentence );
  
  /*
  ** Operators
  */
  
  virtual const GST& operator = ( const GST& source );
};

#endif // GST_CLASS_HEADER
