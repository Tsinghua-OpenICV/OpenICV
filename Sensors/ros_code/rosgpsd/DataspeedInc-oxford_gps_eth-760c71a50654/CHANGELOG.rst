^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oxford_gps_eth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-09-07)
------------------
* Use C++11 or newer
* Publish a sensor_msgs/TimeReference message with GPS time converted to UTC
* Publish a position type string more detailed than the NavSatStatus in the fix message
* Added UTM position and heading to odometry output; Odometry twist now in local frame
* Contributors: Micho Radovnikovich, Kevin Hallenbeck

0.0.6 (2018-03-26)
------------------
* Changed default listen address from broadcast to any
* Added unit tests and rostests
* Added launch file
* Contributors: Kevin Hallenbeck

0.0.5 (2017-08-10)
------------------
* Fixed velocity utm east-north-up orientation
* Properly handle unknown covariance and fields that are not present
* Updated license for year 2017
* Updated package.xml format to version 2
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

0.0.4 (2016-04-11)
------------------
* Updated license year for 2016
* Contributors: Kevin Hallenbeck

0.0.3 (2015-12-21)
------------------
* Added fix for Ubuntu Saucy
* Contributors: Kevin Hallenbeck

0.0.2 (2015-12-14)
------------------
* Added fix for Ubuntu Saucy
* Contributors: Kevin Hallenbeck

0.0.1 (2015-12-10)
------------------
* Ready for public release
* Contributors: Kevin Hallenbeck
