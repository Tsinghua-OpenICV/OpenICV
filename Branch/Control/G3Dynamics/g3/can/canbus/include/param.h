/**
 * @file param.h
 * @brief ros get yaml file param.
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-11-05
 */

#ifndef __PARAM_H_
#define __PARAM_H_

//#include <iostream>
#include "ros/ros.h"

/**
 *  *  *  * @brief for to get config parameter from .yaml file.
 *   *   *   *
 *    *    *    * @tparam T
 *     *     *     * @param name
 *      *      *      * @param default
 *       *       *       *
 *        *        *        * @return related config parameter. 
 *         *         *         */

template < class T > T getParam (const std::string & name, const T & defaultValue)
{
  T v;
  if (ros::param::get (name, v))	// get paramater by name depend on ROS.
    {
      ROS_INFO_STREAM ("Found parameter: " << name << ", \tvalue: " << v);
      return v;
    }
  else
    ROS_WARN_STREAM ("Cannot find value for parameter: " << name <<
		     ", \tassigning default: " << defaultValue);
  return defaultValue;		// if the param haven't set, set the default value.

}

#endif
