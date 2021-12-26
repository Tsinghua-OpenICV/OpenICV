/**
 * Copyright (C) 2015-2017 by QXSI, All rights reserved.
 */

#ifndef __QXWZ_RTCM_H
#define __QXWZ_RTCM_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <time.h>

typedef enum {
    QXWZ_STATUS_NTRIP_CONNECTED=1000,//已连接到ntrip服务器
    QXWZ_STATUS_NTRIP_DISCONNECTED=1001,//已断开与ntrip服务器的连接
    QXWZ_STATUS_APPKEY_IDENTIFY_FAIL=1002,//APP KEY认证失败
    QXWZ_STATUS_APPKEY_IDENTIFY_SUCCESS=1003,//APP KEY认证成功
    QXWZ_STATUS_NETWORK_ERROR=1004, //网络异常
    QXWZ_STATUS_NTRIP_MAX_USER=1005, //NTRIP用户达到上限
    QXWZ_STATUS_NTRIP_USER_NO_EXIST=1006, //NTRIP用户不存在
    QXWZ_STATUS_NTRIP_USER_IDENTIFY_SUCCESS=1007, //NTRIP用户认证成功
    QXWZ_STATUS_ILLEGAL_GGA=1011,  //发送的GGA不合法
    QXWZ_STATUS_GGA_SEND_TIMEOUT=1012,  //GGA发送超时
    
    //SDK 0.0.3新增状态码
    QXWZ_STATUS_NTRIP_CONNECTING=1013,  //正在连接ntrip服务器
    QXWZ_STATUS_NTRIP_RECEIVING_DATA=1014,  //正在接收ntrip服务器数据
    QXWZ_STATUS_ILLEGAL_APP_KEY=1015,  //非法APP KEY
    QXWZ_STATUS_ILLEGAL_APP_SECRET=1016,  //非法APP SECRET
    QXWZ_STATUS_ILLEGAL_DEVICE_TYPE=1017,  //非法Device type
    QXWZ_STATUS_ILLEGAL_DEVICE_ID=1018,  //非法Device id
    QXWZ_STATUS_ACQUIRE_NTRIP_USER_FAILURE=1019,  //无法获取差分用户
    QXWZ_STATUS_SDK_INTERNAL_ERROR=1020,  //SDK内部错误
    QXWZ_STATUS_NTRIP_RTCM_SUCCESS=1021,  //Ntrip播发数据正常
    QXWZ_STATUS_NTRIP_UNAUTHORIZED=1022,  //Ntrip认证失败
    QXWZ_STATUS_CONFIG_NULL=1035,  //Config为空
    QXWZ_STATUS_NO_SETTING_INIT_FUNCTION=1036,  //开发者没有调用setting函数
    QXWZ_STATUS_ACCOUNT_IS_ACTIVATED=1051,      //账号激活成功
    QXWZ_STATUS_ACCOUNT_ACTIVATE_FAILURE=1052,  //账号激活失败
    
    //SDK 0.0.5新增状态码
    QXWZ_STATUS_OPENAPI_OK=2000,  //接口调用成功
    QXWZ_STATUS_OPENAPI_PARAM_MISSING=2001,  //缺少参数
    QXWZ_STATUS_OPENAPI_ACCOUNT_NOT_EXIST=2002,  //账号不存在
    QXWZ_STATUS_OPENAPI_DUPLICATE_ACCOUNT=2003,  //账号重复
    QXWZ_STATUS_OPENAPI_INCORRECT_PASSWORD=2004,  //错误密码
    QXWZ_STATUS_OPENAPI_DISABLED_ACCOUNT=2005,  //账号不可用
    QXWZ_STATUS_OPENAPI_NO_AVAILABLE_ACCOUNT=2006,  //没有有效的账号
    QXWZ_STATUS_OPENAPI_NO_RELATED_POPUSER=2007,  //POPUser不存在
    QXWZ_STATUS_OPENAPI_SYSTEM_ERROR=2008,  //服务端内部错误
    QXWZ_STATUS_NTRIP_SERVER_DISCONNECTED=2009,  //Ntrip服务器断开Socket连接
    QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED = 2010, //账号已过期，需续费
    QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE = 2011, //账号即将过期
    QXWZ_STATUS_BIND_MODE_MISMATCH = 2012, //The current account does not automatically binding
}qxwz_rtcm_status;

/**
 *  qxwz_rtcm 数据，包括数据（buffer)，和数据长度(length)
 */
typedef struct{
    char* buffer;
    size_t length;
}qxwz_rtcm;

typedef struct{
    char buffer[1000];
    size_t length;
}RTCM_data_trans;

/**
 *  启动SDK配置信息
 *  deviceId   设备ID号
 *  deviceType 设备类型
 */
typedef struct{
    char * appkey;
    char * appSecret;
    char * deviceId;
    char * deviceType;
}qxwz_config;

/**
 * SDK 账号信息
 */
typedef struct {
	char *appkey;
	char *deviceID;
	char *deviceType;
	time_t expire_time;  /*自1970年1月1日的秒数*/
#if _ENABLE_CHISHUI
	char *dsk;
	char *dss;
#else
    char *NtripUserName;
    char *NtripPassword;
#endif
} qxwz_account_info;

/**
 *  RTCM差分数据回调函数
 *
 *  @param qxwz_rtcm_response
 */
typedef void (*qxwz_rtcm_response)(qxwz_rtcm data);


/**
 *  RTCM服务状态码回调函数
 *
 *  @param qxwz_status_response
 */
typedef void (*qxwz_status_response)(qxwz_rtcm_status code);


/**
 *  配置SDK
 */
void qxwz_setting(qxwz_config* config);

/**
 *  启动rtcm服务
 */
void qxwz_rtcm_start(qxwz_rtcm_response rtcm_rsp, qxwz_status_response status_rsp);


/**
 *  关闭rtcm服务
 */
void qxwz_rtcm_stop();

/**
 *  向ntrip服务器发送GGA字符串用来获取rtcm数据
 *
 *  @param ggastring the string of gga format
 */
void qxwz_rtcm_sendGGAWithGGAString(char* ggastring);



/**
 *  Get account-related information
 *
 *  @return  qxwz_account_info*
 */
qxwz_account_info* getqxwzAccount(void);

#ifdef __cplusplus
}
#endif

#endif /* __QXWZ_RTCM_H */
