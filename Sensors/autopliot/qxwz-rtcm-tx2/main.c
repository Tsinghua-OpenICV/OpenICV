/**
 * RTCM SDK Demo
 * Copyright (C) 2015-2017 by QXSI, All rights reserved.
 */

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#include "qxwz_rtcm.h"

#undef QXLOGI
#define QXLOGI printf

qxwz_account_info *p_account_info = NULL;
qxwz_config config;
qxwz_rtcm_dataTrans data_new;

void get_qxwz_sdk_account_info(void) {
    p_account_info = getqxwzAccount();
    if(p_account_info->appkey != NULL) {
        printf("appkey=%s\n",p_account_info->appkey);
    }
    if(p_account_info->deviceID != NULL) {
        printf("deviceID=%s\n",p_account_info->deviceID);
    }
    if(p_account_info->deviceType != NULL) {
        printf("deviceType=%s\n",p_account_info->deviceType);
    }
#if _ENABLE_CHISHUI
    if(p_account_info->dsk != NULL) {
        printf("dsk=%s\n",p_account_info->dsk);
    }
#else
    if(p_account_info->NtripUserName != NULL) {
        printf("dsk=%s\n",p_account_info->NtripUserName);
    }
#endif
    printf("expire_time=%ld\n",p_account_info->expire_time);
}

/**
 * Callback to fill rtcm data. The rtcm data is a binary stream.
 * User can send rtcm data into GPS chip according the length returned
 * in the structure of qxwz_rtcm.
 */
void qxwz_rtcm_response_callback(qxwz_rtcm data) {
    QXLOGI("QXWZ_RTCM_DATA_LEN:%ld\n",data.length);
    QXLOGI("QXWZ_RTCM_DATA:");
    if (!data.buffer) {
        QXLOGI("no rtcm data\n");
        return;
    }

	data_new.length = data.length;
	
    
    int idx = 0;
    for (idx = 0; idx < data.length; idx++) {
	data_new.buffer[idx] = data.buffer[idx];
        if (idx % 32 == 0)
            QXLOGI("\n");
        //QXLOGI("%02x ", (unsigned char)(data.buffer[idx]));
    }
    QXLOGI("\n");
}

#if _ACTIVE_ACCOUNT
void qxwz_active_account_cb(qxwz_rtcm_status code, const char* msg) {
    if (code == QXWZ_STATUS_ACCOUNT_IS_ACTIVATED) {
        QXLOGI("active account:%d, msg,%s\n",code, msg);
    } else if (code == QXWZ_STATUS_ACCOUNT_ACTIVATE_FAILURE) {
        QXLOGI("active accoun:%d, msg,%s\n",code, msg);
    }
}
#endif

void qxwz_status_response_callback(qxwz_rtcm_status code) {
    QXLOGI("QXWZ_RTCM_STATUS:%d\n",code);
    int ret = 0;
	//test account expire
	if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE) {
		get_qxwz_sdk_account_info();
	}else if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED) {
		get_qxwz_sdk_account_info();
    }
#if _ACTIVE_ACCOUNT
    else if(code == QXWZ_STATUS_OPENAPI_DISABLED_ACCOUNT) {
        ret = qxwz_activeAccount(qxwz_active_account_cb, config.deviceId, config.deviceType);
        QXLOGI("active account result:%d\n", ret);
    }
#endif
}

int main(int argc, const char * argv[]) {
    //ÉèÖÃappKeyºÍappSecret
    //apapKeyÉêÇëÏêÏ¸¼ûËµÃ÷ÎÄµµ
    config.appkey="751454";
    config.appSecret="80869e2424b3f1ada803c3f93f90febf63bc2a1e94ff44e00d97b07c032632ba";
    config.deviceId="wbd05";
    config.deviceType="wbd";

    //[1] Set sdk configs
    qxwz_setting(&config);
    //[2] Start rtcm sdk
    qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);

    //[3] Send gga every second, gga is hard code in this demo
    int i;
    for (i = 0; i < 120000; i++) {
        //qxwz_rtcm_sendGGAWithGGAString("$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B\r\n");
        qxwz_rtcm_sendGGAWithGGAString("$GPGGA,053323.10,3959.73322770,N,11619.35729550,E,4,20,0.9,57.5650,M,-9.2928,M,02,2964*48\r\n");
    int idx = 0;
    for (idx = 0; idx < data_new.length; idx++) {
        if (idx % 32 == 0)
            QXLOGI("\n");
        QXLOGI("%02x ", (unsigned char)(data_new.buffer[idx]));
    }
    QXLOGI("\n");

        QXLOGI("Send GGA done\r\n");
        sleep(1);
    }
    QXLOGI("qxwz_rtcm_stop here\r\n");
    //[4] Stop rtcm sdk
    qxwz_rtcm_stop();
    QXLOGI("qxwz_rtcm_stop done\r\n");
    return 0;
}

