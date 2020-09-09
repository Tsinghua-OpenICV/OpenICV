
#include <common.h>

std::ofstream outfile;


class transformMainTenance{
    public:
        transformMainTenance(){
             memset(transformSum,0,sizeof(transformSum)); memset(transformIncre,0,sizeof(transformIncre)); memset(transformMapped,0,sizeof(transformMapped));
              memset(transformBefMapped,0,sizeof(transformBefMapped)); memset(transformAftMapped,0,sizeof(transformAftMapped));

        }

        maintenanceBack transformRecorder( laserOdometryBack laserOdometryBackValue, laserMappingBack laserMappingBackValue){
           
           
           transformSum[0]=laserOdometryBackValue.transformSum[0];
           transformSum[1]=laserOdometryBackValue.transformSum[1];
           transformSum[2]=laserOdometryBackValue.transformSum[2];
           transformSum[3]=laserOdometryBackValue.transformSum[3];
           transformSum[4]=laserOdometryBackValue.transformSum[4];
           transformSum[5]=laserOdometryBackValue.transformSum[5];
           

           transformAftMapped[0] = laserMappingBackValue.transformAftMapped[0];
           transformAftMapped[1] = laserMappingBackValue.transformAftMapped[1];
           transformAftMapped[2] = laserMappingBackValue.transformAftMapped[2];
           transformAftMapped[3] = laserMappingBackValue.transformAftMapped[3];
           transformAftMapped[4] = laserMappingBackValue.transformAftMapped[4];
           transformAftMapped[5] = laserMappingBackValue.transformAftMapped[5];
           
           
           maintenanceBack maintenanceBackValue;
           transformAssociateToMap();
           maintenanceBackValue.transformMapped[0] = transformMapped[0];
           maintenanceBackValue.transformMapped[1] = transformMapped[1];
           maintenanceBackValue.transformMapped[2] = transformMapped[2];
           maintenanceBackValue.transformMapped[3] = transformMapped[3];
           maintenanceBackValue.transformMapped[4] = transformMapped[4];
           maintenanceBackValue.transformMapped[5] = transformMapped[5];
            return maintenanceBackValue;



        }



    private:
        float transformSum[6] ;
        float transformIncre[6] ;
        float transformMapped[6] ;
        float transformBefMapped[6] ;
        float transformAftMapped[6] ;


        void transformAssociateToMap()
        {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                    - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                    - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                    - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                    + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                    + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                    + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                    - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                    + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                    + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                    - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                    + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                    crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                    crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                            - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                            - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
        }

     



};








