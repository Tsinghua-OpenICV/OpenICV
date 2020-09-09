typedef struct
{
 bool ACC_Optischer_Hinweis; //23|1@1+ (1,0) [0|0] "" Vector__XXX
 bool Akustik_Hinweis; //22|1@1+ (1,0) [0|0] "" Vector__XXX
 bool Lenkwinkel_VZ; //5|1@1+ (1,0) [0|0] "" Vector__XXX
 float Lenkwinkel; //40|13@1+ (0.1,0) [0|819.1] "" Vector__XXX
 bool PLA_aktiv; // 2|1@1+ (1,0) [0|0] "" Vector__XXX
 bool HCA_Lenkmom_VZ; //4|1@1+ (1,0) [0|0] "" Vector__XXX
 float HCA_Lenkmom_Anf; //24|9@1+ (0.01,0) [0|5.11] "" Vector__XXX
 bool HCA_aktiv; //1|1@1+ (1,0) [0|0] "" Vector__XXX
 float ACC_Beschleunigung_Anf; //8|11@1+ (0.005,-7.22) [-7.22|3.005] "" Vector__XXX
 bool ACC_aktiv; // 0|1@1+ (1,0) [-7.22|4.25] "" Vector__XXX
}GL1000Message;

typedef struct
{
 bool LWI_QBit_Lenkradwinkel; //: 15|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float LWI_Lenkradwinkel; //: 16|13@1+ (0.1,0) [0|800] "Unit_DegreOfArc"  Frontradar,MQB_MFK_2
 bool LWI_VZ_Lenkradwinkel; //: 29|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}SteeringWheelAngleMessage;