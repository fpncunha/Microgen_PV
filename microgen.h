#ifndef microgen_h
#define microgen_h

typedef struct{
    int DCBUS;
    int VPV;
    int IPV;
    int PWR;
}pv2rpi;




typedef struct{
    int APOWER;
    //uint16_t RPOWER;
    //uint16_t dcLINK;
    //uint16_t urede;
    //uint16_t check_sum;
    //uint16_t VDCG;
    //uint16_t IDG;
    //uint16_t IQG;
    //uint16_t APWG;
    //uint16_t RPWG;
    //uint16_t FREQG;
    //uint16_t DCREF;
    //uint16_t IQREF;
    //uint16_t STMG;
}grid2rpi;


typedef struct{
    int dcLINK_ref;
	//uint16_t iq_ref;
	//uint16_t status;  
}rpi2grid;


typedef struct{
    int BYTE1;   //#VDCB#04#
    int BYTE2;   //#IDCB#04#
    //int16_t BYTE3;   //#VBTB#04#
    //int16_t BYTE4;   //#STMB#04#
    //int16_t BYTE5;   //#SOCB#04#
    //int16_t BYTE6;   //
    //int16_t BYTE7;   //
    //int16_t BYTE8;   //
}bat2rpi;


typedef struct{
    int BYTE1;   //#IDCB#04#
    //uint16_t BYTE2;   //#CHAR#04# = 1; #DISC#04# = 2; #STOP#04# = 0
    //uint16_t BYTE3;   //#ENK1#04# = 200; #DIK1#04# = 10;
    //uint16_t BYTE4;   //#ENK2#04# = 200; #DIK2#04# = 10;
    //uint16_t BYTE5;   //#ENK3#04# = 200; #DIK3#04# = 10;
    //uint16_t BYTE6;   //
    //uint16_t BYTE7;   //
    //uint16_t BYTE8;   //#ENAB#04#
}rpi2bat;


#endif

