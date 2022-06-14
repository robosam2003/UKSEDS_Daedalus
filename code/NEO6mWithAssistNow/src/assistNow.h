//
// Created by Samuel scott (robosam2003) on 30/04/2022.
//

#ifndef NEO6MWITHASSISTNOW_ASSISTNOW_H
#define NEO6MWITHASSISTNOW_ASSISTNOW_H

// class IDs
enum ubxClassIDs {
    NAV = 0x01,
    RXM = 0x02,
    INF = 0x04,
    ACK = 0x05,
    CFG = 0x06,
    MON = 0x0A,
    AID = 0x0B,
    TIM = 0x0D
};

// NAV
enum navID {
    NAV_AOPSTATUS = 0x60,
    NAV_CLOCK = 0x22,
    NAV_DGPS = 0x31,
    NAV_DOP = 0x04,
    NAV_EKFSTATUS = 0x40,
    NAV_POSECEF = 0x01,
    NAV_POSLLH = 0x02,
    NAV_SBAS = 0x32,
    NAV_SOL = 0x06,
    NAV_STATUS = 0x03,
    NAV_SVINFO = 0x30,
    NAV_TIMEGPS = 0x20,
    NAV_TIMEUTC = 0x21,
    NAV_VELECEF = 0x11,
    NAV_VELNED = 0x12
};

enum navIDPayloadLengths {
    NAV_AOPSTATUS_PAYLOAD_LENGTH = 20,
    NAV_CLOCK_PAYLOAD_LENGTH = 20,
    NAV_DGPS_PAYLOAD_LENGTH = 16+12*50, // 16 bytes + 12 bytes per channel - change according to your needs
    NAV_DOP_PAYLOAD_LENGTH = 18,
    NAV_EKFSTATUS_PAYLOAD_LENGTH = 36,
    NAV_POSECEF_PAYLOAD_LENGTH = 20,
    NAV_POSLLH_PAYLOAD_LENGTH = 28,
    NAV_SBAS_PAYLOAD_LENGTH = 12+12*1, // 12 bytes + 12 bytes per "cnt" - not sure what "cnt" is - see https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
    NAV_SOL_PAYLOAD_LENGTH = 52,
    NAV_STATUS_PAYLOAD_LENGTH = 16,
    NAV_SVINFO_PAYLOAD_LENGTH = 8+12*50, // 8 bytes + 12 bytes per channel - change according to your needs
    NAV_TIMEGPS_PAYLOAD_LENGTH = 16,
    NAV_TIMEUTC_PAYLOAD_LENGTH = 20,
    NAV_VELECEF_PAYLOAD_LENGTH = 20,
    NAV_VELNED_PAYLOAD_LENGTH = 36
};

// RXM
enum rxmID {
    RXM_ALM = 0x30,
    RXM_EPH = 0x31,
    RXM_PMREQ = 0x41,
    RXM_RAW = 0x10,
    RXM_SFRB = 0x11,
    RXM_SVSI = 0x20
};

// INF
enum infID {
    INF_DEBUG = 0x04,
    INF_ERROR = 0x00,
    INF_NOTICE = 0x02,
    INF_TEST = 0x03,
    INF_WARNING = 0x01
};

// ACK
enum ACKID {
    ACK_ACK = 0x01,
    ACK_NAK = 0x00
};

// CFG
enum cfgID {
    CFG_ANT = 0x13,
    CFG_CFG = 0x09,
    CFG_DAT = 0x06,
    CFG_EKF = 0x12,
    CFG_ESFGWT = 0x29,
    CFG_FXN = 0x0E,
    CFG_INF = 0x02,
    CFG_ITFM = 0x39,
    CFG_MSG = 0x01,
    CFG_NAV5 = 0x24,
    CFG_NAVX5 = 0x23,
    CFG_NMEA = 0x17,
    CFG_PM2 = 0x3B,
    CFG_PM = 0x32,
    CFG_PRT = 0x00,
    CFG_RATE = 0x08,
    CFG_RINV = 0x34,
    CFG_RST = 0x04,
    CFG_RXM = 0x11,
    CFG_SBAS = 0x16,
    CFG_TMODE2 = 0x3D,
    CFG_TMODE = 0x1D,
    CFG_TP5 = 0x31,
    CFG_TP = 0x07,
    CFG_USB = 0x1B
};

// MON
enum monID {
    MON_HW2 = 0x0B,
    MON_HW = 0x09,
    MON_IO = 0x02,
    MON_MSGPP = 0x06,
    MON_RXBUF = 0x07,
    MON_RXR = 0x21,
    MON_TXBUF = 0x08,
    MON_VER = 0x04
};

// AID
enum aidID {
    AID_ALM = 0x30,
    AID_ALPSRV = 0x32,
    AID_ALP = 0x50,
    AID_AOP = 0x33,
    AID_DATA = 0x10,
    AID_EPH = 0x31,
    AID_HUI = 0x02,
    AID_INI = 0x01,
    AID_REQ = 0x00
};

// TIM
enum timID {
    TIM_SVIN = 0x04,
    TIM_TM2 = 0x03,
    TIM_TP = 0x01,
    TIM_VRFY = 0x06
};


#endif //NEO6MWITHASSISTNOW_ASSISTNOW_H
