#include <Arduino.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>


#define ADR_SENS 0x28

elapsedMicros t1;
elapsedMillis tmillis;
// Prototypes




enum data{ // Read two bytes when neccessary
    ACC_OFFSET_X_LSB = 0x55,
    ACC_OFFSET_X_MSB = 0x56,
    ACC_OFFSET_Y_LSB = 0x57,
    ACC_OFFSET_Y_MSB = 0x58,
    ACC_OFFSET_Z_LSB = 0x59,
    ACC_OFFSET_Z_MSB = 0x5A,

    MAG_OFFSET_X_LSB = 0x5B,
    MAG_OFFSET_X_MSB = 0x5C,
    MAG_OFFSET_Y_LSB = 0x5D,
    MAG_OFFSET_Y_MSB = 0x5E,
    MAG_OFFSET_Z_LSB = 0x5F,
    MAG_OFFSET_Z_MSB = 0x60,

    GYR_OFFSET_X_LSB = 0x61,
    GYR_OFFSET_X_MSB = 0x62,
    GYR_OFFSET_Y_LSB = 0x63,
    GYR_OFFSET_Y_MSB = 0x64,
    GYR_OFFSET_Z_LSB = 0x65,
    GYR_OFFSET_Z_MSB = 0x66,


    ACC_DATA_X = 0x08,
    ACC_DATA_Y = 0x0A,
    ACC_DATA_Z = 0x0C,

    MAG_DATA_X = 0x0E,
    MAG_DATA_Y = 0x10,
    MAG_DATA_Z = 0x12,

    GRY_DATA_X = 0x14,
    GRY_DATA_Y = 0x16,
    GRY_DATA_Z = 0x18,

    EUL_HEADING = 0x1A,
    EUL_ROLL = 0x1C,
    EUL_PITCH = 0x1E,

    QUA_DATA_W = 0x20,
    QUA_DATA_X = 0x22,
    QUA_DATA_Y = 0x24,
    QUA_DATA_Z = 0x26,

    LIA_DATA_X = 0x28,
    LIA_DATA_Y = 0x2A,
    LIA_DATA_Z = 0x2C,

    GRV_DATA_X = 0x2E,
    GRV_DATA_Y = 0x30,
    GRV_DATA_Z = 0x32,

    TEMP = 0x34,

    CALIB_STAT = 0x35,

    UNIT_SEL = 0x3B,

    OPR_MODE = 0x3D,
    PWR_MODE = 0x3E,

    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,

    PAGE_ID = 0x07,

    // PAGE 1

    ACC_CONFIG = 0x08,


};


void REGSET(byte reg, byte data) {

    Wire.beginTransmission(ADR_SENS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission(true);
}


unsigned short READREG(byte reg, int numbytes) {
    byte datahigh = 0;
    byte datalow = 0;
    unsigned short data = 0 ;
    if (numbytes == 1 || numbytes==2) {
        Wire.beginTransmission(ADR_SENS);
        Wire.write(reg);
        Wire.endTransmission(true);

        Wire.requestFrom(ADR_SENS, numbytes); // sends a stop
        if (!Wire.available()) { Serial.println("WIRE NOT AVAILABLE "); Serial.println(Wire.available()); }

        if (numbytes == 1){

            data = Wire.read();

        }
        else {
            datalow = Wire.read();
            datahigh = Wire.read();


            data = (datahigh << 8) | datalow;


        }
    }
    else{
        Serial.println("CANNOT REQUEST MORE THAT 2 BYTES OF DATA");
    }

    return data;
}


void READVECT(byte reg, int numdata, int numbytes, short returnvect[] ) {
    byte datahigh = 0;
    byte datalow = 0;

    Wire.beginTransmission(ADR_SENS);
    Wire.write(reg);
    Wire.endTransmission(true);

    Wire.requestFrom(ADR_SENS, numdata*numbytes);   // sends a stop
    if (!Wire.available()) { Serial.println("WIRE NOT AVAILABLE "); Serial.println(Wire.available()); }

    for (int i =0; i<numdata; i++){
        datalow = Wire.read();
        datahigh = Wire.read();
        short temp = ((datahigh << 8) | datalow);
        returnvect[i] = temp ;

    }

}


byte calibrate() {
    uint8_t magcalstat = 0, acccalstat = 0, gyrcalstat = 0, syscalstat = 0, calstat = 0;

    while (( (magcalstat < 3) || (acccalstat < 3) || (gyrcalstat < 3) || (syscalstat < 2) ) && (tmillis<20000)){
        // now includes timout for accelerometer
        calstat = READREG(CALIB_STAT, 1);
        magcalstat = calstat & 0b0011;
        acccalstat = (calstat >> 2) & 0b0011;
        gyrcalstat = (calstat >> 4) & 0b0011;
        syscalstat = (calstat >> 6) & 0b0011;

        if (Serial) {Serial.printf("MAG: %d,   ACC: %d,   GRY: %d,   SYS: %d\n", magcalstat, acccalstat, gyrcalstat, syscalstat); }

    }
    delay(3000);
    return calstat;
}


void init() {
    delay(2000);
    REGSET(OPR_MODE, 0b00000000); // CONFIG MODE
    delay(20);

    REGSET(PAGE_ID, 0x01);
    REGSET(ACC_CONFIG, 0b00000011); // 16G page 1
    REGSET(PAGE_ID, 0x00);

    REGSET(PWR_MODE, 0b00000000); // normal mode

    REGSET(UNIT_SEL, 0b00000000); //Celcius, degrees, dps, m/s^2

    REGSET(AXIS_MAP_CONFIG, 0b00100100);
    REGSET(AXIS_MAP_SIGN, 0b00000000);
    REGSET(OPR_MODE, 0b00001100); // NdoF mode
}

void remove_offsets() {  // TODO: Integrate this with the heading, pitch and roll to get true biases (for if calibration is done on a slope).
    bool offsets_calibrated = false;
    int counter = 0;

    const int num = 100;
    signed short acc_biases_x[num] = {0};
    signed short acc_biases_y[num] = {0};
    signed short acc_biases_z[num] = {0};

    REGSET(OPR_MODE, 0b00000000); //config mode
    delay(20);
    REGSET(ACC_OFFSET_X_LSB, 0);
    REGSET(ACC_OFFSET_X_MSB, 0);
    REGSET(ACC_OFFSET_Y_LSB, 0);
    REGSET(ACC_OFFSET_Y_MSB, 0);
    REGSET(ACC_OFFSET_Z_LSB, 0);
    REGSET(ACC_OFFSET_Z_MSB, 0);

    REGSET(OPR_MODE, 0b00001100); // NDOF for reading

    int avg_x=0, avg_y=0, avg_z=0;


    while (!offsets_calibrated) {
        signed short acc_offset_VECT[3] = {};
        READVECT(ACC_DATA_X, 3, 2, acc_offset_VECT);
        acc_biases_x[counter % num] = acc_offset_VECT[0];
        acc_biases_y[counter % num] = acc_offset_VECT[1];
        acc_biases_z[counter % num] = acc_offset_VECT[2];
        bool within_range_x = true;
        bool within_range_y = true;
        bool within_range_z = true;
        if (counter > (2*num)) {

            for (short ax: acc_biases_x) { avg_x += ax; }
            for (short ay: acc_biases_y) { avg_y += ay; }
            for (short az: acc_biases_z) { avg_z += az; }

            avg_x /= num;
            avg_y /= num;
            avg_z /= num;

            int thres = 5;
            for (auto a: acc_biases_x) { if (abs((a - avg_x) > thres)){ within_range_x = false; } }
            for (auto b: acc_biases_y) { if (abs((b - avg_y) > thres)){ within_range_y = false; } }
            for (auto c: acc_biases_z) { if (abs((c - avg_z) > thres)){ within_range_z = false; } }
            if ((within_range_x & within_range_y) & within_range_z) {
                avg_z -= 980; // for if you are using acc data
                //avg_z = -avg_z;
                offsets_calibrated = true;

            }
        }
        counter++;

        if (Serial) { for (auto x:acc_offset_VECT) { Serial.printf("%d,     ", x ) ;} Serial.printf("%d, %d, %d       %d, %d, %d, %d    \n", avg_x, avg_y, avg_z, (counter>num), within_range_x, within_range_y, within_range_z); }
        delay(10);

    }


    byte acc_offset_addresses[6] = {ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB, ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB, ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB};
    byte mag_offset_addresses[6] = {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB, MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB, MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB};
    byte gyr_offset_addresses[6] = {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB, GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB, GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB};

    signed short addresses[3][9] = {{ACC_OFFSET_X_LSB, ACC_OFFSET_X_MSB,
                                            ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_MSB,
                                            ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_MSB,
                                            static_cast<short>(avg_x), static_cast<short>(avg_y), static_cast<short>(avg_z)},
                                    {MAG_OFFSET_X_LSB, MAG_OFFSET_X_MSB,
                                            MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_MSB,
                                            MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_MSB,
                                            0, 0, 0},
                                    {GYR_OFFSET_X_LSB, GYR_OFFSET_X_MSB,
                                            GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_MSB,
                                            GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_MSB,
                                            0, 0 ,0}};


    // Sets offsets for Accelerometer, Gyroscope, and Magnetometer
    for (int i = 0; i<3; i++) {
        signed short x = addresses[i][6];
        signed short y = addresses[i][7];
        signed short z = addresses[i][8];

        byte datalow_x = x & 0xFF;
        byte datahigh_x = (x >> 8) & 0xFF;
        byte datalow_y = y & 0xFF;
        byte datahigh_y = (y >> 8) & 0xFF;
        byte datalow_z = z & 0xFF;
        byte datahigh_z = (z >> 8) & 0xFF;

        REGSET(OPR_MODE, 0b00000000);  // config mode for writing regs
        delay(20);

        REGSET(addresses[i][0], datalow_x);
        REGSET(addresses[i][1], datahigh_x);
        delay(10);
        REGSET(addresses[i][2], datalow_y);
        REGSET(addresses[i][3], datahigh_y);
        delay(10);
        REGSET(addresses[i][4], datalow_z);
        REGSET(addresses[i][5], datahigh_z);


        REGSET(OPR_MODE, 0b00001100);  // NDOF mode for reading

        //Serial.printf("%d,  %d,  %d  \n", acc_biases_x[0], acc_biases_x[1], acc_biases_x[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_y[0], acc_biases_y[1], acc_biases_y[2]);
        //Serial.printf("%d,  %d,  %d  \n", acc_biases_z[0], acc_biases_z[1], acc_biases_z[2]);
        Serial.printf("x: %d, y: %d, z: %d  \n", x, y, z);

        Serial.println("ALL OFFSETS CALIBRATED");
        delay(1000);
    }





    //delay(2000);

    //REGSET(OPR_MODE, 0b00001100);

}


void setup(void) {
    delay(2000);
    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    init();
    calibrate();
    remove_offsets();

    delay(1000);


}

int counter = 0;

SimpleKalmanFilter kf = SimpleKalmanFilter(0.1, 0.1, 0.01);
double x_pos=0, y_pos=0, z_pos=0;
double x_vel=0, y_vel=0, z_vel=0;

void loop() {
    unsigned long a = t1;

    // initial conditions:

    signed short read1[12] = {};
    READVECT(ACC_DATA_X, 12, 2, read1);
    double acc_VECT[3] = {static_cast<double>(read1[0])/100,
                          static_cast<double>(read1[1])/100,
                          static_cast<double>(read1[2])/100}; // WHY 655

    double mag_VECT[3] = {static_cast<double>(read1[3]),
                          static_cast<double>(read1[4]),
                          static_cast<double>(read1[5])}; // scaling??

    double gyr_VECT[3] = {static_cast<double>(read1[6])/900,
                          static_cast<double>(read1[7])/900,
                          static_cast<double>(read1[8])/900}; // /900 for rps,   /16 for dps

    double ori_VECT[3] = {static_cast<double>(read1[9])/16,
                          static_cast<double>(read1[10])/16,
                          static_cast<double>(read1[11])/16};

    // omitting quaternion data - i doubt i will use it.
    signed short read2[6] = {};
    READVECT(LIA_DATA_X, 6, 2, read2);
    double lia_VECT[3] = {static_cast<double>(read2[0])/100,
                          static_cast<double>(read2[1])/100,
                          static_cast<double>(read2[2])/100};
    double grav_VECT[3] = {static_cast<double>(read2[3])/100,
                           static_cast<double>(read2[4])/100,
                           static_cast<double>(read2[5])/100};



    double mag_acc = sqrt(pow(acc_VECT[0], 2) + pow(acc_VECT[1], 2) + pow(acc_VECT[2], 2));
    if (mag_acc < 2) {
        //Serial.write("APOGEE DETECTED!!!!!!!!!!!!!!!!!! DEPLYOYING PARACHUTES !!!!!!!!");
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
    double mag_grav = sqrt(pow(grav_VECT[0], 2) + pow(grav_VECT[1], 2) + pow(grav_VECT[2], 2));
    double mag_lia = sqrt(pow(lia_VECT[0], 2) + pow(lia_VECT[1], 2) + pow(lia_VECT[2], 2));

    //dead reckoning attempt
    unsigned long b = t1;

    if ((counter>0) && (abs(lia_VECT[0]) > 0.2)){
        float est = kf.updateEstimate(lia_VECT[0]);
        x_vel += est*0.01;
        x_pos += x_vel*0.01;
        Serial.printf("XPOS:  %lf         lia:%lf\n", x_pos*100, est);
    }



    if (Serial) {
        for (int i = 0; i < 3; i++) {
            //Serial.printf("acc%d: %lf   ", i, acc_VECT[i]);
        }
        //Serial.printf("MAGnitude: %lf      ", mag_acc);
        for (int i = 0; i < 3; i++) {
            //Serial.printf("grav%d: %8.5lf,  ", i, grav_VECT[i]);
        }
        //Serial.printf("MAGnitude: %lf      ", mag_grav);
        for (int i = 0; i < 3; i++) {
            //Serial.printf("lia%d: %lf  ", i, lia_VECT[i]);
        }
        //Serial.printf("MAGnitude: %lf      ", mag_lia);
        for (int i = 0; i < 3; i++) {
            //Serial.printf("ori%d: %lf  ", i, ori_VECT[i]);
        }

        //Serial.println("\n");



    }





    delay(10 - (b - a) / 1000);
    counter ++;


}
