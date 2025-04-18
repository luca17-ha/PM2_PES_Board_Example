#ifndef LSM9DS1_H_
#define LSM9DS1_H_

#include "mbed.h"

/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP     0x68
#define WHO_AM_I_M_RSP      0x3D


// The LSM9DS1 functions over both I2C or SPI. This library supports both.
// But the interface mode used must be sent to the LSM9DS1 constructor. Use
// one of these two as the first parameter of the constructor.
enum interface_mode
{
    IMU_MODE_SPI,
    IMU_MODE_I2C,
};

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
    A_SCALE_2G, // 00:  2g
    A_SCALE_16G,// 01:  16g
    A_SCALE_4G, // 10:  4g
    A_SCALE_8G  // 11:  8g
};

// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum gyro_scale
{
    G_SCALE_245DPS,     // 00:  245 degrees per second
    G_SCALE_500DPS,     // 01:  500 dps
    G_SCALE_2000DPS,    // 11:  2000 dps
};

// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale
{
    M_SCALE_4GS,    // 00:  4Gs
    M_SCALE_8GS,    // 01:  8Gs
    M_SCALE_12GS,   // 10:  12Gs
    M_SCALE_16GS,   // 11:  16Gs
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr
{
    //! TODO 
    G_ODR_PD,   // Power down (0)
    G_ODR_149,  // 14.9 Hz (1)
    G_ODR_595,  // 59.5 Hz (2)
    G_ODR_119,  // 119 Hz (3)
    G_ODR_238,  // 238 Hz (4)
    G_ODR_476,  // 476 Hz (5)
    G_ODR_952   // 952 Hz (6)
};
// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
    XL_POWER_DOWN,  // Power-down mode (0x0)
    XL_ODR_10,      // 10 Hz (0x1)
    XL_ODR_50,      // 50 Hz (0x02)
    XL_ODR_119,     // 119 Hz (0x3)
    XL_ODR_238,     // 238 Hz (0x4)
    XL_ODR_476,     // 476 Hz (0x5)
    XL_ODR_952      // 952 Hz (0x6)
};

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
    A_ABW_408,      // 408 Hz (0x0)
    A_ABW_211,      // 211 Hz (0x1)
    A_ABW_105,      // 105 Hz (0x2)
    A_ABW_50,       //  50 Hz (0x3)
};


// mag_odr defines all possible output data rates of the magnetometer:
enum mag_odr
{
    M_ODR_0625, // 0.625 Hz (0)
    M_ODR_125,  // 1.25 Hz (1)
    M_ODR_250,  // 2.5 Hz (2)
    M_ODR_5,    // 5 Hz (3)
    M_ODR_10,   // 10 Hz (4)
    M_ODR_20,   // 20 Hz (5)
    M_ODR_40,   // 40 Hz (6)
    M_ODR_80    // 80 Hz (7)
};

enum interrupt_select
{
    XG_INT1 = INT1_CTRL,
    XG_INT2 = INT2_CTRL
};

enum interrupt_generators
{
    INT_DRDY_XL = (1<<0),    // Accelerometer data ready (INT1 & INT2)
    INT_DRDY_G = (1<<1),     // Gyroscope data ready (INT1 & INT2)
    INT1_BOOT = (1<<2),  // Boot status (INT1)
    INT2_DRDY_TEMP = (1<<2),// Temp data ready (INT2)
    INT_FTH = (1<<3),        // FIFO threshold interrupt (INT1 & INT2)
    INT_OVR = (1<<4),        // Overrun interrupt (INT1 & INT2)
    INT_FSS5 = (1<<5),       // FSS5 interrupt (INT1 & INT2)
    INT_IG_XL = (1<<6),  // Accel interrupt generator (INT1)
    INT1_IG_G = (1<<7),  // Gyro interrupt enable (INT1)
    INT2_INACT = (1<<7),     // Inactivity interrupt output (INT2)
};  

enum accel_interrupt_generator
{
    XLIE_XL = (1<<0),
    XHIE_XL = (1<<1),
    YLIE_XL = (1<<2),
    YHIE_XL = (1<<3),
    ZLIE_XL = (1<<4),
    ZHIE_XL = (1<<5),
    GEN_6D = (1<<6)
};

enum gyro_interrupt_generator
{
    XLIE_G = (1<<0),
    XHIE_G = (1<<1),
    YLIE_G = (1<<2),
    YHIE_G = (1<<3),
    ZLIE_G = (1<<4),
    ZHIE_G = (1<<5)
};

enum mag_interrupt_generator
{
    ZIEN = (1<<5),
    YIEN = (1<<6),
    XIEN = (1<<7)
};

enum h_lactive
{
    INT_ACTIVE_HIGH,
    INT_ACTIVE_LOW
};

enum pp_od
{
    INT_PUSH_PULL,
    INT_OPEN_DRAIN
};

enum fifoMode_type
{
    FIFO_OFF = 0,
    FIFO_THS = 1,
    FIFO_CONT_TRIGGER = 3,
    FIFO_OFF_TRIGGER = 4,
    FIFO_CONT = 5
};

struct gyroSettings
{
    // Gyroscope settings:
    uint8_t enabled;
    uint16_t scale; // Changed this to 16-bit
    uint8_t sampleRate;
    // New gyro stuff:
    uint8_t bandwidth;
    uint8_t lowPowerEnable;
    uint8_t HPFEnable;
    uint8_t HPFCutoff;
    uint8_t flipX;
    uint8_t flipY;
    uint8_t flipZ;
    uint8_t orientation;
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    uint8_t latchInterrupt;
};

struct deviceSettings
{
    uint8_t commInterface; // Can be I2C, SPI 4-wire or SPI 3-wire
    uint8_t agAddress;  // I2C address or SPI CS pin
    uint8_t mAddress;   // I2C address or SPI CS pin
};

struct accelSettings
{
    // Accelerometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
    // New accel stuff:
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    int8_t  bandwidth;
    uint8_t highResEnable;
    uint8_t highResBandwidth;
};

struct magSettings
{
    // Magnetometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
    // New mag stuff:
    uint8_t tempCompensationEnable;
    uint8_t XYPerformance;
    uint8_t ZPerformance;
    uint8_t lowPowerEnable;
    uint8_t operatingMode;
};

struct temperatureSettings
{
    // Temperature settings
    uint8_t enabled;
};

struct IMUSettings
{
    deviceSettings device;
    
    gyroSettings gyro;
    accelSettings accel;
    magSettings mag;
    
    temperatureSettings temp;
};


#define LSM9DS1_AG_ADDR(sa0)    ((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)     ((sa1) == 0 ? 0x1C : 0x1E)

enum lsm9ds1_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};

class LSM9DS1
{
public:
    IMUSettings settings;
    
    // We'll store the gyro, accel, and magnetometer readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call updateGyro(), updateAcc(), and updateMag() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
    int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
    int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
    int16_t temperature; // Chip temperature
    float gBias[3], aBias[3], mBias[3];
    int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
    
    /*
    float gyroX, gyroY, gyroZ; // x, y, and z axis readings of the gyroscope (float value)
    float accX, accY, accZ; // x, y, and z axis readings of the accelerometer (float value)
    float magX, magY, magZ; // x, y, and z axis readings of the magnetometer (float value)
    */
    
    // LSM9DS1 -- LSM9DS1 class constructor
    // The constructor will set up a handful of private variables, and set the
    // communication mode as well.
    /**Input:
    *  - interface = Either IMU_MODE_SPI or IMU_MODE_I2C, whichever you're using
    *              to talk to the IC.
    *  - xgAddr = If IMU_MODE_I2C, this is the I2C address of the accel/gyroscope.
    *              If IMU_MODE_SPI, this is the chip select pin of the gyro (CS_AG)
    *  - mAddr = If IMU_MODE_I2C, this is the I2C address of the magnetometer.
    *              If IMU_MODE_SPI, this is the cs pin of the magnetometer (CS_M)
    
    */
    LSM9DS1(PinName sda, PinName scl, uint8_t xgAddr, uint8_t mAddr);
    LSM9DS1(PinName sda, PinName scl);
    //LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
    //LSM9DS1();
       
    
    /** begin() -- Initialize the gyro, accelerometer, and magnetometer.
    *This will set up the scale and output rate of each sensor. The values set
    * in the IMUSettings struct will take effect after calling this function.
    */
    uint16_t begin();
    
    float readGyroX();
    float readGyroY();
    float readGyroZ();
    float readAccX();
    float readAccY();
    float readAccZ();
    float readMagX();
    float readMagY();
    float readMagZ();
    
    void calibrate(bool autoCalc = true);
    void calibrateMag(bool loadIn = true);
    void magOffset(uint8_t axis, int16_t offset);
    
    /** accelAvailable() -- Polls the accelerometer status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t accelAvailable();
    
    /** gyroAvailable() -- Polls the gyroscope status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t gyroAvailable();
    
    /** gyroAvailable() -- Polls the temperature status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t tempAvailable();
    
    /** magAvailable() -- Polls the accelerometer status register to check
    * if new data is available.
    * Input:
    *  - axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
    *    on one specific axis. Or ALL_AXIS (default) to check for new data
    *    on all axes.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);
    
    /** updateGyro() -- Read the gyroscope output registers.
    * This function will read all six gyroscope output registers.
    * The readings are stored in the class' gx, gy, and gz variables. Read
    * those _after_ calling updateGyro().
    */
    void updateGyro();
    
    /** int16_t updateGyro(axis) -- Read a specific axis of the gyroscope.
    * [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    * Input:
    *  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    * Output:
    *  A 16-bit signed integer with sensor data on requested axis.
    */
    int16_t updateGyro(lsm9ds1_axis axis);
    
    /** updateAcc() -- Read the accelerometer output registers.
    * This function will read all six accelerometer output registers.
    * The readings are stored in the class' ax, ay, and az variables. Read
    * those _after_ calling updateAcc().
    */
    void updateAcc();
    
    /** int16_t updateAcc(axis) -- Read a specific axis of the accelerometer.
    * [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    * Input:
    *  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    * Output:
    *  A 16-bit signed integer with sensor data on requested axis.
    */
    int16_t updateAcc(lsm9ds1_axis axis);
    
    /** updateMag() -- Read the magnetometer output registers.
    * This function will read all six magnetometer output registers.
    * The readings are stored in the class' mx, my, and mz variables. Read
    * those _after_ calling updateMag().
    */
    void updateMag();
    
    /** int16_t updateMag(axis) -- Read a specific axis of the magnetometer.
    * [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    * Input:
    *  - axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    * Output:
    *  A 16-bit signed integer with sensor data on requested axis.
    */
    int16_t updateMag(lsm9ds1_axis axis);

    /** readTemp() -- Read the temperature output register.
    * This function will read two temperature output registers.
    * The combined readings are stored in the class' temperature variables. Read
    * those _after_ calling readTemp().
    */
    void readTemp();
    
    /** calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
    * This function reads in a signed 16-bit value and returns the scaled
    * DPS. This function relies on gScale and gRes being correct.
    * Input:
    *  - gyro = A signed 16-bit raw reading from the gyroscope.
    */
    float calcGyro(int16_t gyro);
    
    /** calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
    * This function reads in a signed 16-bit value and returns the scaled
    * g's. This function relies on aScale and aRes being correct.
    * Input:
    *  - accel = A signed 16-bit raw reading from the accelerometer.
    */
    float calcAccel(int16_t accel);
    
    /** calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
    * This function reads in a signed 16-bit value and returns the scaled
    * Gs. This function relies on mScale and mRes being correct.
    * Input:
    *  - mag = A signed 16-bit raw reading from the magnetometer.
    */
    float calcMag(int16_t mag);
    
    /** setGyroScale() -- Set the full-scale range of the gyroscope.
    * This function can be called to set the scale of the gyroscope to 
    * 245, 500, or 200 degrees per second.
    * Input:
    *  - gScl = The desired gyroscope scale. Must be one of three possible
    *      values from the gyro_scale.
    */
    void setGyroScale(uint16_t gScl);
    
    /** setAccelScale() -- Set the full-scale range of the accelerometer.
    * This function can be called to set the scale of the accelerometer to
    * 2, 4, 6, 8, or 16 g's.
    * Input:
    *  - aScl = The desired accelerometer scale. Must be one of five possible
    *      values from the accel_scale.
    */
    void setAccelScale(uint8_t aScl);
    
    /** setMagScale() -- Set the full-scale range of the magnetometer.
    * This function can be called to set the scale of the magnetometer to
    * 2, 4, 8, or 12 Gs.
    * Input:
    *  - mScl = The desired magnetometer scale. Must be one of four possible
    *      values from the mag_scale.
    */
    void setMagScale(uint8_t mScl);
    
    /** setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    * Input:
    *  - gRate = The desired output rate and cutoff frequency of the gyro.
    */
    void setGyroODR(uint8_t gRate);
    
    // setAccelODR() -- Set the output data rate of the accelerometer
    // Input:
    //  - aRate = The desired output rate of the accel.
    void setAccelODR(uint8_t aRate);    
    
    // setMagODR() -- Set the output data rate of the magnetometer
    // Input:
    //  - mRate = The desired output rate of the mag.
    void setMagODR(uint8_t mRate);
    
    // configInactivity() -- Configure inactivity interrupt parameters
    // Input:
    //  - duration = Inactivity duration - actual value depends on gyro ODR
    //  - threshold = Activity Threshold
    //  - sleepOn = Gyroscope operating mode during inactivity.
    //    true: gyroscope in sleep mode
    //    false: gyroscope in power-down
    void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);
    
    // configAccelInt() -- Configure Accelerometer Interrupt Generator
    // Input:
    //  - generator = Interrupt axis/high-low events
    //    Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
    //  - andInterrupts = AND/OR combination of interrupt events
    //    true: AND combination
    //    false: OR combination
    void configAccelInt(uint8_t generator, bool andInterrupts = false);
    
    // configAccelThs() -- Configure the threshold of an accelereomter axis
    // Input:
    //  - threshold = Interrupt threshold. Possible values: 0-255.
    //    Multiply by 128 to get the actual raw accel value.
    //  - axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
    //  - duration = Duration value must be above or below threshold to trigger interrupt
    //  - wait = Wait function on duration counter
    //    true: Wait for duration samples before exiting interrupt
    //    false: Wait function off
    void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);
    
    // configGyroInt() -- Configure Gyroscope Interrupt Generator
    // Input:
    //  - generator = Interrupt axis/high-low events
    //    Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
    //  - aoi = AND/OR combination of interrupt events
    //    true: AND combination
    //    false: OR combination
    //  - latch: latch gyroscope interrupt request.
    void configGyroInt(uint8_t generator, bool aoi, bool latch);
    
    // configGyroThs() -- Configure the threshold of a gyroscope axis
    // Input:
    //  - threshold = Interrupt threshold. Possible values: 0-0x7FF.
    //    Value is equivalent to raw gyroscope value.
    //  - axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
    //  - duration = Duration value must be above or below threshold to trigger interrupt
    //  - wait = Wait function on duration counter
    //    true: Wait for duration samples before exiting interrupt
    //    false: Wait function off
    void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);
    
    // configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
    // Input:
    //  - interrupt = Select INT1 or INT2
    //    Possible values: XG_INT1 or XG_INT2
    //  - generator = Or'd combination of interrupt generators.
    //    Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
    //    INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
    //  - activeLow = Interrupt active configuration
    //    Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
    //  - pushPull =  Push-pull or open drain interrupt configuration
    //    Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
    void configInt(interrupt_select interupt, uint8_t generator,
                   h_lactive activeLow = INT_ACTIVE_LOW, pp_od pushPull = INT_PUSH_PULL);
                   
    /** configMagInt() -- Configure Magnetometer Interrupt Generator
    * Input:
    *  - generator = Interrupt axis/high-low events
    *    Any OR'd combination of ZIEN, YIEN, XIEN
    *  - activeLow = Interrupt active configuration
    *    Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
    *  - latch: latch gyroscope interrupt request.
    */  
    void configMagInt(uint8_t generator, h_lactive activeLow, bool latch = true);
    
    /** configMagThs() -- Configure the threshold of a gyroscope axis
    * Input:
    *  - threshold = Interrupt threshold. Possible values: 0-0x7FF.
    *    Value is equivalent to raw magnetometer value.
    */
    void configMagThs(uint16_t threshold);
    
    //! getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
    uint8_t getGyroIntSrc();
    
    //! getGyroIntSrc() -- Get contents of accelerometer interrupt source register
    uint8_t getAccelIntSrc();
    
    //! getGyroIntSrc() -- Get contents of magnetometer interrupt source register
    uint8_t getMagIntSrc();
    
    //! getGyroIntSrc() -- Get status of inactivity interrupt
    uint8_t getInactivity();
    
    /** sleepGyro() -- Sleep or wake the gyroscope
    * Input:
    *  - enable: True = sleep gyro. False = wake gyro.
    */
    void sleepGyro(bool enable = true);
    
    /** enableFIFO() - Enable or disable the FIFO
    * Input:
    *  - enable: true = enable, false = disable.
    */
    void enableFIFO(bool enable = true);
    
    /** setFIFO() - Configure FIFO mode and Threshold
    * Input:
    *  - fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
    *    Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
    *  - fifoThs: FIFO threshold level setting
    *    Any value from 0-0x1F is acceptable.
    */
    void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
    
    //! getFIFOSamples() - Get number of FIFO samples
    uint8_t getFIFOSamples();
        

protected:  
    // x_mAddress and gAddress store the I2C address or SPI chip select pin
    // for each sensor.
    uint8_t _mAddress, _xgAddress;
    
    // gRes, aRes, and mRes store the current resolution for each sensor. 
    // Units of these values would be DPS (or g's or Gs's) per ADC tick.
    // This value is calculated as (sensor scale) / (2^15).
    float gRes, aRes, mRes;
    
    // _autoCalc keeps track of whether we're automatically subtracting off
    // accelerometer and gyroscope bias calculated in calibrate().
    bool _autoCalc;
    
    // init() -- Sets up gyro, accel, and mag settings to default.
    // - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
    // - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip 
    //   select pin connected to the CS_XG pin.
    // - mAddr - Sets either the I2C address of the magnetometer or SPI chip 
    //   select pin connected to the CS_M pin.
    void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
    
    // initGyro() -- Sets up the gyroscope to begin reading.
    // This function steps through all five gyroscope control registers.
    // Upon exit, the following parameters will be set:
    //  - CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
    //      95 Hz ODR, 12.5 Hz cutoff frequency.
    //  - CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
    //      set to 7.2 Hz (depends on ODR).
    //  - CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
    //      active high). Data-ready output enabled on DRDY_G.
    //  - CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
    //      address. Scale set to 245 DPS. SPI mode set to 4-wire.
    //  - CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
    void initGyro();
    
    // initAccel() -- Sets up the accelerometer to begin reading.
    // This function steps through all accelerometer related control registers.
    // Upon exit these registers will be set as:
    //  - CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
    //  - CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
    //      all axes enabled.
    //  - CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
    //  - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
    void initAccel();
    
    // initMag() -- Sets up the magnetometer to begin reading.
    // This function steps through all magnetometer-related control registers.
    // Upon exit these registers will be set as:
    //  - CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
    //  - CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
    //      requests don't latch. Temperature sensor disabled.
    //  - CTRL_REG6_XM = 0x00:  2 Gs scale.
    //  - CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
    //  - INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
    void initMag();
    
    // gReadByte() -- Reads a byte from a specified gyroscope register.
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested address.
    uint8_t mReadByte(uint8_t subAddress);
    
    // gReadBytes() -- Reads a number of bytes -- beginning at an address
    // and incrementing from there -- from the gyroscope.
    // Input:
    //  - subAddress = Register to be read from.
    //  - * dest = A pointer to an array of uint8_t's. Values read will be
    //      stored in here on return.
    //  - count = The number of bytes to be read.
    // Output: No value is returned, but the `dest` array will store
    //  the data read upon exit.
    void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
    
    // gWriteByte() -- Write a byte to a register in the gyroscope.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void mWriteByte(uint8_t subAddress, uint8_t data);
    
    // xmReadByte() -- Read a byte from a register in the accel/mag sensor
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested register.
    uint8_t xgReadByte(uint8_t subAddress);
    
    // xmReadBytes() -- Reads a number of bytes -- beginning at an address
    // and incrementing from there -- from the accelerometer/magnetometer.
    // Input:
    //  - subAddress = Register to be read from.
    //  - * dest = A pointer to an array of uint8_t's. Values read will be
    //      stored in here on return.
    //  - count = The number of bytes to be read.
    // Output: No value is returned, but the `dest` array will store
    //  the data read upon exit.
    void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
    
    // xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void xgWriteByte(uint8_t subAddress, uint8_t data);
    
    // calcgRes() -- Calculate the resolution of the gyroscope.
    // This function will set the value of the gRes variable. gScale must
    // be set prior to calling this function.
    void calcgRes();
    
    // calcmRes() -- Calculate the resolution of the magnetometer.
    // This function will set the value of the mRes variable. mScale must
    // be set prior to calling this function.
    void calcmRes();
    
    // calcaRes() -- Calculate the resolution of the accelerometer.
    // This function will set the value of the aRes variable. aScale must
    // be set prior to calling this function.
    void calcaRes();
    
    //////////////////////
    // Helper Functions //
    //////////////////////
    void constrainScales();
    
    ///////////////////
    // SPI Functions //
    ///////////////////
    // initSPI() -- Initialize the SPI hardware.
    // This function will setup all SPI pins and related hardware.
    void initSPI();
    
    // SPIwriteByte() -- Write a byte out of SPI to a register in the device
    // Input:
    //  - csPin = The chip select pin of the slave device.
    //  - subAddress = The register to be written to.
    //  - data = Byte to be written to the register.
    void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data);
    
    // SPIreadByte() -- Read a single byte from a register over SPI.
    // Input:
    //  - csPin = The chip select pin of the slave device.
    //  - subAddress = The register to be read from.
    // Output:
    //  - The byte read from the requested address.
    uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress);
    
    // SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
    // Input:
    //  - csPin = The chip select pin of a slave device.
    //  - subAddress = The register to begin reading.
    //  - * dest = Pointer to an array where we'll store the readings.
    //  - count = Number of registers to be read.
    // Output: No value is returned by the function, but the registers read are
    //      all stored in the *dest array given.
    void SPIreadBytes(uint8_t csPin, uint8_t subAddress, 
                            uint8_t * dest, uint8_t count);
    
    ///////////////////
    // I2C Functions //
    ///////////////////
    // initI2C() -- Initialize the I2C hardware.
    // This function will setup all I2C pins and related hardware.
    void initI2C();
    
    // I2CwriteByte() -- Write a byte out of I2C to a register in the device
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to be written to.
    //  - data = Byte to be written to the register.
    void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
    
    // I2CreadByte() -- Read a single byte from a register over I2C.
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to be read from.
    // Output:
    //  - The byte read from the requested address.
    uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
    
    // I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to begin reading.
    //  - * dest = Pointer to an array where we'll store the readings.
    //  - count = Number of registers to be read.
    // Output: No value is returned by the function, but the registers read are
    //      all stored in the *dest array given.
    uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
    
private:
    I2C i2c;
    float gyroX, gyroY, gyroZ; // x, y, and z axis readings of the gyroscope (float value)
    float accX, accY, accZ; // x, y, and z axis readings of the accelerometer (float value)
    float magX, magY, magZ; // x, y, and z axis readings of the magnetometer (float value)
};

#endif  /* LSM9DS1_H_ */
