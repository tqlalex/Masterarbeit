#include <Arduino.h>
//#include <bluefruit.h>
#include <Wire.h>
#include "bmi270.h"
#include "bmm150.h"
#include "sparkfun52840mini.h"
#include <SoftwareSerial.h>


#define ACC UINT8_C(0x00)
#define GYR UINT8_C(0x01)
#define AUX UINT8_C(0x02)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)

//BLEUart bleuart;

const int number_of_sensors = 2;
//static struct bmi2_dev imu1;
//static struct bmi2_dev imu2;
//static struct bmi2_dev imus[2] = {imu1, imu2};

//uint8_t address1 = BMI2_I2C_SEC_ADDR;   //0x69
//static struct bmi2_dev *imu1_ptr = &imus[0];

//uint8_t address2 = BMI2_I2C_PRIM_ADDR;  //0x68
//static struct bmi2_dev *imu2_ptr = &imus[1];

//(Receiver Rx | Transmitter 17)
SoftwareSerial EEBlue(21, 23);


static uint8_t dev_addr1;
static uint8_t dev_addr2;


static struct bmi2_dev imu1;
static struct bmi2_dev imu2;
static struct bmi2_dev imus[2] = {imu1, imu2};


static struct bmm150_dev aux_bmm150_dev1;
static struct bmm150_dev aux_bmm150_dev2;



struct bmi2_sens_data sensor_data_first = {{0}};
struct bmi2_sens_data sensor_data_second = {{0}};
float imu_data_first[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float imu_data_second[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool send_raw;

void setup(void)
{
    Serial.begin(115200);
    delay(2000);

    EEBlue.begin(115200);
    Serial.println("[MAIN] Config start");

    Serial.println("The Bluetooth gates are open.");
    Serial.println("Connect to HC-05 with 1234 as key!");

    /*
   // Initialize Bluetooth:
    Bluefruit.begin();
    // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
    Bluefruit.setTxPower(4);
    Bluefruit.setName("SparkFun_nRF52840");
    bleuart.begin();

    // Start advertising device and bleuart services
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);
    Bluefruit.ScanResponse.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);

    // Set advertising interval (in unit of 0.625ms):
    Bluefruit.Advertising.setInterval(32, 244);
    // number of seconds in fast mode:
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);  
    */
    

    i2c_setup();

    

    // Enable LED and connect button 13 with toggle imu
    pinMode(LED_BUILTIN, OUTPUT); // enable status led
    pinMode(PIN_DFU, INPUT);      // enable button 13
    attachInterrupt(digitalPinToInterrupt(PIN_DFU), toggle_imu, FALLING);

    start_send_raw();

    // Config IMU1 (0x68)
    //bmi270_setup(&imu, BMI2_I2C_PRIM_ADDR);
    // Config IMU2 (0x69)
    bmi270_setup_first(&imu1, BMI2_I2C_SEC_ADDR);
    bmi270_setup_second(&imu2, BMI2_I2C_PRIM_ADDR);

    //bmi270_setup_first(&imu1, BMI2_I2C_SEC_ADDR);

    //bmi270_setup_first(&imu1, BMI2_I2C_SEC_ADDR);

    Serial.println("[MAIN] Config done");
}

void loop(void)
{
    //bmi270_read_sensor_data(&imu1, imu_data);
    bmi270_read_sensor_data_first(&imu1, imu_data_first);
    bmi270_read_sensor_data_second(&imu2, imu_data_second);
    if (send_raw)
    {
        send_raw_data();
        //send_plotter_data();      
      
      }

    else
        //send_plotter_data_first();
        //send_plotter_data_second();
        send_plotter_data();

    delay(10);
}

void send_raw_data()
{
    for (uint8_t i = 0; i < 9; i++)
    {
        byte *b = (byte *)&imu_data_first[i];
        //bleuart.write(b, 4);
        
        //EEBlue.write(b, 4);
        Serial.write(b, 4);
    }
    
    for (uint8_t i = 0; i < 9; i++)
    {
        byte *b = (byte *)&imu_data_second[i];
        //bleuart.write(b, 4);
        
        //EEBlue.write(b, 4);
        Serial.write(b, 4);
    }
    
}

void send_plotter_data_first()
{
    Serial.print("First Sensor: ");
    for (uint8_t i = 0; i < 9; i++)
    {
        Serial.print(imu_data_first[i]);

        if (i < 8)
            Serial.print(",");
        else
            Serial.print("\r\n");
    }
}

void send_plotter_data_second()
{   
    Serial.print("Second Sensor: ");
    for (uint8_t i = 0; i < 9; i++)
    {
        Serial.print(imu_data_second[i]);

        if (i < 8)
            Serial.print(",");
        else
            Serial.print("\r\n");
    }
}

void send_plotter_data()
{   
    Serial.print("all sensors: ");
    for (uint8_t i = 0; i < 9; i++)
    {
        Serial.print(imu_data_first[i]);

        if (i < 8)
            Serial.print(",");
        else
            Serial.print("\r\n");
    }
    for (uint8_t i = 0; i < 9; i++)
    {
        Serial.print(imu_data_second[i]);

        if (i < 8)
            Serial.print(",");
        else
            Serial.print("\r\n");
    }
}


void start_send_raw(void)
{
    digitalWrite(LED_BUILTIN, LOW);
    send_raw = true;
}

void stop_send_raw(void)
{
    digitalWrite(LED_BUILTIN, HIGH);
    send_raw = false;
}

void toggle_imu(void)
{
    if (send_raw)
        stop_send_raw();
    else
        start_send_raw();
}

/*!
 * @brief Start i2c
 *
 * @return void
 */
void i2c_setup()
{
    Wire.begin();
}

/*!
 * @brief Read data from i2c
 *
 * @return int2_t        : -1 = no data received, 0 = transmition completed
 */

/* 
BMI2_INTF_RETURN_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    uint8_t bytes_received;

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission() == 0)
    {
        bytes_received = Wire.requestFrom(dev_addr, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++)
            reg_data[i] = Wire.read();
    }
    else
        return -1;

    return 0;
}

*/

BMI2_INTF_RETURN_TYPE i2c_read_first(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    uint8_t bytes_received;

    Wire.beginTransmission(dev_addr1);
    Wire.write(reg_addr);
    if (Wire.endTransmission() == 0)
    {
        bytes_received = Wire.requestFrom(dev_addr1, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++)
            reg_data[i] = Wire.read();
    }
    else
        return -1;

    return 0;
}

BMI2_INTF_RETURN_TYPE i2c_read_second(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    uint8_t bytes_received;

    Wire.beginTransmission(dev_addr2);
    Wire.write(reg_addr);
    if (Wire.endTransmission() == 0)
    {
        bytes_received = Wire.requestFrom(dev_addr2, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++)
            reg_data[i] = Wire.read();
    }
    else
        return -1;

    return 0;
}



/*!
 * @brief Write data to i2c
 *
 * @return int8_t        :-1 = send data incomplete, 0 = transmition completed
 */

/* 
BMI2_INTF_RETURN_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);

    for (uint16_t i = 0; i < len; i++)
        Wire.write(reg_data[i]);

    if (Wire.endTransmission() != 0)
        return -1;

    return 0;
}

*/

BMI2_INTF_RETURN_TYPE i2c_write_first(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    Wire.beginTransmission(dev_addr1);
    Wire.write(reg_addr);

    for (uint16_t i = 0; i < len; i++)
        Wire.write(reg_data[i]);

    if (Wire.endTransmission() != 0)
        return -1;

    return 0;
}


BMI2_INTF_RETURN_TYPE i2c_write_second(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
        return -1;

    Wire.beginTransmission(dev_addr2);
    Wire.write(reg_addr);

    for (uint16_t i = 0; i < len; i++)
        Wire.write(reg_data[i]);

    if (Wire.endTransmission() != 0)
        return -1;

    return 0;
}


/*!
 * @brief This function reads the data from auxiliary sensor in data mode.
 */

/* 
static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, len, &imu);

    return rslt;
}
*/


static int8_t aux_i2c_read_first(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;
    //using loop to from intf_ptr to find the dev

    struct bmi2_dev goal_imu;

    uint8_t dev_addr_local = *((uint8_t*)intf_ptr);
    
    for(int j = 0; j < number_of_sensors; j++){

      if(dev_addr_local == *((uint8_t*)(imus[j].intf_ptr))){
        goal_imu = imus[j];
        break;     
      }
        
      
    }

    
    //rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, len, &imu);
    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, len, &imu1);

    return rslt;
}


static int8_t aux_i2c_read_second(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;
    //using loop to from intf_ptr to find the dev

    struct bmi2_dev goal_imu;

    uint8_t dev_addr_local = *((uint8_t*)intf_ptr);
    
    for(int j = 0; j < number_of_sensors; j++){

      if(dev_addr_local == *((uint8_t*)(imus[j].intf_ptr))){
        goal_imu = imus[j];
        break;     
      }
        
      
    }

    
    //rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, len, &imu);
    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, len, &imu2);

    return rslt;
}



/*!
 * @brief This function writes the data to auxiliary sensor in data mode.
 */

/*
static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, len, &imu);

    return rslt;
}
*/

/*!
 * @brief Setup IMU
 *
 * @param[in] dev                      : Pointer to device
 * @param[in] address                  : Address of the IMU
 *
 * @return void
 */

static int8_t aux_i2c_write_first(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;
    //rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, len, &imu);


    struct bmi2_dev goal_imu;

    uint8_t dev_addr_local = *((uint8_t*)intf_ptr);
    
    for(int j = 0; j < number_of_sensors; j++){

      if(dev_addr_local == *((uint8_t*)(imus[j].intf_ptr))){
        goal_imu = imus[j];
        break;     
      }
        
      
    }

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, len, &imu1);

    return rslt;
}


static int8_t aux_i2c_write_second(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt;
    //rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, len, &imu);


    struct bmi2_dev goal_imu;

    uint8_t dev_addr_local = *((uint8_t*)intf_ptr);
    
    for(int j = 0; j < number_of_sensors; j++){

      if(dev_addr_local == *((uint8_t*)(imus[j].intf_ptr))){
        goal_imu = imus[j];
        break;     
      }
        
      
    }

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, len, &imu2);

    return rslt;
}
 
void bmi270_setup_first(struct bmi2_dev *dev, uint8_t address)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

    /* Initialize the interrupt status of accel, gyro and aux. */
    uint16_t int_status = 0;

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config sens_cfg[3];

    /* interrupt pin configuration */
    struct bmi2_int_pin_config int_pin_cfg;

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    dev_addr1 = address;

    sens_cfg[ACC].type = BMI2_ACCEL;
    sens_cfg[GYR].type = BMI2_GYRO;
    sens_cfg[AUX].type = BMI2_AUX;

    /* To enable the i2c interface settings for bmm150. */
    uint8_t aux_bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    aux_bmm150_dev1.intf_ptr = &aux_bmm150_dev_addr;
    aux_bmm150_dev1.read = aux_i2c_read_first;   // todo: change to i2c_read and delete aux_i2c_read
    aux_bmm150_dev1.write = aux_i2c_write_first; // todo: change to i2c_write and delete aux_i2c_write
    aux_bmm150_dev1.delay_us = delay_us;
    aux_bmm150_dev1.intf = BMM150_I2C_INTF; // As per datasheet, aux interface with bmi270 will support only for I2C

    dev->intf = BMI2_I2C_INTF;   // To initialize the user I2C function
    dev->intf_ptr = &dev_addr1;   // I2C-Address
    dev->read = i2c_read_first;        // Callback functions for i2c read (defined in this file)
    dev->write = i2c_write_first;      // Callback functions for i2c write (defined in this file)
    dev->delay_us = delay_us;    // Callback functions for delay (defined in this file)
    dev->read_write_len = 30;    // Limitation of the Wire library
    dev->config_file_ptr = NULL; // Assign to NULL to load the default config file.

    // Initialise IMU and show error if exists
    rslt = bmi270_init(dev);
    bmi270_error_codes_print_result(rslt);

    /* Pull-up resistor 2k is set to the trim register */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, dev);
    bmi270_error_codes_print_result(rslt);

    /* Enable the accel, gyro and aux sensor. */
    rslt = bmi270_sensor_enable(sensor_list, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    // // Pin configuration
    // int_pin_cfg.pin_type = BMI2_INT1;
    // int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    // int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    // int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    // int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_INPUT_DISABLE;
    // int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    // // Configure interrupt PINs
    // rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    // bmi270_error_codes_print_result(rslt);

    /* Configurations for acc. */
    sens_cfg[ACC].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[ACC].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    sens_cfg[ACC].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[ACC].cfg.acc.range = BMI2_ACC_RANGE_4G;

    /* Configurations for gyro. */
    sens_cfg[GYR].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[GYR].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[GYR].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    sens_cfg[GYR].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[GYR].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    sens_cfg[GYR].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;

    /* Configurations for aux. */
    sens_cfg[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    sens_cfg[AUX].cfg.aux.manual_en = BMI2_ENABLE;
    sens_cfg[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    sens_cfg[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    sens_cfg[AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;
    sens_cfg[AUX].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    sens_cfg[AUX].cfg.aux.aux_en = BMI2_ENABLE;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_set_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Initialize bmm150. */
    rslt = bmm150_init(&aux_bmm150_dev1);
    bmm150_error_codes_print_result(rslt);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &aux_bmm150_dev1);
    bmm150_error_codes_print_result(rslt);

    rslt = bmi270_get_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Disable manual mode so that the data mode is enabled. */
    sens_cfg[AUX].cfg.aux.manual_en = BMI2_DISABLE;

    /* Set the aux configurations. */
    rslt = bmi270_set_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    // /* Map data ready interrupt to interrupt pin. */
    // rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    // bmi270_error_codes_print_result(rslt);

    Serial.println("MAGNETOMETER, ACC, AND GYR DATA IN DATA MODE");
}

void bmi270_setup_second(struct bmi2_dev *dev, uint8_t address)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

    /* Initialize the interrupt status of accel, gyro and aux. */
    uint16_t int_status = 0;

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config sens_cfg[3];

    /* interrupt pin configuration */
    struct bmi2_int_pin_config int_pin_cfg;

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    dev_addr2 = address;

    sens_cfg[ACC].type = BMI2_ACCEL;
    sens_cfg[GYR].type = BMI2_GYRO;
    sens_cfg[AUX].type = BMI2_AUX;

    /* To enable the i2c interface settings for bmm150. */
    uint8_t aux_bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    aux_bmm150_dev2.intf_ptr = &aux_bmm150_dev_addr;
    aux_bmm150_dev2.read = aux_i2c_read_second;   // todo: change to i2c_read and delete aux_i2c_read
    aux_bmm150_dev2.write = aux_i2c_write_second; // todo: change to i2c_write and delete aux_i2c_write
    aux_bmm150_dev2.delay_us = delay_us;
    aux_bmm150_dev2.intf = BMM150_I2C_INTF; // As per datasheet, aux interface with bmi270 will support only for I2C

    dev->intf = BMI2_I2C_INTF;   // To initialize the user I2C function
    dev->intf_ptr = &dev_addr2;   // I2C-Address
    dev->read = i2c_read_second;        // Callback functions for i2c read (defined in this file)
    dev->write = i2c_write_second;      // Callback functions for i2c write (defined in this file)
    dev->delay_us = delay_us;    // Callback functions for delay (defined in this file)
    dev->read_write_len = 30;    // Limitation of the Wire library
    dev->config_file_ptr = NULL; // Assign to NULL to load the default config file.

    // Initialise IMU and show error if exists
    rslt = bmi270_init(dev);
    bmi270_error_codes_print_result(rslt);

    /* Pull-up resistor 2k is set to the trim register */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, dev);
    bmi270_error_codes_print_result(rslt);

    /* Enable the accel, gyro and aux sensor. */
    rslt = bmi270_sensor_enable(sensor_list, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    // // Pin configuration
    // int_pin_cfg.pin_type = BMI2_INT1;
    // int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    // int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    // int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    // int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_INPUT_DISABLE;
    // int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    // // Configure interrupt PINs
    // rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    // bmi270_error_codes_print_result(rslt);

    /* Configurations for acc. */
    sens_cfg[ACC].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[ACC].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    sens_cfg[ACC].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[ACC].cfg.acc.range = BMI2_ACC_RANGE_4G;

    /* Configurations for gyro. */
    sens_cfg[GYR].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[GYR].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[GYR].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    sens_cfg[GYR].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[GYR].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    sens_cfg[GYR].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;

    /* Configurations for aux. */
    sens_cfg[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    sens_cfg[AUX].cfg.aux.manual_en = BMI2_ENABLE;
    sens_cfg[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    sens_cfg[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    sens_cfg[AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;
    sens_cfg[AUX].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    sens_cfg[AUX].cfg.aux.aux_en = BMI2_ENABLE;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_set_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Initialize bmm150. */
    rslt = bmm150_init(&aux_bmm150_dev2);
    bmm150_error_codes_print_result(rslt);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &aux_bmm150_dev2);
    bmm150_error_codes_print_result(rslt);

    rslt = bmi270_get_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    /* Disable manual mode so that the data mode is enabled. */
    sens_cfg[AUX].cfg.aux.manual_en = BMI2_DISABLE;

    /* Set the aux configurations. */
    rslt = bmi270_set_sensor_config(sens_cfg, 3, dev);
    bmi270_error_codes_print_result(rslt);

    // /* Map data ready interrupt to interrupt pin. */
    // rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    // bmi270_error_codes_print_result(rslt);

    Serial.println("MAGNETOMETER, ACC, AND GYR DATA IN DATA MODE");
}



/*!
 * @brief Read data from IMU
 *
 * @param[in] dev    : Pointer to device
 * @param[in] result : Pointer to data array
 *
 * @return void
 */

/*
void bmi270_read_sensor_data(struct bmi2_dev *dev, float result[])
{
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, dev);
    bmi270_error_codes_print_result(rslt);

    struct bmm150_mag_data mag_data;
    rslt = bmm150_aux_mag_data(sensor_data.aux_data, &mag_data, &aux_bmm150_dev1);
    bmi270_error_codes_print_result(rslt);

     //Converting lsb to meter per second squared for 16 bit accelerometer at 4G range. 
    result[ACC * 3 + 0] = lsb_to_mps2(sensor_data.acc.x, 4, dev->resolution);
    result[ACC * 3 + 1] = lsb_to_mps2(sensor_data.acc.y, 4, dev->resolution);
    result[ACC * 3 + 2] = lsb_to_mps2(sensor_data.acc.z, 4, dev->resolution);

    // Converting lsb to degree per second for 16 bit gyro at 2000dps range. 
    result[GYR * 3 + 0] = lsb_to_dps(sensor_data.gyr.x, 2000, dev->resolution);
    result[GYR * 3 + 1] = lsb_to_dps(sensor_data.gyr.y, 2000, dev->resolution);
    result[GYR * 3 + 2] = lsb_to_dps(sensor_data.gyr.z, 2000, dev->resolution);

    // Compensating the raw auxiliary data to uT available from the BMM150 API. 
    result[AUX * 3 + 0] = mag_data.x;
    result[AUX * 3 + 1] = mag_data.y;
    result[AUX * 3 + 2] = mag_data.z;
}
*/

void bmi270_read_sensor_data_first(struct bmi2_dev *dev, float result[])
{
    int8_t rslt = bmi2_get_sensor_data(&sensor_data_first, dev);
    bmi270_error_codes_print_result(rslt);

    struct bmm150_mag_data mag_data;
    rslt = bmm150_aux_mag_data(sensor_data_first.aux_data, &mag_data, &aux_bmm150_dev1);
    bmi270_error_codes_print_result(rslt);

    /* Converting lsb to meter per second squared for 16 bit accelerometer at 4G range. */
    result[ACC * 3 + 0] = lsb_to_mps2(sensor_data_first.acc.x, 4, dev->resolution);
    result[ACC * 3 + 1] = lsb_to_mps2(sensor_data_first.acc.y, 4, dev->resolution);
    result[ACC * 3 + 2] = lsb_to_mps2(sensor_data_first.acc.z, 4, dev->resolution);

    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
    result[GYR * 3 + 0] = lsb_to_dps(sensor_data_first.gyr.x, 2000, dev->resolution);
    result[GYR * 3 + 1] = lsb_to_dps(sensor_data_first.gyr.y, 2000, dev->resolution);
    result[GYR * 3 + 2] = lsb_to_dps(sensor_data_first.gyr.z, 2000, dev->resolution);

    /* Compensating the raw auxiliary data to uT available from the BMM150 API. */
    result[AUX * 3 + 0] = mag_data.x;
    result[AUX * 3 + 1] = mag_data.y;
    result[AUX * 3 + 2] = mag_data.z;
}




void bmi270_read_sensor_data_second(struct bmi2_dev *dev, float result[])
{
    int8_t rslt = bmi2_get_sensor_data(&sensor_data_second, dev);
    bmi270_error_codes_print_result(rslt);

    struct bmm150_mag_data mag_data;
    rslt = bmm150_aux_mag_data(sensor_data_second.aux_data, &mag_data, &aux_bmm150_dev2);
    bmi270_error_codes_print_result(rslt);

    /* Converting lsb to meter per second squared for 16 bit accelerometer at 4G range. */
    result[ACC * 3 + 0] = lsb_to_mps2(sensor_data_second.acc.x, 4, dev->resolution);
    result[ACC * 3 + 1] = lsb_to_mps2(sensor_data_second.acc.y, 4, dev->resolution);
    result[ACC * 3 + 2] = lsb_to_mps2(sensor_data_second.acc.z, 4, dev->resolution);

    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
    result[GYR * 3 + 0] = lsb_to_dps(sensor_data_second.gyr.x, 2000, dev->resolution);
    result[GYR * 3 + 1] = lsb_to_dps(sensor_data_second.gyr.y, 2000, dev->resolution);
    result[GYR * 3 + 2] = lsb_to_dps(sensor_data_second.gyr.z, 2000, dev->resolution);

    /* Compensating the raw auxiliary data to uT available from the BMM150 API. */
    result[AUX * 3 + 0] = mag_data.x;
    result[AUX * 3 + 1] = mag_data.y;
    result[AUX * 3 + 2] = mag_data.z;
}


/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}

/*!
 * @brief Callback function for delay processing
 *
 * @param[in] period : Delay in microseconds
 *
 * @return void
 */
void delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

/*!
 * @brief Flash blue led in case of any error
 *
 * @return void
 */
void bmi270_panic_led_trap(void)
{
    while (1)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi270_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
    // No error, do nothing
    case BMI2_OK:
        return; /* Do nothing */
        break;

    // Several error codes
    case BMI2_E_NULL_PTR:
        Serial.println("Error [" + String(rslt) + "] : Null pointer");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_COM_FAIL:
        Serial.println("Error [" + String(rslt) + "] : Communication failure");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_DEV_NOT_FOUND:
        Serial.println("Error [" + String(rslt) + "] : Device not found");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_OUT_OF_RANGE:
        Serial.println("Error [" + String(rslt) + "] : Out of range");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_ACC_INVALID_CFG:
        Serial.println("Error [" + String(rslt) + "] : Invalid accel configuration");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_GYRO_INVALID_CFG:
        Serial.println("Error [" + String(rslt) + "] : Invalid gyro configuration");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
        Serial.println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_SENSOR:
        Serial.println("Error [" + String(rslt) + "] : Invalid sensor");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_CONFIG_LOAD:
        Serial.println("Error [" + String(rslt) + "] : Configuration loading error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_PAGE:
        Serial.println("Error [" + String(rslt) + "] : Invalid page ");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_FEAT_BIT:
        Serial.println("Error [" + String(rslt) + "] : Invalid feature bit");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_INT_PIN:
        Serial.println("Error [" + String(rslt) + "] : Invalid interrupt pin");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_SET_APS_FAIL:
        Serial.println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_AUX_INVALID_CFG:
        Serial.println("Error [" + String(rslt) + "] : Invalid auxilliary configuration");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_AUX_BUSY:
        Serial.println("Error [" + String(rslt) + "] : Auxilliary busy");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_SELF_TEST_FAIL:
        Serial.println("Error [" + String(rslt) + "] : Self test failed");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_REMAP_ERROR:
        Serial.println("Error [" + String(rslt) + "] : Remapping error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
        Serial.println("Error [" + String(rslt) + "] : Gyro user gain update failed");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_SELF_TEST_NOT_DONE:
        Serial.println("Error [" + String(rslt) + "] : Self test not done");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_INPUT:
        Serial.println("Error [" + String(rslt) + "] : Invalid input");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_STATUS:
        Serial.println("Error [" + String(rslt) + "] : Invalid status");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_CRT_ERROR:
        Serial.println("Error [" + String(rslt) + "] : CRT error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_ST_ALREADY_RUNNING:
        Serial.println("Error [" + String(rslt) + "] : Self test already running");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
        Serial.println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_DL_ERROR:
        Serial.println("Error [" + String(rslt) + "] : DL error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_PRECON_ERROR:
        Serial.println("Error [" + String(rslt) + "] : PRECON error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_ABORT_ERROR:
        Serial.println("Error [" + String(rslt) + "] : Abort error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
        Serial.println("Error [" + String(rslt) + "] : Gyro self test error");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
        Serial.println("Error [" + String(rslt) + "] : Gyro self test timeout");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
        Serial.println("Error [" + String(rslt) + "] : Write cycle ongoing");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
        Serial.println("Error [" + String(rslt) + "] : Write cycle timeout");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_ST_NOT_RUNING:
        Serial.println("Error [" + String(rslt) + "] : Self test not running");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_DATA_RDY_INT_FAILED:
        Serial.println("Error [" + String(rslt) + "] : Data ready interrupt failed");
        bmi270_panic_led_trap();
        break;
    case BMI2_E_INVALID_FOC_POSITION:
        Serial.println("Error [" + String(rslt) + "] : Invalid FOC position");
        bmi270_panic_led_trap();
        break;
    default:
        Serial.println("Error [" + String(rslt) + "] : Unknown error code");
        bmi270_panic_led_trap();
        break;
    }
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
static void bmm150_error_codes_print_result(int8_t rslt)
{
    if (rslt != BMM150_OK)
    {
        switch (rslt)
        {
        case BMM150_E_NULL_PTR:
            Serial.println("Error [" + String(rslt) + "] : Null pointer error.");
            Serial.println("It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.");
            bmi270_panic_led_trap();
            break;

        case BMM150_E_COM_FAIL:
            Serial.println("Error [" + String(rslt) + "] : Communication failure error.");
            Serial.println("It occurs due to read/write operation failure and also due to power failure during communication");
            bmi270_panic_led_trap();
            break;

        case BMM150_E_DEV_NOT_FOUND:
            Serial.println("Error [" + String(rslt) + "] : Device not found error. It occurs when the device chip id is incorrectly read");
            bmi270_panic_led_trap();
            break;

        case BMM150_E_INVALID_CONFIG:
            Serial.println("Error [" + String(rslt) + "] : Invalid sensor configuration.");
            Serial.println("It occurs when there is a mismatch in the requested feature with the available one");
            bmi270_panic_led_trap();
            break;

        default:
            Serial.println("Error [" + String(rslt) + "] : Unknown error code");
            bmi270_panic_led_trap();
            break;
        }
    }
}
