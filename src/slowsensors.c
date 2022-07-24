#include <zephyr.h>
#include <kernel.h>

#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>

#include "slowsensors.h"
#include "graph.h"
#include "sensorlog.h"
#include "bt.h"
#include <bluetooth/services/bas.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(slowsensors);

#define SENSORS_PRIORITY             0
#define SENSORS_STACKSIZE         1024

K_THREAD_STACK_DEFINE(sensors_stack, SENSORS_STACKSIZE);

static struct k_thread sensors_thread_data;

static k_tid_t sensors_tid;

static inline uint32_t ms_to_cycles(uint32_t ms){
    return (1 + (ms * sys_clock_hw_cycles_per_sec()) / 1000);
}

typedef struct voltToPercent_s 
{
    int16_t mv;
    int16_t percent;
} voltToPercent_t;

static voltToPercent_t voltConvTable[] = {
    {4200, 100},
    {4100, 95},
    {4000, 85},
    {3900, 68},
    {3800, 50},
    {3700, 25},
    {3650, 15},
    {3600, 5},
    {3550, 1},
};

static int voltToPercent(int vbat){
    int vMax = 0;
    int vMin = 0;
    int pMax = 0, pMin = 0, percent;
    for (int i = 0; i < sizeof(voltConvTable)/sizeof(voltConvTable[0]); ++i){
        if (vbat < voltConvTable[i].mv){
            vMax = voltConvTable[i].mv;
            pMax = voltConvTable[i].percent;
        } else {
            vMin = voltConvTable[i].mv;
            pMin = voltConvTable[i].percent;
            break;
        }
    }
    if (vMax > 0){
        if (vMin > 0){
            percent = (pMin * (vMax-vbat) + pMax * (vbat - vMin))/(vMax - vMin);
        } else {
            percent = 0;
        }
    } else {
        percent = 101;
    }
    return percent;
}

static int batt_volt_mv;
static int batt_percent;
static uint32_t adc_deadline;

static uint8_t calc_CRC8(uint8_t *data, int size, uint8_t polynomial){
    uint8_t crc = 0;
    for(int i = size - 1; i >= 0; --i){
        crc ^= data[i];
        for (int bit = 8; bit > 0; --bit){
            if (crc & 0x80){
                crc = (crc << 1) ^ polynomial;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static int getAdcValue(struct device *dev, int ref, int gain, int channel) {
    const struct adc_channel_cfg cfg = { gain, ref, ADC_ACQ_TIME_DEFAULT, channel, 0, channel, channel };
    int16_t value;
    const struct adc_sequence sequence = {
        .channels    = BIT(channel),
        .buffer      = &value,
        .buffer_size = sizeof(value),
        .resolution  = 12,
    };

    int ret = adc_channel_setup(dev, &cfg);
    if (ret) {
        LOG_ERR("Setting up of the %d channel failed with code %d", channel, ret);
        return 0;
    }

    ret = adc_read(dev, &sequence);
    if (ret) {
        LOG_ERR("Read ADC failed %d", ret);
        return 0;
    }
    return value;
}

#define _ADC_GAIN_NOM(nom) ADC_GAIN_ ## nom
#define ADC_GAIN_NOM(nom) _ADC_GAIN_NOM(nom)
#define _ADC_GAIN_NOM_DENOM(nom,denom) ADC_GAIN_ ## nom ## _ ## denom
#define ADC_GAIN_NOM_DENOM(nom,denom) _ADC_GAIN_NOM_DENOM(nom, denom)

#ifdef DT_INST_0_ADC_VBAT_ADCS_CONTROLLER

#if DT_INST_0_ADC_VBAT_GAIN_DENOM
#define ADC_VBAT_GAIN ADC_GAIN_NOM_DENOM(DT_INST_0_ADC_VBAT_GAIN_NOM, DT_INST_0_ADC_VBAT_GAIN_DENOM)
#else
#define ADC_VBAT_GAIN ADC_GAIN_NOM(DT_INST_0_ADC_VBAT_GAIN_NOM)
#endif
#endif // DT_INST_0_ADC_VBAT_ADCS_CONTROLLER

int getBattPercentage(){
    return batt_percent;
}

static int updateAdcSensors(int force){
    int rv = 0;
#ifdef DT_INST_0_ADC_VBAT_ADCS_CONTROLLER
    struct device * vbat_adc_dev = device_get_binding(DT_INST_0_ADC_VBAT_ADCS_CONTROLLER);
    if ((force) || (((int32_t)(k_cycle_get_32() - adc_deadline)) >= 0)){
        int adc_counts = getAdcValue(vbat_adc_dev, ADC_REF_INTERNAL, ADC_VBAT_GAIN , DT_INST_0_ADC_VBAT_CHANNEL);
        batt_volt_mv = ((adc_counts)*(DT_INST_0_ADC_VBAT_MULTIPLYER)) >> (DT_INST_0_ADC_VBAT_SHIFT);
        // TODO: averaging
        batt_percent = voltToPercent(batt_volt_mv);
#ifdef CONFIG_BT_GATT_BAS
        bt_gatt_bas_set_battery_level(batt_percent);
#endif        
        LOG_WRN("Battery voltage is %dmv, charge=%d%%", batt_volt_mv, batt_percent);
        adc_deadline = k_cycle_get_32() + ms_to_cycles(DT_INST_0_ADC_VBAT_PERIOD);
        ++rv;
    }
#endif // DT_INST_0_ADC_VBAT_ADCS_CONTROLLER

    return rv;
}

// ************************************************************************************************************
// I2C Barometer MS561101BA or MS5803
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// MS 5803 has different 2nd order correction

//MS5611/5803/5837 I2C sensor
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME  
// thresholds for error protection based on measurement results comparison
#ifndef BARO_DP_THRESHOLD
#define BARO_DP_THRESHOLD 150L      // 150Pa is ok for mos cases excepr for fast elevators
#endif

#ifndef BARO_DT_THRESHOLD
#define BARO_DT_THRESHOLD 200L       //2 deg C is OK even for fast slew rate
#endif

#ifndef BARO_PROTECT_COUNTER_MAX
#define BARO_PROTECT_COUNTER_MAX 15 // maximum shift 15 bits 
#endif

#define BARO_TAB_SIZE        16

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E
#define MS561101BA_PROM        0xA2
#define MS561101BA_RES         0x00

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
    // sensor registers pRef the MS561101BA datasheet
    struct device * i2c_master;
    uint16_t c[7];
    uint32_t ut; //uncompensated T
    uint32_t up; //uncompensated P
    uint8_t state;
    uint8_t i2c_address;
    uint8_t osr;
    uint32_t deadline;
    int8_t baru_protect_counter;
    int32_t baro_pressure;       // in 0.01 bar
    int32_t baro_pressureSum;
    int32_t baro_temperature;    // in 0.01 deg C
    int32_t baro_pressure_tab[BARO_TAB_SIZE];
    uint8_t baro_count;
    uint8_t baro_tab_idx;    
} ms5xxx_ctx;

static int i2c_ms5xxx_reset(){
    uint8_t cmd = MS561101BA_RESET;
    int err = i2c_write(ms5xxx_ctx.i2c_master, &cmd, 1, ms5xxx_ctx.i2c_address);
    if (err) {
        LOG_ERR("MS5xxx Baro sensor reset write failed");
    }
    return err;
}

static void i2c_ms5xxx_read_calibration(){
    if (!ms5xxx_ctx.i2c_master) return;
    uint8_t raw[2];
    for(int i = 0; i < 6;i++) {
        int rv = i2c_burst_read(ms5xxx_ctx.i2c_master, ms5xxx_ctx.i2c_address, MS561101BA_PROM + (i << 1), &raw[0], 2);
        if (rv < 0){
            LOG_ERR("MS5xxx Baro sensor PROM read failed at step %d",i);
        } else {
            uint16_t res = (raw[0] << 8) + raw[1];
            ms5xxx_ctx.c[i+1] = res;
            LOG_DBG("MS5xxx Baro sensor PROM[%d]=0x%x", i, res);
        }
    }
}

static void ms5xxx_init(const char * devname, uint8_t addr, uint8_t osr, uint8_t doCal) {
    ms5xxx_ctx.i2c_master = device_get_binding(devname);
    if (!ms5xxx_ctx.i2c_master) {
        LOG_ERR("i2c master %s not found", devname);
        return;
    }
    ms5xxx_ctx.i2c_address = addr;
    ms5xxx_ctx.osr = osr;
    if (i2c_ms5xxx_reset()){
        ms5xxx_ctx.i2c_master = NULL;
        return;
    }
    k_sleep(K_MSEC(10)); //*** reset seems to take 2.8ms, we wait 10ms to guarantee device is OK
    if (doCal) {
        ms5xxx_ctx.state = 0xff;
    }else {
        ms5xxx_ctx.state = 0xfe;
    }
    ms5xxx_ctx.baru_protect_counter = BARO_PROTECT_COUNTER_MAX;
    ms5xxx_ctx.deadline = k_cycle_get_32() + ms_to_cycles(100);
}

// read uncompensated temperature value: send command first
static void i2c_ms5xxx_ut_start() {
    if (!ms5xxx_ctx.i2c_master) return;
    uint8_t cmd = MS561101BA_TEMPERATURE + ms5xxx_ctx.osr;
    int err = i2c_write(ms5xxx_ctx.i2c_master, &cmd, 1, ms5xxx_ctx.i2c_address);
    if (err) {
        LOG_INF("MS5xxx Baro sensor UT start failed");
    }    
}

// read uncompensated pressure value: send command first
static void i2c_ms5xxx_up_start () {
    if (!ms5xxx_ctx.i2c_master) return;
    uint8_t cmd = MS561101BA_PRESSURE + ms5xxx_ctx.osr;
    int err = i2c_write(ms5xxx_ctx.i2c_master, &cmd, 1, ms5xxx_ctx.i2c_address);
    if (err) {
        LOG_INF("MS5xxx Baro sensor UP start failed");
    }        
}

static void i2c_ms5xxx_result_read(uint32_t * storage){
    if (!ms5xxx_ctx.i2c_master) return;
    uint8_t raw[3];
    int rv = i2c_burst_read(ms5xxx_ctx.i2c_master, ms5xxx_ctx.i2c_address, MS561101BA_RES, &raw[0], 3);
    uint32_t res = 0;
    if (rv < 0){
        LOG_ERR("MS5xxx Baro sensor result read failed");
    } else {    
        for (int i = 0; i < 3; ++i){
            res = (res << 8) + raw[i];
        }
        *storage = res;
        LOG_DBG("MS5xxx Baro sensor raw read 0x%x", res);
    }
}

// read uncompensated pressure value: read result bytes
static inline void i2c_ms5xxx_up_read () {
    i2c_ms5xxx_result_read(&ms5xxx_ctx.up);
}

// read uncompensated temperature value: read result bytes
static inline void i2c_ms5xxx_ut_read() {
    i2c_ms5xxx_result_read(&ms5xxx_ctx.ut);
}

static void i2c_ms5xxx_calculate() {
    int32_t off2, sens2, delt, newbaro_temperature, newbaro_pressure;
    int64_t T2 = 0;
    int64_t dT = (int32_t)ms5xxx_ctx.ut - ((int32_t)ms5xxx_ctx.c[5] << 8);
    newbaro_temperature = 2000 + ((dT * ms5xxx_ctx.c[6])>>23);
    int64_t off = ((uint32_t)ms5xxx_ctx.c[2] <<16) + ((dT * ms5xxx_ctx.c[4]) >> 7);
    int64_t sens = ((uint32_t)ms5xxx_ctx.c[1] <<15) + ((dT * ms5xxx_ctx.c[3]) >> 8);
    LOG_DBG("MS5xxx Baro sensor rp=0x%x rt=0x%d", ms5xxx_ctx.up, ms5xxx_ctx.ut);
#if (DT_INST_0_MEAS_MS5XXX_MODEL_NUM == 5803)                 // MS5803 barometer
    if (newbaro_temperature < 2000){ // temperature lower than 20st.C
//        T2=((dT*dT)>>31);
        delt = newbaro_temperature - 2000;
        delt = delt*delt;
        off2 = 3 * delt;
        sens2 = (7 * delt) >> 3;
        if (newbaro_temperature < -1500) { // temperature lower than -15st.C
            delt = newbaro_temperature + 1500;
            delt = delt*delt;
            sens2 += (delt << 1);
        }
    }else{
        if (newbaro_temperature >= 4500){
            delt = newbaro_temperature - 4500;
            delt = delt*delt;
            sens2 =-(delt >> 3);
         }else{
            sens2 = 0;
         }
        off2 = 0;
    }
    off -= off2; 
    sens -= sens2;
#else // MS5611 barometer
    if (newbaro_temperature < 2000) { // temperature lower than 20st.C
//        T2=((dT*dT)>>31);
        delt = newbaro_temperature - 2000;
        delt = 5 * delt*delt;
        off2 = delt >> 1;
        sens2 = delt >> 2;
        if (newbaro_temperature < -1500) { // temperature lower than -15st.C
            delt = newbaro_temperature + 1500;
            delt = delt*delt;
            off2 += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
        off -= off2; 
        sens -= sens2;
    }
#endif
    
    newbaro_pressure = (( (ms5xxx_ctx.up * sens ) >> 21) - off) >> 15;
    if (newbaro_temperature < 2000) { // T2 Correction is the same for both 5611 and 5803 sensors
        T2 = ((dT*dT)>>31);
        newbaro_temperature -= T2;
    }
    // finished calculation, compare new and old results
    int32_t pdp = (newbaro_pressure - ms5xxx_ctx.baro_pressure);
    if (pdp < 0) pdp = -pdp;
    int32_t pdt = (newbaro_temperature - ms5xxx_ctx.baro_temperature);
    if (pdt < 0) pdt = -pdt;
    if ((pdp > ((BARO_DP_THRESHOLD) << ms5xxx_ctx.baru_protect_counter)) || 
        (pdt > ((BARO_DT_THRESHOLD) << ms5xxx_ctx.baru_protect_counter))){
        //deltas are too high, this can mean measure error, so we increase counter and thresholds 
        ms5xxx_ctx.baru_protect_counter++;
        if(ms5xxx_ctx.baru_protect_counter > BARO_PROTECT_COUNTER_MAX) {
            ms5xxx_ctx.baru_protect_counter = BARO_PROTECT_COUNTER_MAX;
        }
    }else {
        // normal operation - update values
        ms5xxx_ctx.baro_pressure = newbaro_pressure;             
        ms5xxx_ctx.baro_temperature = newbaro_temperature;
        ms5xxx_ctx.baru_protect_counter = 0;
    }
    LOG_DBG("MS5xxx Baro sensor p=%dpa T=%d.%02dC c=%d", newbaro_pressure, 
        newbaro_temperature / 100, newbaro_temperature % 100,
        ms5xxx_ctx.baru_protect_counter);
}

static void ms5xxx_baro_tab_update() {
    ms5xxx_ctx.baro_pressureSum -= ms5xxx_ctx.baro_pressure_tab[ms5xxx_ctx.baro_tab_idx];
    ms5xxx_ctx.baro_pressure_tab[ms5xxx_ctx.baro_tab_idx] = ms5xxx_ctx.baro_pressure;
    ms5xxx_ctx.baro_pressureSum += ms5xxx_ctx.baro_pressure;
    ms5xxx_ctx.baro_tab_idx++;
    ms5xxx_ctx.baro_count++;
    if (ms5xxx_ctx.baro_tab_idx == BARO_TAB_SIZE) ms5xxx_ctx.baro_tab_idx = 0;
    if (ms5xxx_ctx.baro_count >= BARO_TAB_SIZE) ms5xxx_ctx.baro_count = BARO_TAB_SIZE;
}

//return 0: no data available, no computation ;    1: new value available    ; 2: no new value, but computation time
static int ms5xxx_update(int force) {
    if (!ms5xxx_ctx.i2c_master) return 0;
    if ((!force) && (((int32_t)(k_cycle_get_32() - ms5xxx_ctx.deadline)) < 0) ) return 0;
    ms5xxx_ctx.deadline = k_cycle_get_32() + ms_to_cycles(10);    // UT and UP conversion take 8.5ms so we do next reading after 10ms
    if (ms5xxx_ctx.state == 0) {
        i2c_ms5xxx_ut_read(); 
        i2c_ms5xxx_up_start(); 
        ms5xxx_ctx.state = 1;
        return 1;
    } else if (ms5xxx_ctx.state == 1){
        i2c_ms5xxx_up_read();
        i2c_ms5xxx_ut_start(); 
        i2c_ms5xxx_calculate();
        ms5xxx_baro_tab_update();
        ms5xxx_ctx.state = 0; 
        return 2;
    }else if (ms5xxx_ctx.state == 0xff) {
        i2c_ms5xxx_read_calibration();
        i2c_ms5xxx_ut_start(); 
        ms5xxx_ctx.state=0;
        return 0;
    }else{
        i2c_ms5xxx_ut_start(); 
        ms5xxx_ctx.state = 0;
        return 0;
    }
}

static int ms5xxx_measure(int force) {
    if (!ms5xxx_ctx.i2c_master) return 0;
    int delay_ms = 10;
    if ((!force) && (((int32_t)(k_cycle_get_32() - ms5xxx_ctx.deadline)) < 0) ) return 0;
    ms5xxx_ctx.deadline = k_cycle_get_32() + ms_to_cycles(delay_ms);    // UT and UP conversion take 8.5ms so we do next reading after 10ms
    if (ms5xxx_ctx.state == 0) {
        i2c_ms5xxx_up_start(); 
        ms5xxx_ctx.state = 1;
    } else if (ms5xxx_ctx.state == 1){
        i2c_ms5xxx_ut_start(); 
        ms5xxx_ctx.state = 0; 
    }else if (ms5xxx_ctx.state == 0xff) {
        i2c_ms5xxx_read_calibration();
        i2c_ms5xxx_ut_start(); 
        ms5xxx_ctx.state=0;
    }else{
        i2c_ms5xxx_ut_start(); 
        ms5xxx_ctx.state = 0;
    }
    return delay_ms;
}

static int ms5xxx_fetch(int force) {
    if (!ms5xxx_ctx.i2c_master) return 0;
    if ((!force) && (((int32_t)(k_cycle_get_32() - ms5xxx_ctx.deadline)) < 0) ) return 0;
    if (ms5xxx_ctx.state == 0) {
        i2c_ms5xxx_ut_read();
        return 0;
    } else if (ms5xxx_ctx.state == 1){
        i2c_ms5xxx_up_read();
        i2c_ms5xxx_calculate();
        ms5xxx_baro_tab_update();
        return 1;
    }
    return 0;
}

#endif
// *** End MS5611/5803/5837


// ************************************************************************************************************
// I2C Humidity Sensor SHT21
// ************************************************************************************************************
//
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME

// registers of the device  

#define SHT21_START_TEMP 0xF3 // Trigger T measurement, no hold master
#define SHT21_START_RH   0xF5 // Trigger RH measurement, no hold master
#define SHT21_WRITE_UR   0xE6 // Write User Reg
#define SHT21_READ_UR    0xE7 // Read User Reg
#define SHT21_RESET      0xFE // Reset

// constants
#define SHT21_POLYNOMIAL   0x31 //0x131
#define SHT21_TEMP_PEAK     100 //1 deg per measure
#define SHT21_RH_PEAK    10*256 // 10 percent per measure
#define SHT21_PEAK_LIM        2 //resets before accept peak is expected to be true

static struct {
    struct device * i2c_master;
    uint32_t deadline;
    int temperature;
    int RH;
    int new_temp;
    uint8_t status;
    uint8_t peak_count;
    uint8_t i2c_address;
} sht21_ctx;

// Start temp measure
static inline void sht21_reset() {
    if (!sht21_ctx.i2c_master) return;
    uint8_t cmd = SHT21_RESET;
    LOG_DBG("SHT2x Reset");
    i2c_write(sht21_ctx.i2c_master, &cmd, 1, sht21_ctx.i2c_address);
    sht21_ctx.status = 0xff;
    k_sleep(K_MSEC(100)); //sht21 seems to hold i2c line after reset
}
// Start temp measure
static inline void sht21_temp_start() {
    if (!sht21_ctx.i2c_master) return;
    uint8_t cmd = SHT21_START_TEMP;
    LOG_DBG("SHT2x Temp start");
    i2c_write(sht21_ctx.i2c_master, &cmd, 1, sht21_ctx.i2c_address);
}

// Start RH measure
static inline void sht21_RH_start() {
    if (!sht21_ctx.i2c_master) return;
    uint8_t cmd = SHT21_START_RH;
    LOG_DBG("SHT2x RH start");
    i2c_write(sht21_ctx.i2c_master, &cmd, 1, sht21_ctx.i2c_address);
}

static inline uint16_t sht21_read_result(){
    if (!sht21_ctx.i2c_master) return 0;
    uint16_t val; uint8_t raw[2];
    if (i2c_read(sht21_ctx.i2c_master, &raw[0], 2, sht21_ctx.i2c_address)){
        LOG_ERR("SHT2x: result read failed");
        return 0;
    }
    val = raw[1] + (raw[0] << 8);
    return val;
}

static inline uint8_t sht21_read_result_CRC(uint16_t * res){
    if (!sht21_ctx.i2c_master) return 0;
    uint16_t val;
    uint8_t raw[4];
    if (i2c_read(sht21_ctx.i2c_master, &raw[0], 3, sht21_ctx.i2c_address)){
        LOG_ERR("SHT2x: result read failed");
        return 0;
    }
    val = raw[1] + (raw[0] << 8);
    uint8_t crc_expected = calc_CRC8((uint8_t*) &val, 2, SHT21_POLYNOMIAL);
    if (crc_expected == raw[2]){
      *res = val;
      return 1;
    }else{
        LOG_WRN("SHT21: bad CRC: 0x%x-0x%x-0x%x, expected 0x%x", raw[0], raw[1], raw[2], crc_expected);
      return 0;
    }
}

static void sht21_sensor_init(const char * devname, uint8_t addr){
    sht21_ctx.i2c_master = device_get_binding(devname);
    if (!sht21_ctx.i2c_master) {
        LOG_ERR("i2c master %s not found", devname);
        return;
    }
    sht21_ctx.i2c_address = addr;
    sht21_reset();
    sht21_ctx.peak_count = 1 + SHT21_PEAK_LIM;
    sht21_ctx.deadline = k_cycle_get_32();
}

static int sht21_sensor_update(int force){
    uint16_t sensor_data;
    if ((!force) && (((int32_t)(k_cycle_get_32() - sht21_ctx.deadline)) < 0) ) return 0;
    LOG_DBG("SHT2x Update entry, state %x deadline updated to %d", sht21_ctx.status, sht21_ctx.deadline);
    sht21_ctx.deadline = k_cycle_get_32() + ms_to_cycles(100);
    LOG_DBG("SHT2x Update entry, state %x deadline updated to %d", sht21_ctx.status, sht21_ctx.deadline);
    if (1 == sht21_ctx.status){
        if (sht21_read_result_CRC(&sensor_data)){
            if ((sensor_data & 0x02) == 0x00) {
                sht21_ctx.new_temp = -4685 + ((17572L * sensor_data) >> 16);
                int16_t delta_T = sht21_ctx.new_temp - sht21_ctx.temperature;
                if ((delta_T < -(SHT21_TEMP_PEAK)) || (delta_T > (SHT21_TEMP_PEAK))){
                    if (0 == sht21_ctx.peak_count) sht21_ctx.peak_count = 1;
                }else{
                    sht21_ctx.temperature = sht21_ctx.new_temp;
                }
                LOG_DBG("SHT2x: temp=%d.%d, peak=%d", sht21_ctx.new_temp / 100, sht21_ctx.new_temp % 100, sht21_ctx.peak_count);
            }
        }
        sht21_RH_start();
        sht21_ctx.status = 2;
        return 1;
    } else if (2 == sht21_ctx.status){
        if (sht21_read_result_CRC(&sensor_data)){
            if ((sensor_data & 0x02) == 0x02) {
                int16_t new_sht21_RH = -6*256 + ((125 * sensor_data) >> 8);
                int16_t delta_RH = new_sht21_RH - sht21_ctx.RH;
                if ((delta_RH < -(SHT21_RH_PEAK)) || (delta_RH > (SHT21_RH_PEAK))){
                    if (0 == sht21_ctx.peak_count) sht21_ctx.peak_count = 1;
                } else {
                    sht21_ctx.RH = new_sht21_RH;
                }
                LOG_DBG("SHT2x: RH=%d%%, peak=%d", new_sht21_RH, sht21_ctx.peak_count);
                if (sht21_ctx.peak_count > SHT21_PEAK_LIM){
                    sht21_ctx.RH = new_sht21_RH;
                    sht21_ctx.temperature = sht21_ctx.new_temp;
                    sht21_ctx.peak_count = 0;
                    LOG_WRN("SHT2x: Peak overide: temp=%d.%d RH=%d%%", sht21_ctx.temperature / 100, sht21_ctx.temperature % 100, new_sht21_RH >> 8);
                }
            }
        }
        if (sht21_ctx.peak_count) {
            sht21_reset();
            LOG_WRN("SHT2x: peak=%d, resetting device",sht21_ctx.peak_count);
            ++sht21_ctx.peak_count;
            return 0;
        } else {
            sht21_temp_start();
            sht21_ctx.status = 1;
            return 2;    
        }
    }else{
        // after Reset
        sht21_temp_start();
        sht21_ctx.status = 1;
        return 0;
    }
}

static int sht21_sensor_measure(int force){
    if ((!force) && (((int32_t)(k_cycle_get_32() - sht21_ctx.deadline)) < 0) ) return 0;
    LOG_DBG("SHT2x Update entry, state %x deadline updated to %d", sht21_ctx.status, sht21_ctx.deadline);
    int delay_ms = 100;
    sht21_ctx.deadline = k_cycle_get_32() + ms_to_cycles(delay_ms);
    LOG_DBG("SHT2x Update entry, state %x deadline updated to %d", sht21_ctx.status, sht21_ctx.deadline);
    if (1 == sht21_ctx.status){
        sht21_RH_start();
        sht21_ctx.status = 2;
    } else if (2 == sht21_ctx.status){
        if (sht21_ctx.peak_count) {
            sht21_reset();
            LOG_WRN("SHT2x: peak=%d, resetting device",sht21_ctx.peak_count);
            ++sht21_ctx.peak_count;
        } else {
            sht21_temp_start();
            sht21_ctx.status = 1;
        }
    }else{
        // after Reset
        sht21_temp_start();
        sht21_ctx.status = 1;
    }
    return delay_ms;
}

static int sht21_sensor_fetch(int force){
    uint16_t sensor_data;
    if ((!force) && (((int32_t)(k_cycle_get_32() - sht21_ctx.deadline)) < 0) ) return 0;
    int rv = 0;
    if (1 == sht21_ctx.status){
        if (sht21_read_result_CRC(&sensor_data)){
            if ((sensor_data & 0x02) == 0x00) {
                sht21_ctx.new_temp = -4685 + ((17572L * sensor_data) >> 16);
                int16_t delta_T = sht21_ctx.new_temp - sht21_ctx.temperature;
                if ((delta_T < -(SHT21_TEMP_PEAK)) || (delta_T > (SHT21_TEMP_PEAK))){
                    if (0 == sht21_ctx.peak_count) sht21_ctx.peak_count = 1;
                }else{
                    sht21_ctx.temperature = sht21_ctx.new_temp;
                    rv = 1;
                }
                LOG_DBG("SHT2x: temp=%d.%d, peak=%d", sht21_ctx.new_temp / 100, sht21_ctx.new_temp % 100, sht21_ctx.peak_count);
            }
        }
    } else if (2 == sht21_ctx.status){
        if (sht21_read_result_CRC(&sensor_data)){
            if ((sensor_data & 0x02) == 0x02) {
                int16_t new_sht21_RH = -6*256 + ((125 * sensor_data) >> 8);
                int16_t delta_RH = new_sht21_RH - sht21_ctx.RH;
                if ((delta_RH < -(SHT21_RH_PEAK)) || (delta_RH > (SHT21_RH_PEAK))){
                    if (0 == sht21_ctx.peak_count) sht21_ctx.peak_count = 1;
                } else {
                    sht21_ctx.RH = new_sht21_RH;
                    rv = 1;
                }
                LOG_DBG("SHT2x: RH=%d%%, peak=%d", new_sht21_RH, sht21_ctx.peak_count);
                if (sht21_ctx.peak_count > SHT21_PEAK_LIM){
                    sht21_ctx.RH = new_sht21_RH;
                    sht21_ctx.temperature = sht21_ctx.new_temp;
                    sht21_ctx.peak_count = 0;
                    rv = 1;
                    LOG_WRN("SHT2x: Peak overide: temp=%d.%d RH=%d%%", sht21_ctx.temperature / 100, sht21_ctx.temperature % 100, new_sht21_RH >> 8);
                }
            }
        }
    }
    return rv;
}
#endif
// *** End SHT21

// ************************************************************************************************************
// I2C Light and color sensor VEML3328
// ************************************************************************************************************
//
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME

// registers of the device  

#define VEML3328_REG_CTRL      0x00
#define VEML3328_REG_ID        0x0C

#define VEML3328_REG_OUT_BASE  0x4

#define VEML3328_INDEX_CLEAR   0x0
#define VEML3328_INDEX_RED     0x1
#define VEML3328_INDEX_GREEN   0x2
#define VEML3328_INDEX_LUX     0x2
#define VEML3328_INDEX_BLUE    0x3
#define VEML3328_INDEX_IR      0x4
#define VEML3328_INDEX_MAX     0x5

#define VEML3328_REG_OUT_CLEAR   (VEML3328_INDEX_CLEAR + VEML3328_REG_OUT_BASE)
#define VEML3328_REG_OUT_REG     (VEML3328_INDEX_RED   + VEML3328_REG_OUT_BASE)
#define VEML3328_REG_OUT_GREEN   (VEML3328_INDEX_GREEN + VEML3328_REG_OUT_BASE)
#define VEML3328_REG_OUT_LUX     (VEML3328_INDEX_LUX   + VEML3328_REG_OUT_BASE)
#define VEML3328_REG_OUT_BLUE    (VEML3328_INDEX_BLUE  + VEML3328_REG_OUT_BASE)
#define VEML3328_REG_OUT_IR      (VEML3328_INDEX_IR    + VEML3328_REG_OUT_BASE)

// constants
#define VEML3328_ID_VAL        0x28

#define VEML3328_SAFE_VAL_THR 0x3FFF

#define VEML3328_DG_1x  0x0000
#define VEML3328_DG_2x  0x1000
#define VEML3328_DG_4x  0x2000

#define VEML3328_GAIN_d2  0x0C00
#define VEML3328_GAIN_1x  0x0000
#define VEML3328_GAIN_2x  0x0400
#define VEML3328_GAIN_4x  0x0800

#define VEML3328_IT_050  0x0000
#define VEML3328_IT_100  0x0010
#define VEML3328_IT_200  0x0020
#define VEML3328_IT_400  0x0030
#define VEML3328_IT_MSK  0x0030

#define VEML3328_SENS_NORM  0x0000
#define VEML3328_SENS_LOW   0x0040

#define VEML3328_TRIG       0x4
#define VEML3328_AF         0x8
#define VEML3328_SHUTDOWN   0x8001

typedef struct {
    uint16_t val;
    uint16_t mul;
} veml3328_range_t;

static const veml3328_range_t veml3328_ranges[] = {
    {VEML3328_DG_4x | VEML3328_GAIN_4x | VEML3328_IT_400 | VEML3328_SENS_NORM, 1*1*1*1},
    {VEML3328_DG_1x | VEML3328_GAIN_4x | VEML3328_IT_400 | VEML3328_SENS_NORM, 4*1*1*1},
    {VEML3328_DG_1x | VEML3328_GAIN_d2 | VEML3328_IT_400 | VEML3328_SENS_NORM, 4*8*1*1},
    {VEML3328_DG_1x | VEML3328_GAIN_d2 | VEML3328_IT_050 | VEML3328_SENS_NORM, 4*8*8*1},
    {VEML3328_DG_1x | VEML3328_GAIN_d2 | VEML3328_IT_050 | VEML3328_SENS_LOW , 4*8*8*3},
};

#define VEML3328_RANGES (sizeof(veml3328_ranges)/sizeof(veml3328_ranges[0]))

static struct {
    struct device * i2c_master;
    uint32_t deadline;
    uint32_t good_data[VEML3328_INDEX_MAX];
    uint16_t raw_data[VEML3328_INDEX_MAX];
    uint8_t i2c_address;
    uint8_t measure_range_num;
    uint8_t peak_count[VEML3328_INDEX_MAX];
} veml3328_ctx;

// Start measure
static inline void veml3328_cmd(uint16_t cmd) {
    if (!veml3328_ctx.i2c_master) return;
    uint8_t raw [3];
    raw[0] = VEML3328_REG_CTRL;
    raw[1] = (cmd & 0xff);
    raw[2] = (cmd >> 8);
    i2c_write(veml3328_ctx.i2c_master, &raw[0], 3, veml3328_ctx.i2c_address);
}

static inline int veml3328_start() {
    if (!veml3328_ctx.i2c_master) return 0;
    uint16_t cmd = VEML3328_TRIG | VEML3328_AF | veml3328_ranges[veml3328_ctx.measure_range_num].val;
    LOG_DBG("VEML3328 start (0x%04x)", cmd);
    veml3328_cmd(cmd);
    int ms_delay = 50;
    switch(cmd & VEML3328_IT_MSK) {
        case VEML3328_IT_050:
            ms_delay += 50;
            break;
        case VEML3328_IT_100:
            ms_delay += 100;
            break;
        case VEML3328_IT_200:
            ms_delay += 200;
            break;
        default:
            ms_delay += 400;
            break;
    }
    veml3328_ctx.deadline = k_cycle_get_32() + ms_to_cycles(ms_delay);
    return ms_delay;
}

//get and check result for overflow
static inline int veml3328_fetch_result(){
    if (!veml3328_ctx.i2c_master) return 0;
    uint8_t raw[2];
    if (i2c_burst_read(veml3328_ctx.i2c_master, veml3328_ctx.i2c_address,
                        VEML3328_REG_CTRL, &raw[0], 2)){
        LOG_ERR("VEML3328: CTRL reg read failed");
        return 0;
    } else {
        LOG_DBG("VEML3328: CTRL REG IS 0x%02x%02x", raw[1], raw[0]);
        if (raw[0] & VEML3328_TRIG) return 0;
    }
    for (int i = 0; i < VEML3328_INDEX_MAX; ++i){
        if (i2c_burst_read(veml3328_ctx.i2c_master, veml3328_ctx.i2c_address,
                            i + VEML3328_REG_OUT_BASE, &raw[0], 2)){
            LOG_ERR("VEML: result read %d failed", i);
            return 0;
        }
        uint16_t val = (raw[1] << 8) + raw[0];
        veml3328_ctx.raw_data[i] = val;
    }
    return 1;
}

static inline void veml3328_import_result(){
    LOG_DBG("VEML3328: index %d mul %d", veml3328_ctx.measure_range_num, veml3328_ranges[veml3328_ctx.measure_range_num].mul);
    for (int i = 0; i < VEML3328_INDEX_MAX; ++i){
        if (veml3328_ctx.raw_data[i] != 0xFFFF){
            veml3328_ctx.good_data[i] = veml3328_ctx.raw_data[i] *  veml3328_ranges[veml3328_ctx.measure_range_num].mul;
            veml3328_ctx.peak_count[i] = 0;
        } else {
            if (veml3328_ctx.peak_count[i] < 0xff) ++veml3328_ctx.peak_count[i];
        }
        LOG_DBG("VEML3328 ch %d : count=0x%04x val=%d", i, veml3328_ctx.raw_data[i], veml3328_ctx.good_data[i]);
    }
}

static void veml3328_sensor_init(const char * devname, uint8_t addr){
    veml3328_ctx.i2c_master = device_get_binding(devname);
    if (!veml3328_ctx.i2c_master) {
        LOG_ERR("i2c master %s not found", devname);
        return;
    }
    veml3328_ctx.i2c_address = addr;
    uint8_t val;
    if (i2c_burst_read(veml3328_ctx.i2c_master, veml3328_ctx.i2c_address,
                            VEML3328_REG_ID, &val, 1)){
        LOG_ERR("VEML3328: read ID reg failed");
    }
    if (VEML3328_ID_VAL != val){
        LOG_ERR("VEML3328: ID reg: got 0x%x expected 0x%x", val, VEML3328_ID_VAL);
    }
    veml3328_cmd(VEML3328_SHUTDOWN);
}

static void veml3328_lpmode(){
    veml3328_cmd(VEML3328_SHUTDOWN);
}

static inline uint32_t veml3328_max_raw(int imin, int imax){
    uint32_t max = 0;
    for (int i = imin; i < imax; ++i){
        if (max < veml3328_ctx.raw_data[i]) max = veml3328_ctx.raw_data[i];
    }
    return max;
}

static inline int veml3328_all_valid(void){
    for (int i = 0; i < VEML3328_INDEX_MAX; ++i) {
        if (veml3328_ctx.peak_count[i]) return 0;
    }
    return 1;
}

static int veml3328_sensor_update(int force){
    if ((!force) && (((int32_t)(k_cycle_get_32() - veml3328_ctx.deadline)) < 0) ) return 0;
    if (! veml3328_fetch_result()) return 0;
    veml3328_import_result();
    if (0xffff == veml3328_max_raw(0, VEML3328_INDEX_MAX)){
        veml3328_ctx.measure_range_num = VEML3328_RANGES - 1;
    } else {
        uint32_t max = veml3328_max_raw(1, VEML3328_INDEX_MAX);
        uint32_t cmul = veml3328_ranges[veml3328_ctx.measure_range_num].mul;
        uint32_t new_range_num = veml3328_ctx.measure_range_num;
        for (int i = veml3328_ctx.measure_range_num - 1; i >= 0; --i){
            if (max * cmul < veml3328_ranges[i].mul * VEML3328_SAFE_VAL_THR) new_range_num = i;
        }
        veml3328_ctx.measure_range_num = new_range_num;
    }
    LOG_DBG("VEML3328: set range id %d", veml3328_ctx.measure_range_num );

    veml3328_start();
    return 1;
}

static int veml3328_sensor_measure(int force){
    if ((!force) && (((int32_t)(k_cycle_get_32() - veml3328_ctx.deadline)) < 0) ) return 0;
    return veml3328_start();
}

static int veml3328_sensor_fetch(int force){
    if ((!force) && (((int32_t)(k_cycle_get_32() - veml3328_ctx.deadline)) < 0) ) return 0;
    if (! veml3328_fetch_result()) return 0;
    veml3328_import_result();
    if (0xffff == veml3328_max_raw(0, VEML3328_INDEX_MAX)){
        veml3328_ctx.measure_range_num = VEML3328_RANGES - 1;
    } else {
        uint32_t max = veml3328_max_raw(1, VEML3328_INDEX_MAX);
        uint32_t cmul = veml3328_ranges[veml3328_ctx.measure_range_num].mul;
        uint32_t new_range_num = veml3328_ctx.measure_range_num;
        for (int i = veml3328_ctx.measure_range_num - 1; i >= 0; --i){
            if (max * cmul < veml3328_ranges[i].mul * VEML3328_SAFE_VAL_THR) new_range_num = i;
        }
        veml3328_ctx.measure_range_num = new_range_num;
    }
    LOG_DBG("VEML3328: set range id %d", veml3328_ctx.measure_range_num );

    veml3328_lpmode();
    return 1;
}

#endif
// *** End VEML3328

int32_t slowsensorsGetTemperature(uint32_t flags){
    if (flags & FLAG_SENSOR_TEMPERATURE_BAROSENSOR){
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME        
        return ms5xxx_ctx.baro_temperature;
#else 
        return 0;
#endif
    } else {
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
        return sht21_ctx.temperature;
#else
        return 0;
#endif
    }
}

int32_t slowsensorsGetPressure(uint32_t flags){
    int32_t result = 0; 
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME    
    if (flags & FLAG_SENSOR_PRESSURE_SMOOTH) {
        result = (ms5xxx_ctx.baro_count) ? ms5xxx_ctx.baro_pressureSum / ms5xxx_ctx.baro_count : 0;
    } else {
        result = ms5xxx_ctx.baro_pressure;
    }
    
    if (flags & FLAG_SENSOR_PRESSURE_MMHG) result = pascals2torr(result);
#endif
    return result;
}

uint32_t slowsensorsGetHumidity(uint32_t flags){
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    return (FLAG_SENSOR_RH_SHIFT8 & flags) ? sht21_ctx.RH : sht21_ctx.RH >> 8;
#else
    return 0;
#endif
}

uint32_t slowsensorsGetLux(uint32_t flags, uint32_t multiplyer){ // multiplyer=0 result is 10e-4 lux
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
//static const uint8_t pwmul[VEML3328_INDEX_MAX] = {1000/57, 1000/41, 1000/39, 1000/34, 1000/25}; // white, red, green, blue, ir
static const uint8_t pwmul[VEML3328_INDEX_MAX] = {1000/57, 1000/57, 1000/57, 1000/57, 1000/25}; // white, red, green, blue, ir
    if (flags) {
        int channelIndex = flags &  FLAG_SENSOR_LUX_CHANNEL_MASK;
        if (channelIndex >  VEML3328_INDEX_MAX) return 0;
        if (!multiplyer) multiplyer = pwmul[channelIndex];
        uint64_t tval = (veml3328_ctx.good_data[channelIndex] * multiplyer);
        return (tval >> 6);
    } else {
        if (!multiplyer) multiplyer = DT_INST_0_VISHAY_VEML3328_LUX_MUL;
        return veml3328_ctx.good_data[VEML3328_INDEX_LUX] * multiplyer;
    }
        
#else
    return 0;
#endif
}

int slowsensorsLuxInSync(void){
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    return veml3328_all_valid();
#else
    return 0;
#endif
}

void slowsensorsGetColorXY(uint32_t *px, uint32_t *py){
    uint32_t x = 0;
    uint32_t y = 0;
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    uint32_t r = veml3328_ctx.good_data[VEML3328_INDEX_RED];
    uint32_t g = veml3328_ctx.good_data[VEML3328_INDEX_GREEN];
    uint32_t b = veml3328_ctx.good_data[VEML3328_INDEX_BLUE];
    uint32_t m =r;
    if (g > m) m = g;
    if (b > m) m = b;
    uint32_t div = m >> 16;
    if (div > 0){
        div += 1;
        r /= div;
        g /= div;
        b /= div;
    }
/*    int32_t X = ( 349 * r + 380 * g - 153 * b);
    int32_t Y = ( 140 * r + 470 * g - 073 * b);
    int32_t Z = (-281 * r + 183 * g + 657 * b);*/
    int32_t X = ( 419 * r + 285 * g -  69 * b);
    int32_t Y = ( 264 * r + 301 * g +  76 * b);
    int32_t Z = (-299 * r + 208 * g + 635 * b);
    if (X < 0) X = 0;
    if (Y < 0) Y = 0;
    if (Z < 0) Z = 0;
    uint32_t S = (X + Y + Z) >> 10;
    if (S > 0){
        x = (X << 6) / S;
        y = (Y << 6) / S;
    }
#endif
    if (px) *px = x;
    if (py) *py = y;
    return;
}

static inline void sensorsBusPwrSet(uint32_t device_power_state){
# if 0    
    struct device * dev = 0;
    struct device * adev = 0;
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME    
    adev = ms5xxx_ctx.i2c_master;
    if (adev != dev){
        device_set_power_state(adev, device_power_state, NULL, NULL);
        dev = adev;
    }
#endif
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    adev = sht21_ctx.i2c_master;
    if (adev != dev){
        device_set_power_state(adev, device_power_state, NULL, NULL);
        dev = adev;
    }
#endif
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    adev =  veml3328_ctx.i2c_master;
    if (adev != dev){
        device_set_power_state(adev, device_power_state, NULL, NULL);
        dev = adev;
    }
#endif
#endif    
}

static inline void slowsensorsUpdate(void){
    updateAdcSensors(0);
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME    
    ms5xxx_update(0);
#endif
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    sht21_sensor_update(1);
#endif
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    veml3328_sensor_update(0);
#endif
}

static inline int slowsensorsMeasure(void){
    updateAdcSensors(0);
    int msec = 0;
    int cmsec;
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME    
    cmsec = ms5xxx_measure(0);
    if (cmsec > msec) msec = cmsec;
#endif
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    cmsec = sht21_sensor_measure(1);
    if (cmsec > msec) msec = cmsec;
#endif
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    cmsec = veml3328_sensor_measure(0);
    if (cmsec > msec) msec = cmsec;
#endif
    return msec;
}

static inline void slowsensorsFetch(void){
    updateAdcSensors(0);
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME    
    ms5xxx_fetch(0);
#endif
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    sht21_sensor_fetch(1);
#endif
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    veml3328_sensor_fetch(0);
#endif
}

static inline void slowsensorsHwInit(void){
#ifdef DT_INST_0_MEAS_MS5XXX_BUS_NAME
    ms5xxx_init(DT_INST_0_MEAS_MS5XXX_BUS_NAME, DT_INST_0_MEAS_MS5XXX_BASE_ADDRESS, OSR, 1);
#endif
#ifdef DT_INST_0_SENSIRION_SHT2X_BUS_NAME
    sht21_sensor_init(DT_INST_0_SENSIRION_SHT2X_BUS_NAME, DT_INST_0_SENSIRION_SHT2X_BASE_ADDRESS);
#endif
#ifdef DT_INST_0_VISHAY_VEML3328_BUS_NAME
    veml3328_sensor_init(DT_INST_0_VISHAY_VEML3328_BUS_NAME, DT_INST_0_VISHAY_VEML3328_BASE_ADDRESS);
#endif
}

static int slowsensorsSleepMs = SENSORS_SLEEP_TIME_MS;

void slowsensorsSetFastRun(int fast){
    slowsensorsSleepMs = (fast) ? SENSORS_SLEEP_TIME_SHORT_MS : SENSORS_SLEEP_TIME_MS;
    if (fast) k_wakeup(sensors_tid);
}

void sensors_entry(void* p1, void * p2, void * p3){
    (void) p1;
    (void) p2;
    (void) p3;
    slowsensorsHwInit();
    int lastT = 0;
    int lastP = 0;
    int lastRH = 0;
    while (1){
        int fetch_sleep_ms = slowsensorsMeasure();
        if (fetch_sleep_ms > 0) {
            uint32_t ms_left = fetch_sleep_ms;
            do{
                ms_left = k_sleep(K_MSEC(ms_left));
            } while (ms_left);           
        }
        slowsensorsFetch();
        int curP = slowsensorsGetPressure(FLAG_SENSOR_PRESSURE_MMHG);
        int curT = slowsensorsGetTemperature(0);
        int curRH = slowsensorsGetHumidity(0);
        uint32_t mask = 0;
#if UI_PRESENT        
        int rv;
        do{
            rv = graphProcessData(curP, curT, curRH);
        } while ((rv));
#endif
        if (curP) { //Pressure is NZ if we have pressure sensor and it is running
            if (sensorLogProcessData(curP, curT, curRH)) mask |= BIT(LOCAL_ATTR_MLS_INFO);
        }
        if (curP != lastP) mask |= BIT(LOCAL_ATTR_PRESSURE);
        if (curT != lastT) mask |= BIT(LOCAL_ATTR_TEMPERATURE);
        if (curRH != lastRH) mask |= BIT(LOCAL_ATTR_HUMIDITY);
        notify_by_mask(mask);
        lastP = curP;
        lastT = curT;
        lastRH = curRH;
        int ms_delay = slowsensorsSleepMs;
        if (fetch_sleep_ms > 0) ms_delay -= fetch_sleep_ms; 
        k_sleep(K_MSEC(ms_delay));
    }
}

void slowsensors_init(void)
{
    sensors_tid = k_thread_create(&sensors_thread_data, sensors_stack,
                                 K_THREAD_STACK_SIZEOF(sensors_stack),
                                 sensors_entry,
                                 NULL, NULL, NULL,
                                 SENSORS_PRIORITY, 0, K_NO_WAIT);
    if (sensors_tid) k_thread_name_set(sensors_tid, "slowsensors");
}

k_tid_t slowsensors_get_tid(void){
    return sensors_tid;
}