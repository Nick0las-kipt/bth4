#include <zephyr.h>
#include <kernel.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>
#include "rtc.h"
#include "slowsensors.h"
#include "bt.h"
#include "sensorlog.h"

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(bt);

#define NOTIFY_STACK_SIZE    1024
#define NOTIFY_PRIORITY         2


typedef struct local_attr_info_s {
    uint16_t size;
    uint8_t *data_ptr;
    const struct bt_gatt_attr *attr_ptr;
    int (*getter)(uint8_t *data, uint16_t size);
    int (*setter)(const uint8_t *data, uint16_t size);
    uint8_t properties;
} local_attr_info_t;

typedef struct static_attr_info_s {
    uint16_t size;
    const void * data;
} static_attr_info_t;

uint8_t cts_storage[10];

static int setter_cts_time(const uint8_t *data, uint16_t size);
static int getter_cts_time(uint8_t *data, uint16_t size);

static int getter_ess_temperature(uint8_t *data, uint16_t size);
static int getter_ess_humidity(uint8_t *data, uint16_t size);
static int getter_ess_pressure(uint8_t *data, uint16_t size);

static int getter_mls_info(uint8_t *data, uint16_t size);

local_attr_info_t local_attrs[]={
    [LOCAL_ATTR_TIME - 1] = {
        .size = 10,
        .data_ptr =&cts_storage[0],
        .getter = getter_cts_time,
        .setter = setter_cts_time,
    },
    [LOCAL_ATTR_TEMPERATURE - 1] = {
        .size = 2,
        .data_ptr = NULL,
        .getter = getter_ess_temperature,
    },    
    [LOCAL_ATTR_PRESSURE - 1] = {
        .size = 4,
        .data_ptr = NULL,
        .getter = getter_ess_pressure,
    },    
    [LOCAL_ATTR_HUMIDITY - 1] = {
        .size = 2,
        .data_ptr = NULL,
        .getter = getter_ess_humidity,
    },
    [LOCAL_ATTR_MLS_INFO - 1] = {
        .size = sizeof(mls_info_t),
        .data_ptr = NULL,
        .getter = getter_mls_info,
    },
};

#define IS_ATTR_LOCAL(arg) (((uint32_t)arg >= (uint32_t)&local_attrs) && \
    ((uint32_t)arg < ((uint32_t)&local_attrs) + sizeof(local_attrs)) && \
    ( ((local_attr_info_t*)arg)->size ))

static int get_local_id(const local_attr_info_t * ptr){
    return (IS_ATTR_LOCAL(ptr)) ? 1 + (ptr - &local_attrs[0]) : 0;
}

static const local_attr_info_t * get_attr_by_local_id(uint16_t local_id){
    if (!(local_id)) return NULL;
    const local_attr_info_t * ptr = &local_attrs[local_id-1];
    return IS_ATTR_LOCAL(ptr) ? ptr: NULL;
}

static uint16_t find_static_handle_for_attr(const struct bt_gatt_attr *attr)
{
    uint16_t handle = 1;

    Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
        for (int i = 0; i < static_svc->attr_count; i++, handle++) {
            if (attr == &static_svc->attrs[i]) {
                return handle;
            }
        }
    }
    return 0;
}

static const struct bt_gatt_attr *find_static_attr_by_handle(uint16_t f_handle)
{
    if (f_handle < 1) return NULL;
    uint16_t handle = 1;

        Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
            /* Skip ahead if start is not within service handles */
            if (handle + static_svc->attr_count < f_handle) {
                handle += static_svc->attr_count;
                continue;
            }
            
            int i = f_handle - handle;
            return &static_svc->attrs[i];
        }
    return NULL;
}

static ssize_t read_static(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, uint16_t len, uint16_t offset)
{
    if (!attr->user_data) return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);

    const static_attr_info_t * desc = (static_attr_info_t*) attr->user_data;

    uint16_t size = desc->size;

    const uint8_t * value = (uint8_t*)desc->data;

    LOG_INF("Read static handle=0x%x ofs=0x%x len=0x%x", attr->handle , offset, len);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, size);
}

static ssize_t read_def(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, uint16_t len, uint16_t offset)
{
    if (!IS_ATTR_LOCAL(attr->user_data)) return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);

    const local_attr_info_t * info = attr->user_data;
    uint8_t *value = info->data_ptr;
    const uint16_t size = info->size;

    LOG_INF("Read attr local_id=0x%x ofs=0x%x len=0x%x", get_local_id(info) , offset, len);

    if (value){
        if (info->getter){
            if (info->getter(value, size) < 0) return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }
        return bt_gatt_attr_read(conn, attr, buf, len, offset, value, size);
    } else if (info->getter) {
        uint8_t tmp_buf[CONFIG_BT_L2CAP_TX_MTU];
        value = &tmp_buf[0];
        if (info->getter(value, size) < 0) return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        return bt_gatt_attr_read(conn, attr, buf, len, offset, value, size);
    }
    return BT_GATT_ERR(BT_ATT_ERR_READ_NOT_PERMITTED);
}

static ssize_t write_def(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, uint16_t len, uint16_t offset,
             uint8_t flags)
{
    if (!IS_ATTR_LOCAL(attr->user_data)) return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
    
    const local_attr_info_t * info = attr->user_data;
    uint8_t *value = info->data_ptr;

    if (!value) return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);

    const uint16_t size = info->size;

    if (offset + len > size) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    if ((size - offset - len) > 0) memset(value + offset + len, 0, size - offset - len );

    LOG_INF("Write attr local_id=0x%x ofs=0x%x len=0x%x", get_local_id(info), offset, len);
    
    if (info->setter){
        if (info->setter(value, size) < 0) return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return len;
}

static ssize_t notify_def(int local_id, uint8_t * data){
    const local_attr_info_t * info = get_attr_by_local_id(local_id);
    if (!info) return -EINVAL;
    if (info->data_ptr){
        int res = 0;
        if (data) {
            memcpy(info->data_ptr, data, info->size);
            res = info->size;
        } else {
            if (info->getter) res = info->getter(info->data_ptr, info->size);
        }
        return (res > 0) ? bt_gatt_notify(NULL, info->attr_ptr, info->data_ptr, info->size) : -ENOENT;
    } else {
        uint8_t tmp_buf[CONFIG_BT_L2CAP_TX_MTU];
        int res = 0;
        if (data) {
            memcpy(tmp_buf, data, MIN(info->size, CONFIG_BT_L2CAP_TX_MTU));
            res = info->size;
        } else {
            if (info->getter) res = info->getter(&tmp_buf[0], info->size);
        }
        return (res > 0) ? bt_gatt_notify(NULL, info->attr_ptr, &tmp_buf[0], info->size) : -ENOENT;
    }
}

static void ccc_cfg_changed_def(const struct bt_gatt_attr *attr, uint16_t value)
{
    const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(attr->handle - 1);
    LOG_WRN("Configuration for local id 0x%x: [%s%s]",
         get_local_id((local_attr_info_t *)(attr1->user_data)),
         (BT_GATT_CCC_NOTIFY == value) ? "Notify":"",
         (BT_GATT_CCC_INDICATE == value) ? "Indicate":"");
}

BT_GATT_SERVICE_DEFINE(cts_cvs,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
    BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME, BT_GATT_CHRC_READ |
                   BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN,
                   read_def, write_def, &local_attrs[LOCAL_ATTR_TIME - 1]),
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

typedef struct  __attribute__((__packed__)) ess_measurement_desc_s  {
    uint8_t flags_lo;
    uint8_t flags_hi;
    uint8_t sampling_function;
    uint8_t period[3];
    uint8_t interval[3];
    uint8_t application;
    uint8_t uncertainity;
} ess_measurement_desc_t;

static const ess_measurement_desc_t ess_measurement_desc_def = {
    .flags_lo = 0,
    .flags_hi = 0,
    .sampling_function = 1, //instantaneous
    .period = {4, 0, 0},
    .interval = {4, 0, 0},
    .application = 1, //air
    .uncertainity = 1,
};

static const uint8_t ess_configuration = 1;
static const uint8_t ess_trigger_setting = {3};

static const static_attr_info_t ess_measurement_desc_info = {
    .size = sizeof(ess_measurement_desc_def),
    .data = &ess_measurement_desc_def,
};

static const static_attr_info_t ess_configuration_desc_info = {
    .size = sizeof(ess_configuration),
    .data = &ess_configuration,
};

static const static_attr_info_t ess_trigger_setting_info = {
    .size = sizeof(ess_trigger_setting),
    .data = &ess_trigger_setting,
};

BT_GATT_SERVICE_DEFINE(ess,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_READ,
                   read_def, NULL, &local_attrs[LOCAL_ATTR_TEMPERATURE - 1]),
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_measurement_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_CONFIGURATION, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_configuration_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_trigger_setting_info),

    BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_READ,
                   read_def, NULL, &local_attrs[LOCAL_ATTR_PRESSURE - 1]),
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_measurement_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_CONFIGURATION, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_configuration_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_trigger_setting_info),

    BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_READ,
                   read_def, NULL, &local_attrs[LOCAL_ATTR_HUMIDITY - 1]),
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_measurement_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_CONFIGURATION, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_configuration_desc_info),
    BT_GATT_ATTRIBUTE(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ, read_static, NULL, (void*)&ess_trigger_setting_info),
);

static ssize_t write_mls_message(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, uint16_t len, uint16_t offset,
             uint8_t flags);

BT_GATT_SERVICE_DEFINE(mls,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(UUID_MLS_SERVICE_INITIALIZER)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(UUID_MLS_INFO_INITIALIZER), BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_READ,
                   read_def, NULL, &local_attrs[LOCAL_ATTR_MLS_INFO - 1]),
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),     
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(UUID_MLS_MESSAGE_INITIALIZER), BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_WRITE_AUTHEN,
                   NULL, write_mls_message, NULL),    
    BT_GATT_CCC(ccc_cfg_changed_def, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),    
);


static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME,
              0x1A, 0x18, 0x05, 0x18),
};

#if CONFIG_LOG
char * bt_uuid_str(const struct bt_uuid *uuid){
    char buf[50];
    bt_uuid_to_str(uuid, buf, sizeof(buf));
    return log_strdup(buf);
}
#else
char * bt_uuid_str(const struct bt_uuid *uuid){
    static const char astr[] = "str";
    return (char*) astr;
}
#endif

static int setter_cts_time(const uint8_t *data, uint16_t size){
    int year = data[0] + (data[1] << 8);
    if ((year < 2000) || (year>=2100)) return -EINVAL;
    year -=2000;
    rtc_set_date_time(data[3], data[2], year, data[4], data[5], data[6]);
    return 0;
}

static int getter_cts_time(uint8_t *data, uint16_t size){
    if (size < 8) return -EINVAL;
    rtc_get_date_time_dow(&data[3],&data[2],&data[0],&data[4],&data[5],&data[6], &data[7]);
    uint16_t year_corr = 2000 + data[0];
    data[0] = year_corr & 0xff;
    data[1] = (year_corr >> 8);
    memset(&data[8], 0, size - 8);
    return 0;
}

static inline uint8_t * put_data_le(uint8_t * data, uint32_t to_put, int num){
    while(num--) {
        *data++ = to_put;
    to_put >>= 8;
    };
    return data;
}

static int getter_ess_temperature(uint8_t *data, uint16_t size){
    if (size < 2) return -EINVAL;
    int16_t temp16d2 = slowsensorsGetTemperature(0);
    /*data[0] = temp16d2 & 0xff;
    data[1] = temp16d2 >> 8;*/
    put_data_le(data, (uint32_t) temp16d2, 2);
    return 2;
}

static int getter_ess_humidity(uint8_t *data, uint16_t size){
    if (size < 2) return -EINVAL;
    uint32_t humidity16d2 = (slowsensorsGetHumidity(FLAG_SENSOR_RH_SHIFT8) * 100) >> 8;
    /*data[0] = humidity16d2 & 0xff;
    data[1] = humidity16d2 >> 8;*/
    put_data_le(data, humidity16d2, 2);
    return 2;
}

static int getter_ess_pressure(uint8_t *data, uint16_t size){
    if (size < 4) return -EINVAL;
    uint32_t pressure32d1 = slowsensorsGetPressure(0) * 10;
    /*data[0] = pressure32d1 & 0xff;
    data[1] = (pressure32d1 >> 8) & 0xff;
    data[2] = (pressure32d1 >> 16) & 0xff;
    data[3] = (pressure32d1 >> 24) & 0xff;*/
    put_data_le(data, pressure32d1, 4);
    return 4;
}

static int getter_mls_info(uint8_t *data, uint16_t size){
    if (size < sizeof(mls_info_t)) return -EINVAL;
    int used = getLogUsed();
    int pos;
    uint32_t time;
    getLogPosTime(&pos, &time);
    data = put_data_le(data, used, 2);
    data = put_data_le(data, LOG_MINUTE_INTERVAL * 60, 2);
    data = put_data_le(data, time, 4);
    data = put_data_le(data, getLogPoint(pos), 4);
    if (--pos < 0) pos = getLogSize() - 1;
    data = put_data_le(data, (used) ? getLogPoint(pos): 0, 4);
    return sizeof(mls_info_t);
}

K_THREAD_STACK_DEFINE(notify_stack, NOTIFY_STACK_SIZE);

static struct k_work_q notify_work_q;
static struct k_work mls_notify_work;
static uint32_t mls_notify_arg;
static struct bt_conn * mls_conn;


static ssize_t write_mls_message(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, uint16_t len, uint16_t offset,
             uint8_t flags)
{
    if ((0 != offset) || (6 != len)) return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);

    uint8_t * data = (uint8_t*) buf;
    uint32_t req_time = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24) ;
    int req_count = data[4] + (data[5] << 8);
    int cur_pos = 0;
    uint32_t cur_time = 0;
    getLogPosTime(&cur_pos, &cur_time);

    if (req_time > cur_time){
        LOG_WRN("request in future: req_time=%d cur_time=%d", req_time, cur_time);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
    }
    int req_pos = cur_pos - (cur_time - req_time)/(60 * LOG_MINUTE_INTERVAL);

    if (req_pos < 0) req_pos += getLogSize();
    if (req_pos < 0) {
        LOG_WRN("request too far in the past: req_time=%d cur_time=%d", req_time, cur_time);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);        
    }
    if (mls_conn){
        LOG_WRN("Another request in progress");    
        return BT_ATT_ERR_PREPARE_QUEUE_FULL;
    }

    LOG_INF("Start meteo log serivce send from pos %d, count %d", req_pos, req_count);

    mls_notify_arg = (req_pos << 16) + req_count;
    mls_conn = conn;

    k_work_submit_to_queue(&notify_work_q, &mls_notify_work);
    
    return len;
}

void get_mtu_min(struct bt_conn *conn, void *data){
    int *mtu = (int *) data;
    int cmtu = bt_gatt_get_mtu(conn);
    if (cmtu > 0) {
        if ((0 == *mtu) || (*mtu > cmtu)) *mtu = cmtu;
    }
}

static void mls_send(struct k_work *item){
    (void) item;
    uint32_t points_count = mls_notify_arg;
    int32_t pos = points_count >> 16;
    points_count &= 0xffff;
    int mtu = 0;
    bt_conn_foreach(BT_CONN_TYPE_ALL, get_mtu_min, &mtu);
    if (!mtu) mtu = 23;
    int points_per_msg = (mtu - 3 - 2)/4; //TODO: derive from MTU;
    uint8_t buf[CONFIG_BT_L2CAP_TX_MTU];
    int packet = 0;
    LOG_INF("MLS: start");
    while (points_count) {
        uint8_t * dptr = put_data_le(&buf[0], packet, 2);
        int num_points = MIN(points_count, points_per_msg);
        for (int point = 0; point < num_points; ++point){
            dptr = put_data_le(dptr, getLogPoint(pos--), 4);
            if (pos < 0) pos = getLogSize() - 1;
        }
        points_count -= num_points;
        bt_gatt_notify(mls_conn, &mls.attrs[4], buf, 2 + (num_points << 2));
        LOG_DBG("MLS: sent packet %d, %d points left", packet, points_count);         
        ++packet;
    }
    LOG_INF("MLS: sent %d packets", packet);
    mls_conn = 0;
}

static uint8_t local_adv;
static uint8_t local_conn;

static uint32_t bt_sec_levels[CONFIG_BT_MAX_CONN];

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_INF("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
        ++local_conn;
        if (conn){
            int index = bt_conn_index(conn);
            bt_sec_levels[index] = 0x100;
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    if (local_conn > 0) --local_conn;
    if (conn){
        int index = bt_conn_index(conn);
        bt_sec_levels[index] = 0;
    }
}

static void param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout){
        int mtu = bt_gatt_get_mtu(conn);

        int ibp = (5 * interval) / 4;
        int iap = (125*interval) % 100;

        int lbp = (5 * latency) / 4;
        int lap = (125*latency) % 100;
    
        LOG_WRN("Connection params updated, MTU = %u, interval=%d.%02dms, latency=%d.%02dms, timeout = %dms", mtu,
            ibp, iap, lbp, lap, 10 * timeout);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err){
    LOG_INF("Security changed to (%d) result:%d", level, err);
    if (conn){
        int index = bt_conn_index(conn);
        bt_sec_levels[index] = 0x100 + level;
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = param_updated,
    .security_changed = security_changed,
};

static int g_passkey = -1;

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    g_passkey = passkey; 

    LOG_WRN("Passkey for %s: %06u", log_strdup(addr), passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    g_passkey = -1;

    LOG_INF("Pairing cancelled: %s", log_strdup(addr));
}

static void auth_complete(struct bt_conn *conn, bool bonded)
{
    g_passkey = -1;

    LOG_INF("Pairing result: %s", bonded ? "OK":"Fail");
}

static void auth_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    g_passkey = -1;

    LOG_INF("Pairing failed: reason:%d", reason);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
    .pairing_complete = auth_complete,
    .pairing_failed = auth_failed,
};

int passkey_get(void){
    return g_passkey;
}

int start_le_adv(){
    local_adv = 1;
    return bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
}

int stop_le_adv(){
    local_adv = 0;
    return bt_le_adv_stop();
}

static void bt_ready(int err){
    if (err) LOG_ERR("Bluetooth init failed (err %d)", err);
    settings_load();

    LOG_INF("Bluetooth initialized");

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);
    err = start_le_adv();
    if (err) {
        LOG_INF("Advertising failed to start (err %d)", err);
        return;
    }
    local_adv = 1;

    LOG_INF("Advertising successfully started");    
}

static void attrs_post_init(){
    Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
        for (int i = 0; i < static_svc->attr_count; i++) {
            const struct bt_gatt_attr * attr_ptr = &static_svc->attrs[i];
            
            if (IS_ATTR_LOCAL(attr_ptr->user_data)){
                local_attr_info_t * info = (local_attr_info_t*) attr_ptr->user_data;
                info->attr_ptr = attr_ptr;
                uint16_t handle = find_static_handle_for_attr(attr_ptr);
                const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(handle - 1);
            
                if ((attr1) && !bt_uuid_cmp(attr1->uuid, BT_UUID_GATT_CHRC)) {
                    struct bt_gatt_chrc *chrc = attr1->user_data;
                    info->properties = chrc->properties;
                }                
                LOG_DBG("Local ID 0x%x bound to UUID %s Properties=0x%x handle=%d",
                    get_local_id(info), bt_uuid_str(attr_ptr->uuid), info->properties, handle);
            }
        }
    }
}

int bt_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    bt_ready(err);

    attrs_post_init();

    k_work_q_start(&notify_work_q, notify_stack, K_THREAD_STACK_SIZEOF(notify_stack), NOTIFY_PRIORITY);
    k_work_init(&mls_notify_work, mls_send);
    k_thread_name_set(&notify_work_q.thread, "Custom BT notify");

    return 0;
}

int bt_unpair_all(){
    return bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
}

void _diconnect(struct bt_conn * conn, void * data){
    (void) data;
    if (conn) bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_POWER_OFF);
}

int bt_adv_enable_set(int enable){
    if (enable){
        return start_le_adv();
    } else {
        int rv = stop_le_adv();
        bt_conn_foreach(BT_CONN_TYPE_ALL, _diconnect, 0);
        return rv;
    }
}

int bt_adv_status_get(){
    return local_adv;
}

int bt_conn_count_get(){
    return local_conn;
}

int bt_max_sec_get(){
    int rv = -1;
    for (int i = 0; i < CONFIG_BT_MAX_CONN; ++i){
        if (bt_sec_levels[i]){
            int level = bt_sec_levels[i] - 0x100;
            if (level > rv) rv = level;
        }
    }
    return rv;
}

void notify_by_mask(uint32_t mask){
    if (!local_conn){
        LOG_DBG("Skip notify by mask 0x%x, no connection", mask);
        return;
    }
    for (int i = LOCAL_ATTR_WRONG_ID + 1; i <LOCAL_ATTR_LAST; ++i){
        if (mask & BIT(i)) {
            int rv = notify_def(i, NULL);
            LOG_DBG("Notify local_id=%d res = %d", i, rv);
        }
    }
}