#ifndef BT_H
#define BT_H
#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    LOCAL_ATTR_WRONG_ID = 0,
    LOCAL_ATTR_TIME = 1,
    LOCAL_ATTR_TEMPERATURE = 2,
    LOCAL_ATTR_HUMIDITY = 3,
    LOCAL_ATTR_PRESSURE = 4,
    LOCAL_ATTR_MLS_INFO = 5,
    LOCAL_ATTR_LAST = 6,
} attr_id_e;

int bt_init(void);
int passkey_get(void);
void bas_notify_demo(void);

int bt_unpair_all();
int bt_adv_enable_set(int enable);
int bt_adv_status_get();
int bt_conn_count_get();
int bt_max_sec_get();
void notify_by_mask(uint32_t mask);


// abcd0123-2fd0-11ea-978f-2e728ce88125
#define UUID_MLS_SERVICE_INITIALIZER   0x25, 0x81, 0xe8, 0x8c, 0x72, 0x2e, 0x8f, 0x97, 0xea, 0x11, 0xd0, 0x2f, 0x00, 0x00, 0xcd, 0xab
#define UUID_MLS_INFO_INITIALIZER      0x25, 0x81, 0xe8, 0x8c, 0x72, 0x2e, 0x8f, 0x97, 0xea, 0x11, 0xd0, 0x2f, 0x01, 0x00, 0xcd, 0xab
#define UUID_MLS_MESSAGE_INITIALIZER   0x25, 0x81, 0xe8, 0x8c, 0x72, 0x2e, 0x8f, 0x97, 0xea, 0x11, 0xd0, 0x2f, 0x02, 0x00, 0xcd, 0xab

typedef struct __attribute__((__packed__)) mls_info_s {
    uint16_t number_of_points;
    uint16_t interval_secs;
    uint32_t timestamp;
    uint32_t cur_point;
    uint32_t prev_point;
} mls_info_t;

#ifdef __cplusplus
}
#endif

#endif