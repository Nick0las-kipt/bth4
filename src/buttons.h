#ifndef BUTTONS_H
#define BUTTONS_H


#ifdef __cplusplus
extern "C" {
#endif

void buttons_init(void);
void get_button_status(int32_t *button_num, int32_t *bcntr);
void buttons_set_listener(k_tid_t _wake_tid);
uint32_t process_button_status(int32_t *button_num, int32_t *bcntr, uint32_t nevents);

#ifdef __cplusplus
}
#endif

#endif