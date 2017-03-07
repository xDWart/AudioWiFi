enum {
  EVNT_UART_RECEIVE,
  EVNT_UART_SEND,
  EVNT_VAD_DETECT,
  EVNT_LED_SIGNAL,
  CNT_EVENTS  //count of registered events
};

#define MEASURE_PERIOD                  (uint32_t)1   //ms
#define EVENT_DONE                      0
#define EVENT_CLR                       -1

void SetEvent(uint8_t event_id, uint32_t msec);
void RunEvents(void);
void ClrEvents(void);
int TestEvent_Done(unsigned int event_id);
int TestEvent_Clr(unsigned int event_id);
void ClrEvent(unsigned int event_id);
int32_t GetEventTime(uint32_t event_id);
int TestEvent(unsigned int event_id);