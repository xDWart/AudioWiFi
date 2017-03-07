#include "..\public.h"
#include "events.h"

//******************************************************************************
//                    vars
//******************************************************************************
volatile int32_t Events[CNT_EVENTS];
//******************************************************************************
//                    functions
//******************************************************************************
void SetEvent(uint8_t event_id, uint32_t msec){
  Events[event_id] = msec/MEASURE_PERIOD;
}
//******************************************************************************
void RunEvents(void){
  int i;
  
  for (i=0; i<CNT_EVENTS;i++){
    if (Events[i]>EVENT_DONE) Events[i]--;
  }
}
//******************************************************************************
void ClrEvents(void){
  int i;
  
  for (i=0; i<CNT_EVENTS;i++){
    Events[i] = EVENT_CLR;
  }
}
//******************************************************************************
int TestEvent_Done(unsigned int event_id){
  if (Events[event_id] == EVENT_DONE){
    Events[event_id] = EVENT_CLR;
    return true;
  }else{
    return false;
  }
}
//******************************************************************************
int TestEvent(unsigned int event_id){
  if (Events[event_id] == EVENT_DONE){
    return true;
  }else{
    return false;
  }
}
//******************************************************************************
int TestEvent_Clr(unsigned int event_id){
  if (Events[event_id] <= EVENT_CLR){
    return true;
  }else{
    return false;
  }
}
//******************************************************************************
void ClrEvent(unsigned int event_id){
  Events[event_id] = EVENT_CLR;
}
//******************************************************************************
int32_t GetEventTime(uint32_t event_id)
{
  return Events[event_id]*MEASURE_PERIOD;
}
//******************************************************************************