#include "../public.h"
#include "wifi.h"
#include "events.h"
#include "speexx.h"

#define sizeOfBuf 32 //должна быть степень двойки
#define BUF_MASK (sizeOfBuf-1)
uart_nxt UART_lastoperate;
uart_nxt UART_nextoperate[sizeOfBuf];
uint16_t idxIN = 0;
uint16_t idxOUT = 0;

#define sizeOfRecv 512
uint8_t UART_RxByte;
uint8_t UART_Receive_Data[sizeOfRecv];
uint16_t UART_Receive_Data_Size = 0;
uint8_t UART_Answer[sizeOfRecv];

uint8_t UART_Sending = false;
uint8_t ReadySendData = false;

uint8_t AT_ATE0_str[]="ATE0\r\n"; //проверка связи/выключить эхо, ответ OK
uint8_t AT_RST_str[]="AT+RST\r\n";  //перезагрузка модуля, в ответе куча мусора
uint8_t AT_CWLAP_str[] = "AT+CWLAP\r\n"; //список доступных точек доступа 
uint8_t AT_CWJAP_str[] = "AT+CWJAP=\"iPhoneWiFi\",\"12345678\"\r\n";
uint8_t AT_CIPSTART_str[] = "AT+CIPSTART=\"TCP\",\"test.ru\",80\r\n";
uint8_t AT_CIPSEND_str[]="AT+CIPSEND=1929\r\n"; //будем слать данные
uint8_t POST_SEND_str[] = "POST /ogg.php HTTP/1.1\r\nHost: test.ru\r\nContent-Type: audio/x-speex\r\nConnection: keep-alive\r\nContent-length: 1801\r\n\r\n\x00";
//Connection: keep-alive или close

typedef struct {
uint8_t * str;
uint8_t size;
}ATstr;

ATstr String_AT[] = {
  {AT_ATE0_str,sizeof(AT_ATE0_str)-1}, 
  {AT_RST_str,sizeof(AT_RST_str)-1},
  {AT_CWLAP_str,sizeof(AT_CWLAP_str)-1}, 
  {AT_CWJAP_str,sizeof(AT_CWJAP_str)-1},
  {AT_CIPSTART_str,sizeof(AT_CIPSTART_str)-1},
  {AT_CIPSEND_str,sizeof(AT_CIPSEND_str)-1},
  {POST_SEND_str,sizeof(POST_SEND_str)-1},
}; 

void Change_AT_CIPSEND_str(uint16_t num)
{
  uint8_t * tmp = AT_CIPSEND_str;
  uint8_t i;
  num += sizeof(POST_SEND_str)-1;
  for (i=0;i<4;i++){
    *(tmp+14-i) = num%10 + 0x30;
    num /= 10;
  }
}

void Change_POST_SEND_str(uint16_t num, uint8_t isitnew)
{
  uint8_t * tmp = POST_SEND_str;
  uint8_t i;
  num += 1;
  for (i=0;i<4;i++){
    *(tmp+123-i) = num%10 + 0x30;
    num /= 10;
  }
  *(tmp+128) = isitnew;
}

enum
{
  ASETTING = 0,
  AALREADY_CONN,
  AERROR,
  AWIFI_DISCONNECTED,  
  ACLOSED,
  ASEND_FAIL,
  AFAIL,
  AWIFI_GOT_IP,
  AWIFI_CONNECTED,
  ACONNECT,
  ARECV,
  AOK,
  ACOUNT
};

uint8_t * ESP_answ[] = {
  "Settings=",
  "ALREADY CCONNECTED",
  "ERROR", 
  "WIFI DISCONNECT",  
  "CLOSED",
  "SEND FAIL",
  "FAIL",
  "WIFI GOT IP",
  "WIFI CONNECTED",  
  "CONNECT",  
  "Recv",
  "OK",
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UART_Receive_Data[UART_Receive_Data_Size] = UART_RxByte;
  UART_Receive_Data_Size++;
  SetEvent(EVNT_UART_RECEIVE,3);
  HAL_UART_Receive_IT(&huart3,&UART_RxByte,1);
}

void Get_UART_Answer(void)
{
  uint16_t i;
  for (i=0;i<UART_Receive_Data_Size;i++) {
    UART_Answer[i] = UART_Receive_Data[i];
  }
  UART_Receive_Data_Size = 0;
}

void UART_Transmit(uart_nxt Nextoperate)
{
  if (!UART_Sending)
  {
    UART_Sending = true; 
    HAL_UART_Transmit_DMA(&huart3,Nextoperate.pData,Nextoperate.Size);
  }
}

void Add_UART_nextoperate(uint8_t Operate, uint8_t *pData, uint16_t Size, uint8_t wait)
{
  UART_nextoperate[idxIN].Operate = Operate;
  UART_nextoperate[idxIN].pData = pData;
  UART_nextoperate[idxIN].Size = Size;
  UART_nextoperate[idxIN].WaitAnswer = wait;
    
  idxIN++;
  idxIN &= BUF_MASK;
}

void Add_UART_nextoperate_AT(uint8_t comm, uint8_t wait)
{
  Add_UART_nextoperate(comm,String_AT[comm].str,String_AT[comm].size,wait);
}

uint8_t Check_UART_Nextoperate(void)
{
  if (idxIN != idxOUT) return true;
                  else return false;
}

uart_nxt Get_UART_Nextoperate(void)
{
  return UART_nextoperate[idxOUT];
}

void Del_UART_Lastoperate(void)
{
  if (Check_UART_Nextoperate()) {
    idxOUT++;
    idxOUT &= BUF_MASK;
  }
}

void Del_UART_ALL_Nextoperate(void)
{
  idxIN = idxOUT = 0;
}

void UART_send(void)
{
  if (Check_UART_Nextoperate()) {
    UART_lastoperate = Get_UART_Nextoperate();
    UART_Transmit(UART_lastoperate);  
    if (UART_lastoperate.Operate == CIPSEND_DATA) Del_UART_Lastoperate();
  }
}

void Get_VAD_Parameter(uint8_t * Pos)
{
        uint16_t tmp;
        tmp = 0; Pos += 8;
        while (*(++Pos) != ',') tmp = tmp*10 + *Pos - 0x30;
        if ((tmp>0)&&(tmp<160)) VAD_COUNT = tmp;
        tmp = 0; 
        while (*(++Pos) != ',') tmp = tmp*10 + *Pos - 0x30;  
        if (tmp>0) VAD_SENS = tmp;
        tmp = 0; 
        while (*(++Pos) != ',') tmp = tmp*10 + *Pos - 0x30;
        if (tmp>0) VAD_LENGTH = tmp; 
}

void UART_parser(void)
{  
    Get_UART_Answer();
    uint8_t Answer;
    uint8_t * Pos;
    //printf((char const*)UART_Answer);
    for (Answer=0;Answer<ACOUNT;Answer++){
      Pos = (uint8_t *)strstr((char const*)UART_Answer,(char const*)ESP_answ[Answer]);
      if (Pos>0) break;
    }
    switch (Answer){
    case ASETTING:
      Get_VAD_Parameter(Pos);
      ReadySendData = true; //предыдущие данные отправлены, можно отправлять следующие
      UART_lastoperate.WaitAnswer = false;
      break;
    case AALREADY_CONN:
      Del_UART_Lastoperate();
      UART_lastoperate.WaitAnswer = false;
      ReadySendData = true; 
      break;
    case ARECV: //данные были приняты, больше их слать не надо
      Del_UART_Lastoperate();
      break;
    case AERROR: 
      break;
    case ACLOSED:  //соединение закрыто, надо подключаться заного
      if (UART_lastoperate.Operate != AT_CWJAP) {
        UART_lastoperate.WaitAnswer = false;
        ReadySendData = false; 
        VAD_detect = false;
        Del_UART_ALL_Nextoperate();
        Add_UART_nextoperate_AT(AT_CIPSTART,true);
      }
      break;
    case AWIFI_CONNECTED: //подключились к WiFi, ждем получения IP     
      nop();
      break;
    case AWIFI_DISCONNECTED: //потеряли WiFi
      Led_off; 
      ClrEvent(EVNT_LED_SIGNAL);      
      ReadySendData = false;
      UART_lastoperate.WaitAnswer = true; 
      VAD_detect = false;
      Del_UART_ALL_Nextoperate();
      Add_UART_nextoperate_AT(AT_CWJAP,true);
      break;
    case AWIFI_GOT_IP: //IP получили 
      SetEvent(EVNT_LED_SIGNAL,EVENT_DONE);
      Add_UART_nextoperate_AT(AT_CIPSTART,true);
      //если это автоподключение, можно начинать слать данные, иначе будет еще ОК на команду подключения
      if (UART_lastoperate.Operate != AT_CWJAP) {         
        UART_lastoperate.WaitAnswer = false;
      } 
      break;
    case ACONNECT:  //подключились к TCP
      if (UART_lastoperate.Operate == AT_CIPSTART) {
        Del_UART_Lastoperate();
        UART_lastoperate.WaitAnswer = false;
        ReadySendData = true; 
        SetEvent(EVNT_LED_SIGNAL,EVENT_DONE);
      }
      break;
    case AOK: //нашли ОК
      switch (UART_lastoperate.Operate) { 
      case AT_CWJAP:
        if (Get_UART_Nextoperate().Operate == AT_CWJAP) Del_UART_Lastoperate();
        UART_lastoperate.WaitAnswer = false;
        Add_UART_nextoperate_AT(AT_CIPSTART,true);
        break;
      default:
        Del_UART_Lastoperate();
        UART_lastoperate.WaitAnswer = false;
        break;
      }      
      break;
    case AFAIL: //ошибка подключения, попробуем еще
      UART_lastoperate.WaitAnswer = false;
      break;
    }    
    
    memset(UART_Answer, 0, sizeOfRecv);
    
    if (!UART_lastoperate.WaitAnswer && Check_UART_Nextoperate()) SetEvent(EVNT_UART_SEND,EVENT_DONE);
                                                             else ClrEvent(EVNT_UART_SEND);  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) //Функция окончания передачи UART
{
  UART_Sending = false;
  if (!UART_lastoperate.WaitAnswer) {
    Del_UART_Lastoperate();
    if (Check_UART_Nextoperate()) SetEvent(EVNT_UART_SEND,EVENT_DONE);
  } 
}