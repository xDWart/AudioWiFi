enum 
{
   ATE0,
   AT_RST,
   AT_CWLAP,
   AT_CWJAP,
   AT_CIPSTART,
   AT_CIPSEND,
   POST_SEND,
   CIPSEND_DATA
};

typedef struct { 
         uint8_t Operate;
         uint8_t * pData;
         uint16_t Size;
         uint8_t WaitAnswer;
}uart_nxt ;

void Change_AT_CIPSEND_str(uint16_t num);
void Change_POST_SEND_str(uint16_t num, uint8_t isitnew);

void Add_UART_nextoperate(uint8_t Operate, uint8_t *pData, uint16_t Size, uint8_t wait);
void Add_UART_nextoperate_AT(uint8_t comm, uint8_t wait);
void Del_UART_ALL_Nextoperate(void);

extern uint8_t UART_RxByte;

extern uint8_t UART_Sending;
extern uint8_t ReadySendData;

void UART_send(void);
void UART_parser(void);
