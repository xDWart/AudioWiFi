#include "../public.h"
#include "speexx.h"
#include "wifi.h"
#include "events.h"

//SPEEX variables
__IO uint16_t IN_Buffer[2][FRAME_SIZE];
__IO uint8_t Start_Encoding = 0;
uint8_t Index_Encoding = 0;
uint32_t Encoded_Frames = 0;
float Average = 1891;

uint8_t REC_DATA[2][MAX_REC_FRAMES*ENCODED_FRAME_SIZE]; //сюда сохраняются закодированные данные
uint8_t* Rec_Data_ptr = &REC_DATA[0][0]; //указатель на кодируемые данные
uint8_t* Trm_Data_ptr; //указатель на передаваемые данные

int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */
SpeexBits bits; /* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */

uint16_t VAD_COUNT = 30;
uint16_t VAD_SENS = 2000;
uint16_t VAD_LENGTH = 20; //сек
uint8_t VAD_detect = false;
uint8_t new_VAD_detect = false;

void Speex_Init(void)
{
  /* Speex encoding initializations */ 
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
}

float Flt_RC(float val_tek, float val_pred, float count)
{
    return ((val_pred * count / (count + 1)) + (val_tek / (count + 1)));      
}

void Add_Operates(uint8_t *pData, uint16_t size)
{
  if (VAD_detect && ReadySendData) {  
          ReadySendData = false;
          Change_AT_CIPSEND_str(size);
          Add_UART_nextoperate_AT(AT_CIPSEND,true);
          Change_POST_SEND_str(size,new_VAD_detect);
          new_VAD_detect = false; 
          Add_UART_nextoperate_AT(POST_SEND,false);          
          Add_UART_nextoperate(CIPSEND_DATA,pData,size,true);     
          SetEvent(EVNT_UART_SEND,EVENT_DONE);
  }
}

void EncodingVoice(void)
{
    uint8_t i;
    uint8_t vad_check=0;    
    int16_t INB[FRAME_SIZE];
    float NewAverage = 0;
    
    //====================Если одна из половинок буфера заполнена======================
    if(Start_Encoding > 0)
      { 
        Index_Encoding = Start_Encoding - 1;
        for (i=0;i<FRAME_SIZE;i++) {
          NewAverage += IN_Buffer[Index_Encoding][i];
          INB[i] = (IN_Buffer[Index_Encoding][i]-(int16_t)Average)*14;
          if (fabs(INB[i])>VAD_SENS) vad_check++;                          
        }
        Average = Flt_RC(NewAverage/FRAME_SIZE,Average,9);
        if (vad_check>VAD_COUNT) {
          if (!VAD_detect) new_VAD_detect = true; 
          VAD_detect = true;
          SetEvent(EVNT_VAD_DETECT,VAD_LENGTH*1000);
        }
        /* Flush all the bits in the struct so we can encode a new frame */
        speex_bits_reset(&bits);
        /* Encode the frame */
        speex_encode_int(enc_state, (spx_int16_t*)INB, &bits);
        /* Copy the bits to an array of char that can be decoded */
        speex_bits_write(&bits, (char *)Rec_Data_ptr, ENCODED_FRAME_SIZE);
          
        Rec_Data_ptr += ENCODED_FRAME_SIZE;
        Encoded_Frames += 1;
        
        Start_Encoding = 0;	
      }
    
    if (Encoded_Frames == MAX_REC_FRAMES) {        
        Add_Operates(&REC_DATA[0][0],1800);
    }
    
    if (Encoded_Frames == MAX_REC_FRAMES*2) {
        Rec_Data_ptr = &REC_DATA[0][0];
        Encoded_Frames = 0;
        Add_Operates(&REC_DATA[1][0],1800);
    }
}