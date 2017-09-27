//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-ir.h"

#define REMOTE_RESYNC_TICKS 14
#define REMOTE_START_BIT_TICK 3
#define REMOTE_DATA_BIT_TICK 7
#define REMOTE_STOP_BIT_TICK 14
#define REMOTE_BIT_TIMEOUT_TICK 18
#define REMOTE_BYTE_STOP_TICKS 8
#define REMOTE_STOP_BIT_MIN_TICKS 2


REMOTE_DECODE remote_decode[REMOTE_DECODER_MAX];

void remote_resync_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_RESYNC;
  decode_p->timer = 0;
}

void remote_byte_start_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_BYTE_START_WAIT;
}

void remote_start_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_START_BIT;
  decode_p->timer = 0;
}

void remote_data_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_DATA_BIT;
  decode_p->count = 0;
}

void remote_stop_bit_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_STOP_BIT_WAIT;
}

void remote_stop_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_STOP_BIT;
  decode_p->count = 0;
  decode_p->timer = 0;
}

void remote_start_bit_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_START_BIT_WAIT;
}

void remote_byte_stop_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_BYTE_STOP_WAIT;
  decode_p->timer = 0;
}

/****************************************************************
*Function   :  remote_decode_init
*Description:  获取接收头接收到遥控器的数据 
*Input      :  u16 instance:接收头ID            
*Output     :  无
*Return     :  -1,出错 0正确
*Others     :  
******************************************************************/
u16 remote_decode_init( u16 instance )
{
 	
  REMOTE_DECODE *decode_p;

  if (instance >= REMOTE_DECODER_MAX)
  {
    return(-1);
  }
  decode_p = &remote_decode[instance];

  remote_resync_init(decode_p);

  return(0);
   
}

/****************************************************************
*Function   :  remote_decode_ir
*Description:  获取接收头接收到遥控器的数据 
*Input      :  u16 instance:接收头ID   
*              u16 ir_state:接收头采集到的电平
*Output     :  无
*Return     :  -1:未解出正确的编码 0:解出正确的编码
*Others     :  此处红外的编码方式是"1"为高电平持续3ms,低电平持续1ms
                                   "0"为高电平持续1ms,低电平持续3ms
******************************************************************/
U8 BIT16=0;
int16_t remote_decode_ir( u16 instance,
                           u16 ir_state )
{
 
  REMOTE_DECODE *decode_p;
  int16_t     remoteStatus = 0;
  

  if (instance >= REMOTE_DECODER_MAX)
  {
    return(-1);
  }
  decode_p = &remote_decode[instance];


  decode_p->timer++;

  switch (decode_p->state)
  {
  case REMOTE_RESYNC:
   if (!ir_state)     //已经取反       
   { 
        decode_p->count=0;
        remote_resync_init(decode_p);
    }
    else 
    {
      decode_p->count++;
    }
    if (decode_p->count>=12)//3.12ms 低电平
    {   
        
        decode_p->count=0;
        decode_p->timer=0;
        remote_byte_start_wait_init(decode_p);
     }
    break;
    
  case REMOTE_BYTE_START_WAIT:   
    if (!ir_state)
    {
      decode_p->count++;
    }
    if((decode_p->count==0)&&ir_state)
    {
      decode_p->timer=0;
      break;
    }
    if ((decode_p->timer==decode_p->count)&&(decode_p->count>=10)&&(decode_p->count<=13))//2.88高电平
    {
        decode_p->count=0;
        decode_p->timer=0;
        remote_start_bit_init(decode_p);
    }
    else if (decode_p->timer!=decode_p->count)
     { 
        decode_p->count=0;
        decode_p->timer=0;
        remote_resync_init(decode_p);
    }
    break;
      
  case REMOTE_START_BIT:        //  前导码完成
    if (ir_state)//是否有低电平，已取反 58Ous 低电平
    {
      decode_p->count++;
    }
      if((decode_p->count==0)&&(!ir_state))
    {
      decode_p->timer=0;
      break;
    }
    if(decode_p->count!=decode_p->timer)
    {
        decode_p->count=0;
        decode_p->timer=0;
        remote_resync_init(decode_p);
    }
    else if((decode_p->count==decode_p->timer) && (decode_p->count>=1) && (decode_p->count<=3))
    {
       remote_data_bit_init(decode_p);
    }
    break;
    
  case REMOTE_DATA_BIT:
    if (!ir_state)
    {
      decode_p->count++;
    }
      if((decode_p->count==0)&&(ir_state))
    {
      decode_p->timer=0;
      break;
    }

    else if(decode_p->count!=decode_p->timer)
    {
         decode_p->rxByte <<= 1;
          switch (decode_p->count)
          {
            case 0:
                decode_p->count=0;
                decode_p->timer=0;
                remote_resync_init(decode_p);
                break;
            case 1:
            case 2:
            case 3:
              remote_stop_bit_init(decode_p);//取一位保存，低电平
              break;
           case 4:
           case 5:
                decode_p->count=0;
                decode_p->timer=0;
                remote_resync_init(decode_p);
                break;
           case 6:    
           case 7: 
              decode_p->rxByte++;
              remote_stop_bit_init(decode_p);
              break;
            default:
              remote_resync_init(decode_p);
              break;
          }
    }
    break;

  case REMOTE_STOP_BIT:
     if (ir_state)
    {
      decode_p->count++;
    }
     if((decode_p->count==0)&&(ir_state))
    {
      decode_p->timer=0;
      break;
    }
    if (decode_p->count > 0)
    {
              decode_p->count=0;
              decode_p->timer=0;
	      decode_p->bitsDecoded++;
              if((BIT16 == 1)&&(decode_p->bitsDecoded>8))
                printf("bitsDecoded=%d\r\n",decode_p->bitsDecoded);
#if 1             
	      if (decode_p->bitsDecoded >= 8)
	      {
                decode_p->bitsDecoded=0;
              if((decode_p->rxByte==0xC5)||(decode_p->rxByte==0xA6)||(decode_p->rxByte==0x59)&&(!BIT16))
              {             
                  remoteStatus = 1;
                  remote_resync_init(decode_p);
              }
              else if(decode_p->rxByte==0xDD)
                {
                  decode_p->rxByte = 0x00;
                  BIT16=1;
                  remote_data_bit_init(decode_p);
                  remoteStatus = 0;
                  printf("DDDD\r\n");
                }
              else if(BIT16)
              {
                BIT16=0;
                decode_p->bitsDecoded=0;
                remoteStatus = 2;
                remote_resync_init(decode_p);

              }
                
              
//                remoteStatus = 0;//成功，
//               else
//               remoteStatus = -1;//失败
//                decode_p->bitsDecoded=0;
//	        remote_resync_init(decode_p);
	      }
#endif             
#if 0          
	      if (decode_p->bitsDecoded >= 16)
	      {
               //printf("decode_p->rxByte&0x00ff=%x\r\n",decode_p->rxByte&0x00ff);
                if(((decode_p->rxByte&0x00ff)==0xC5)||((decode_p->rxByte&0x00ff)==0xA6)||((decode_p->rxByte&0x00ff)==0x59)||((decode_p->rxByte&0xff00)==0xDD00))
               {
                // printf("decode_p->rxByte=%x\r\n",decode_p->rxByte);
                 remoteStatus = 0;//成功，
                
               }
                else
                remoteStatus = -1;//失败
                decode_p->bitsDecoded=0;
	        remote_resync_init(decode_p);
	      }
#endif
	      else
	      { 
	        remote_data_bit_init(decode_p);
	      }
      } 
    break;
    
  default:
    break;
  }
  
  return(remoteStatus);
   
}
  



