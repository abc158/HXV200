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
U8 remote_signal=0;
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
   if (!ir_state)            
   { 
        decode_p->count=0;
        remote_resync_init(decode_p);
    }
    else 
    {
      decode_p->count++;
    }
    if (decode_p->count>=12)
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
    if ((decode_p->timer==decode_p->count)&&(decode_p->count>=10)&&(decode_p->count<=13))
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
    if (ir_state)
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

//    if(decode_p->count==decode_p->timer)
//    {
//         if(decode_p->count>3&&decode_p->count<=5)
//         {
//           decode_p->count=0;
//           decode_p->timer=0;
//           remote_resync_init(decode_p);
//         }
//    }
    else if(decode_p->count!=decode_p->timer)
    {
//          if(remote_signal)
//    {
//        remote_signal=0;
//        printf("count %d timer %d \r\n",decode_p->count,decode_p->timer);
//    }
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
              remote_stop_bit_init(decode_p);
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
//     if (ir_state)
//    {
      decode_p->count++;
//    }
//     if((decode_p->count==0)&&(ir_state))
//    {
//      decode_p->timer=0;
//      break;
//    }
    if (decode_p->count > 0)
    {
              decode_p->count=0;
              decode_p->timer=0;
	      decode_p->bitsDecoded++;
             // if(remote_signal)
                
	      if (decode_p->bitsDecoded >= 8)
	      {
             
                if(decode_p->rxByte==0xDD)
                { 
                  remote_signal=1;
                  remote_data_bit_init(decode_p);
                  remoteStatus=0;
                  printf("DDDD\r\n");
                }
                //else if(((decode_p->rxByte==0xC5)||(decode_p->rxByte==0xA6)||(decode_p->rxByte==0x59))&&(!remote_signal))
                else if(decode_p->rxByte!=0xDD)
                {
                  if(((decode_p->rxByte==0xC5)||(decode_p->rxByte==0xA6)||(decode_p->rxByte==0x59))&&(!remote_signal))
                    remoteStatus = 1;
                  else if(remote_signal == 1)
                  {
                    remoteStatus = 2;
                    printf("remote_signal=%d\r\n",remote_signal);
                    printf("rxByte=%x\r\n",decode_p->rxByte);
                  }
                    remote_resync_init(decode_p);
                    remote_signal=0;
                    
                }
                else
                remoteStatus = 0;
                decode_p->bitsDecoded=0;
	       
	      }
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
  



