
#include <mega128.h>

unsigned char outdata;

void Putch(char data)
{
   while(!(UCSR1A & 0x20)); //송신완료 flag 비트가 1되면 정지
   UDR1=data;
}

char Getch(void)
{
   while(!(UCSR1A & 0x80)); //수신완료 flag 비트가 1되면 정지
   return UDR1; //UART0번 사용
} 


void main()
{ 
   DDRA = 0x00;
   
   UCSR1A=0x00; //flag 레지스터를 사용하지 않음
   UCSR1B=0x18; //수신 enable, 송신 enable, 전송비트 8bit
   UCSR1C=0x06; //비동기식 통신
   UBRR1H=0;                      
   UBRR1L=103; //9600bps   

   while(1)
   {     
 
          outdata = PINA;                 
          
          switch(outdata)
          {
          case  1: Putch('1');
                       break;
          case  2: Putch('2');
                        break;
          case  4: Putch('3');
                         break;
          case  8: Putch('4');
                       break;
          case  16: Putch('5');
                        break;
          default:  Putch('0');
                    break;
      }
   }   
}
