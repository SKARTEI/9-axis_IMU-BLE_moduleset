
#include <mega128.h>

unsigned char indata;

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
   DDRA=0xff;

   UCSR1A=0x00; //flag 레지스터를 사용하지 않음
   UCSR1B=0x18; //수신 enable, 송신 enable, 전송비트 8bit
   UCSR1C=0x06; //비동기식 통신
   UBRR1H=0;
   UBRR1L=103; //9600bps

   while(1)
   {
       indata = Getch();
       switch(indata){
          case '1': PORTA=0x01;
                       break;
          case '2': PORTA=0x02;
                       break;
          case '3': PORTA=0x04;
                      break;
          case '4': PORTA=0x08;
                     break;
          case '5': PORTA=0x10;
                    break;
          default:
          }
   }
}
