
#include <mega128.h>

unsigned char outdata;

void Putch(char data)
{
   while(!(UCSR1A & 0x20)); //�۽ſϷ� flag ��Ʈ�� 1�Ǹ� ����
   UDR1=data;
}

char Getch(void)
{
   while(!(UCSR1A & 0x80)); //���ſϷ� flag ��Ʈ�� 1�Ǹ� ����
   return UDR1; //UART0�� ���
} 


void main()
{ 
   DDRA = 0x00;
   
   UCSR1A=0x00; //flag �������͸� ������� ����
   UCSR1B=0x18; //���� enable, �۽� enable, ���ۺ�Ʈ 8bit
   UCSR1C=0x06; //�񵿱�� ���
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
