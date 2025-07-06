
#include <mega128.h>

unsigned char indata;

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
   DDRA=0xff;

   UCSR1A=0x00; //flag �������͸� ������� ����
   UCSR1B=0x18; //���� enable, �۽� enable, ���ۺ�Ʈ 8bit
   UCSR1C=0x06; //�񵿱�� ���
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
