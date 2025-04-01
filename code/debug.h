/***
 * !!! ��Ҫ����Ŀ�а������ļ�������ֻ�����ڴ�������� !!!
 */

// �� KEIL ��������������Ժ�
/*-�жϺ�
INT0        0
Timer0        1
INT1        2
Timer1        3
UART1        4
ADC 5
LVD        6
UART2        8
SPI        9
INT2        10
INT3        11
Timer2        12
INT4        16
UART3        17
UART4        18
Timer3        19
Timer4        20
CMP        21
I2C        24
USB        25
PWMA        26
PWMB        27
CANBUS        28
CAN2BUS        29
LINBUS        30
RTC        36
P0 �ж�        37
P1 �ж�        38
P2 �ж�        39
P3 �ж�        40
P4 �ж�        41
P5 �ж�        42
P6 �ж�        43
P7 �ж�        44
DMA_M2M �ж�        47
DMA_ADC �ж�        48
DMA_SPI        49
DMA_UR1T        50
DMA_UR1R        51
DMA_UR2T        52
DMA_UR2R        53
DMA_UR3T        54
DMA_UR3R        55
DMA_UR4T        56
DMA_UR4R        57
DMA_LCM        58
LCM        59
DMA_I2CT        60
DMA_I2CR        61
I2S        62
DMA_I2ST        63
DMA_I2SR        64

*/
#define interrupt
// �����жϡ�ʹ�á���ַ�����ȼ���

#define using      /*--ʹ��--*/
#define _at_       /*--@��ַ--*/
#define _priority_ /*--���ȼ�--*/
#define _task_     /*--����--*/

// ����洢���ͺ�

#define reentrant // �����뺯�� (����ݹ�д��,���κ���)
#define compact   // ʹ���ⲿRAM��
#define small     // ȫ��ȱʡ������λ���ⲿRAM����һҳ,�����ǿռ��SmallΪ��ԣ�ٶȽ�Small������largeҪ�죬��һ���м�״̬��
#define large     // ȫ��ȱʡ�����ɷ����ⲿRAM���������ǿռ�󣬿ɴ�����࣬ȱ�����ٶȽ�������ʾ���洢ģʽ��C51������ѡ����ѡ��
#define data      // Ĭ�ϵ����ݴ洢�ռ�
#define bdata     // ���ڶ���ֻռ��һ��λ��bit���ı���
#define idata     // ����洢�ռ�
#define pdata     // λ���ڴ��ַ��Χ0x00~0xFF֮������ݿռ䡣�����ڴ洢��̬�����������ջ���;ֲ�����
#define xdata     // ��һ������Ĵ洢���ռ䣬��λ��8051оƬ���ⲿRAM��,�����data�ռ����
#define code      // ����������ROM��Flash����

// �ڲ����ͺ꣬����λ(bit)������λ(sbit)�����⹦�ܼĴ���(sfr)

typedef bit bit;     /*--λ--*/
typedef sbit sbit;   /*--����λ--*/
typedef sfr sfr;     /*--���⹦�ܼĴ���--*/
typedef sfr16 sfr16; /*--���⹦�ܼĴ���16--*/
typedef sfr32 sfr32; /*--���⹦�ܼĴ���32--*/
