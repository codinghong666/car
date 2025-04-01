/***
 * !!! 不要在项目中包含此文件！，它只能用于代码分析！ !!!
 */

// 非 KEIL 编译器，定义调试宏
/*-中断号
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
P0 中断        37
P1 中断        38
P2 中断        39
P3 中断        40
P4 中断        41
P5 中断        42
P6 中断        43
P7 中断        44
DMA_M2M 中断        47
DMA_ADC 中断        48
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
// 定义中断、使用、地址和优先级宏

#define using      /*--使用--*/
#define _at_       /*--@地址--*/
#define _priority_ /*--优先级--*/
#define _task_     /*--任务--*/

// 定义存储类型宏

#define reentrant // 可重入函数 (列如递归写法,修饰函数)
#define compact   // 使用外部RAM区
#define small     // 全部缺省变量均位于外部RAM区的一页,优势是空间较Small为宽裕速度较Small慢，较large要快，是一种中间状态。
#define large     // 全部缺省变量可放在外部RAM区，优势是空间大，可存变量多，缺点是速度较慢。提示：存储模式在C51编译器选项中选择
#define data      // 默认的数据存储空间
#define bdata     // 用于定义只占用一个位（bit）的变量
#define idata     // 特殊存储空间
#define pdata     // 位于内存地址范围0x00~0xFF之间的数据空间。它用于存储动态变量（例如堆栈）和局部变量
#define xdata     // 是一种特殊的存储器空间，它位于8051芯片的外部RAM中,相比于data空间更大
#define code      // 将变量放在ROM（Flash）中

// 内部类型宏，包括位(bit)、特殊位(sbit)和特殊功能寄存器(sfr)

typedef bit bit;     /*--位--*/
typedef sbit sbit;   /*--特殊位--*/
typedef sfr sfr;     /*--特殊功能寄存器--*/
typedef sfr16 sfr16; /*--特殊功能寄存器16--*/
typedef sfr32 sfr32; /*--特殊功能寄存器32--*/
