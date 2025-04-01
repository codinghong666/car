#include "./H_Group/bno08x.h" //���������
#include "./H_Group/can.h"	  //ͨѶ���
#include "./H_Group/config.h" //��������
#include "./H_Group/pid.h"	  //�㷨���
#include "./H_Group/spi.h"	  //flash���
#include "./H_Group/uart.h"	  //ͨѶ���

// ���̹߳���
// ����߳���
#define Task_Max 20
// �߳�ָ��
u8 Task = 0;
// �߳�״̬��
u8 Task_This[Task_Max] = {0};
// �߳�˽�ж�ʱ��
u16 Task_Timer[Task_Max] = {0};
u8 camer_ctrl_dat = 0;
#define XYABLBRB_Dat 1
#define TBLRBS_Dat 0

#define Key_X 0x40
#define Key_Y 0x80
#define Key_A 0x10
#define Key_B 0x20
#define Key_LB 0x01
#define Key_RB 0x02
#define Key_Up 0x01
#define Key_Down 0x02
#define Key_Left 0x04
#define Key_Right 0x08
#define Key_Back 0x20
#define Key_Start 0x10

// ��������
void Core_Init(void);
void GPIO_Init(void);
void Delay(unsigned int Time);
void Get_Delay(void);
void Timer0_Init(void);
void Timer4_Init(void);
void Auto_Fre_Calibrator(void);
// ���ƺ���
void Set_Moto(int left_speed, int right_speed);
void Set_DuoJi(int Dat);
float LowPassFilter(float x, LowPassConfig *p, float Tf);

// �����������
int RT, LT;					 // ����
char Left_Right, Top_Bottom; // ������ٶ�
float angle = 0;
// �����������
bit save_flag = 0;		  // �����־λ
bit save_run = 0;		  // �������������
#define diff_addr 130816L // ƫ�Ƶ�ַ,�洢����
u32 this_addr = 0;
// ��ʱ����
u8 full[4] = {0xff, 0xff, 0xff, 0xff}; // ����ָ��
float save_in[200];
float save_out[200];			// ���뻺��
u8 in_index = 0, out_index = 0; // ����ָ��
u8 d_in_index = 0, d_out_index = 0;
u8 _in_index = 0; // ����дָ����ֵ
u8 d__in_index = 0;
bit write_in_flag = 0;								 // д��æ��־λ
float save_left_postion = 0, save_right_postion = 0; // ��㶨��

// �򻬼���������
float a_diff = 0, a_imu = 0, a_speed = 0; // ���ٶȲ�ֵ��IMU���ٶȣ��ٶȴ��������ٶ�
long T_diff = 0, T_start = 0;			  // �������ڣ�������ʼʱ���
float v_diff = 0, v_imu = 0, v_speed = 0; // �ٶȲ�ֵ��IMU�ٶȻ��֣��ٶȴ������ٶ�
float v_imu_last = 0, v_speed_last = 0;	  // IMU�ٶ���ʷֵ���ٶȴ������ٶ���ʷֵ
float v_out = 0, v_out_last = 0;		  // �ٶ�������ֵ,�ٶ�������ֵ��ʷֵ
long v_T_start = 0, v_T_diff = 0;
// �������
int left_out, right_out;
float diff_left_gain = 0;
float diff_right_gain = 0;
float diff_all = 0;			   // ��pidʹ�õ�
float get_diff = 0, _get_diff; // �����ò���ֵ
long d_ldiff = 0, d_rdiff = 0; // �����ֵ
// ͨѶ��������
int moto_left_speed = 0, moto_right_speed = 0;
int servo_positon = 0;
bit Read_Finish = 0;
u8 n = 0, can_cnt = 0;
// �趨yaw
u16 long_index = 0; // Զ��ǰհ��
float set_yaw = 0;
float long_yaw = 0; // Զ��ǰհ
float err_yaw = 0;
bit A_M_Flag = 0; // �Զ�ģʽ��

bit high_speed_flag = 0;
bit middle_speed_flag = 0;
bit low_speed_flag = 0;
bit load_run_flag = 0;
bit stop_key_flag = 0;

int Auto_Angle = 0;
float add_pos = 0;
u32 save_index = 0;
u32 stop_index = 0;
// �����Զ�yaw
PID_Register yaw_pid, gz_pid;
void PID_Init(void)
{
	yaw_pid.P = 500 * 2.7;
	yaw_pid.I = 250 * 2.7;
	yaw_pid.D = 62 * 2.7;
	yaw_pid.I_limit = 500;
	yaw_pid.limit = 999;
	yaw_pid.output_ramp = 10000;

	gz_pid.P = 5;
	gz_pid.I = 2.5;
	gz_pid.D = 0.62;
	gz_pid.I_limit = 10;
	gz_pid.limit = 100; // ֻ�������ģ��޷�ƫ��100
	gz_pid.output_ramp = 100000;
}
// ����ʹ��
bit left_en = 0, right_en = 0, servo_en = 0;

float all_speed = 0; // ȫ�ֺ���ٶ�

// ͼ�����
int left_error = 0, right_error = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float qi = 0, qj = 0, qk = 0, qr = 0;
float roll = 0, pitch = 0, yaw = 0;
float d_roll = 0, d_pitch = 0, d_yaw = 0;
float yaw_cnt = 0, last_yaw = 0;
#define M_PI 3.14159265358979323846
LowPassConfig gz_loss, diff_loss, ay_loss, speed_loss;
bit dec_speed_flag = 0;
u8 light_state = 0, _light_state = 0;		// �ƹ��Ч,��Чģʽ����
u8 light_left_cnt = 0, light_right_cnt = 0; // ��˸������

void quatToEuler(Quaternion *q, float *yaw, float *pitch, float *roll)
{
	// ����roll (����)�����ر�ע���ĸΪ0�����������򻯴���ʵ��Ӧ���п�����Ҫ��ϸ�µ��ж�
	*roll = atan2(2 * (q->w * q->x + q->y * q->z), q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z);
	// ����yaw (ƫ��)
	*yaw = atan2(2 * (q->w * q->z - q->x * q->y), 1 - 2 * (q->y * q->y + q->z * q->z));
	// ����pitch (����)
	*pitch = 0;
}

Quaternion this_q;

float get_add_yaw(void) // �����ʷ�ۼƺ����
{
	return ((yaw_cnt * 2 * M_PI) + yaw);
}
float in_dat = 0;
void Timer3_Init(void);
void main(void)
{
	Core_Init(); // ϵͳ��ʼ��
	Timer3_Init();
	Uart1_Init(); // ���Դ��ڳ�ʼ��
	UART_Init();  // ���ڲ��ֳ�ʼ��
	PID_Init();
	Can_Init();
	PWM_Transform_Timer_Init(); // ��PWM��Ϊ��ʱ��
	I2C_Init();
	SPI_init();
	GPIO_Init(); // ����ʼ��GPIO����ֹ�����쳣��ƽ
	if (key1 == 0)
	{
		while (1)
			;
	}
	softReset();
	Delay_ms(1);
	// enableRotationVector(2500);
	enableGameRotationVector(2500); // 400hz
	enableGyro(2500);
	// enableLinearAccelerometer(2500);
	Auto_Fre_Calibrator();
	left_en = 1;
	right_en = 1;
	servo_en = 1;
	EA = 1;
	FlashCheckID();
	while (1)
	{
		Task = 0; // �߳�0���ܳ������ٶȾ���
		switch (Task_This[Task])
		{
		case 0:
			if (A_M_Flag == 1)
			{
				if (save_run == 1)
				{
					if (load_run_flag)
					{
						if (low_speed_flag)
						{
							if (fabs(long_yaw - set_yaw) < 0.5)
							{
								if (all_speed < 200)
									all_speed += 0.5;
							}
							else
							{
								if (all_speed > 180)
									all_speed -= 1.1;
								dec_speed_flag = 1;
							}
							if (all_speed < 0)
								all_speed = 0;
						}
						if (middle_speed_flag)
						{
							if (fabs(long_yaw - set_yaw) < 0.5)
							{
								if (all_speed < 500)
									all_speed += 0.5;
							}
							else
							{
								if (all_speed > 400)
									all_speed -= 1.1;
								dec_speed_flag = 1;
							}
							if (all_speed < 0)
								all_speed = 0;
						}
						if (high_speed_flag)
						{
							if (fabs(long_yaw - set_yaw) < 0.5)
							{
								if (all_speed < 900)
									all_speed += 0.8;
							}
							else
							{
								if (all_speed > 750)
									all_speed -= 1.5;
								dec_speed_flag = 1;
							}
							if (all_speed < 0)
								all_speed = 0;
						}
					}
					else
					{
						if (high_speed_flag == 0 && middle_speed_flag == 0 && low_speed_flag == 0)
							all_speed = (int)((float)(LT) * 3 + (float)((float)(RT) * 2.0));
					}
					left_out = all_speed;
					right_out = -all_speed;
					angle = ((float)servo_positon / 1637);
					diff_right_gain = fabs(((130 / tan(angle)) - (45)) / (sqrt(4225 + pow(130 / tan(angle), 2))));
					diff_left_gain = fabs(((130 / tan(angle)) + (45)) / (sqrt(4225 + pow(130 / tan(angle), 2))));
					Set_Moto(left_out * diff_left_gain, right_out * diff_right_gain);
					// ���ټ���
					// Set_Moto(left_out * diff_all, right_out * (1 - diff_all));
				}
				else
				{
					Set_Moto(0, 0);
				}
				Delay(1);
			}
			break;
		case 2:
			Task_This[Task] = 0;
			break;
		default:
			Get_Delay();
			break;
		}
		Task = 1; // �߳�1�������ǽ���
		switch (Task_This[Task])
		{
		case 0:
			if (dataAvailable() == 1)
			{
				Task_This[Task] = 3;
			}
			else
			{
				Delay(2);
			}
			break;
		case 2:
			Task_This[Task] = 0;
			break;
		case 3:
			gz = getGyroZ();
			// ay = getLinAccelY() * -300.0f;
			// T_diff = (Micros() - T_start);
			// T_start = Micros();
			// v_diff = (all_speed - v_speed_last) / (float)(T_diff * 1e-6);
			// v_speed_last = all_speed;
			// v_diff = LowPassFilter(v_diff, &speed_loss, 0.15); // ���ٶȵ�ͨ�˲�
			// ay = LowPassFilter(ay, &ay_loss, 0.15);			   // ���ٶȵ�ͨ�˲�
			// a_imu += ay / (float)(T_diff);
			// v_imu += a_imu;
			// gz = LowPassFilter(gz, &gz_loss, 0.08);
			this_q.x = getQuatI();
			this_q.y = getQuatJ();
			this_q.z = getQuatK();
			this_q.w = getQuatReal();
			quatToEuler(&this_q, &yaw, &pitch, &roll);
			if ((yaw - last_yaw) > 3)
			{
				yaw_cnt--;
			}
			if ((yaw - last_yaw) < -3)
			{
				yaw_cnt++;
			}
			last_yaw = yaw;
			if (save_run == 1 && A_M_Flag == 1)
			{
				long_index = (int)(all_speed / 10.0f) * 4;
				SPI_Read_Nbytes(this_addr, (u8 *)&set_yaw, 4);
				if ((this_addr + long_index) < stop_index)
				{
					SPI_Read_Nbytes(this_addr + long_index, (u8 *)&long_yaw, 4);
				}
				servo_positon = PID_Ctrl(get_add_yaw() - set_yaw, &yaw_pid);
				// diff_all = PID_Ctrl(((((save_table[save_index + 1] - save_table[save_index])) * (float)(all_speed) * 0.06)) - gz, &gz_pid);
			}
			if (save_flag == 0)
			{
				if (A_M_Flag == 1)
				{
					if (save_run == 0 && save_flag == 0)
						light_state = state_auto_norun;
					else
					{
						if (dec_speed_flag == 1) // ���ٶȲ����ٸĵ�
						{
							dec_speed_flag = 0;
							light_state = state_stop;
						}
						else
						{
							if (light_state != state_stop)
							{
								if (gz > 1)
									light_state = state_left;
								if (gz < -1)
									light_state = state_right;
							}
						}
					}
				}
				else
				{
					if (light_state == state_noaction)
					{
						if (gz > 1)
							light_state = state_left;
						if (gz < -1)
							light_state = state_right;
						if (all_speed == 0)
							light_state = state_stop;
						if (light_state == state_stop && all_speed != 0)
							light_state = state_noaction;
					}
				}
			}
			else
			{
				light_state = state_double;
			}
			Task_This[Task] = 0;
			break;
		default:
			Get_Delay();
			break;
		}
		// �߳�2,���ڵ��Գ���
		// Task = 2;
		// switch (Task_This[Task])
		// {
		// case 0:
		// 	// if (fabs(ay - v_diff) > 300)
		// 	// 	buzz = 0;
		// 	// else
		// 	// 	buzz = 1;
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%f,%f\r\n", ay, v_diff);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%.2f,%.2f,%ld,%ld,%f\r\n", get_add_yaw(), set_yaw, this_addr, stop_index, all_speed);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%d,%d,%d\r\n", moto_left_speed, moto_right_speed, (int)A_M_Flag);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%d,%d,%d,%d,%d\r\n", save_index, stop_index,(int)save_flag,(int)save_run,(int)A_M_Flag);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%d,%d,%d,%d,%d\r\n", (int)(camera_dat_l - left_error), (int)(camera_dat_r - right_error), (int)camera_state_l, (int)camera_state_r, (int)road_state);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%d,%f,%f,%f,%f,%f\r\n", (int)(xy_angle * 57.29577), xy_lenth, ax, ay, sx_i, sy_i);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%d,%d,%d,%d,%f,%f\r\n", (int)(camera_dat_l - left_error), (int)(camera_dat_r - right_error),(int)ccd_pd.e1, (int)Auto_Angle,set_yaw,get_add_yaw());
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%f,%d,%f,%f,%f\r\n", ((float)servo_positon / -300), servo_positon, gz, speed_diff.e1, speed_diff.output);
		// 	// Tx_Cnt = sprintf(TX1_Buffer, "%f,%f,%d\r\n", get_add_yaw(), set_yaw, servo_positon);
		// 	// DMA_Send_Start();
		// 	//Delay(20);
		// 	break;
		// case 2:
		// 	Task_This[Task] = 0;
		// 	break;
		// default:
		// 	Get_Delay();
		// 	break;
		// }
		Task = 3;
		switch (Task_This[Task])
		{
		case 0:
			if (uart_dat_pack_done_flag == 0)
				Task_This[Task]++;
			break;
		case 1:
			Left_Right = Ctrl[5];
			Top_Bottom = Ctrl[4];
			RT = Ctrl[3];
			LT = Ctrl[2];
			if (A_M_Flag == 0)
			{
				if (Left_Right == 0 && Top_Bottom == 0)
					angle = 0;
				else if (Left_Right == 0 && Top_Bottom != 0)
				{
					angle = 0;
				}
				else
					angle = atan2((float)Left_Right, fabs((float)Top_Bottom));
				if (angle > 1.5)
					angle = 1.5;
				if (angle < -1.5)
					angle = -1.5;
				Set_DuoJi((int)((angle * 666.6))); // 0�У�+-1000��Χ
				all_speed = (int)(((float)(sqrt(pow(Top_Bottom, 2) + pow(Left_Right, 2))) * 6.0) + (float)((float)(RT) * 2.0));

				left_out = all_speed;
				right_out = -all_speed;
				if (Top_Bottom == 0 && Left_Right == 0)
				{
					Set_Moto(0, 0);
				}
				else if (Left_Right == 0 && Top_Bottom != 0)
				{
					if (Top_Bottom >= 0)
						Set_Moto(left_out, right_out);
					else
						Set_Moto(left_out * -1, right_out * -1);
				}
				else
				{
					angle = ((float)servo_positon / 1909);
					diff_right_gain = fabs(((130 / tan(angle)) - (45)) / (sqrt(4225 + pow(130 / tan(angle), 2))));
					diff_left_gain = fabs(((130 / tan(angle)) + (45)) / (sqrt(4225 + pow(130 / tan(angle), 2))));
					if (Top_Bottom >= 0)
						Set_Moto(left_out * diff_left_gain, right_out * diff_right_gain);
					else
						Set_Moto(left_out * diff_left_gain * -1, right_out * diff_right_gain * -1);
				}
			}
			Delay(1);
			break;
		case 3:
			Task_This[Task] = 0;
			break;
		default:
			Get_Delay();
			break;
		}
		Task = 4; // ͬ��CAN���ݴ���
		switch (Task_This[Task])
		{
		case 0:
			CAN1_Tx.FF = STANDARD_FRAME; // ��׼֡
			CAN1_Tx.RTR = 0;			 // 0������֡��1��Զ��֡
			CAN1_Tx.DLC = 0x08;			 // ���ݳ���
			CAN1_Tx.ID = 0x0001;		 // CAN ID
			if (servo_en)
			{
				CAN1_Tx.DataBuffer[0] = 0xa0; // ��������
			}
			else
			{
				CAN1_Tx.DataBuffer[0] = 0x20; // ��������
			}
			CAN1_Tx.DataBuffer[1] = 0x05;
			CAN1_Tx.DataBuffer[2] = 0x00;
			CAN1_Tx.DataBuffer[3] = 0x00;
			CAN1_Tx.DataBuffer[4] = (char)((long)(servo_positon) >> 24);
			CAN1_Tx.DataBuffer[5] = (char)((long)(servo_positon) >> 16);
			CAN1_Tx.DataBuffer[6] = (char)((long)(servo_positon) >> 8);
			CAN1_Tx.DataBuffer[7] = (char)((long)(servo_positon));
			CanSendMsg(&CAN1_Tx);
			Task_This[Task] = 1;
			break;
		case 1:
			if (B_Can1Send == 0) // ������ɱ�־
				Task_This[Task] = 2;
			break;
		case 2:
			CAN1_Tx.FF = STANDARD_FRAME; // ��׼֡
			CAN1_Tx.RTR = 0;			 // 0������֡��1��Զ��֡
			CAN1_Tx.DLC = 0x08;			 // ���ݳ���
			CAN1_Tx.ID = 0x0003;		 // CAN ID
			if (left_en)
			{
				CAN1_Tx.DataBuffer[0] = 0xb0; // ��������
			}
			else
			{
				CAN1_Tx.DataBuffer[0] = 0x30; // ��������
			}
			CAN1_Tx.DataBuffer[1] = 0x01;
			CAN1_Tx.DataBuffer[2] = 0x00;
			CAN1_Tx.DataBuffer[3] = 0x00;
			CAN1_Tx.DataBuffer[4] = (char)((long)(moto_left_speed) >> 24);
			CAN1_Tx.DataBuffer[5] = (char)((long)(moto_left_speed) >> 16);
			CAN1_Tx.DataBuffer[6] = (char)((long)(moto_left_speed) >> 8);
			CAN1_Tx.DataBuffer[7] = (char)((long)(moto_left_speed));
			CanSendMsg(&CAN1_Tx);
			Task_This[Task] = 3;
			break;
		case 3:
			if (B_Can1Send == 0) // ������ɱ�־
				Task_This[Task] = 4;
			break;
		case 4:
			// //====��ʼ������=====
			CAN1_Tx.FF = STANDARD_FRAME; // ��׼֡
			CAN1_Tx.RTR = 0;			 // 0������֡��1��Զ��֡
			CAN1_Tx.DLC = 0x08;			 // ���ݳ���
			CAN1_Tx.ID = 0x0002;		 // CAN ID
			if (right_en)
			{
				CAN1_Tx.DataBuffer[0] = 0xb0; // ��������
			}
			else
			{
				CAN1_Tx.DataBuffer[0] = 0x30; // ��������
			}
			CAN1_Tx.DataBuffer[1] = 0x01;
			CAN1_Tx.DataBuffer[2] = 0x00;
			CAN1_Tx.DataBuffer[3] = 0x00;
			CAN1_Tx.DataBuffer[4] = (char)((long)(moto_right_speed) >> 24);
			CAN1_Tx.DataBuffer[5] = (char)((long)(moto_right_speed) >> 16);
			CAN1_Tx.DataBuffer[6] = (char)((long)(moto_right_speed) >> 8);
			CAN1_Tx.DataBuffer[7] = (char)((long)(moto_right_speed));
			CanSendMsg(&CAN1_Tx);
			Task_This[Task] = 5;
			break;
		case 5:
			if (B_Can1Send == 0) // ������ɱ�־
				Task_This[Task] = 6;
			break;
		case 6:							  // ͬ������
			CAN1_Tx.FF = STANDARD_FRAME;  // ��׼֡
			CAN1_Tx.RTR = 0;			  // 0������֡��1��Զ��֡
			CAN1_Tx.DLC = 0x08;			  // ���ݳ���
			CAN1_Tx.ID = 0x0000;		  // CAN ID
			CAN1_Tx.DataBuffer[0] = 0x00; // ��������
			CAN1_Tx.DataBuffer[1] = 0x00;
			CAN1_Tx.DataBuffer[2] = 0x00;
			CAN1_Tx.DataBuffer[3] = 0x55;
			CAN1_Tx.DataBuffer[4] = 0x00;
			CAN1_Tx.DataBuffer[5] = 0x00;
			CAN1_Tx.DataBuffer[6] = 0x00;
			CAN1_Tx.DataBuffer[7] = 0x00;
			CanSendMsg(&CAN1_Tx);
			Task_This[Task] = 7;
			break;
		case 7:
			if (B_Can1Send == 0) // ������ɱ�־
				Task_This[Task] = 0;
			break;
		// case 9:										// ͬ������
		// 	CAN1_Tx.FF = STANDARD_FRAME;			// ��׼֡
		// 	CAN1_Tx.RTR = 0;						// 0������֡��1��Զ��֡
		// 	CAN1_Tx.DLC = 0x01;						// ���ݳ���
		// 	CAN1_Tx.ID = 0x000a;					// CAN ID
		// 	CAN1_Tx.DataBuffer[0] = camer_ctrl_dat; // ��������
		// 	camer_ctrl_dat = 0;						// ִ��һ�ξ����
		// 	CanSendMsg(&CAN1_Tx);
		// 	Task_This[Task]++;
		// 	break;
		// case 10:
		// 	if (B_Can1Send == 0) // ������ɱ�־
		// 		Delay(1);
		// 	break;
		// case 12:									// ͬ������
		// 	CAN1_Tx.FF = STANDARD_FRAME;			// ��׼֡
		// 	CAN1_Tx.RTR = 0;						// 0������֡��1��Զ��֡
		// 	CAN1_Tx.DLC = 0x01;						// ���ݳ���
		// 	CAN1_Tx.ID = 0x000b;					// CAN ID
		// 	CAN1_Tx.DataBuffer[0] = camer_ctrl_dat; // ��������
		// 	camer_ctrl_dat = 0;						// ִ��һ�ξ����
		// 	CanSendMsg(&CAN1_Tx);
		// 	Task_This[Task]++;
		// 	break;
		// case 13:
		// 	if (B_Can1Send == 0) // ������ɱ�־
		// 		Task_This[Task]++;
		// 	break;
		// case 14:						 // ͬ������
		// 	CAN1_Tx.FF = STANDARD_FRAME; // ��׼֡
		// 	CAN1_Tx.RTR = 1;			 // 0������֡��1��Զ��֡
		// 	CAN1_Tx.DLC = 0x01;			 // ���ݳ���
		// 	CAN1_Tx.ID = 0x000a;		 // CAN ID
		// 	CanSendMsg(&CAN1_Tx);
		// 	Task_This[Task]++;
		// 	break;
		// case 15:
		// 	if (B_Can1Send == 0) // ������ɱ�־
		// 	{
		// 		Read_Finish = 1;
		// 		Task_This[Task]++;
		// 	}
		// 	break;
		// case 16:
		// 	if (Read_Finish == 0) // ��ȡ��ɱ�־
		// 		Task_This[Task]++;
		// 	break;
		// case 17:						 // ͬ������
		// 	CAN1_Tx.FF = STANDARD_FRAME; // ��׼֡
		// 	CAN1_Tx.RTR = 1;			 // 0������֡��1��Զ��֡
		// 	CAN1_Tx.DLC = 0x01;			 // ���ݳ���
		// 	CAN1_Tx.ID = 0x000b;		 // CAN ID
		// 	CanSendMsg(&CAN1_Tx);
		// 	Task_This[Task]++;
		// 	break;
		// case 18:
		// 	if (B_Can1Send == 0) // ������ɱ�־
		// 	{
		// 		Read_Finish = 1;
		// 		Task_This[Task]++;
		// 	}
		// case 19:
		// 	if (Read_Finish == 0) // ������ɱ�־
		// 		Task_This[Task] = 0;
		// 	break;
		default:
			Get_Delay();
			break;
		}
		Task = 5; // ͨѶ
		switch (Task_This[Task])
		{
		case 0:
			if (B_Can1Read)
			{
				B_Can1Read = 0;
				CANSEL = CAN1;			 // ѡ��CAN1ģ��
				n = CanReadMsg(CAN1_Rx); // ��ȡ��������
				for (can_cnt = 0; can_cnt < n; can_cnt++)
				{
					Can_Dat_Handle(&CAN1_Rx[can_cnt]);
				}
				// Read_Finish = 0;
			}
			break;
		default:
			Get_Delay();
			break;
		}
		// �ֶ��Զ�ģʽ�л�
		Task = 6;
		switch (Task_This[Task])
		{
		case 0:
			if (((Ctrl[XYABLBRB_Dat] & Key_A) || key1 == 0) && stop_key_flag == 0)
			{
				Delay(1);
			}
			break;
		case 2:
			if (((Ctrl[XYABLBRB_Dat] & Key_A) || key1 == 0) && stop_key_flag == 0)
			{
				A_M_Flag = ~A_M_Flag;
				set_yaw = get_add_yaw();
				// if (camera_state_l != 1 || camera_state_r != 1)
				// 	A_M_Flag = 0;
				if (A_M_Flag) // �����Զ�ģʽǰ���ȶ�pid�������㣬��ֹ����
				{
					all_speed = 0;
					// left_error = camera_dat_l;
					// right_error = camera_dat_r; // �����Զ�ģʽǰ��������һ�ξ���
				}
				else
				{
					left_en = 1;
					right_en = 1;
					servo_en = 1; // �ָ����еı�־λ
					save_run = 0;
					save_flag = 0;
					load_run_flag = 0;
					low_speed_flag = 0;
					middle_speed_flag = 0;
					high_speed_flag = 0;
				}
				Task_This[Task]++;
			}
			else
			{
				Task_This[Task] = 0;
			}
			break;
		case 3:
			if ((Ctrl[XYABLBRB_Dat] & Key_A) == 0 && key1 == 1)
			{
				Task_This[Task] = 0;
			}
			break;
		default:
			Get_Delay();
			break;
		}
		// ������Ӧ
		Task = 7;
		switch (Task_This[Task])
		{
		case 0:
			if (((Ctrl[XYABLBRB_Dat] & Key_X) || key3 == 0) && stop_key_flag == 0)
			{
				Delay(1);
			}

		case 2:
			if (((Ctrl[XYABLBRB_Dat] & Key_X) || key3 == 0) && stop_key_flag == 0)
			{
				save_flag = 0;
				save_run = 0;
				// �˳�����
				stop_index = this_addr;
				Task_This[Task]++;
			}
			else
			{
				Task_This[Task] = 0;
			}

		case 3:
			if ((Ctrl[XYABLBRB_Dat] & Key_X) == 0 || key3 == 1)
			{
				Task_This[Task] = 0;
			}
		default:
			Get_Delay();
			break;
		}
		// ��ʼ����·��
		Task = 8;
		switch (Task_This[Task])
		{
		case 0:
			if (((Ctrl[TBLRBS_Dat] & Key_Start) || key2 == 0) && stop_key_flag == 0)
			{
				Delay(1);
			}
			break;
		case 2:
			if (((Ctrl[TBLRBS_Dat] & Key_Start) || key2 == 0) && stop_key_flag == 0)
			{
				// ���еĳ���
				if (A_M_Flag == 1)
				{
					save_flag = 1;
					in_index = 1; // ǿ��д1
					add_pos = 0;  // ���λ�û���
					save_run = 0;
					// �������
					// yaw_cnt = 0;
					err_yaw = yaw_cnt;
					set_yaw = get_add_yaw(); // ����
					FlashSectorErase(0UL);	 // ��Ϊ32K�����
					servo_en = 0;
					this_addr = 0;
					left_en = 0;
					right_en = 0;
				};
				Task_This[Task]++;
			}
			else
			{
				Task_This[Task] = 0;
			}
			break;
		case 3:
			if ((Ctrl[TBLRBS_Dat] & Key_Start) == 0 || key2 == 1)
			{
				Task_This[Task] = 0;
			}
			break;
		default:
			Get_Delay();
			break;
		}
		// ��ʼ����·��
		Task = 9;
		switch (Task_This[Task])
		{
		case 0:
			if (((Ctrl[TBLRBS_Dat] & Key_Back) || key4 == 0) && stop_key_flag == 0)
			{
				Delay(1);
			}
			break;
		case 2:
			if (((Ctrl[TBLRBS_Dat] & Key_Back) || key4 == 0) && stop_key_flag == 0)
			{
				// ���еĳ���
				if (A_M_Flag == 1)
				{
					load_run_flag = 0;
					// save_left_postion = read_left_postion;
					// save_right_postion = read_right_postion;
					save_run = 1;
					save_flag = 0;
					// yaw_cnt = 0;
					// this_addr = 13 * 4; // ��ͷ��ʼ,���복����
					this_addr = 24; // ��ͷ��ʼ,���복����
					yaw_cnt = err_yaw;
					SPI_Read_Nbytes(this_addr, (u8 *)&set_yaw, 4); // ����
					servo_en = 1;
					left_en = 1;
					right_en = 1;
				}
				Task_This[Task]++;
			}
			else
			{
				Task_This[Task] = 0;
			}
			break;
		case 3:
			if ((Ctrl[TBLRBS_Dat] & Key_Back) == 0 && key4 == 1)
			{
				Task_This[Task] = 0;
			}
			break;
		default:
			Get_Delay();
			break;
		}
		// �ƹ��źŷ�Ӧ���������
		Task = 10;
		switch (Task_This[Task])
		{
		case 0:
			switch (light_state)
			{
			case state_noaction:
			{
				// �ƹ�ʡ��
				P2M0 |= 0xcf;
				P2M1 |= 0xcf;
				P3M0 |= 0xc0;
				P3M1 |= 0xc0;
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0; // �ƹ�ȫ��
			}
			break;
			case state_left:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 1;
				Y5 = Y6 = 0;
				Delay(200);
			}
			break;
			case state_right:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 1;
				Delay(200);
			}
			break;
			case state_stop:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 1;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
				Delay(1000);
			}
			break;
			case state_double:
			{
				// �ƹ�ʡ��
				P2M0 |= 0xcf;
				P2M1 |= 0xcf;
				P3M0 |= 0xc0;
				P3M1 |= 0xc0;
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 1;
				Y5 = Y6 = 1;
				Delay(200);
			}
			break;
			case state_auto_norun:
			{
				// �ƹ�ʡ��
				P2M0 |= 0xcf;
				P2M1 |= 0xcf;
				P3M0 |= 0xc0;
				P3M1 |= 0xc0;
				R1 = R2 = R3 = R4 = 1;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
			}
			break;
			default:
				break;
			}
			_light_state = light_state; // ��ʼ����һ��
			break;
		case 2:
			switch (light_state)
			{
			case state_left:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
				Delay(200);
			}
			break;
			case state_right:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
				Delay(200);
			}
			break;
			case state_stop:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
				light_state = state_noaction; // �Զ����
				Task_This[Task] = 0;
			}
			break;
			case state_double:
			{
				// �ƹ�ʡ��
				P2M0 |= 0xcf;
				P2M1 |= 0xcf;
				P3M0 |= 0xc0;
				P3M1 |= 0xc0;
				R1 = R2 = R3 = R4 = 0;
				Y1 = Y2 = 0;
				Y5 = Y6 = 0;
				Delay(200);
			}
			break;
			default:
				break;
			}
			break;
		case 4:
			switch (light_state)
			{
			case state_left:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				light_left_cnt++;
				if (light_left_cnt >= 2)
				{
					light_left_cnt = 0;
					light_state = state_noaction;
				}
				Task_This[Task] = 0;
			}
			break;
			case state_right:
			{
				if (A_M_Flag)
				{
					// �ƹ�����
					P2M0 |= 0xcf;
					P2M1 &= ~0xcf;
					P3M0 |= 0xc0;
					P3M1 &= ~0xc0;
				}
				else
				{
					// �ƹ�ʡ��
					P2M0 |= 0xcf;
					P2M1 |= 0xcf;
					P3M0 |= 0xc0;
					P3M1 |= 0xc0;
				}
				light_right_cnt++;
				if (light_right_cnt >= 2)
				{
					light_right_cnt = 0;
					light_state = state_noaction;
				}
				Task_This[Task] = 0;
			}
			break;
			case state_double:
			{
				// �ƹ�ʡ��
				P2M0 |= 0xcf;
				P2M1 |= 0xcf;
				P3M0 |= 0xc0;
				P3M1 |= 0xc0;
				Task_This[Task] = 0;
			}
			break;
			default:
				break;
			}
			break;
		default:
		{
			if (_light_state != light_state)
			{
				// ģʽ�л���ǿ�д��
				light_left_cnt = 0;
				light_right_cnt = 0; // ��ռ�����
				Task_This[Task] = 0;
			}
			else
			{
				Get_Delay();
			}
		}
		break;
		}
		// �յ���־�����б���
		Task = 2; // ���ô����߳�
		switch (Task_This[Task])
		{
		case 0:
			if (write_in_flag == 0 && save_flag == 1 && in_index > 0) // ����״̬���������
			{
				EA = 0;
				_in_index = in_index;
				memcpy(save_out, save_in, (4 * _in_index)); // flaot����ռ�ĸ��ֽ�
				in_index = 0;								// ���㸴λ
				EA = 1;
				out_index = 0;
				Task_This[Task]++;
			}
			break;
		case 1:
			// ��ʼд��
			in_dat = save_out[out_index];
			out_index++;
			SPI_Write_Nbytes(this_addr, (u8 *)&in_dat, 4);
			this_addr += 4;
			if (out_index == _in_index)
			{
				Task_This[Task] = 0;
			}
			break;
		default:
			Get_Delay();
			break;
		}
		// ��ʼ�ٶȾ���
		Task = 15;
		switch (Task_This[Task])
		{
		case 0:
			if (key5 == 0)
			{
				Delay(1);
			}
			break;
		case 2:
			if (key5 == 0)
			{
				// low_speed_flag = 1;
				// Task_This[Task] = 0;
				Task_This[Task]++;
			}
			else
			{
				Task_This[Task] = 0;
			}
			break;
		case 3:
			load_run_flag = 0;
			stop_key_flag = 1;
			// ��ʱ��������Ϊ���١����١����٣�����ʱ���ٰ������ļ����������Ϲ���
			if (key1 == 0)
			{
				low_speed_flag = 1;
				middle_speed_flag = 0;
				high_speed_flag = 0;
				Task_This[Task] = 4;
				while (key1 == 0)
					;
				stop_key_flag = 0;
			}
			if (key2 == 0)
			{
				low_speed_flag = 0;
				middle_speed_flag = 1;
				high_speed_flag = 0;
				Task_This[Task] = 4;
				while (key2 == 0)
					;
				stop_key_flag = 0;
			}
			if (key3 == 0)
			{
				low_speed_flag = 0;
				middle_speed_flag = 0;
				high_speed_flag = 1;
				Task_This[Task] = 4;
				while (key3 == 0)
					;
				stop_key_flag = 0;
			}
			break;
		case 4:
			buzz = 0;
			Delay(100);
			break;
		case 6:
			buzz = 1;
			Delay(100);
			break;
		case 8:
			buzz = 0;
			Delay(100);
			break;
		case 10:
			buzz = 1;
			Delay(100);
			break;
		case 12:
			buzz = 0;
			Delay(100);
			break;
		case 14:
			buzz = 1;
			Delay(100);
			break;
		case 16:
			buzz = 0;
			Delay(100);
			break;
		case 18:
			buzz = 1;
			Delay(100);
			break;
		case 20:
			load_run_flag = 1;
			Task_This[Task] = 0;
			break;
		default:
			Get_Delay();
			break;
		}
	};
}

void Delay_ms(u16 time) //@52MHz
{
	unsigned long edata i;
	for (; time > 0; time--)
	{
		_nop_();
		_nop_();
		_nop_();
		i = 12998UL;
		while (i)
			i--;
	}
}

void ICacheOn(void) // ��ICACHE����
{
	bit fEA;

	if (WTST > 0)
	{
		fEA = EA;
		EA = 0; // �ر��жϣ���ֹд��������������;�����ж�
		_nop_();
		_nop_();
		TA = 0xaa;	   // д�봥����������1
					   // �˴������������κ�ָ��
		TA = 0x55;	   // д�봥����������2
					   // �˴������������κ�ָ��
		ICHECR = 0x01; // д������ʱ�رգ������޸�ICHECR�е�ENλ
					   // ENΪ�ٴν���д����״̬
		_nop_();
		_nop_();
		EA = fEA;
	}
}

void ICacheOff(void) // �ر�ICACHE����
{
	bit fEA;

	fEA = EA;
	EA = 0; // �ر��жϣ���ֹд��������������;�����ж�
	_nop_();
	_nop_();
	TA = 0xaa;	   // д�봥����������1
				   // �˴������������κ�ָ��
	TA = 0x55;	   // д�봥����������2
				   // �˴������������κ�ָ��
	ICHECR = 0x00; // д������ʱ�رգ������޸�ICHECR�е�ENλ
				   // ENλ�ٴν���д����״̬
	_nop_();
	_nop_();
	EA = fEA;
}

void Timer4_Init(void) // �ٶ��������@1ms�����ȼ�0
{
	TM4PS = 0x00;  // ���ö�ʱ��ʱ��Ԥ��Ƶ ( ע��:��������ϵ�ж��д˼Ĵ���,������鿴�����ֲ� )
	T4T3M |= 0x20; // ��ʱ��ʱ��1Tģʽ
	T4L = 0xE0;	   // ���ö�ʱ��ʼֵ
	T4H = 0x34;	   // ���ö�ʱ��ʼֵ
	T4T3M |= 0x80; // ��ʱ��4��ʼ��ʱ
	IE2 |= 0x40;   // ʹ�ܶ�ʱ��4�ж�
}

float LowPassFilter(float x, LowPassConfig *p, float Tf)
{
	unsigned long int time_temp = Micros();
	float dt = (time_temp - p->last_time) * 1e-6f;
	float alpha, y;
	if (dt < 0.0f)
		dt = 1e-3f;
	else if (dt > 0.3f)
	{
		p->last_result = x;
		p->last_time = time_temp;
		return x;
	}
	alpha = Tf / (Tf + dt);
	y = alpha * p->last_result + (1.0f - alpha) * x;
	p->last_result = y;
	p->last_time = time_temp;
	return y;
}

void Core_Init(void)
{
	EAXFR = 1;	  // ʹ�ܷ����ⲿXFR
	CKCON = 0x00; // �ⲿ���������ٶ�Ϊ���

	// ����HPLLʱ����
	USBCLK |= 0xe0; // ��PLL��PLL����ʱ��52/8=6.5Mhz
	HPLLCR |= 14;	// HPLL���Ϊ260Mhz
	HPLLCR |= 0x80; // ����HPLL

	Timer4_Init();
	// PWM����ӿڵ�ƽԤȷ��

	ICacheOn(); // �򿪸��ٻ���
}

void Auto_Fre_Calibrator(void)
{
	// ��CRE�Զ�׷Ƶ
	X32KCR = 0x80; // ���ⲿ32K����
	while (!(X32KCR & 1))
		;		   // �ȴ��ȶ���־λ
	IRTRIM = 0x80; // ��ʼ��ΪƵ�ε��м�Ƶ��
	WTST = WAIT;
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	CLKDIV = DIV; // ����ϵͳ��Ƶϵ��
	IRCBAND &= ~BAND_MASK;
	IRCBAND |= BAND; // ����IRCƵ�η�Χ���û��������Ӱ��USBʱ������
	// ����׷Ƶ����
	CRECNTH = CNT >> 8;
	CRECNTL = CNT;
	CRERES = XRES; // ����CREУ׼���
	CRECR = 0x90 | CREHF;
	while (CRECR & 0x01 == 0)
		;
}

void GPIO_Init(void)
{
	// IO��ʼ��
	P0M0 = 0x04;
	P0M1 = 0xf8;
	P1M0 = 0x00;
	P1M1 = 0x3e;

	P2M0 = 0xff;
	P2M1 = 0xff;
	P3M0 = (P3M0 & ~0x08) | 0xf4;
	P3M1 = (P3M1 & ~0x3c) | 0xc0;
	P4M0 = 0x00;
	P4M1 = 0xff;
	P5M0 = 0x00;
	P5M1 = 0xff;

	P0PU = 0x08;
	P1PU = 0x38;
	P2PU = 0xff;
	P3PU = 0xc0;
	P3SR = 0xc3;
	P3DR = 0xcb;
	P5PU = 0x10;
}

void Set_DuoJi(int Dat)
{
	if (Dat >= 1000)
		Dat = 1000;
	if (Dat <= -1000)
		Dat = -1000;
	if (A_M_Flag == 0) // �ֶ�ģʽ
	{
		servo_positon = Dat;
	}
}

void Set_Moto(int left_speed, int right_speed)
{
	//====��ʼ������=====
	moto_left_speed = left_speed;
	moto_right_speed = right_speed;
}

// ���÷Ƕ�����ʱ���̶�1ms
void Delay(unsigned int Time)
{
	Task_Timer[Task] = Time;
	Task_This[Task]++;
}

// ��ȡ��ǰ��ʱ��״̬
void Get_Delay(void)
{
	if (Task_Timer[Task] == 0)
	{
		Task_This[Task]++;
	}
	else
	{
		// �ղ���
	}
}

// ����λ�ô���
// ����ȫ�����м�Ϊ��׼��������������ʻ����Ϊ100��110ʱ�򣬾����ֵΪ5��Ϊ���Ҳ�ֵ��һ��
long l_ldiff = 0, l_rdiff = 0;			  // ��ʷ����
float this_postion = 0, last_postion = 0; // ���䵱ǰλ��ֵ,��ʷֵ
float _get_yaw = 0;
void Timer3_Isr(void) interrupt 19
{
	this_postion = ((float)(fabs(read_left_postion) + fabs(read_right_postion)) / 2.0);
	add_pos += (this_postion - last_postion); // ��λcm
	// this_postion = ((float)(fabs(read_left_postion - save_left_postion) + fabs(read_right_postion - save_right_postion)) / 2.0);
	last_postion = this_postion;

	if (fabs(add_pos) >= 36384.0)
		add_pos = 0; // �ϴ�ֱֵ������
	// if (fabs(speed_add_pos) >= 36384.0)
	// 	speed_add_pos = 0;				  // �ϴ�ֱֵ������
	// if (fabs(speed_add_pos) >= (16384.0)) // ��Լ1cm�ж�һ��
	// {
	// 	v_T_diff = (Micros() - v_T_start);
	// 	v_T_start = Micros();
	// 	v_speed = ((add_pos*1.3038836697) / (float)v_T_diff)*100;//��λcm/s
	// 	speed_add_pos -= 16384.0;
	// }
	if (fabs(add_pos) >= (800.0)) // ��Լ1cm�ж�һ��
	{
		// ��ʼ����·��
		if (save_flag == 1)
		{
			_get_yaw = get_add_yaw();
			servo_en = 0;
			write_in_flag = 1;
			save_in[in_index] = _get_yaw;
			if (in_index < 190)
				in_index++;
			write_in_flag = 0;
		}
		// ��ʼ����·��
		if (save_run == 1)
		{
			servo_en = 1;
			left_en = 1;
			right_en = 1;
			if (stop_index > 100)
			{
				if (this_addr > (stop_index - 52))
				{
					save_run = 0;
					save_flag = 0;
					this_addr = 0;
					load_run_flag = 0;
					high_speed_flag = 0;
					middle_speed_flag = 0;
					low_speed_flag = 0;
				}
			}
			this_addr += 4;
		}
		add_pos -= 800.0; // �������㣬���Ҳ������������Ҫ��һ�������ɺ���
	}
}

void Timer3_Init(void) // 1����@52MHz
{
	T4T3M |= 0x02; // ��ʱ��ʱ��1Tģʽ
	T3L = 0xE0;	   // ���ö�ʱ��ʼֵ
	T3H = 0x34;	   // ���ö�ʱ��ʼֵ
	T4T3M |= 0x08; // ��ʱ��3��ʼ��ʱ
	IE2 |= 0x20;   // ʹ�ܶ�ʱ��3�ж�
}

// �����������ȼ�
void Timer4_Isr(void) interrupt 20
{
	u8 i;
	// ���������̶߳�ʱ��
	for (i = 0; i < Task_Max; i++)
	{
		if (Task_Timer[i] > 0)
		{
			Task_Timer[i]--;
		}
	}
}
