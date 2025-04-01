#include "./H_Group/pid.h"



// ???
u32 Tick_ms = 0;

void PWM_Transform_Timer_Init(void)
{
    // PWMB???
    PWMB_PSCRH = (u8)(52 >> 8); // 1Mhz1us?
    PWMB_PSCRL = (u8)(52);
    PWMB_ARRH = (u8)(50000 >> 8); // 50ms????
    PWMB_ARRL = (u8)(50000);
    PWMB_CR1 = 0x05; // ??
    PWMB_IER = 0x01; // ?
    PPWMBH = 1;
    PPWMB = 1; // ?
}
void PWMB_Isr(void) interrupt PWMB_VECTOR
{
    PWMB_SR1 = 0x00; // ??
    Tick_ms += 50;   // ÿ50ms
}

u16 Read_Timer_Cnt(void)
{
    u8 cnth, cntl;
    cnth = PWMB_CNTRH;
    cntl = PWMB_CNTRL;
    return (u16)cnth << 8 | (u8)cntl;
}

// 1us???
u32 Micros(void)
{
    return (Read_Timer_Cnt() + (Tick_ms * 1000));
}

void IPID_Ctrl(float ek, IPID *p) // PID??
{
    float d_uk = 0;                 // ?
    if (fabs(p->ki * (ek)) > p->ei) // ?
        d_uk = (p->kp * (ek - p->ek_1)) + (p->ki * (ek)) + (p->kd * (ek - (2 * p->ek_1) + p->ek_2));
    else
        d_uk = (p->kp * (ek - p->ek_1)) + (p->kd * (ek - (2 * p->ek_1) + p->ek_2));
    p->ek_2 = p->ek_1;
    p->ek_1 = ek;
    if (fabs(d_uk) > p->eo) // 
        p->output += d_uk;
    p->output = _constrain(p->output, -(p->limit), (p->limit));
}

u16 code List_X[] = {0, 2, 3, 10, 20, 50};
float code List_Y[] = {1.0, 2.0, 5.0, 15.5, 60.5, 70.5};

int Fast_Seek(u16 *dat, u16 num)
{
    int l = 0;
    int n = sizeof(dat) / sizeof(dat[0]);
    int r = n - 1;
    int mid = 0;
    while (l < r)
    {
        mid = l + r >> 1;
        if (dat[mid] >= num)
        {
            r = mid;
        }
        else
        {
            l = mid + 1;
        }
    }
    return l;
}

// float List_PID_Ctrl(float error)
// {
//     int x = Fast_Seek(List_X, 1);
//     float y = List_Y[x];
//     float y1,out;
//     if (List_X[x] == 1)
//     {
//         y1 = List_Y[x - 1];
//         out =
//     }
// }

float Fast_PID(float error, FPID *p)
{
    float _p, _p2, _d, _d2; // 
    _p = p->kp * error;
    _p2 = p->kp2 * (fabs(error) * error);
    _d = p->kd * (error - p->ek_1);                             // ??
    _d2 = p->kd2 * gy;                                          // ?
    p->output = _p + _p2 + _d + _d2;                            // ??
    p->output = _constrain(p->output, -(p->limit), (p->limit)); // ?
    p->ek_1 = error;
    return p->output;
}

float PID_Ctrl(float error, PID_Register *p)
{
    // ????
    unsigned long int timestamp_now = Micros();
    float Ts = (timestamp_now - p->timestamp_prev) * 1e-6f;
    float proportional;
    float integral;
    float derivative;
    float output;
    float output_rate;
    // P
    proportional = p->P * error;
    derivative = p->D * (p->error_prev - error);

    // P,I,D??
    output = proportional + derivative;
    output = _constrain(output, -(p->limit), p->limit);

    if (p->output_ramp > 0)
    {
        // PID???
        output_rate = (output - p->output_prev) / Ts;
        if (output_rate > p->output_ramp)
            output = p->output_prev + p->output_ramp * Ts;
        else if (output_rate < -(p->output_ramp))
            output = p->output_prev - p->output_ramp * Ts;
    }
    // ????
    p->integral_prev = integral;
    p->output_prev = output;
    p->error_prev = error;
    p->timestamp_prev = timestamp_now;
    return output;
}

void PD_Ctrl(float error, PD_Register *p)
{
    float _out;
    p->i_add+= error;
    if(fabs(error - p->e1)<p->i_th)p->i_add = 0;
    _out = (p->kp * error) + ((error - p->e1) * p->kd)+(p->i_add*p->ki);
    p->output = _constrain(_out, -p->limit, p->limit);
    p->e1 = error;
}

void ADJ_PID_Register(u8 state, ADJ_Register *r)
{
    if (state == 0)
    {
        // µ
        r->max = r->out;
    }
    else
    {
        r->min = r->out;
        // ?
    }
    r->out = (r->max + r->min) / 2;
}

/*
1????II??Z??
I????????????
???out???????????
?????????
*/

void I_Ctrl(float error, I_Register *p)
{
    float _out = 0;
    _out = p->output + (error * p->ki);
    p->output = _constrain(_out, -p->limit, p->limit);
}

/*
2??????PI????P
???I?????????????
???????ý?²?
??????????ôP????
?????????????
???PID????????????
????????
*/

/*
3???????????
??????õ??
???????????????
???PI+?????????
?PI???????´?PI????
*/