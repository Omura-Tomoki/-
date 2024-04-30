/******************************************************************************/
/*                ライン走行                                                  */
/******************************************************************************/

 

#include <xc.h>

 

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (Power-up Timer is enabled)
#pragma config MCLRE = ON
#pragma config BOREN = OFF
#pragma config CP = OFF         // Code Protection bit (Code protection disabled)
#pragma config LVP = OFF        // 低電圧プログラミング機能使用しない   added for 648A !! (Low-voltage programming disabled))
#pragma config CPD = OFF

 

//*** グローバル定数の定義 ***********************

 

#define _XTAL_FREQ  10000000
#define INTV_20ms 195     //19.968ms (int clock 102.4micro * 195)

 

//*** 定義  **************************************

 

// PWM 大きいほどスピードが大
#define PWM_CYCLE  30

 

#define SPD_H0   22           // --- 直進 --------------------
#define SPD_H1   20           // --- 補正１ -外輪のスピード --
#define SPD_L1   16         //          内輪のスピード
#define SPD_H2   20           // --- 補正２ -外輪のスピード --
#define SPD_L2   12          //          内輪のスピード
#define SPIN     13
#define right    0
#define left     1

 

#define STOP      0           // ------- 停止 ------------

 

#define ON  1  // オン
#define OFF 0 // オフ
#define CON 2 // 継続

 

//void interrupt IntTMR0(void);
static volatile unsigned char  Intv_count;   // 20msカウント用

 

// モータ用
unsigned char PWM_count;     //  PWM周期のカウント
static volatile unsigned char Flag_20ms;        // 20msec フラグ
unsigned char Sensor;        // センサ読取り値(Bit2:左 Bit1：中 Bit0:右)
unsigned char M_direction;   // 車輪の前進・後進
unsigned char M_action;      // モータの正・逆・ブレーキ
unsigned char R_duty ;       // 右モータのデューティ
unsigned char L_duty ;       // 左モータのデューティ
unsigned int roll;
unsigned int star=0;
// void interrupt IntTMR0(void);
void PWM(void);
void set_direction(char R, char L);
void onoff_motor(char R, char L);
void read_sensor(void);
void drive_motor(void);
void trace(void);
void turn_corner(void);
/*********   タイマ0割込み処理  *******/
void interrupt IntTMR0(void)
{       
    T0IE=0;              //  タイマ割り込み禁止    
    T0IF=0 ;             // タイマ0割込みフラグリセット
    TMR0=0xFF;

    PWM();

 

    if(++Intv_count==INTV_20ms) {
        Flag_20ms=1;
        Intv_count=0;
    }

    T0IE=1;                 //  タイマ割り込み可    
}

 

/************* main  **************/
int main(void)
{
    INTCON=0;
    PCLATH=0;
    CMCON=0x07;    // added for 648A
    TRISA=0x10;
    TRISB=0x19;
    OPTION_REG=0x07;  
    PORTA=0x0F;
    PORTB=0xE0;   // all LED ON

 

    TMR0=0xFF;  

    Intv_count=0;  // 20msec counter
    Flag_20ms=0;  // 20msec flag
    PWM_count=0;

    set_direction('F', 'F');

    while(RA4) {     // wait until SW2 is pushed (loop while RA4==1)
        read_sensor();
    }
    L_duty=SPD_H0;
    R_duty=SPD_H0;
    T0IE=1;
    GIE=1;  

    while(1){ 
        if(Flag_20ms==1) {
            T0IE=0;
            Flag_20ms=0;
            read_sensor();
            trace(); 
            T0IE=1;
        }
    }
}

 

/**************  PWM制御：デュティ比でモータをON, OFF ****************/
void PWM()
{   
    ++PWM_count;

    if(PWM_count==PWM_CYCLE){
        PWM_count=0;
        onoff_motor(ON, ON);   // M_actionにセット
    }

 

    if(PWM_count==R_duty)  
        onoff_motor(CON, OFF);
    if(PWM_count==L_duty)
        onoff_motor(OFF, CON); 

    drive_motor();    // M_action-> PORTA
    return;
}

 


/********** 前進・後退の指定 -> M_direction *************************/
void set_direction( char L, char R)
{
     if(L=='F') {
        M_direction=(M_direction & 0x03) | 0x04;  // ****  01**
    }
    else if(L=='B') 
        M_direction=(M_direction & 0x03) | 0x08;  // ****  10**

    if(R=='F') {
        M_direction=(M_direction & 0x0C) | 0x01;  // ****  **01
    }
    else if(R=='B') 
        M_direction=(M_direction & 0x0C) | 0x02;  // ****  **10

    return;
}

 

/************ (モータのオン・オフ + M_direction))-> M_action  ***********/
void onoff_motor(char L, char R)
{
//   L, Rごとに，指定された動作を，M_actionに設定
    // ON:   M_directionで指定された回転（前進 or 後退）-> M_action 
    // OFF:  ブレーキ
    // CON:  そのまま

    switch(L) {
        case ON: 
            M_action=(M_action & 0x03) | (M_direction & 0x0C); // M_direction の3, 2ビット
            break;
        case OFF:
            M_action=(M_action & 0x03) | 0x0C; // M_action = ****  11**
            break;
        case CON:
            break;
    }

    switch(R) {
        case ON: 
            M_action=(M_action & 0x0C) | (M_direction & 0x03); // M_direction の下位2ビット
            break;
        case OFF:
            M_action=(M_action & 0x0C) | 0x03; // M_direction = ****  **11
            break;
        case CON:
            break;
    }
    return;
}

 

/**********   M_action-> PORTAへ出力  *****************/
void drive_motor()
{
    unsigned char porta;

    porta=(PORTA & 0xF0);      //下位4ビットのクリア
    PORTA=(M_action | porta);  // 下位4ビットの書換え                 
    __delay_us(1);  // nop相当

}

 

/*************  センサ読み込み，LEDの点灯 ********************/
void read_sensor(void)  
{
    unsigned char buf=0;
    unsigned char sensor=0;

 

    if(RB4){                 // left sensor
        buf=(buf | 0x80);    // 
        sensor=(sensor|0x04);
    }
    if(RB3){                 // center sensor
        buf=(buf | 0x40);    //
        sensor=(sensor|0x02);
    }
    if(RB0){                 // right sensor          
        buf=(buf | 0x20);    //
        sensor=(sensor|0x01);
    }

    buf=(buf | (PORTB & 0x1F));
    PORTB=buf;                // LED
    __delay_us(1);
    Sensor=sensor;
    return;
}

 

/************  センサの値によりデュティ値を設定  *******************/
void trace() 
{
    switch(Sensor){
        case 0:         // 〇〇〇 反応なし
            //L_duty=STOP; 
            //R_duty=STOP;
            turn_corner();
            break;
        case 1:         // 〇〇● 右反応
            L_duty=SPD_H2;
            R_duty=SPD_L2;
            break;
        case 2:         // 〇●〇 中反応
            L_duty=SPD_H0;
            R_duty=SPD_H0;
            break; 
        case 3:         // 〇●● 中，右反応
            L_duty=SPD_H1; 
            R_duty=SPD_L1;
            break;
        case 4:         // ●〇〇 左反応
            L_duty=SPD_L2;
            R_duty=SPD_H2;
            break;  
        case 5:         // ●〇● 左，右反応
            break;    
        case 6:         // ●●〇 左，中反応
            L_duty=SPD_L1;
            R_duty=SPD_H1;
            break;   
        case 7:         // ●●●
//            L_duty=STOP;
//            R_duty=STOP;
            break;
    }
}

 

void turn_corner()
{
    T0IE=0;  //  タイマ割り込み禁止
    if(star==0){
        set_direction('B','B');//後退
        while(!(Sensor & 0x06) &&!(Sensor & 0x03)){//左右どちらかのセンサが光る
            PWM();
            __delay_us(100);
            read_sensor();
        }
        if(Sensor & 0x03) roll=right;
        else if(Sensor & 0x06) roll=left;


        set_direction('F','F');//左右ともに前進
        L_duty=SPD_H2;
        R_duty=SPD_H2;
        for(int i=0;i<1500;i++){
            PWM();
            __delay_us(100);
            read_sensor();
        }
        star=1;
    }
    else if(star==1){
        if(roll==right) roll=left;
        else if(roll==left) roll=right;
        set_direction('F','F');//左右ともに前進
        L_duty=SPD_H2;
        R_duty=SPD_H2;
        for(int i=0;i<1500;i++){
            PWM();
            __delay_us(100);
            read_sensor();
        }
        star=0;
    }
    if(roll==left) set_direction('B','F');
    if(roll==right) set_direction('F','B');
    L_duty=SPIN;
    R_duty=SPIN;
    while(1){
        PWM();
        __delay_us(100);
        read_sensor();
        if(Sensor & 0x02){
            break;
        }
    }

        set_direction('F','F');
    T0IE=1;                 //  タイマ割り込み可    
    return;
}