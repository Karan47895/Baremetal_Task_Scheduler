#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


#include <stdio.h>
#include "stdint.h"
#include "main.h"
#include "stm32f407xx.h"
#include "string.h"
#include "stm32f407xx_ADC_driver.h"
#include "systick.h"
#include "timer.h"


#define DHT_PORT GPIOC
#define DHT_PIN GPIO_PIN_4


void GPIO_Initt(void);


void GPIO_Initt() {
    // Configuring PD5 for generating pulse sent to the trig pin
    RCC->AHB1ENR |= 1 << 3; // Enable GPIOD clock (bit 3 corresponds to GPIOD)
    GPIOD->MODER |= 1 << 10; // Set the PD5 pin to output mode (01 for output)

    // Configuring PD6 as input mode for the echo pin
    GPIOD->MODER &= ~(0x00003000); // Set PD6 to input mode (00 for input)
}



uint32_t data;
double time, dist;



void Timer3_Init(void) {
    // Enable clock for Timer 3
    TIMER_PeriClockControl(TIM3, ENABLE);

    // Configure Timer 3
    TIM3->PSC = 16 - 1;  // Prescaler for 1us tick (with 16MHz clock)
    TIM3->ARR = 0xFFFFFFFF;  // Max auto-reload value
    TIMER_Enable(TIM3, ENABLE);
}


void delay(uint32_t value);


void delay(uint32_t delay) {
    TIM3->CNT = 0;  // Reset counter
    while (TIM3->CNT < delay);  // Wait until the delay is reached
}


void Set_Pin_Input() {
    GPIO_PeriClockControl(GPIOC, DISABLE);
    GPIO_Handle_t out;
    out.pGPIOX = GPIOC;
    out.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    out.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    out.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;
    out.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    out.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&out);
}


void Set_Pin_Output() {
    GPIO_PeriClockControl(GPIOC, DISABLE);
    GPIO_Handle_t out;
    out.pGPIOX = GPIOC;
    out.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    out.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    out.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;
    out.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    out.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&out);
}


void DHT11_Start (void)
{
    Set_Pin_Output ();  // set the pin as output
    GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_4, 0); // pull the pin low
    delay(18000);  // wait for 18ms
    GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_4, 1); // pull the pin high
    delay(20);   // wait for 20us
    Set_Pin_Input();    // set as input
}


uint8_t DHT11_Check_Response (void)
{
    uint8_t Response = 0;
    delay(40);
    if (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)))
    {
        delay(80);
        if ((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4))) Response = 1;
        else Response = -1; // 255
    }
    while ((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)));   // wait for the pin to go low

    return Response;
}


uint8_t DHT11_Read (void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        while (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)));   // wait for the pin to go high
        delay(40);  // wait for 40 us
        if (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)))   // if the pin is low
        {
            i &= ~(1 << (7 - j)); // write 0
        }
        else i |= (1 << (7 - j)); // if the pin is high, write 1
        while ((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)));  // wait for the pin to go low
    }
    return i;
}


USART_Handle_t usart2_handle;


uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;





void ADC1_GPIO_INIT();
void ADC1_INIT();

TIMER_handle_t handlerr;


ADC_Handle_t handler;
ADC_Handle_t handler1;



void ADC1_GPIO_INIT() {

    GPIO_Handle_t adc1;

    adc1.pGPIOX = GPIOA;
    adc1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    adc1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;

    GPIO_Init(&adc1);
}



void ADC1_INIT() {

    handler.pADCx = ADC1;
    handler.ADCConfig.ADC_Channel = 1;
    handler.ADCConfig.ADC_DataAlign = ADC_DataAlign_RIGHT;
    handler.ADCConfig.ADC_SampleTime = ADC_SampleTime_3_Cycle;

    ADC_Init_SingleChannel(&handler);
}



void ADC2_GPIO_INIT() {
    GPIO_Handle_t adc2_gpio;

    // Configure PA5 (D5 on the Arduino pinout) as analog mode for ADC2
    adc2_gpio.pGPIOX = GPIOA;
    adc2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;  // PA5 (Pin D5)
    adc2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG; // Set to analog mode

    GPIO_Init(&adc2_gpio);
}


void ADC2_INIT() {

    // Configure ADC2 for PA5 (Channel 5)
    handler1.pADCx = ADC2;
    handler1.ADCConfig.ADC_Channel = 1;                           // Channel 5 (corresponding to PA5)
    handler1.ADCConfig.ADC_DataAlign = ADC_DataAlign_RIGHT;        // Right-aligned data
    handler1.ADCConfig.ADC_SampleTime = ADC_SampleTime_3_Cycle;    // Set sample time (adjust if necessary)

    ADC_Init_SingleChannel(&handler1);
}


uint32_t result;



USART_Handle_t usart2_handle;

void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOX = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    //USART2 TX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    //USART2 RX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);


}

void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void idle_task(void);


void init_systick_timer(uint32_t tick_hz);
__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
void init_tasks_stack(void);
void enable_processor_faults(void);
__attribute__((naked)) void switch_sp_to_psp(void);
uint32_t get_psp_value(void);

void task_delay(uint32_t tick_count);


/* This variable tracks the current_task being executed on the CPU */
uint8_t current_task = 1; //

typedef struct {
    uint32_t psp_value;
    uint32_t block_count;
    uint8_t current_state;
    void (*task_handler)(void);
} TCB_t;

TCB_t user_tasks[MAX_TASKS];

int main(void)
{

    GPIO_Initt();
    GPIOD->BSRR = 0x00000000; // Setting trig pin to low to initialize the module

    /* Loop forever */
    USART2_GPIOInit();

    USART2_Init();

    Timer3_Init();

    USART_PeripheralControl(USART2, ENABLE);

    enable_processor_faults();

    init_scheduler_stack(SCHED_STACK_START);

    init_tasks_stack();

    init_systick_timer(TICK_HZ);

    switch_sp_to_psp();

    task1_handler();

    for (;;);
}


int cnt = 0;
char str[20];
char newline[] = "\n";
void idle_task(void) {
    while (1) {

    }
}


void task1_handler(void) {
    ADC1_GPIO_INIT();
    ADC1_INIT();
    ADC_PeripheralEnable(ADC1, 1);
    while (1) {
        INTERRUPT_DISABLE();
        ADC_StartConversion(&handler);
        result = ADC_ReadConversion(&handler);
        printf("result is %d\n", (int) result);
        sprintf(str, "%ld", result);
        strcat(str, newline);
        USART_SendData(&usart2_handle, (uint8_t*)str, strlen(str));
        INTERRUPT_ENABLE();
        task_delay(1000);
    }
}


void task2_handler(void) {
    ADC2_GPIO_INIT();
    ADC2_INIT();
    ADC_PeripheralEnable(ADC2, 1);
    while (1) {
        INTERRUPT_DISABLE();
        ADC_StartConversion(&handler1);
        result = ADC_ReadConversion(&handler);
        printf("result1 is %d\n", (int) result);
        sprintf(str, "%ld", result);
        strcat(str, newline);
        USART_SendData(&usart2_handle, (uint8_t*)str, strlen(str));
        INTERRUPT_ENABLE();
        task_delay(1000);
    }

}




void task3_handler(void) {

    while (1) {
        INTERRUPT_DISABLE();
        printf("Temp=%.1f\n", Temperature);
        printf("Humidity=%.1f\n", Humidity);

        DHT11_Start();
        Presence = DHT11_Check_Response();
        Rh_byte1 = DHT11_Read ();
        Rh_byte2 = DHT11_Read ();
        Temp_byte1 = DHT11_Read ();
        Temp_byte2 = DHT11_Read ();
        SUM = DHT11_Read();

        TEMP = Temp_byte1;
        RH = Rh_byte1;

        Temperature = (float)(((Temp_byte1 & 0x7F) << 8 | Temp_byte2) / (10.0));
        Humidity = (float)(((Rh_byte1 << 8) | Rh_byte2) / (10.0));
        sprintf(str, "%f", Temperature);
        strcat(str, newline);
        USART_SendData(&usart2_handle, (uint8_t*)str, strlen(str));
        sprintf(str, "%f", Humidity);
        strcat(str, newline);
        USART_SendData(&usart2_handle, (uint8_t*)str, strlen(str));
        INTERRUPT_ENABLE();
        task_delay(1000);
    }
}

void Delay(uint32_t delay) {
    RCC->APB1ENR |= 1; // Enable clock for TIM2
    TIM2->ARR = (int)(delay / 0.0625); // Set auto-reload value for desired delay
    TIM2->CNT = 0; // Reset the counter
    TIM2->CR1 |= 1; // Start the timer
    while (!(TIM2->SR & (1 << 0))) {} // Wait for update interrupt flag
    TIM2->SR &= ~(0x0001); // Clear the update interrupt flag
}

void task4_handler(void)
{
    while (1) {
        INTERRUPT_DISABLE();
        // 1. Sending 10us pulse to trig pin (PD5)
        GPIOD->BSRR &= 0x00000000; // PD5 is low
        Delay(2);
        GPIOD->BSRR |= 0x00000020; // PD5 set to High (bit 5)
        Delay(10); // Wait for 10us
        GPIOD->BSRR |= 0x00200000; // Make PD5 low again (reset bit 5)

        // 2. Measure the pulse width of the pulse sent from the echo pin (PD6) by polling IDR for port D
        while (GPIOD->IDR & (1 << 6)) { // Check PD6 (bit 6)
            data++;
        }

        // 3. Converting the gathered data into distance in cm
        if (data > 0) {
            time = data * (0.0625 * 0.000001); // Convert timer ticks to time in seconds
            dist = ((time * 340) / 2) * 100; // Calculate distance in cm
            //printf("distance is %f\n",dist);
        }
        Delay(4);
        data = 0; // Reset the data for the next measurement
        sprintf(str, "%f", dist);
        strcat(str, newline);
        USART_SendData(&usart2_handle, (uint8_t*)str, strlen(str));
        INTERRUPT_ENABLE();
        task_delay(1000);
    }
}


uint32_t g_tick_count = 0;

void schedule(void) {
    // pend the pendsv exception.
    uint32_t *pICSR = (uint32_t*)0xE000ED04;
    *pICSR |= (1 << 28);

}
void task_delay(uint32_t tick_count) {
    INTERRUPT_DISABLE();
    if (current_task) {
        user_tasks[current_task].block_count = g_tick_count + tick_count;
        user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
        schedule();
    }
    INTERRUPT_ENABLE();
}

void init_systick_timer(uint32_t tick_hz) {
    uint32_t *pSRVR = (uint32_t*)0xE000E014;
    uint32_t *pSCSR = (uint32_t*)0xE000E010;
    uint32_t count_value = (SYSTICK_TIM_CLK / tick_hz) - 1;
    // clear the value of svr
    *pSRVR &= ~(0x00FFFFFFFF);
    // load the value into SVR
    *pSRVR |= count_value;
    // do some settings
    *pSCSR |= (1 << 1); // enavles systick exception request.
    *pSCSR |= (1 << 2); // indicates the clock source,processor clock source.

    // enable the systick.
    *pSCSR |= (1 << 0);
}


__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack) {

    __asm volatile("MSR MSP,%0": :"r"(sched_top_of_stack) : );
    __asm volatile ("BX LR");
}



void init_tasks_stack(void)
{
    user_tasks[0].current_state = TASK_READY_STATE;
    user_tasks[1].current_state = TASK_READY_STATE;
    user_tasks[2].current_state = TASK_READY_STATE;
    user_tasks[3].current_state = TASK_READY_STATE;
    user_tasks[4].current_state = TASK_READY_STATE;

    user_tasks[0].psp_value = IDLE_STACK_START;
    user_tasks[1].psp_value = T1_STACK_START;
    user_tasks[2].psp_value = T2_STACK_START;
    user_tasks[3].psp_value = T3_STACK_START;
    user_tasks[4].psp_value = T4_STACK_START;


    user_tasks[0].task_handler = idle_task;
    user_tasks[1].task_handler = task1_handler;
    user_tasks[2].task_handler = task2_handler;
    user_tasks[3].task_handler = task3_handler;
    user_tasks[4].task_handler = task4_handler;
    uint32_t *pPSP;

    for (int i = 0; i < MAX_TASKS; i++) {
        pPSP = (uint32_t*)user_tasks[i].psp_value;

        pPSP--;
        *pPSP = DUMMY_XPSR; ///0x01000000

        pPSP--;//PC
        *pPSP = (uint32_t)user_tasks[i].task_handler;

        pPSP--;///LR
        *pPSP = 0xFFFFFFFD;

        for (int j = 0; j < 13; j++) {
            pPSP--;
            *pPSP = 0;
        }
        user_tasks[i].psp_value = (uint32_t)pPSP;
    }
}



void enable_processor_faults() {
    // 1)Enable all the configurable exceptions like usage fault,mem manage fault and bus fault
    uint32_t *pSHCSR = (uint32_t*)0xE000ED24;

    *pSHCSR |= (1 << 16); // mem manage
    *pSHCSR |= (1 << 17); // bus fault
    *pSHCSR |= (1 << 18); //usage fault

}


uint32_t get_psp_value(void) {
    return user_tasks[current_task].psp_value;
}

void save_psp_value(uint32_t current_psp_value) {
    user_tasks[current_task].psp_value = current_psp_value;
}

void update_next_task(void) {
    int state = TASK_BLOCKED_STATE;

    for (int i = 0 ; i < (MAX_TASKS) ; i++)
    {
        current_task++;
        current_task %= MAX_TASKS;
        state = user_tasks[current_task].current_state;
        if ( (state == TASK_READY_STATE) && (current_task != 0) )
            break;
    }

    if (state != TASK_READY_STATE)
        current_task = 0;
}

__attribute__((naked)) void switch_sp_to_psp(void) {
    //1)initialize the psp with task1 stack start.
    //get the value of psp of current tasks.
    __asm volatile("PUSH {LR}");//preserve LR to connect back to main.
    __asm volatile ("BL get_psp_value");
    __asm volatile ("MSR PSP,R0");//initialize psp
    __asm volatile ("POP {LR}");
    //2)change sp to psp using control register
    __asm volatile ("MOV R0,#0X02");
    __asm volatile ("MSR CONTROL,R0");
    __asm volatile ("BX LR");
}

void update_global_tick_count(void) {
    g_tick_count++;
}


void unblock_tasks() {
    for (int i = 0; i < MAX_TASKS; i++) {
        if (user_tasks[i].current_state != TASK_READY_STATE) {
            if (user_tasks[i].block_count == g_tick_count) {
                user_tasks[i].current_state = TASK_READY_STATE;
            }
        }
    }
}

void SysTick_Handler(void) {
    uint32_t *pICSR = (uint32_t*)0xE000ED04;
    update_global_tick_count();
    unblock_tasks();
    *pICSR |= (1 << 28);
}

__attribute__((naked))void PendSV_Handler() {
    // save the context of current task.
    //1)Get the PSP value of current task.
    __asm volatile("MRS R0,PSP");
    //2) using that psp value store SF2(r4 to r11)
    __asm volatile("STMDB R0!,{R4-R11}");
    //3)save the current value of PSP
    __asm volatile("PUSH {LR}");


    __asm volatile("BL save_psp_value");

    // retrieve the context of next task
    //1) Decide the next task to run.
    __asm volatile("BL update_next_task");
    //2)get its past PSP value
    __asm volatile("BL get_psp_value");
    //3)Using the PSP value retrieve SF2(R4 TO R11)
    __asm volatile ("LDMIA R0!,{R4-R11}");
    //4)update PSP and exit.
    __asm volatile("MSR PSP,R0");

    __asm volatile("POP {LR}");

    __asm volatile("BX LR");
}
//2. implement the fault handlers
void HardFault_Handler(void)
{
    printf("Exception : Hardfault\n");
    while (1);
}


void MemManage_Handler(void)
{
    printf("Exception : MemManage\n");
    while (1);
}

void BusFault_Handler(void)
{
    printf("Exception : BusFault\n");
    while (1);
}
