/*
 * Archivo:      DSP_Lab1.c
 * Descripción:  Procesamiento Digital de Señales - Laboratorio N°1
 *
 * Consigna: Realizar un programa aplicativo que sea capaz de digitalizar una señal
 * a través del módulo del ADC disponible en la board FRDM-K64F a distintas velocidades
 * de muestreo, las velocidades requeridas son 8K/S, 16K/S 22K/S, 44K/S y 48K/S.
 * Los cambios de las velocidades de muestreo serán realizados con una de las teclas
 * de la placa de evaluación, en forma de un buffer circular.
 * Cada velocidad de muestreo se indicará a través de un color RGB del LED.
 * Con otra tecla de la placa se habilitara la adquisición o se parara la misma (Run/Stop).
 * Los valores adquiridos serán almacenados en memoria en un buffer circular de 512 muestras
 *  del tipo q15 (fraccional 15bits) y a su vez serán enviados a través del DAC (de 12bits).
 */

/* Include Files */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "arm_math.h"

/* Definitions and declarations */
#define 	ticks_SR_8K  	7500U
#define 	ticks_SR_16K    3750U
#define 	ticks_SR_22K    2727U
#define 	ticks_SR_44K    1364U
#define 	ticks_SR_48K    1250U
#define 	buffer_size     512U
//#define     offset       (uint16_t)     32767

/* Variable que almacena conversion del ADC. */
volatile uint16_t adc_value;
/* Variable que almacena conversion del ADC en formato fraccional */
q15_t adc_value_frac;
/* Buffer circular para almacenar 512 muestras */
q15_t buffer[buffer_size];
/* Variable que almacena valor a enviar por DAC */
uint16_t dac_value;

/* Function Prototypes */
void envioUART(uint16_t);
void DAC_Test(void);

/*
 * Main program.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitButtonsPins();
    BOARD_InitLEDsPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    /* Inicialización en 8K/S - LED ROJO */
    GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
    GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
    GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    while(1) {
    }
    return 0 ;
}

/*
 * Switch SW2 Interrupt Handler
 * Cambia frecuencia del Timer en cada presión del pulsador
 */
void BOARD_SW2_IRQ_HANDLER(void){

	/* Variable de control para cambiar frecuencia de muestreo */
	uint8_t static sampling_Freq = 0;
	/* Lectura de banderas de interrupción por GPIOC */
	uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOC);

	/* Verificación del pin que produjo la interrupción */
	if(pin_flags & (1U << BOARD_SW2_GPIO_PIN)){
		switch(sampling_Freq){
		case 0: // 16K/S : BLUE LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, ticks_SR_16K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);
			break;
		case 1: // 22K/S : GREEN LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, ticks_SR_22K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0);
			break;
		case 2: // 44K/S : YELLOW LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, ticks_SR_44K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0);
			break;
		case 3: // 48K/S : VIOLET LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, ticks_SR_48K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);
			break;
		case 4:	// 8K/S : RED LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, ticks_SR_8K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);
		default:
			break;
		}

		sampling_Freq++;
		if (sampling_Freq > 4){
			sampling_Freq = 0;
		}
	}

	/* Limpia bandera de interrupción en SW2 */
	GPIO_PortClearInterruptFlags(GPIOC, (1U << BOARD_SW2_GPIO_PIN));

	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
	#endif

	return;
}

/*
 * Switch SW3 Interrupt Handler
 * Habilita/Deshabilita la adquisición de datos disparando/deteniendo el Timer
 */
void BOARD_SW3_IRQ_HANDLER(void){

	/* Bandera para control del Timer */
	static uint8_t startFlag = 0;
	/* Lectura de banderas de interrupción por GPIOA */
	uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOA);

	/* Verificación del pin que produjo la interrupción */
	if(pin_flags & (1U << BOARD_SW3_GPIO_PIN)){
		if(startFlag == 0){
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);	// Run
			EnableIRQ(PIT_CHANNEL_0_IRQN);
			startFlag = 1;
		}
		else {
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);	// Stop
			DisableIRQ(PIT_CHANNEL_0_IRQN);
			startFlag = 0;
		}
	}
	/* Limpia bandera de interrupción en SW3 */
	GPIO_PortClearInterruptFlags(GPIOA, (1U << BOARD_SW3_GPIO_PIN));

	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
	#endif

	return;
}

/*
 *  PIT CHANNEL 0 Interrupt Handler
 */
void PIT_CHANNEL_0_IRQHANDLER(void)
{
	uint32_t intStatus = 0;
	/* Lectura de banderas de interrupción del registro Status */
	intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
	/* Limpia banderas de interrupción por PIT */
    PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

    /* Inicialización de la conversion del ADC */
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP, &ADC0_channelsConfig[0]);

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
       Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
	#endif

    return;
}

/*
 * 	ADC0 Interrupt Handler
 */
void ADC0_IRQHANDLER(void) {
	/* Inicializacion variable que almacena la conversión del ADC */
	adc_value = 0;
	/* Indice que indica la posición del arreglo (dirección del buffer) */
	static uint16_t index_buffer = 0;

	/* Lectura de bandera Done */
	uint16_t status = ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP);
	if ( status == kADC16_ChannelConversionDoneFlag){
		/* Lectura del valor convertido por el ADC */
		adc_value = (ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP)) & 0xFFFF;
		/* Transformación del valor convertido por el ADC de 16 bits en q15_t */
		adc_value_frac =  (q15_t) adc_value;
		/* Almacena dato en buffer circular de 512 muestras */
		buffer[index_buffer] = adc_value_frac;
		index_buffer++;
		if(index_buffer == buffer_size){
			index_buffer = 0;
		}
		/* Transformación del valor de 16 bits para transmitirlo por DAC de 12 bits.
		 * Se descartan los 4 LSB */
	        dac_value = (adc_value >> 4) & 0x0FFF ;
		/* Envía datos al DAC */
		DAC_SetBufferValue(DAC0_PERIPHERAL, 0U, dac_value);
		/* --- Serial Oscilloscope --- */
		PRINTF("%d\r\n", adc_value);

		/* --------- Lab View ---------*/
		/* Envia por UART valor adquirido por ADC */
		envioUART(adc_value);
	}

  	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  	#if defined __CORTEX_M && (__CORTEX_M == 4U)
    	__DSB();
    #endif

    return;
}

/*
 *  Función que transmite el dato adquirido por UART.
 */
void envioUART(uint16_t ADC_Value){
	uint8_t data = 0;
	// Transmite parte alta del byte
	data = (uint8_t) ((ADC_Value >> 8) & (0x00FF));
	UART_WriteByte(UART0_PERIPHERAL, data);
	// Transmite parte baja del byte
	data = (uint8_t) (ADC_Value & (0x00FF));
	UART_WriteByte(UART0_PERIPHERAL, data);
	return;
}
