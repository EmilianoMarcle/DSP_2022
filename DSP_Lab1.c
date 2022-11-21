/*
 * Archivo:      DSP_Lab1.c
 * Descripción:  Procesamiento Digital de Señales - Laboratorio N°1
 */

/*Include Files*/
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "arm_math.h"


/* Definitions and declarations */
#define 	SR_8K   	7500
#define 	SR_16K		3750
#define 	SR_22K		2727
#define 	SR_44K		1364
#define 	SR_48K		1250
#define 	buffer_size 512

volatile uint16_t ADC_Value;	// Variable que almacena conversion del ADC.
uint16_t DAC_Value;				// Variable que almacena valor a transmitir por DAC.
q15_t buffer[buffer_size];		// Buffer circular.

/* Function Prototypes */
void cargaBuffer(q15_t buffer_data);

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
	if(pin_flags & (1<<BOARD_SW2_GPIO_PIN)){
		switch(sampling_Freq){
		case 0: // 16K/S : BLUE LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, SR_16K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0);
			break;
		case 1: // 22K/S : GREEN LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, SR_22K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);
			break;
		case 2: // 44K/S : YELLOW LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, SR_44K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1);
			break;
		case 3: // 48K/S : VIOLET LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, SR_48K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0);
			break;
		case 4:	// 8K/S : RED LED
			PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, SR_8K);
			PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
			GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1);
			GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0);
			GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0);
		default:
			break;
		}

		sampling_Freq++;
		if (sampling_Freq > 4){
			sampling_Freq = 0;
		}
	}

	/* Limpia bandera de interrupción en SW2 */
	GPIO_PortClearInterruptFlags(GPIOC, (1<<BOARD_SW2_GPIO_PIN));

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
	if(pin_flags & (1<<BOARD_SW3_GPIO_PIN)){
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
	GPIO_PortClearInterruptFlags(GPIOA, (1<<BOARD_SW3_GPIO_PIN));

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
	/* Variable temporal que almacena resultado de la conversion del ADC en formato q15 */
	q15_t ADC_Value_fix;
	/* Inicialización variable que almacena resultado de la conversión del ADC */
	ADC_Value = 0;
	/* Lectura de bandera Done */
	uint16_t status = ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP);
	if ( status == kADC16_ChannelConversionDoneFlag){
		/* Lectura del valor convertido por el ADC */
		ADC_Value = (ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP) & (0xFFFF));
	}
	/* Transformación del valor convertido */
	ADC_Value_fix =  (q15_t) ADC_Value;
	/* Almacena dato en buffer circular en formato q15*/
	cargaBuffer(ADC_Value_fix);
	/* Transformacion del valor para transmitirlo por DAC de 12 bits*/
	DAC_Value = ((ADC_Value >> 4) & (0xFFF)); // Se descartan los 4 bits menos significativos.
  	/* Envia datos al DAC */
	DAC_SetBufferValue(DAC0_PERIPHERAL, 0U, DAC_Value);

  	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  	#if defined __CORTEX_M && (__CORTEX_M == 4U)
    	__DSB();
    #endif
}

/*
 * Función que implementa el buffer circular.
 */
void cargaBuffer(q15_t buffer_data){
	/* Indice que indica la posición del arreglo (direccion del buffer) */
	static uint16_t index_buffer = 0;

	buffer[index_buffer] = buffer_data;
	  	index_buffer++;
	  	if(index_buffer == buffer_size){
	  		index_buffer = 0;
	  	}
	return;
}


