#include "driverlib.h"
#include "EVR/evr.h"
#include <stdint.h>

EVR_t evr = {0};
uint16_t adcBufferChannels[2] = {0};
void sendCalibrationConstants();
void ADC_Init(void);
void Init_Clock();

void GetAdcValue(uint16_t *buffer)
{
    // Start ADC conversion sequence at MEMORY_0 and include MEMORY_1
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM0,
                            ADC12_B_SEQOFCHANNELS);

    // Wait until ADC conversion is complete
    uint32_t timeout = 500000; // Define a suitable timeout value (adjust as needed)
    while (ADC12_B_isBusy(ADC12_B_BASE) == ADC12BUSY)
    {
        if (--timeout == 0)
        {
            EVR("Error: ADC conversion timed out.\n\r");
            return;
        }
    }

    // Retrieve results from both memory buffers
    buffer[0] = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
    buffer[1] = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
}

void main (void)
{
    // Stop Watchdog Timer and unlock ports
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

    // Configure P1.3 and P1.4 as ADC input pins
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN4 | GPIO_PIN3,
        GPIO_TERNARY_MODULE_FUNCTION
    );

    // Initialize EVR and ADC
    EVR_Init(&evr);
    ADC_Init();

    while (1) {
        GetAdcValue(adcBufferChannels);
        EVR("ADC Memory0: %d, Memory1: %d\n\r", adcBufferChannels[0], adcBufferChannels[1]);
        __delay_cycles(100000);  // Delay between conversions
    }
}

void ADC_Init(void)
{
    // Initialize the ADC12B Module with software trigger and internal oscillator
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_BATTMAP | ADC12_B_TEMPSENSEMAP;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Configure sampling timer for multiple samples
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_128_CYCLES,
                               ADC12_B_CYCLEHOLD_128_CYCLES,
                               ADC12_B_MULTIPLESAMPLESENABLE);

    // Configure Memory 0 (A4 as input source)
    ADC12_B_configureMemoryParam configureMemory0 = {0};
    configureMemory0.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemory0.inputSourceSelect = ADC12_B_INPUT_A4;
    configureMemory0.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemory0.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemory0.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemory0.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemory0);

    // Configure Memory 1 (Temperature Sensor as input source)
    ADC12_B_configureMemoryParam configureMemory1 = {0};
    configureMemory1.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    configureMemory1.inputSourceSelect = ADC12_B_INPUT_TCMAP;
    configureMemory1.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemory1.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemory1.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemory1.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemory1);

    // Clear interrupt flags for both memory buffers
    ADC12_B_clearInterrupt(ADC12_B_BASE, ADC12_B_MEMORY_0 | ADC12_B_MEMORY_1,
                           ADC12_B_IFG0 | ADC12_B_IFG1);

    // Enable interrupts for both memory buffers
    ADC12_B_enableInterrupt(ADC12_B_BASE, ADC12_B_IE0 | ADC12_B_IE1,
                            ADC12_B_MEMORY_0 | ADC12_B_MEMORY_1, 0);

    // Configure internal reference voltage and enable temperature sensor
    uint32_t timeout = 500000; // Define a timeout value (adjust as needed)
    while(Ref_A_isRefGenBusy(REF_A_BASE))
    {
        if (--timeout == 0)
        {
            EVR("Error: Reference generator is busy. Initialization failed.\n\r");
            return;
        }
    }

    Ref_A_enableTempSensor(REF_A_BASE);
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF2_5V);
    Ref_A_enableReferenceVoltage(REF_A_BASE);
}

/*
 * ADC12 Interrupt Service Routine
 * Exits LPM3 when Temperature/Voltage data is ready
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  switch(__even_in_range(ADC12IV,76))
  {
    case  ADC12IV_NONE: break;                // Vector  0:  No interrupt
    case  ADC12IV_ADC12OVIFG: break;          // Vector  2:  ADC12MEMx Overflow
    case  ADC12IV_ADC12TOVIFG: break;         // Vector  4:  Conversion time overflow
    case  ADC12IV_ADC12HIIFG: break;          // Vector  6:  ADC12HI
    case  ADC12IV_ADC12LOIFG: break;          // Vector  8:  ADC12LO
    case ADC12IV_ADC12INIFG: break;           // Vector 10:  ADC12IN
    case ADC12IV_ADC12IFG0:                   // Vector 12:  ADC12MEM0
        ADC12IFGR0 &= ~ADC12IFG0;             // Clear interrupt flag
        __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case ADC12IV_ADC12IFG1:                   // Vector 14:  ADC12MEM1
        ADC12IFGR0 &= ~ADC12IFG1;             // Clear interrupt flag
        __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case ADC12IV_ADC12IFG2: break;            // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3: break;            // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4: break;            // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5: break;            // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6: break;            // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7: break;            // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8: break;            // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9: break;            // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10: break;           // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11: break;           // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12: break;           // Vector 36:  ADC12MEM12
    case ADC12IV_ADC12IFG13: break;           // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14: break;           // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15: break;           // Vector 42:  ADC12MEM15
    case ADC12IV_ADC12IFG16: break;           // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17: break;           // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18: break;           // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19: break;           // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20: break;           // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21: break;           // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22: break;           // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23: break;           // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24: break;           // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25: break;           // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26: break;           // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27: break;           // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28: break;           // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29: break;           // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30: break;           // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31: break;           // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG: break;          // Vector 76:  ADC12RDY
    default: break;
  }
}
