/**************************************************************************************

<Hw5.c >
<Semaphores, Task Synchronizing and Interrupt Management>
<By Yamini Yamini>

Hw5 indicates the concept of the basic task synchronizing techniques along with interrupts and semaphores. The assignment also focuses on proving code correctness in the time domain by using discrete signals and an oscilloscope. 
The assignment has two parts. Part 1 uses counting semaphore technique to capture button cycles. Part 2 focuses on measuring kernel performance and analyzing performance challenges with other debugging techniques.

Scope:
1) Modified the code from example 14 to meet the requirements of the assignment.
2) A button handle task is blocked by a semaphore and becomes unblocked after the button (RD6) ISR has been pressed and released.
3) The periodic task in the example was modified.
4) Task managememnt of Kernel was used to measure the time taken to output the print statement in the periodic task.

Test Plan
Modified the code that used polling technique with tasks using semaphore
Question 1) Does the output of the measured counts (printed) match your counts? 
Answer: Yes, the output of the measured counts matched my count.
Part B:
A) It takes 189ms to measure the time it takes output the print statement in periodic task.
B) The period rate of the periodic task is 190ms.
C) The task excutes once.
D) A context switch could take about few 100 nanoseconds to few microseconds depending upon the CPU architecture and the size of the context that is to be saved and restored.
E) The kernel takes about the same time as one periodic task cycle time to take or give a semaphore.

/* MPLAB includes. */
#include <p32xxxx.h>
#include <plib.h>

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes. */
#include "basic_io.h"

/* PIC32 configuration of harware and debug */
#pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2
#pragma config FWDTEN = OFF, FPBDIV = DIV_2, POSCMOD = XT, FNOSC = PRIPLL, CP = OFF
#pragma config FSRSSEL = PRIORITY_7

/* Software interrupt bit used within the CAUSE register to generate an 
interrupt. */
#define mainSW1_CAUSE_BIT            ( 0x01UL << 9 )

/* The software interrupt bit within the Interrupt Flag Status Register. */
#define mainSW1_INT_BIT                ( 0x04UL )

/* Macro to force a software interrupt. */
#define mainTRIGGER_INTERRUPT()                 \
{                                               \
unsigned long ulStatus;                         \
                                                \
    /* Trigger software interrupt. */           \
    ulStatus = _CP0_GET_CAUSE();                \
    ulStatus |= mainSW1_CAUSE_BIT;              \
    _CP0_SET_CAUSE( ulStatus );                 \
}

/* Macro to clear the same software interrupt. */
#define mainCLEAR_INTERRUPT()                   \
{                                               \
unsigned long ulStatus;                         \
                                                \
    /* Trigger software interrupt. */           \
    ulStatus = _CP0_GET_CAUSE();                \
    ulStatus &= ~mainSW1_CAUSE_BIT;             \
    _CP0_SET_CAUSE( ulStatus );                 \
}

/* The tasks to be created for creating integer and print statement */
static void vIntegerGenerator( void *pvParameters );
static void vnewStringPrinter( void *pvParameters );

/* The service routine for the interrupt. The C compiler extensions are used to
indicate that this is an interrupt entry point to use for the 
_CORE_SOFTWARE_1_VECTOR vector location.  The assembly wrapper for the 
interrupt (vSW1_ISR_Wrapper()) is defined in ISR_Wrapper.S, the C portion of
the handler (vSW1_ISR_Handler()) is defined in this file. */
void __attribute__( (interrupt(ipl1), vector(_CORE_SOFTWARE_1_VECTOR))) vSW1_ISR_Wrapper( void );

/* Basic hardware and debug interface configuration. */
void vSetupEnvironment( void );

/*-----------------------------------------------------------*/

unsigned long ulNext = 0;
unsigned long ulCount;
unsigned long ul[ 100 ];

/* Declare two variables of type xQueueHandle.  One queue will be read from
within an ISR, the other will be written to from within an ISR. */
xQueueHandle xIntegerQueue, xStringQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure both the hardware and the debug interface. */
	xMutex = xSemaphoreCreateMutex();
	srand(567);

    /* Creating two queues. One queue can hold variables of type unsigned long, 
	the other queue can hold variables of type char*.  Both queues can hold a 
	maximum of 10 items. It also checks if the queue has been created succesfully */
    xIntegerQueue = xQueueCreate( 10, sizeof( unsigned long ) );
	xStringQueue = xQueueCreate( 10, sizeof( char * ) );
	//checks if the queue is empty
	if (xMutex != NULL)
	{

		/* Create the task that uses a queue to pass integers to the interrupt service
		routine.  The task is created at priority 1. It generates an integer value  */
		xTaskCreate(vIntegerGenerator, "IntGen", 240, NULL, 1, NULL);

		/* Create the task that prints out the strings sent to it from the interrupt
		service routine.  This task is created at the higher priority of 2. It generates a string */
		xTaskCreate(vnewStringPrinter, "String", 240, NULL, 2, NULL);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}
		
    // Insuffient memory for the created resource. Returns a null value.
	for( ;; );
	return 0;
}
/*-----------------------------------------------------------*/

static void vIntegerGenerator( void *pvParameters )
{
portTickType xLastExecutionTime;
unsigned portLONG ulValueToSend = 0;
char *pcStringtoPrint;
/* Two instanes of the task are created so it can be sent to vnewStringPrinter() parameter. It is also cast to the required type.*/
ticks = millisec / portTICK_t;
/* This is a periodic task.  Block until it is time to run again.
The task will execute every 200ms. */

xLastExecutionTime = xTaskGetTickCount();

xLastExecutionTime = xTaskGetTickCount();
/* Initialize the variable used by the call to vTaskDelayUntil(). */
vTaskDelayUntil(&xLastExecutionTime, 200 / portTICK_RATE_MS);
	
	if (xSemaphore != NULL)//checks if the semaphore is null
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.	
		if (xSemaphoreTake(xSemaphore, (portTickType_t)10) == pdTRUE)
		{
			
			xSemaphoreGive(xSemaphore);
		}
		else
		{
			*Force an interrupt so the interrupt service routine can read the
				values from the queue. * /
				vPrintString("Current Total Button Pushes are\n" accButCycles);
			mainTRIGGER_INTERRUPT();
			vPrintString("Generator task - Interrupt generated.\n\n");

		}
	}


	}

/*-----------------------------------------------------------*/

static void vnewStringPrinter( const char *newString )
{
	/*The mutex is created before the scheduler is started so it starts executing by the time the task is first created. The code checks if the semaphore function returns pdTRUE before accessing the shared resource*/
	xSemaphoreTask(xMutex, portMAX_DELAY);
	{
		//Only one task can have the mutex at one time so only one line will execute at one time.
		printf("%s", newString); 
		printf("%s", accButCycles);
		fflush(stdout);
	}
	xSemaphoreTask(xMutex);//The mutex should given back
}
/*-----------------------------------------------------------*/

void vSW1_ISR_Handler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static unsigned long ulReceivedNumber;

/* The strings are declared static const to ensure they are not allocated to the
interrupt service routine stack, and exist even when the interrupt service routine
is not executing. */
static const char *pcStrings[] 
{
    "String 0\n",
    "String 1\n",
    "String 2\n",
    "String 3\n"
};

    /* Loop keeps running until the queue becomes empty. */
    while( xQueueReceiveFromISR( xIntegerQueue, &ulReceivedNumber, &xHigherPriorityTaskWoken ) != errQUEUE_EMPTY )
    {
        /* Shortened the received value to the last two bits (values 0 to 3 inc.), then
        send the string that corresponds to the smaller value to the other
        queue. */
        ulReceivedNumber &= 0x03;
        xQueueSendToBackFromISR( xStringQueue, &pcStrings[ ulReceivedNumber ], &xHigherPriorityTaskWoken );
    }

    /* Clear the software interrupt flag. */
    mainCLEAR_INTERRUPT();

    /* Clear the interrupt in the interrupt controller. */
    IFS0CLR = mainSW1_INT_BIT;

    /* xHigherPriorityTaskWoken was initialised to pdFALSE.  It will have then
    been set to pdTRUE only if reading from or writing to a queue caused a task
    of equal or greater priority than the currently executing task to leave the
    Blocked state.  When this is the case a context switch should be performed.
    In all other cases a context switch is not necessary.
    
    NOTE: The syntax for forcing a context switch within an ISR varies between 
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the PIC32 port layer for this purpose.  taskYIELD() must never be called 
    from an ISR! */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

void vSetupEnvironment( void )
{
	/* Setup the main clock for maximum performance, and the peripheral clock
	to equal the main clock divided by 2. */
	SYSTEMConfigPerformance( configCPU_CLOCK_HZ );
	mOSCSetPBDIV( OSC_PB_DIV_2 );// set the tick rate
    
    /* Setup the software interrupt used by mainTRIGGER_INTERRUPT(). */
    mConfigIntCoreSW1( CSW_INT_ON | CSW_INT_PRIOR_1 | CSW_INT_SUB_PRIOR_0 );
	
	/* Enable global interrupt handling. */
	INTEnableSystemMultiVectoredInt();

	/* Initialise the debug utils library to enable strings printed from the
	demo source code to be displayed in the MPLAB IDE. */
	DBINIT();
}
/*-----------------------------------------------------------*/







