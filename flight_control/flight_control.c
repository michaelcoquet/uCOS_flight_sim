//#define _CRT_SECURE_NO_WARNINGS
#include "includes.h"
#include <stdio.h>
#include <stdlib.h>
//#include "address_map_nios2.h"

/*
 *********************************************************************************************************
 *                                           MACROS
 *********************************************************************************************************
 */
#define	TASK_STK_SIZE		512       /* Size of each task's stacks (# of WORDs)            */
#define FLIGHT_LIMIT        32

 //Priorities
#define FLIGHT_PRIO         36
#define DISPLAY_PRIO        37
#define INPUT_PRIO          38
#define RUNWAY_PRIO         39


#define INBOUND_COL			29
#define RUNWAY_COL			60

//timer periods
#define MAX_ETA             50
#define MIN_ETA             10

//colors
#define BLUE                0x99
#define GRAY                0x0A

// STATES
#define sim_idle_state              0x01
#define sim_running_state           0x02
#define sim_paused_state            0x04
#define sim_all                     0x07

#define flight_inbound_state        0x01
#define flight_outbound_state       0x02
#define flight_on_runway_state      0x04
#define flight_pending_runway       0x08
#define flight_cleared_runway       0x10
#define flight_free_state           0x20
#define flight_dec_in_row_flag      0x40
#define flight_dec_out_row_flag     0x80
#define flight_all                  0xFF

#define display_dec_out_row_flag    0x01
#define display_dec_in_row_flag     0x02
#define display_all                 0x03

/*
 *********************************************************************************************************
 *                                           GLOBALS
 *********************************************************************************************************
 */
INT8U inboundCount;
INT8U outboundCount;
INT8U row_swap;
INT8U totalFlights = 0;

typedef struct
{
	int ID; // flight ID
	int prio; // task ID
	int direction; // outbound = 0, inbound = 1
	int inbound_ID; // basically its row position in the display task
	int outbound_ID;
	int ETA_start;
	int clear_start;
	int free_start;
	char msg[20]; // message to give display
} Flight;

// stacks
OS_STK	  flight_stack[FLIGHT_LIMIT][TASK_STK_SIZE];
OS_STK	  display_stack[TASK_STK_SIZE];
OS_STK	  input_stack[TASK_STK_SIZE];

//event flags
OS_FLAG_GRP *sim_state;

// semaphores
OS_EVENT *expired_sem;
OS_EVENT *expired_ack_sem;
OS_EVENT *callback_sem;
OS_EVENT *runway_sem;

// mailboxes
OS_EVENT *out_expire_mailbox;
OS_EVENT *in_expire_mailbox;

// message queues
OS_EVENT *outbound_dsp_queue;
void     *outbound_dsp_messages[FLIGHT_LIMIT * 2];
OS_EVENT *inbound_dsp_queue;
void     *inbound_dsp_messages[FLIGHT_LIMIT * 2];

// arrays
Flight     flight_data[FLIGHT_LIMIT];
Flight     display_data[FLIGHT_LIMIT];


/*
 *********************************************************************************************************
 *                                           TASKS
 *********************************************************************************************************
 */
void flight_task(void *data);
void display_task(void *data);
void input_task(void *data);
/*
 *********************************************************************************************************
 *                                           FUNCTION PROTOTYPES
 *********************************************************************************************************
 */

static  void  create_tasks(void);
void VGA_text(int, int, char *);
void VGA_clear();
void VGA_clear_character();
void VGA_box(int x1, int y1, int x2, int y2, short pixel_color);
void clear_bg();

// Callbacks
void u_callback(void *ptmr, void *callback_arg);

/*
 *********************************************************************************************************
 *                                                MAIN
 *********************************************************************************************************
 */

int  main(void)
{
	INT8U err;

	VGA_clear();
	VGA_clear_character();

	OSInit();                                              /* Initialize uC/OS-II                      */

	//                       INITIALIZATIONS
	//event_flags
	sim_state = OSFlagCreate(0x00, &err);
	OSFlagPost(sim_state, sim_all, OS_FLAG_CLR, &err);
	OSFlagPost(sim_state, sim_idle_state, OS_FLAG_SET, &err);

	runway_sem = OSSemCreate(1);

	//semaphores
	expired_sem = OSSemCreate(0);
	expired_ack_sem = OSSemCreate(0);


	//mailboxes
	out_expire_mailbox = OSMboxCreate((void *)0);
	in_expire_mailbox = OSMboxCreate((void *)0);

	//queues
	outbound_dsp_queue = OSQCreate(&outbound_dsp_messages[0], FLIGHT_LIMIT);
	inbound_dsp_queue = OSQCreate(&inbound_dsp_messages[0], FLIGHT_LIMIT);


	inboundCount = 0;
	outboundCount = 0;

	create_tasks();

	OSStart();                        /* Start multitasking   */
	return 0;
}

/*
 *********************************************************************************************************
 *                                             CREATE TASKS
 *********************************************************************************************************
 */

static  void  create_tasks(void)
{
	OSTaskCreate(display_task, (void *)0, &display_stack[TASK_STK_SIZE - 1], DISPLAY_PRIO);
	OSTaskCreate(input_task, (void *)0, &input_stack[TASK_STK_SIZE - 1], INPUT_PRIO);
}

/*
 *********************************************************************************************************
 *                                                  TASKS
 *********************************************************************************************************
 */

void flight_task(void *pdata)
{
	INT16U value;
	INT8U err;
	OS_FLAGS  flags;
	Flight f;

	//char dsp_msg[20];

	//int flightID;

	//int ETA_start = rand() % MAX_ETA + MIN_ETA;
	int ETA_time = 0;

	//int clear_start = 10;
	int clear_time = 0;

	//int free_start = 20;
	int free_time = 0;

	int pending_start = 0;
	int pending_time = 0;

	int wait_time = 0; // time for outbound flight to be ready for takeoff

	f = *((Flight *)pdata);

	//OS_TMR *wait_timer;

	OS_FLAG_GRP *flight_state;

	//OS_EVENT *row_swap_sem;
	//row_swap_sem = OSSemCreate(1);

//	wait_timer = OSTmrCreate(0, 1,
//		OS_TMR_OPT_PERIODIC,
//		(void *)wait_callback,
//		(void *)&wait_time,
//		(INT8U         *)"Wait Timer",
//		&err
//	);

	switch (err)
	{
	case OS_ERR_NONE:
		break;
	case OS_ERR_TMR_INVALID_DLY:
		VGA_text(30, 14, "OS_ERR_TMR_INVALID");
		break;
	case OS_ERR_TMR_INVALID_PERIOD:
		VGA_text(30, 10, "OS_ERR_TMR_INVALID");
		break;
	case OS_ERR_TMR_INVALID_OPT:
		VGA_text(30, 11, "OS_ERR_TMR_INVALID");
		break;
	case OS_ERR_TMR_ISR:
		VGA_text(30, 12, "OS_ERR_TMR_INVALID");
		break;
	case OS_ERR_TMR_NON_AVAIL:
		VGA_text(30, 13, "OS_ERR_TMR_INVALID");
		break;
	}
	//OSTmrStart(wait_timer, &err);

	switch (err)
	{
	case OS_ERR_NONE:
		break;
	case OS_ERR_TMR_INVALID:
		VGA_text(20, 14, "OS_ERR_TMR_INVALID");
		break;
	case OS_ERR_TMR_INVALID_TYPE:
		VGA_text(20, 13, "OS_ERR_TMR_INVALID_TYPE");
		break;
	case OS_ERR_TMR_ISR:
		VGA_text(20, 12, "OS_ERR_TMR_ISR");
		break;
	case OS_ERR_TMR_INACTIVE:
		VGA_text(20, 11, "OS_ERR_TMR_INACTIVE");
		break;
	case OS_ERR_TMR_INVALID_STATE:
		VGA_text(20, 10, "OS_ERR_TMR_INVALID_STATE");
		break;
	}

	if (f.direction == 0)  //make outbound flight
	{
		flight_state = OSFlagCreate(flight_outbound_state, &err);
	}
	else  //make inbound flight
	{
		flight_state = OSFlagCreate(flight_inbound_state, &err);
	}

	//flight_data[f.prio] = f;
//	char s[20];
//	sprintf(s, "ETA %d", flight_data[f.prio].ETA_start);
//	VGA_text(35, 20, s);
	while (1)
	{
		flags = OSFlagAccept(sim_state, sim_running_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			wait_time++;
			flags = OSFlagAccept(flight_state, flight_free_state, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				// send message to scan input task to allow this flight to be reused
				//OSQPost(inbound_dsp_queue, (void *)&f);
				OSSemPost(expired_sem);
				OSSemPend(expired_ack_sem, 0, &err);

				f.ID = totalFlights;
				++totalFlights;

				//f.ETA_start = f.ETA_start; //randomize
				ETA_time = 0;
				//f.clear_start = f.clear_start;
				clear_time = 0;
				//f.free_start = f.free_start;
				free_time = 0;
				pending_time = 0;
				wait_time = 0;

				if (f.direction == 0)
				{
					outboundCount++;
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_outbound_state, OS_FLAG_SET, &err);
				}
				else
				{
					inboundCount++;
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_inbound_state, OS_FLAG_SET, &err);
				}

				//if (f.direction == 0)
				//{
				//	expired_f.ID = f.outbound_ID;
				//	err = OSMboxPost(out_expire_mailbox, (void *)&expired_f);
				//	//OSQPost(outbound_expired_queue, (void *)&expired_f);
				//	//err = OSQFlush(outbound_dsp_queue);
				//}
				//else
				//{
				//	expired_f.ID = f.inbound_ID;
				//	err = OSMboxPost(in_expire_mailbox, (void *)&expired_f);
				//	//OSQPost(inbound_expired_queue, (void *)&expired_f);
				//	//err = OSQFlush(inbound_dsp_queue);
				//}
				//OSSemPost(expired_sem);

				//err = OSTaskDel(f.prio);
				//switch (err)
				//{
				//case OS_NO_ERR:
				//	VGA_text(45, 23, "OS_NO_ERR");
				//case OS_TASK_DEL_IDLE:
				//	VGA_text(35, 22, "OS_TASK_DEL_IDLE");
				//case OS_TASK_DEL_ERR:
				//	VGA_text(35, 21, "OS_TASK_DEL_ERR");
				//case OS_PRIO_INVALID:
				//	VGA_text(35, 20, "OS_PRIO_INVALID");
				//case OS_TASK_DEL_ISR:
				//	VGA_text(35, 19, "OS_TASK_DEL_ISR");
				//default:
				//	break;
				//}

				// delete task, timer and anything else
				//char s[20];
				//sprintf(s, "prio: %d", f.prio);
				//VGA_text(35, 10, s);
			}


			flags = OSFlagAccept(flight_state, flight_inbound_state, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				//OSTmrStart(wait_timer, &err);// restart timer if it was paused
				ETA_time = f.ETA_start - wait_time;

				sprintf(f.msg, "Flight #%03d ETA %d    ", f.ID, ETA_time);
				OSQPost(inbound_dsp_queue, (void *)&f);

				if (ETA_time <= 0)
				{
					ETA_time = 0;

					/* pend runway */
					sprintf(f.msg, "Flight #%03d pending  ", f.ID);
					OSQPost(inbound_dsp_queue, (void *)&f);
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_pending_runway, OS_FLAG_SET, &err);
					pending_start = wait_time;

				}
			}

			flags = OSFlagAccept(flight_state, flight_outbound_state, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				//OSTmrStart(wait_timer, &err);// restart timer if it was paused
				ETA_time = f.ETA_start - wait_time;

				sprintf(f.msg, "Flight #%03d ETA %d  ", f.ID, ETA_time);
				OSQPost(outbound_dsp_queue, (void *)&f);

				if (ETA_time <= 0)
				{
					ETA_time = 0;

					/* pend runway */
					sprintf(f.msg, "Flight #%03d pending    ", f.ID);
					OSQPost(outbound_dsp_queue, (void *)&f);
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_pending_runway, OS_FLAG_SET, &err);
					wait_time = 0;
					pending_start = wait_time;

				}
			}
			flags = OSFlagAccept(flight_state, flight_on_runway_state, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				//OSTmrStart(wait_timer, &err);// restart timer if it was paused
				clear_time = f.clear_start - wait_time;

				if (f.direction == 0)
				{
					sprintf(f.msg, "Flight #%03d (Takeoff)    ", f.ID);
					OSQPost(outbound_dsp_queue, (void *)&f);
				}
				else
				{
					sprintf(f.msg, "Flight #%03d (Landing)  ", f.ID);
					OSQPost(inbound_dsp_queue, (void *)&f);
				}


				if (clear_time <= 0)
				{
					clear_time = 0;
					/* start timer to free memory*/
					/* release runway */
					err = OSSemPost(runway_sem);
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_cleared_runway, OS_FLAG_SET, &err);

					if (f.direction == 0)
					{
						sprintf(f.msg, "Flight #%03d (Clear)    ", f.ID);
						OSQPost(outbound_dsp_queue, (void *)&f);
					}
					else
					{
						sprintf(f.msg, "Flight #%03d (Clear)   ", f.ID);
						OSQPost(inbound_dsp_queue, (void *)&f);
					}


					wait_time = 0;
				}
			}
			flags = OSFlagAccept(flight_state, flight_pending_runway, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				//OSTmrStart(wait_timer, &err);// restart timer if it was paused
				pending_time = wait_time - pending_start;

				if (f.direction == 0)
				{
					sprintf(f.msg, "Flight #%03d Pending %d   ", f.ID, pending_time);
					OSQPost(outbound_dsp_queue, (void *)&f);
				}
				else
				{
					sprintf(f.msg, "Flight #%03d Pending %d   ", f.ID, pending_time);
					OSQPost(inbound_dsp_queue, (void *)&f);
				}



				value = OSSemAccept(runway_sem);
				if (value > 0) {
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_on_runway_state, OS_FLAG_SET, &err);

					if (f.direction == 0)
					{
						sprintf(f.msg, "Flight #%03d (Takeoff)    ", f.ID);
						OSQPost(outbound_dsp_queue, (void *)&f);
					}
					else
					{
						sprintf(f.msg, "Flight #%03d (Landing)    ", f.ID);
						OSQPost(inbound_dsp_queue, (void *)&f);
					}

					/* start on runway timer (aka clear_timer)*/
					wait_time = 0;
				}
			}
			flags = OSFlagAccept(flight_state, flight_cleared_runway, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				//OSTmrStart(wait_timer, &err);// restart timer if it was paused
				free_time = f.free_start - wait_time;

				if (f.direction == 0)
				{
					sprintf(f.msg, "Flight #%03d (Clear)  ", f.ID);
					OSQPost(outbound_dsp_queue, (void *)&f);
				}
				else
				{
					sprintf(f.msg, "Flight #%03d (Clear)   ", f.ID);
					OSQPost(inbound_dsp_queue, (void *)&f);
				}

				if (free_time <= 0)
				{
					free_time = 0;
					if (f.direction == 0)
					{
						outboundCount--;
					}
					else
					{
						inboundCount--;
					}
					/* free up resources */
					OSFlagPost(flight_state, flight_all, OS_FLAG_CLR, &err);
					OSFlagPost(flight_state, flight_free_state, OS_FLAG_SET, &err);

				}
			}

		}

		flags = OSFlagAccept(sim_state, sim_paused_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			//OSTmrStop(wait_timer, OS_TMR_OPT_NONE, (void *)0, &err);
		}

		OSTimeDlyHMSM(0, 0, 1, 0);
	}

}

void display_task(void *pdata)
{
	OS_FLAGS flags;
	INT8U err;
	Flight *outbound_msgs;
	Flight *inbound_msgs;

	OS_SEM_DATA sem_data;
	VGA_clear();
	while (1)
	{
		//VGA_clear_character();
		//VGA_clear_character();
		//VGA_clear();
		flags = OSFlagAccept(sim_state, sim_idle_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			switch (err) {
			case OS_NO_ERR:
				//"                                                                                  "
				VGA_text(0, 1, "                          Press Key0 to Start Simulation                          ");
				break;
			case OS_TIMEOUT:
				//PC_DispStr(1, 1, "error: OS_TIMEOUT", DISP_FGND_RED + DISP_BGND_BLACK);
				break;
			}
		}


		flags = OSFlagAccept(sim_state, sim_running_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			switch (err) {
			case OS_NO_ERR:

				VGA_text(0, 1,  "                           Simulation Running                              ");
				VGA_text(0, 59, "                                                               Pause: Key1 ");
				VGA_text(0, 58, "                                             Generate Random Flights: Key2 ");
				VGA_text(0, 4,  " Outbound Flights             Inbound Flights               Runway Status  ");


				err = OSSemQuery(runway_sem, &sem_data);
				if (err == OS_NO_ERR)
				{
					if (sem_data.OSCnt != 0x00)
					{
						VGA_text(RUNWAY_COL, 6, "AVAILABLE     ");
					}
					else
					{
						VGA_text(RUNWAY_COL, 6, "OCCUPIED     ");
					}
				}



				for (int i = 0; i < outboundCount; i++)
				{
					outbound_msgs = (Flight *)OSQAccept(outbound_dsp_queue, &err);
					if (outbound_msgs != (void *)0)
					{
						//display_data[i] = *outbound_msgs;
						VGA_text(0, 5 + outbound_msgs->outbound_ID, &outbound_msgs->msg[0]);
					}
				}
				err = OSQFlush(outbound_dsp_queue);

				for (int i = 0; i < inboundCount; i++)
				{
					inbound_msgs = (Flight *)OSQAccept(inbound_dsp_queue, &err);
					if (inbound_msgs != (void *)0)
					{
						VGA_text(INBOUND_COL, 5 + inbound_msgs->inbound_ID, &inbound_msgs->msg[0]);
					}
				}
				err = OSQFlush(inbound_dsp_queue);


			}
		}

		flags = OSFlagAccept(sim_state, sim_paused_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			switch (err) {
			case OS_NO_ERR:
				//"                "                            Simulation Running                               "                                                                    "
				VGA_text(0, 1,     "                             Simulation Paused                              ");
				//PC_DispStr(1, 1, "                                    Simulation Paused                                  ", DISP_FGND_YELLOW + DISP_BGND_BLUE);
				VGA_text(0, 59,    "                                                              Resume: Key1 ");
				break;
			case OS_TIMEOUT:
				//PC_DispStr(1, 1, "error: OS_TIMEOUT", DISP_FGND_RED + DISP_BGND_BLACK);
				break;
			}
		}
		OSTimeDly(1);
	}
}

void input_task(void *pdata)
{
	INT8U  err;
	INT16U value;

	OS_FLAGS flags;

	INT16U counter = 0;

	INT8U direction = 0;
	INT8U flightCounter = 0;
	srand(OSTimeGet());

	OS_TMR *utilization_timer;

	utilization_timer = OSTmrCreate(20, 0,
		OS_TMR_OPT_ONE_SHOT,
		(void *)u_callback,
		(void *)&counter,
		(INT8U         *)"utilization Timer",
		&err
	);

	//pdata = pdata;
	while (1)
	{
		volatile int * KEY_ptr = (int *) 0xFF200050;		// pushbutton KEY address
		int KEY_value;

		KEY_value = *(KEY_ptr); 				// read the pushbutton KEY values
		//while (*KEY_ptr);							// wait for pushbutton KEY release

		if (KEY_value == 0x1)					// check KEY1
		{
			OSFlagPost(sim_state, sim_idle_state, OS_FLAG_CLR, &err);
			OSFlagPost(sim_state, sim_running_state, OS_FLAG_SET, &err);
			OSTmrStart(utilization_timer, &err);
		}
		if (KEY_value == 0x2)					// check KEY1
		{
			flags = OSFlagQuery(sim_state, &err);
			if (flags == sim_running_state)
			{
				OSFlagPost(sim_state, sim_running_state, OS_FLAG_CLR, &err);
				OSFlagPost(sim_state, sim_paused_state, OS_FLAG_SET, &err);
			}
			else if (flags == sim_paused_state)
			{
				OSFlagPost(sim_state, sim_paused_state, OS_FLAG_CLR, &err);
				OSFlagPost(sim_state, sim_running_state, OS_FLAG_SET, &err);
			}
			//VGA_clear_character();
		}
		if (KEY_value == 0x4)				// check KEY2
		{
			flags = OSFlagAccept(sim_state, sim_running_state, OS_FLAG_WAIT_SET_ALL, &err);
			if (flags > 0)
			{
				counter++;
				//char s[20];
				//sprintf(s, "out: %d in: %d total: %d limit %d", outboundCount, inboundCount, outboundCount + inboundCount, FLIGHT_LIMIT);
				//VGA_text(30, 15, s);
				if ((outboundCount + inboundCount) != FLIGHT_LIMIT) // cant handle more than a certain number of flights
				{
					flightCounter = outboundCount + inboundCount; // total nubmer of flights currently active
					direction = rand() % 2;
					if (direction == 0)
					{
						if (outboundCount != (FLIGHT_LIMIT / 2))
						{
							value = OSSemAccept(expired_sem);         /* Check resource availability */
							if (value > 0)
							{
								OSSemPost(expired_ack_sem);
							}
							else
							{
								// if no IDS/priorities are available make a new outbound flight

								++outboundCount;
								flight_data[flightCounter].ID = totalFlights;
								flight_data[flightCounter].ETA_start = rand() % MAX_ETA + MIN_ETA;
								flight_data[flightCounter].direction = direction;
								Flight low_eta = flight_data[flightCounter];
								for (int i = 0; i < flightCounter; i++)
								{
									if (flight_data[i].ETA_start < low_eta.ETA_start)
									{
										low_eta = flight_data[i];
									}
								}
								if (flight_data[flightCounter].ETA_start < low_eta.ETA_start)
								{
									flight_data[flightCounter].prio = low_eta.prio - 1; // task ID
								}
								else
								{
									flight_data[flightCounter].prio = FLIGHT_PRIO - flightCounter;
								}
								flight_data[flightCounter].clear_start = 10;
								flight_data[flightCounter].free_start = 10;

								flight_data[flightCounter].outbound_ID = outboundCount; //display row
								OSTaskCreate(flight_task, (void *)&flight_data[flightCounter], &flight_stack[flightCounter][TASK_STK_SIZE - 1], (INT8U)(flight_data[flightCounter].prio));

								++totalFlights; // total number of flights (since this also gives the task its priority start at 0

							}
						}
					}
				}
				else
				{
					// outbounds full try to make inbound
					direction = 1;
				}
			}

			if (direction == 1)
			{
				if (inboundCount != (FLIGHT_LIMIT / 2))
				{

					value = OSSemAccept(expired_sem);         /* Check resource availability */
					if (value > 0)
					{
						OSSemPost(expired_ack_sem);
					}
					else
					{
						// make inbound flight
						// if no IDS/priorities are available make a new outbound flight
						++inboundCount;
						flight_data[flightCounter].ID = totalFlights;
						flight_data[flightCounter].ETA_start = rand() % MAX_ETA + MIN_ETA;
						flight_data[flightCounter].direction = direction;

						Flight low_eta = flight_data[flightCounter];
						for (int i = 0; i < flightCounter; i++)
						{
							if (flight_data[i].ETA_start < low_eta.ETA_start)
							{
								low_eta = flight_data[i];
							}
						}
						if (flight_data[flightCounter].ETA_start < low_eta.ETA_start)
						{
							flight_data[flightCounter].prio = low_eta.prio - 1; // task ID
						}
						else
						{
							flight_data[flightCounter].prio = FLIGHT_PRIO - flightCounter;
						}
						flight_data[flightCounter].clear_start = 10;
						flight_data[flightCounter].free_start = 10;

						flight_data[flightCounter].inbound_ID = inboundCount; //display row
						OSTaskCreate(flight_task, (void *)&flight_data[flightCounter], &flight_stack[flightCounter][TASK_STK_SIZE - 1], (INT8U)(flight_data[flightCounter].prio));


						++totalFlights; // total number of flights (since this also gives the task its priority start at 0
					}

				}
			}
		}
		flags = OSFlagAccept(sim_state, sim_running_state, OS_FLAG_WAIT_SET_ALL, &err);
		if (flags > 0)
		{
			counter++;
		}
		OSTimeDly(1);
	}
}


/*
 *********************************************************************************************************
 *                                                  Callbacks
 *********************************************************************************************************
 */
void u_callback(void *ptmr, void *callback_arg)
{
	INT16U count = 0;
	count = *((INT16U *)callback_arg);
	VGA_box(0,0, 319, 239, GRAY);
	// count = 160 without tasks
	// count = 250 in lowest task
}
//*
//*********************************************************************************************************
//*                                                  Functions
//*********************************************************************************************************
//*
void clear_bg()
{
	VGA_box(0,0, 319, 239, GRAY);
	VGA_box(0, 1, 320, 8, BLUE);
	VGA_box(170, 231, 320, 240, BLUE);
	VGA_box(0, 238, 320, 240, BLUE);
}

void VGA_text(int x, int y, char * text_ptr)
{
	int offset;
	volatile char * character_buffer = (char *) 0x09000000;   // VGA character buffer
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		*(character_buffer + offset) = *(text_ptr);   // write to the character buffer
		++text_ptr;
		++offset;
	}
}

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	int offset, row, col;
	volatile short * pixel_buffer = (short *) 0x08000000;
	// VGA pixel buffer
	/* assume that the box coordinates are valid */
	for (row = y1; row <= y2; row++)
	{
		col = x1;
		while (col <= x2)
		{
			offset = (row << 9) + col;
			*(pixel_buffer + offset) = pixel_color;
			// compute halfword address, set pixel
			++col;
		}
	}
}

void VGA_pixel(int x, int y, short pixel_color)
{
	int offset;
	volatile short * pixel_buffer = (short *) 0x08000000;
	// VGA pixel buffer
	offset = ((y << 9) + x);
	*(pixel_buffer + offset) = pixel_color;
}

void VGA_clear()
{
	int x, y;
	for(x = 0; x < 320; x++)
	{
		for(y = 0; y < 240; y++)
		{
			VGA_pixel(x, y, 0);
		}
	}
	clear_bg();
}

void VGA_clear_character()
{
	for(int x = 0; x < 80; x++)
	{
		for(int y = 0; y < 60; y++)
		{
			VGA_text(x, y, " ");
		}
	}
	clear_bg();
}


