
#ifndef __XENO__
#define __XENO__
#endif

#include "RTClient.h"

//Modify this number to indicate the actual number of motor on the network
#define ELMO_TOTAL 5
#define BIONIC_ARM_DOF 6

hyuEcat::Master ecatmaster;
hyuEcat::EcatElmo ecat_elmo[ELMO_TOTAL];

int serial_fd = -1;
int sercan_fd = -1;


struct LOGGING_PACK
{
	double 	Time;				/**< Global Time			*/
	double 	ActualPos[BIONIC_ARM_DOF]; 	/**< Actual Position in Radian		*/
	double 	ActualVel[BIONIC_ARM_DOF];	/**< Actual Velocity in Radian/second	*/
	short  	ActualToq[BIONIC_ARM_DOF];
	double 	DesiredPos[BIONIC_ARM_DOF];
	double  DesiredVel[BIONIC_ARM_DOF];
	short  	DesiredToq[BIONIC_ARM_DOF];
};

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double double_gt=0; //real global time
float float_dt=0;

// EtherCAT Data (Dual-Arm)
UINT16	StatusWord[BIONIC_ARM_DOF] = {0,};
INT32 	ActualPos[BIONIC_ARM_DOF] = {0,};
INT32 	ActualVel[BIONIC_ARM_DOF] = {0,};
INT16 	ActualTor[BIONIC_ARM_DOF] = {0,};
INT8	ModeOfOperationDisplay[BIONIC_ARM_DOF] = {0,};
INT8	DeviceState[BIONIC_ARM_DOF] = {0,};

INT16 	TargetTor[BIONIC_ARM_DOF] = {0,};		//100.0 persentage
/****************************************************************************/
// Xenomai RT tasks
RT_TASK RTArm_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK tcpip_task;

RT_QUEUE msg_plot;
RT_QUEUE msg_tcpip;

void signal_handler(int signum);
int isSlaveInit(void);

// For RT thread management
unsigned long fault_count=0;
unsigned long ethercat_time=0;
unsigned long worst_time=0;

static double ActualPos_Rad[BIONIC_ARM_DOF] = {0.0,};
static double ActualVel_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetPos_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetVel_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetAcc_Rad[BIONIC_ARM_DOF] = {0.0,};
static double TargetToq[BIONIC_ARM_DOF] = {0.0,};

static double manipulatorpower=0;
static double best_manipulatorpower=0;

int isSlaveInit(void)
{
	int elmo_count = 0;
	int slave_count = 0;

	for(int i=0; i<ELMO_TOTAL; ++i)
	{
		if(ecat_elmo[i].initialized())
		{
			elmo_count++;
		}
	}

	for(int j=0; j<((int)ecatmaster.GetConnectedSlaves()); j++)
	{
		if(ecatmaster.GetSlaveState(j) == 0x08)
		{
			slave_count++;
		}
	}

	if((elmo_count == ELMO_TOTAL) && (slave_count == ((int)ecatmaster.GetConnectedSlaves())))
		return 1;
	else
		return 0;
}

Vector3d ForwardPos[2];
Vector3d ForwardOri[2];
Vector3d ForwardAxis[2];
int NumChain;

// RTArm_task
void RTRArm_run(void *arg)
{
#if defined(_PLOT_ON_)
	int sampling_time 	= 20;	// Data is sampled every 10 cycles.
	int sampling_tick 	= sampling_time;

	void *msg;
	LOGGING_PACK logBuff;
	int len = sizeof(LOGGING_PACK);
#endif
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	short MaxTor = 1200;

	int k=0;

	uint16_t ControlMotion = SYSTEM_BEGIN;
	uint16_t JointState = SYSTEM_BEGIN;

	VectorXd finPos(BIONIC_ARM_DOF);
	finPos.setZero();

	SerialManipulator BionicArm;
	//HYUControl::Controller Control(&BionicArm);
	//HYUControl::Motion motion(&BionicArm);

	BionicArm.UpdateManipulatorParam();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (1)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		previous = rt_timer_read();

		ecatmaster.RxUpdate();

		for(k=0; k < ELMO_TOTAL; k++)
		{
			DeviceState[k] = 				ecat_elmo[k].Elmo_DeviceState();
			StatusWord[k] = 				ecat_elmo[k].status_word_;
			ModeOfOperationDisplay[k] = 	ecat_elmo[k].mode_of_operation_display_;
			ActualPos[k] = 					ecat_elmo[k].position_;
			ActualVel[k] = 					ecat_elmo[k].velocity_;
			ActualTor[k] = 					ecat_elmo[k].torque_;
		}

		if( system_ready )
		{

			BionicArm.ENCtoRAD(ActualPos, ActualPos_Rad);
			BionicArm.VelocityConvert(ActualVel, ActualVel_Rad);

			//BionicArm.pKin->PrepareJacobian(ActualPos_Rad);

			//BionicArm.pKin->GetManipulability( TaskCondNumber, OrientCondNumber );
			//BionicArm.pKin->GetForwardKinematics( ForwardPos, ForwardOri, NumChain );

			//BionicArm.StateMachine( ActualPos_Rad, ActualVel_Rad, finPos, JointState, ControlMotion );
			//motion.JointMotion( TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, finPos, ActualPos_Rad, ActualVel_Rad, double_gt, JointState, ControlMotion );

			//Control.InvDynController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, float_dt );

			//BionicArm.TorqueConvert(TargetToq, TargetTor, MaxTor);

			//write the motor data
			for(int j=0; j < ELMO_TOTAL; ++j)
			{

				if(double_gt >= 1.0)
				{
					//ecat_elmo[j].writeTorque(TargetTor[j]);
				}
				else
				{
					ecat_elmo[j].writeTorque(0);
				}
			}
		}

		ecatmaster.TxUpdate();

#if defined(_USE_DC_MODE_)
		ecatmaster.SyncEcatMaster(rt_timer_read());
#endif

		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

		if ( isSlaveInit() == 1 )
		{
			float_dt = ((float)(long)(p3 - p1))*1e-3; 		// us
			double_gt += ((double)(long)(p3 - p1))*1e-9; 	// s
			ethercat_time = (long) now - previous;

			if( double_gt >= 0.5 )
			{
				system_ready=1;	//all drives have been done

				if ( worst_time<ethercat_time )
					worst_time=ethercat_time;

				if(best_manipulatorpower < manipulatorpower)
					best_manipulatorpower = manipulatorpower;

				if( ethercat_time > (unsigned long)cycle_ns )
				{
					fault_count++;
					worst_time=0;
				}
			}

#if defined(_PLOT_ON_)
			if ( (system_ready==1) && datasocket.hasConnection() && (sampling_tick-- == 0) )
			{
				sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

				logBuff.Time = double_gt;
				for(int k=0; k<BIONIC_ARM_DOF; k++)
				{
					logBuff.ActualPos[k] = ActualPos_Rad[k]*RADtoDEG;
					logBuff.ActualVel[k] = ActualVel_Rad[k]*RADtoDEG;
					logBuff.ActualToq[k] = ActualTor[k];

					logBuff.DesiredPos[k] = TargetPos_Rad[k]*RADtoDEG;
					logBuff.DesiredVel[k] = TargetVel_Rad[k]*RADtoDEG;
					logBuff.DesiredToq[k] = TargetTor[k];
				}

				msg = rt_queue_alloc(&msg_plot, len);
				if(msg == NULL)
					rt_printf("rt_queue_alloc Failed to allocate, NULL pointer received\n");

				memcpy(msg, &logBuff, len);
				rt_queue_send(&msg_plot, msg, len, Q_NORMAL);
			}
#endif
		}
		else
		{
			system_ready = 0;
			double_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
			best_manipulatorpower=0;
		}
	}
}

void can_task_proc(void *arg)
{
	CAN_FRAME txframe, rxframe;
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	float float_dt_can=0;
	unsigned long can_time=0;
	unsigned long can_worst_time=0;

	unsigned int can_cycle_ns = 70e6;
	rt_task_set_periodic(NULL, TM_NOW, can_cycle_ns);

	while(1)
	{
		rt_task_wait_period(NULL);
		previous = rt_timer_read();

		if(system_ready)
		{
			//control code
			//

			float_dt_can = ((float)(long)(p3 - p1))*1e-3; 		// us
			can_time = (long) now - previous;

			if ( can_worst_time<can_time )
				can_worst_time=can_time;

			if( can_time > (unsigned long)can_cycle_ns )
			{
				fault_count++;
				can_worst_time=0;
			}
		}
		else
		{
			can_worst_time = 0;
			can_time = 0;
		}

		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

	}
}

void serial_task_proc(void *arg)
{
	int kchr;
	int chr;
	int nbytes;

	unsigned int serial_cycle_ns = 50e6;
	rt_task_set_periodic(NULL, TM_NOW, serial_cycle_ns);
	for(;;)
	{
		rt_task_wait_period(NULL);

		//write
		if(NRMKkbhit())
		{
			kchr =  getchar();
			write(serial_fd, &kchr, 1);
		}

		//read
		nbytes = read(serial_fd, &chr, 1);
		if(nbytes > 0)
		{

		}

	}
}

void print_run(void *arg)
{
	long stick=0;
	int count=0;

	rt_printf("\nPlease WAIT at least %i (s) until the system getting ready...\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	RTIME PrintPeriod = 5e8;
	rt_task_set_periodic(NULL, TM_NOW, PrintPeriod);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle

		if ( ++count >= roundl(NSEC_PER_SEC/PrintPeriod) )
		{
			++stick;
			count=0;
		}

		if ( system_ready )
		{
			rt_printf("Time=%0.2fs\n", double_gt);
			rt_printf("actTask_dt= %lius, desTask_dt=%0.1fus, Worst_dt= %lius, Fault=%d\n",
					ethercat_time/1000, float_dt, worst_time/1000, fault_count);

			for(int j=0; j<ELMO_TOTAL; ++j)
			{
				rt_printf("\t \nID: %d,", j+1);

#if defined(_DEBUG_)
				//rt_printf(" StatWord: 0x%04X, ",	StatusWord[j]);
				//rt_printf(" DeviceState: %d, ",	DeviceState[j]);
				rt_printf(" ModeOfOp: %d,",		ModeOfOperationDisplay[j]);
				//rt_printf("\n");
#endif
				rt_printf("\tActPos(Deg): %0.2lf,", 	ActualPos_Rad[j]*RADtoDEG);
				//rt_printf("\tTarPos(Deg): %0.2lf,",	TargetPos_Rad[j]*RADtoDEG);
				rt_printf("\tActPos(inc): %d,", 	ActualPos[j]);
				//rt_printf("\n");
				rt_printf("\tActVel(Deg/s): %0.1lf,", 	ActualVel_Rad[j]*RADtoDEG);
				//rt_printf("\tTarVel(Deg/s): %0.1lf,",	TargetVel_Rad[j]*RADtoDEG);
				//rt_printf("\tActVel(inc/s): %d,", 	ActualVel[j]);
				//rt_printf("\n");
				rt_printf("\tActTor(%): %d,",		ActualTor[j]);
				rt_printf("\tCtrlTor(Nm): %0.1lf", 	TargetToq[j]);
				//rt_printf("\tTarTor(%): %d", 		TargetTor[j]);
				//rt_printf("\n");
			}

			rt_printf("\nForward Kinematics -->");
			for(int cNum = 0; cNum < NumChain; cNum++)
			{
				rt_printf("\n Num:%d: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.3lf, v:%0.3lf, w:%0.3lf ",cNum, ForwardPos[cNum](0), ForwardPos[cNum](1), ForwardPos[cNum](2),
						ForwardOri[cNum](0)*RADtoDEG, ForwardOri[cNum](1)*RADtoDEG, ForwardOri[cNum](2)*RADtoDEG);
				//rt_printf("\n Manipulability: Task:%0.2lf, Orient:%0.2lf", TaskCondNumber[cNum], OrientCondNumber[cNum]);
				rt_printf("\n");
			}

			rt_printf("\n");
		}
		else
		{
			if ( count==0 )
			{
				rt_printf("\nReady Time: %i sec", stick);

#if defined(_ECAT_ON_)
				rt_printf("\nMaster State: %s, AL state: 0x%02X, ConnectedSlaves : %d",
						ecatmaster.GetEcatMasterLinkState().c_str(), ecatmaster.GetEcatMasterState(), ecatmaster.GetConnectedSlaves());
				for(int i=0; i<((int)ecatmaster.GetConnectedSlaves()); i++)
				{
					rt_printf("\nID: %d , SlaveState: 0x%02X, SlaveConnection: %s, SlaveNMT: %s ", i,
							ecatmaster.GetSlaveState(i), ecatmaster.GetSlaveConnected(i).c_str(), ecatmaster.GetSlaveNMT(i).c_str());

					rt_printf(" SlaveStatus : %s,", ecat_elmo[i].GetDevState().c_str());
					rt_printf(" StatWord: 0x%04X, ", ecat_elmo[i].status_word_);

				}
#endif
				rt_printf("\n");
			}
		}
	}
}

void plot_run(void *arg)
{
	ssize_t len;
	void *msg;
	LOGGING_PACK logBuff;
	memset(&logBuff, 0, sizeof(LOGGING_PACK));

	int err = rt_queue_bind(&msg_plot, "PLOT_QUEUE", TM_INFINITE);
	if(err)
	{
		fprintf(stderr, "Failed to queue bind, code %d\n", err);
	}

	rt_task_set_periodic(NULL, TM_NOW, 1e7);

	while (1)
	{
		rt_task_wait_period(NULL);

		if ( (len = rt_queue_receive(&msg_plot, &msg, TM_INFINITE)) > 0 )
		{
			memcpy(&logBuff, msg, sizeof(LOGGING_PACK));

			datasocket.updateControlData( logBuff.ActualPos, logBuff.DesiredPos, logBuff.ActualVel, logBuff.DesiredVel, logBuff.ActualToq, logBuff.DesiredToq );
			datasocket.update( logBuff.Time );

			rt_queue_free(&msg_plot, msg);
		}
	}
}

void tcpip_run(void *arg)
{

	ServerSocket sock(SERVER_PORT);
	TCPServer server(new SessionFactory(), sock);

	cout << "Simple TCP Server Application." << endl;
	cout << "maxConcurrentConnections: " << server.maxConcurrentConnections() << endl;

	int current_Thread=0;
	server.start();

	rt_task_set_periodic(NULL, TM_NOW, 1e7);

	while(1)
	{
		  rt_task_wait_period(NULL);
		  current_Thread = server.currentConnections();
		  if(current_Thread != 0)
		  {

		  }
	}
}

/****************************************************************************/
void signal_handler(int signum)
{
	rt_printf("\nSignal Interrupt: %d", signum);

	int res = -1;

	rt_printf("\nConsolPrint RTTask Closing....");
	rt_task_delete(&print_task);
	rt_printf("\nConsolPrint RTTask Closing Success....");

#if defined(_PLOT_ON_)
	rt_queue_unbind(&msg_plot);
	rt_printf("\nPlotting RTTask Closing....");
	rt_task_delete(&plot_task);
#endif

	rt_printf("\nTCPIP RTTask Closing....");
	rt_task_delete(&tcpip_task);
	rt_printf("\nTCPIP RTTask Closing Success....");

#if defined(_CAN_ON_)
	rt_printf("\nSERCAN RTTask Closing....");
	res = rt_task_delete(&can_task);
	if(sercan_fd > 0)
	{
		res = close(sercan_fd);
		res = rt_printf("\nSERCAN RTTask Closing Success...");
	}
#endif

#if defined(_RS232_ON_)
	rt_printf("\nSerial RTTask Closing...");
	res = rt_task_delete(&serial_task);
	if(serial_fd > 0)
	{
		if(close(serial_fd) == 0)
		{
			rt_printf("\nSerial Closing Success...");
		}
		else
		{
			rt_printf("\nSerial Closing Failure...");
		}
	}
#endif

#if defined(_ECAT_ON_)
	rt_printf("\nEtherCAT RTTask Closing....");
	rt_task_delete(&RTArm_task);
	rt_printf("\nEtherCAT RTTask Closing Success....");

	ecatmaster.deactivate();
#endif

	rt_printf("\n\n\t !!RT Arm Client System Stopped!! \n");
	exit(signum);
}

/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
	rt_print_auto_init(1);

	signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGKILL, signal_handler);
	signal(SIGFPE, signal_handler);
	signal(SIGQUIT, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	//cycle_ns = 250000; // nanosecond -> 4kHz
	cycle_ns = 500000; // nanosecond -> 2kHz
	//cycle_ns = 1000000; // nanosecond -> 1kHz
	//cycle_ns = 1250000; // nanosecond -> 800Hz
	period = ((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

#if defined(_RS232_ON_)
	char PortName[30];
	unsigned int SerialBoud = 115200;
	strcpy(PortName, COM1);
	rt_printf("Serial Configuration : port=#%s, boudrate=%i\n", PortName, SerialBoud);
	serial_fd = tp_open_serial_port(PortName, SerialBoud);
	if(serial_fd < 0)
		exit(EXIT_FAILURE);
#endif

#if defined(_CAN_ON_)
	sercan_fd = SERCAN_open();
	if(sercan_fd < 0)
	{
		exit(EXIT_FAILURE);
	}
	else
	{
		rt_printf("Sercan Driver Activated : %d\t", sercan_fd);
		int CANdevBitrate = SERCAN_GetBitRate(sercan_fd, 10);
		rt_printf("Sercan Bitrate : %d\n", CANdevBitrate);
	}
#endif

#if defined(_ECAT_ON_)
	int SlaveNum;
	for(SlaveNum=0; SlaveNum < ELMO_TOTAL; SlaveNum++)
	{
		ecatmaster.addSlave(0, SlaveNum, &ecat_elmo[SlaveNum]);
	}

#if defined(_USE_DC_MODE_)
	ecatmaster.activateWithDC(0, cycle_ns);
#else
	ecatmaster.activate();
#endif

#endif

#if defined(_PLOT_ON_)
	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		rt_printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	rt_printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
#endif

	// RTArm_task: create and start
	rt_printf("Now running rt task ...\n");


#if defined(_PLOT_ON_)
	rt_queue_create(&msg_plot, "PLOT_QUEUE", sizeof(LOGGING_PACK)*20, 20, Q_FIFO|Q_SHARED);

	rt_task_create(&plot_task, "PLOT_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&plot_task, &plot_run, NULL);
#endif

#if defined(_ECAT_ON_)
	rt_task_create(&RTArm_task, "CONTROL_PROC_Task", 1024*1024*4, 99, T_FPU); // MUST SET at least 4MB stack-size (MAXIMUM Stack-size ; 8192 kbytes)
	rt_task_start(&RTArm_task, &RTRArm_run, NULL);
#endif

#if defined(_CAN_ON_)
	rt_task_create(&can_task, "CAN_PROC_TASK", 0, 98, T_FPU);
	rt_task_start(&can_task, &can_task_proc, NULL);

#endif

#if defined(_RS232_ON_)
	rt_task_create(&serial_task, "SERIAL_PROC_TASK", 0, 95, T_FPU);
	rt_task_start(&serial_task, &serial_task_proc, NULL);
#endif

	rt_task_create(&print_task, "CONSOLE_PROC_Task", 0, 60, T_FPU);
	rt_task_start(&print_task, &print_run, NULL);

	rt_task_create(&tcpip_task, "TCPIP_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&tcpip_task, &tcpip_run, NULL);

	// Must pause here
	pause();

	// Finalize
	signal_handler(SIGTERM);

    return 0;
}



