。
/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : ebox [ifname] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This test is specifically build for the E/BOX.
 *
 * (c)Arthur Ketels 2011
 */
//////////////////////////////////////////  修改  /////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////



#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"
////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
#define NSEC_PER_SEC 1000000000



typedef struct PACKED
{
    uint16 status;
    uint32 tPos;
    uint32 velocityOffset;
    uint16 torquteOffset;
    int32 tVel;
    uint16 tTorque;
    uint8  opMode;
    uint8  complement;
} out_ISMC;

typedef struct PACKED
{
    uint16 status;
    int32 Pos;
    int32 vel;
    uint16 tTorque;
    uint8  opMode;
    uint8  complement;
} in_ISMC;

// total samples to capture
#define MAXSTREAM 200000
// sample interval in ns, here 8us -> 125kHz
// maximum data rate for E/BOX v1.0.1 is around 150kHz
#define SYNC0TIME 8000

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff;
int DCdiff;
int os;
uint32 ob;
int16 ob2;
uint8 ob3;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int64 integral = 0;
uint32 cyclecount;

in_ISMC *val;
out_ISMC *target;

in_ISMC *val2;
out_ISMC *target2;

double ain[2];
int ainc;
int streampos;
int16 stream1[MAXSTREAM];
int16 stream2[MAXSTREAM];
/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

int output_cvs(char *fname, int length)
{
    FILE *fp;

    int i;

    fp = fopen(fname, "w");
    if (fp == NULL)
        return 0;
    for (i = 0; i < length; i++)
    {
        fprintf(fp, "%d %d %d\n", i, stream1[i], stream2[i]);
    }
    fclose(fp);

    return 1;
}

static int slave_dc_config(uint16 slave) //ecx_contextt * context, uint16 slave
{
    //ec_dcsync0(slave, TRUE, 2000 * 1000, 4000);
    ec_dcsync01(slave, TRUE, 2000 * 1000, 2000 * 1000, 0); 
    return 0;
}
static int slave_dc_config2(ecx_contextt * context,uint16 slave) //ecx_contextt * context, uint16 slave
{

ec_dcsync0(slave, TRUE, 1000 * 1000, 0);
//  ec_dcsync01(slave, TRUE, 1000 * 1000, 1000 * 1000, 0); 
return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ros::Subscriber velSub;

// std_msgs::Int32MultiArray velMsg;

int32_t vel1,vel2;

void velCallback(std_msgs::Int32MultiArray velMsg)
{
    vel1 = velMsg.data[0];
    vel2 = velMsg.data[1];
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void eboxtest(char *ifname)
{

///////////////////////////////////////////////////////////////////////////////////////////////////
    // usleep(10000);
    ros::NodeHandle nh;
    ros::Subscriber  velSub = nh.subscribe("vel",100,velCallback);
//这里也许可以写一个vel1=0   vel2=0
//////////////////////////////////////////////////////////////////////////////////////////////////
    int cnt, i;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

    printf("Starting E/BOX test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);



            // ec_dcsync0(1, TRUE, 1000 * 1000, 0 * 1000); // SYNC0/1 on slave 1
            // // ec_dcsync0(2, TRUE, 1000 * 1000, 0 * 1000); // SYNC0/1 on slave 2     
            ec_dcsync01(1, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 1
            ec_dcsync01(2, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 2
            ec_config_map(&IOmap);
            ec_config_init(FALSE);



            // check if first slave is an E/BOX
            if ((ec_slavecount >= 1))
            {
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    WRITE(i, 0x6060, 0, buf8, 9, "OpMode"); // 3 profile velocity mode works, 9 Cyclic sync velocity mode didn't work    8 位置  9 速度  10 电流 
                    READ(i, 0x6061, 0, buf8, "OpMode display");

                    READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                    READ(i, 0x1c13, 0, buf32, "txPDO:0");
                    READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                    READ(i, 0x1c13, 1, buf32, "txPDO:1");



                    //配置控制器->驱动器
                    WRITE(i, 0x1c12, 0, buf8, 0x0, "Rx mapping");
                    WRITE(i, 0x1c12, 1, buf16, 0x1600, "Rx mapping");
                    WRITE(i, 0x1c12, 0, buf8, 0x1, "Rx mapping");


                    //配置驱动器->控制器
					
                    WRITE(i, 0x1c13, 0, buf8, 0x0, "Tx mapping");	
                    WRITE(i, 0x1c13, 1, buf16, 0x1a00, "Rx mapping");
                    WRITE(i, 0x1c13, 0, buf8, 0x1, "Tx mapping");



                    WRITE(i, 0x6080, 0, buf32, 1000000, "max motor speed");
                    WRITE(i, 0x6081, 0, buf32, 1000000, "profile velocity");

                    WRITE(i, 0x60c2, 1, buf8, 0x01, "cyc time");
                //    ec_slave[i].PO2SOconfigx = slave_dc_config2;
                }
            }

                //  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            // ec_configdc();


            ec_config_map(&IOmap);
            ec_configdc();
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            usleep(100000);     
            for (i = 1; i <= ec_slavecount; i++)
            {
                // ec_dcsync0(i, TRUE, 1000 * 1000, 0 * 1000); // SYNC0/1 on slave 1
                ec_dcsync01(i, TRUE, 1000 * 1000, 000 * 1000,0); // SYNC0/1 on slave 1
            }

                   
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            usleep(100000);   



            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /* configure DC options for every DC capable slave found in the list */
            printf("DC capable : %d\n", ec_configdc());

            // ec_send_processdata();
            // ec_receive_processdata(EC_TIMEOUTRET);
            // usleep(100000);     
            // // ec_dcsync0(1, TRUE, 1 * 1000, 0 * 1000); // SYNC0/1 on slave 1
            // // ec_dcsync0(2, TRUE, 1 * 1000, 0 * 1000); // SYNC0/1 on slave 2                            
            // ec_dcsync01(1, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 1
            // ec_dcsync01(2, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 2


            for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
            {
               printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
               printf(" Configured address: %x\n", ec_slave[cnt].configadr);
               printf(" Outputs address: %x\n", ec_slave[cnt].outputs);
               printf(" Inputs address: %x\n", ec_slave[cnt].inputs);

               for(int j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
               {
                  printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
                          (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                          ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                          ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
               }
               printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                        ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);

            }

            /* check configuration */
            if (ec_slavecount >= 1)
            {

                /* connect struct pointers to slave I/O pointers */
                target = (out_ISMC *)(ec_slave[1].outputs);
                val = (in_ISMC *)(ec_slave[1].inputs);

                

                target2 = (out_ISMC *)(ec_slave[2].outputs);
                val2 = (in_ISMC *)(ec_slave[2].inputs);
                target->opMode=9;                                          ///////////////////////////////////////////////   169
                target2->opMode=9;
                /* read indevidual slave state and store in ec_slave[] */
                ec_readstate();
                for (cnt = 1; cnt <= ec_slavecount; cnt++)
                {
                    printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                           cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                           ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                }
                printf("Request operational state for all slaves\n");

                /* send one processdata cycle to init SM in slaves */
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                ec_slave[0].state = EC_STATE_OPERATIONAL;
                /* request OP state for all slaves */
                ec_writestate(0);
                /* wait for all slaves to reach OP state */
                ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
                if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    printf("Operational state reached for all slaves.\n");
                    ain[0] = 0;
                    ain[1] = 0;
                    ainc = 0;
                    dorun = 1;
                    usleep(100000);                               // wait for linux to sync on DC
                    ec_dcsync01(1, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 1
                    ec_dcsync01(2, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave 2
                    printf("dc set over.\n");

                    uint8_t startup_step = 0;
                    uint8_t startup_step2 = 0;
                    /* acyclic loop 1ms */
                    for (i = 1; i <= 20000; i++)
                    {
                        /**
                         * Drive state machine transistions
                         *   0 -> 6 -> 7 -> 15
                         */
                        uint16 cur_status1 = val->status; //0x6041
                        switch (startup_step)
                        {
                        case 1:
                            target->status = 0x06; //0x6040
                            if ((cur_status1 == 0x0631) || (cur_status1 == 0x0231) || cur_status1 == 545 || cur_status1 == 1569)
                                startup_step = 2;
                            break;
                        case 2:
                            target->status = 0x07;
                            if (cur_status1 == 0x633 || cur_status1 == 545)
                                startup_step = 3;
                        case 3:
                            target->status = 0x0f;
                            if ((cur_status1 == 0x1637) || (cur_status1 == 0x1633))
                                startup_step = 4;
                        case 4:
                            target->status = 0x0f;
                            target->tVel = 0000;                       /////////////////////////
                            //  target->tTorque=100;

                            break;
                        default:
                            startup_step = 1;
                            target->status = 0x06; //0x6040
                            break;
                        }
                        uint16 cur_status2 = val2->status; //0x6041
                        switch (startup_step2)
                        {
                        case 1:
                            target2->status = 0x06; //0x6040
                            if ((cur_status2 == 0x0631) || (cur_status2 == 0x0231) || cur_status2 == 545 || cur_status1 == 1569)
                                startup_step2 = 2;
                            break;
                        case 2:
                            target2->status = 0x07;
                            if (cur_status2 == 0x633 || cur_status2 == 545)
                                startup_step2 = 3;
                        case 3:
                            target2->status = 0x0f;
                            if ((cur_status2 == 0x1637) || (cur_status2 == 0x1633))
                                startup_step2 = 4;
                        case 4:
                            target2->status = 0x0f;
                            target2->tVel = 0000;                        //////////////////////////
                            // target2->tTorque=100;
                            break;
                        default:
                            startup_step2 = 1;
                            target2->status = 0x06; //0x6040
                            break;
                        }
                        printf("stat1: %d pos1: %d, stat2: %d pos2: %d  ", val->status, val->Pos, val2->status, val2->Pos);
                        printf("tVel 1: %d, tVel 2: %d", target->tVel, target2->tVel);
                        printf("\r");
 /////////////////////////////////////////////////////////////////////
                        ros::spinOnce();
///////////////////////////////////////////////////////////////////////
                        usleep(1000);
                    }
                    dorun = 0;
                    //            printf("\nCnt %d : Ain0 = %f  Ain2 = %f\n", ainc, ain[0] / ainc, ain[1] / ainc);
                }
                else
                {
                    printf("Not all slaves reached operational state.\n");
                }
            }
            else
            {
                printf("E/BOX not found in slave configuration.\n");
            }
            //  ec_dcsync0(1, FALSE, 8000, 0); // SYNC0 off
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            ec_slave[0].state = EC_STATE_PRE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach state */
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            if ((ec_slavecount >= 1) &&
                (strcmp(ec_slave[1].name, "E/BOX") == 0))
            {
                // restore PDO to standard mode
                // this can only be done is pre-op state
                os = sizeof(ob2);
                ob2 = 0x1600;
                ec_SDOwrite(1, 0x1c12, 01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                os = sizeof(ob2);
                ob2 = 0x1a00;
                ec_SDOwrite(1, 0x1c13, 01, FALSE, os, &ob2, EC_TIMEOUTRXM);
            }
            printf("Streampos %d\n", streampos);
            // output_cvs("stream.txt", streampos);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End E/BOX, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
}

/* RT EtherCAT thread */
void* ecatthread(void *ptr)
{
    struct timespec ts;
    struct timeval tp;
    int ht;
    int i;
    int pcounter = 0;
    int64 cycletime;

    pthread_mutex_lock(&mutex);
    gettimeofday(&tp, NULL);

    /* Convert from timeval to timespec */
    ts.tv_sec = tp.tv_sec;
    ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    while (1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        pthread_cond_timedwait(&cond, &mutex, &ts);
        if (dorun > 0)
        {
            gettimeofday(&tp, NULL);

            ec_send_processdata();

            ec_receive_processdata(EC_TIMEOUTRET);

            cyclecount++;

            //  if((in_EBOX->counter != pcounter) && (streampos < (MAXSTREAM - 1)))
            //  {
            //     // check if we have timing problems in master
            //     // if so, overwrite stream data so it shows up clearly in plots.
            //     if(in_EBOX->counter > (pcounter + 1))
            //     {
            //        for(i = 0 ; i < 50 ; i++)
            //        {
            //           stream1[streampos]   = 20000;
            //           stream2[streampos++] = -20000;
            //        }
            //     }
            //     else
            //     {
            //        for(i = 0 ; i < 50 ; i++)
            //        {
            //           stream1[streampos]   = in_EBOX->stream[i * 2];
            //           stream2[streampos++] = in_EBOX->stream[(i * 2) + 1];
            //        }
            //     }
            //     pcounter = in_EBOX->counter;
            //  }

            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
        }
    }
}

int main(int argc, char *argv[])
{
//////////////////////////////////////////////////////////////////////////////////////////////
    int32 vel1 = 0;
    int32 vel2 = 0; 
    ros::init(argc,argv,"soemTest_node");           //疑惑的点   这里的node应该是真实的node吧？

    printf("start\n");
//////////////////////////////////////////////////////////////////////////////////////////////

    int ctime;
    struct sched_param param;
    int policy = SCHED_OTHER;

    printf("SOEM (Simple Open EtherCAT Master)\nE/BOX test\n");

    memset(&schedp, 0, sizeof(schedp));
    /* do not set priority above 49, otherwise sockets are starved */
    schedp.sched_priority = 30;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    do
    {
      usleep(1000);
    } while (dorun);

    if (argc > 1)
    {
        dorun = 1;
        if (argc > 2)
            ctime = atoi(argv[2]);
        else
            ctime = 1000; // 1ms cycle time
        /* create RT thread */
        // pthread_create(&thread1, NULL, (void *)&ecatthread, (void *)&ctime);
        pthread_create(&thread1, NULL, ecatthread, (void *)&ctime);
        memset(&param, 0, sizeof(param));
        /* give it higher priority */
        param.sched_priority = 40;
        pthread_setschedparam(thread1, policy, &param);

        /* start acyclic part */
        eboxtest(argv[1]);
    }
    else
    {
        printf("Usage: ebox ifname [cycletime]\nifname = eth0 for example\ncycletime in us\n");
    }

    schedp.sched_priority = 0;
    sched_setscheduler(0, SCHED_OTHER, &schedp);

    printf("End program\n");



    return (0);

}


