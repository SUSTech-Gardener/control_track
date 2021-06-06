/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int opMode = 0;

typedef struct PACKED
{
    uint16 status;
    uint32 tPos;
    uint32 velocityOffset;
    uint16 torquteOffset;
    int32 tVel;
    uint16 tTorque;
    uint8 opMode;
    uint8 complement;
} out_ISMC;

typedef struct PACKED
{
    uint16 status;
    int32 Pos;
    int32 vel;
    uint16 tTorque;
    uint8 opMode;
    uint8 complement;
} in_ISMC;

in_ISMC *val;
out_ISMC *target;

in_ISMC *val2;
out_ISMC *target2;

#define READ(slaveId, idx, sub, buf, comment)                                                                          \
    {                                                                                                                  \
        buf = 0;                                                                                                       \
        int __s = sizeof(buf);                                                                                         \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s,   \
               (unsigned int)buf, (unsigned int)buf, comment);                                                         \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                  \
    {                                                                                                                  \
        int __s = sizeof(buf);                                                                                         \
        buf = value;                                                                                                   \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                   \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s,       \
               (unsigned int)buf, comment);                                                                            \
    }

#define CHECKERROR(slaveId)                                                                                            \
    {                                                                                                                  \
        ec_readstate();                                                                                                \
        printf("EC> \"%s\" %x - %x [%s] \n", (char *)ec_elist2string(), ec_slave[slaveId].state,                       \
               ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));        \
    }
static int slave_dc_config(uint16 slave)
{

    // ec_dcsync0(1, TRUE, 1000 * 1000, 0 * 1000);
    ec_dcsync01(slave, TRUE, 1000 * 1000, 000 * 1000, 0); // SYNC0/1 on slave 1

    printf("ec_dcsync0 called on slave %u\n", slave);
    return 0;
}

void redtest(char *ifname, char *ifname2)
{
    int cnt, i, j, oloop, iloop;
    opMode = 0;
    printf("Starting Redundant test\n");

    /* initialise SOEM, bind socket to ifname */
    //   if (ec_init_redundant(ifname, ifname2))
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0) // ec_config(FALSE, &IOmap) > 0
        {
            printf("%d slaves found and configured.\n", ec_slavecount);
            /* wait for all slaves to reach SAFE_OP state */
            // ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

            /* configure DC options for every DC capable slave found in the list */
            ec_configdc();
            // dorun = 1;
            // usleep(10000);
            // ec_dcsync01(1, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on slave
            // 1 ec_dcsync01(2, TRUE, 1000 * 1000, 1000 * 1000, 0); // SYNC0/1 on
            // slave 2

            ec_slave[1].PO2SOconfig = &slave_dc_config;
            ec_slave[2].PO2SOconfig = &slave_dc_config;

            ec_config_map(&IOmap);

            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits "
                       "State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits, ec_slave[cnt].state,
                       (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* activate cyclic process data */
            dorun = 1;
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");

                target = (out_ISMC *)(ec_slave[1].outputs);
                val = (in_ISMC *)(ec_slave[1].inputs);
                target2 = (out_ISMC *)(ec_slave[2].outputs);
                val2 = (in_ISMC *)(ec_slave[2].inputs);
                target->opMode = 9;
                target2->opMode = 9;

                opMode = 1;
                inOP = TRUE;

                /* acyclic loop 5000 x 20ms = 10s */
                for (i = 1;; i++) // i <= 50000
                {
                    printf("Processdata cycle %5d , Wck %3d, DCtime %12lld, dt %12lld, O:", dorun, wkc, ec_DCtime,
                           gl_delta);
                    for (j = 0; j < 16; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }
                    printf(" I:");

                    /////////////////////////////////////////

                    printf("stat1: %d pos1: %d, stat2: %d pos2: %d  ", val->status, val->Pos, val2->status, val2->Pos);
                    printf("tVel 1: %d, tVel 2: %d", target->tVel, target2->tVel);
                    // printf("\r");

                    /////////////////////////////////////////////////////
                    printf("\r");
                    fflush(stdout);
                    osal_usleep(1000);
                }
                dorun = 0;
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state,
                               ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End redundant test, close socket\n");
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
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
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
    gl_delta = delta;
}

/* RT EtherCAT thread */
void *ecatthread(void *ptr) // OSAL_THREAD_FUNC_RT
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    opMode = 0;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    ec_send_processdata();
    uint16_t command = 0x004F;
    uint16_t command2 = 0x004F;
    int i = 0;
    while (1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            dorun++;
            /* if we have some digital output, cycle */
            // if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);
            if (inOP)
            {
                i++;
                target->opMode = 9;
                target2->opMode = 9;

                uint16 cur_status1 = val->status; // 0x6041
                if ((cur_status1 & command) == 0x0040)
                {
                    target->status = 0x06; // 0x6040
                    command = 0x006F;
                }

                else if ((cur_status1 & command) == 0x0021)
                {
                    target->status = 0x07;
                    command = 0x006F;
                }

                else if ((cur_status1 & command) == 0x0023)
                {
                    target->status = 0x0f;
                    command = 0x006F;
                }

                else if ((cur_status1 & command) == 0x0027)
                {
                    target->status = 0x1f;
                    target->tVel = -(int32)(sin(i / 500.) * (1000000));
                }

                uint16 cur_status2 = val2->status; // 0x6041

                if ((cur_status2 & command2) == 0x0040)
                {
                    target2->status = 0x06; // 0x6040

                    // set control mode
                    command2 = 0x006F;
                }

                else if ((cur_status2 & command2) == 0x0021)
                {
                    target2->status = 0x07;
                    command2 = 0x006F;
                }

                else if ((cur_status2 & command2) == 0x0023)
                {
                    target2->status = 0x0f;
                    command2 = 0x006F;
                }

                else if ((cur_status2 & command2) == 0x0027)
                {
                    target2->status = 0x1f;
                    target2->tVel = -(int32)(sin(i / 500.) * (1000000));
                }
            }
            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
        }
    }
}

void *ecatcheck(void *ptr) // OSAL_THREAD_FUNC
{
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
    int ctime;

    printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

    if (argc > 3)
    {
        dorun = 0;
        ctime = atoi(argv[3]);

        /* create RT thread */
        // osal_thread_create_rt(&thread1, stack64k * 2, ecatthread, (void
        // *)&ctime);
        // pthread_create(&thread1, NULL, ecatthread, (void *)&ctime);

        int ret;
        pthread_attr_t attr;
        struct sched_param schparam;
        //    pthread_t            *threadp;

        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, stack64k * 2);
        ret = pthread_create(&thread1, &attr, ecatthread, (void *)&ctime);
        pthread_attr_destroy(&attr);
        if (ret < 0)
        {
            return 0;
        }
        memset(&schparam, 0, sizeof(schparam));
        schparam.sched_priority = 99;
        ret = pthread_setschedparam(thread1, SCHED_FIFO, &schparam);
        if (ret < 0)
        {
            return 0;
        }

        // /* create thread to handle slave error handling in OP */
        // osal_thread_create(&thread2, stack64k * 4, ecatcheck, NULL);
        pthread_create(&thread2, NULL, ecatcheck, NULL);
        /* start acyclic part */
        redtest(argv[1], argv[2]);
    }
    else
    {
        printf("Usage: red_test ifname1 ifname2 cycletime\nifname = eth0 for "
               "example\ncycletime in us\n");
    }

    printf("End program\n");

    return (0);
}
