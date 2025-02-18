// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

static int frame_num = 0;

static unsigned int errors_read = 0;
static unsigned int bytes_sent = 0;
static unsigned int swrite_calls = 0;
static unsigned int actual_bytes_sent = 0;

#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define CTRL_SET 0x03
#define CTRL_UA 0x07
#define CTRL_I1 0x80
#define CTRL_I0 0x00
#define CTRL_RR0 0xAA
#define CTRL_RR1 0xAB
#define CTRL_REJ0 0x54
#define CTRL_REJ1 0x55
#define CTRL_DISC 0x0B
#define ADDR_SX 0x03
#define ADDR_RX 0x01

int TIMEOUT = 5;
int MAX_ALARM_REPEATS = 5;

#define TRIES 10
#define RR_LOST_TRIES 2

#define ESCAPE 0x7D
#define SPECIAL_MASK 0x20
#define LLWRITE_EXTRA_BIT_NUM 8

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarm(0);

    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

enum OPEN_STATE
{
    STATE_START = 0,
    STATE_FLAG_RCV = 1,
    STATE_A_RCV = 2,
    STATE_C_RCV = 3,
    STATE_BCC_OK = 4,
};
#define SHORT_MESSAGE_SIZE 5
const unsigned char SET[] = {FLAG, ADDR_SX, CTRL_SET, ADDR_SX ^ CTRL_SET, FLAG};
const unsigned char UA[] = {FLAG, ADDR_SX, CTRL_UA, ADDR_SX ^ CTRL_UA, FLAG};
const unsigned char RR0[] = {FLAG, ADDR_SX, CTRL_RR0, ADDR_SX ^ CTRL_RR0, FLAG};
const unsigned char RR1[] = {FLAG, ADDR_SX, CTRL_RR1, ADDR_SX ^ CTRL_RR1, FLAG};
const unsigned char REJ0[] = {FLAG, ADDR_SX, CTRL_REJ0, ADDR_SX ^ CTRL_REJ0, FLAG};
const unsigned char REJ1[] = {FLAG, ADDR_SX, CTRL_REJ1, ADDR_SX ^ CTRL_REJ1, FLAG};
const unsigned char DISC[] = {FLAG, ADDR_SX, CTRL_DISC, ADDR_SX ^ CTRL_DISC, FLAG};

static int open_port_called = FALSE;

// int last_was_set=0;

int llopen(LinkLayer connectionParameters)
{
    printf("llopen called\n");
    if (!open_port_called)
    {
        open_port_called = TRUE;
        if (openSerialPort(connectionParameters.serialPort,
                           connectionParameters.baudRate) < 0)
        {
            return -1;
        }

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);
    }
    MAX_ALARM_REPEATS = connectionParameters.nRetransmissions;
    TIMEOUT = connectionParameters.timeout;
    printf("Alarm set!\n");

    frame_num = 0;

    enum OPEN_STATE state = STATE_START;
    int run = TRUE;
    unsigned char buf, expected_address_flag, expected_code;

    if (connectionParameters.role == LlTx)
    {
        expected_address_flag = ADDR_SX;
        expected_code = CTRL_UA;
    }
    else
    {
        expected_address_flag = ADDR_SX;
        expected_code = CTRL_SET;
    }

    int has_received_bytes = 0;

    alarmCount = 0;
    while (run)
    {
        if (alarmEnabled == FALSE)
        {
            alarmEnabled = TRUE;
            if (connectionParameters.role == LlTx && !has_received_bytes)
            {
                writeBytesSerialPort(SET, SHORT_MESSAGE_SIZE);

                printf("Wrote set message!\n");
            }
            alarm(TIMEOUT);
        }
        if (alarmCount == MAX_ALARM_REPEATS)
        {
            run = FALSE;
        }
        int bytes = readByteSerialPort(&buf);
        if (bytes == 1)
        {
            has_received_bytes = 1;
            alarmCount = 0;
            switch (state)
            {
            case STATE_START:

                if (buf == FLAG)
                {
                    state = STATE_FLAG_RCV;

                    printf("read flag\n");
                }
                break;

            case STATE_FLAG_RCV:
                if (buf == FLAG)
                {
                    break;
                }
                if (buf == expected_address_flag)
                {
                    state = STATE_A_RCV;
                    printf("read address\n");
                    break;
                }
                state = STATE_START;
                break;

            case STATE_A_RCV:
                if (buf == FLAG)
                {
                    state = STATE_FLAG_RCV;
                    break;
                }
                if (buf == expected_code)
                {
                    state = STATE_C_RCV;
                    printf("read code\n");
                    break;
                }
                state = STATE_START;
                break;

            case STATE_C_RCV:
                if (buf == FLAG)
                {
                    state = STATE_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ expected_code))
                {
                    state = STATE_BCC_OK;
                    printf("read error correction\n");
                    break;
                }
                state = STATE_START;
                break;

            case STATE_BCC_OK:
                if (buf == FLAG)
                {
                    printf("read final flag\n");
                    alarm(0);
                    alarmEnabled = FALSE;
                    if (connectionParameters.role == LlRx)
                    {
                        return writeBytesSerialPort(UA, SHORT_MESSAGE_SIZE) > 0 ? 1 : -1;
                    }
                    else
                    {
                        return 1;
                    }
                }
                else
                {
                    state = STATE_START;
                }
                break;
            }
        }
        else if (bytes == -1)
        {
            return -1;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

enum WRITE_STATE
{
    STATE_WRITE_START = 0,
    STATE_WRITE_FLAG_RCV = 1,
    STATE_WRITE_A_RCV = 2,
    STATE_WRITE_C_RCV = 3,
    STATE_WRITE_BCC_CORRECT = 4,
    STATE_WRITE_REPEAT_UA_RECEIVED = 5,
    STATE_WRITE_REPEAT_UA_BCC_CORRECT = 6
};

int llwrite(const unsigned char *buf, int bufSize)
{
    printf("frame ordering: %d\n", frame_num ? 1 : 0);
    alarmCount = 0;

    unsigned char to_send[bufSize + LLWRITE_EXTRA_BIT_NUM];
    to_send[0] = FLAG;
    to_send[1] = ADDR_SX;
    if (frame_num == 0)
    {
        to_send[2] = CTRL_I0;
    }
    else
    {
        to_send[2] = CTRL_I1;
    }
    to_send[3] = to_send[1] ^ to_send[2];
    int num_bytes = 4;

    unsigned char bcc2 = 0;
    for (size_t i = 0; i < bufSize; i++)
    {
        printf("byte: 0x%2x\n", buf[i]);
        bcc2 ^= buf[i];
        if (buf[i] == ESCAPE || buf[i] == FLAG)
        {
            to_send[num_bytes] = ESCAPE;
            num_bytes++;
            to_send[num_bytes] = buf[i] ^ SPECIAL_MASK;
            num_bytes++;
        }
        else
        {
            to_send[num_bytes] = buf[i];
            num_bytes++;
        }
    }

    printf("bcc2: %d (0x%2x)\n", bcc2, bcc2);
    if (bcc2 == ESCAPE || bcc2 == FLAG)
    {
        to_send[num_bytes] = ESCAPE;
        num_bytes++;
        to_send[num_bytes] = bcc2 ^ SPECIAL_MASK;
        printf("bcc2 transformed: %d\n", bcc2 ^ SPECIAL_MASK);
        num_bytes++;
    }
    else
    {
        to_send[num_bytes] = bcc2;
        num_bytes++;
    }
    to_send[num_bytes] = FLAG;
    num_bytes++;

    bytes_sent += num_bytes;

    enum WRITE_STATE state = STATE_WRITE_START;
    int run = TRUE;
    unsigned char bt, code, num_tries = 0, rrLostTries = 0;

    while (run)
    {
        if (alarmEnabled == FALSE)
        {
            alarmEnabled = TRUE;
            swrite_calls++;
            if (writeBytesSerialPort(to_send, num_bytes) == -1)
            {
                return -1;
            }
            printf("sent message\n");
            actual_bytes_sent += num_bytes;
            alarm(TIMEOUT);
        }
        if (alarmCount >= MAX_ALARM_REPEATS)
        {
            run = FALSE;
        }

        int bytes = readByteSerialPort(&bt);
        if (bytes == 1)
        {
            alarmCount = 0;

            switch (state)
            {
            case STATE_WRITE_START:

                if (bt == FLAG)
                {
                    state = STATE_WRITE_FLAG_RCV;
                    printf("received flag (0x%2x)\n", bt);
                    break;
                }
                printf("Should have been flag\n");
                break;

            case STATE_WRITE_FLAG_RCV:
                if (bt == FLAG)
                {
                    printf("found flag instead of address\n");
                    break;
                }
                if (bt == ADDR_SX)
                {
                    state = STATE_WRITE_A_RCV;
                    printf("read address\n");
                    break;
                }
                printf("wrong address: 0x%2x\n", bt);
                state = STATE_WRITE_START;
                break;

            case STATE_WRITE_A_RCV:
                if (bt == FLAG)
                {
                    printf("found flag instead of command\n");
                    state = STATE_WRITE_FLAG_RCV;
                    break;
                }

                code = bt;
                if (code == CTRL_UA)
                {
                    state = STATE_WRITE_REPEAT_UA_RECEIVED;
                    printf("Random UA received!\n");
                    break;
                }

                if (frame_num == 0)
                {
                    if ((code == CTRL_REJ0) || (code == CTRL_RR1))
                    {
                        printf("Read correct command(0x%2x)\n", code);
                        state = STATE_WRITE_C_RCV;
                        break;
                    }
                    else if ((code == CTRL_REJ1) || (code == CTRL_RR0))
                    {
                        printf("Command read: out of sync!!\n");
                        if (num_tries < TRIES)
                        {
                            if (rrLostTries > RR_LOST_TRIES && code == CTRL_REJ1)
                            {
                                printf("Assumed rr lost, moving to next frame\n");
                                frame_num = !frame_num;
                                return -2;
                            }
                            printf("Retrying rr reception\n");
                            state = STATE_WRITE_START;
                            rrLostTries++;
                            break;
                        }
                        else
                        {
                            printf("Skipping to next frame\n");
                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;
                            return -2;
                        }
                    }
                    else
                    {
                        printf("didn't read the correct command:  0x%2x\n", code);
                        if (rrLostTries < TRIES)
                        {
                            printf("Retrying rr reception\n");
                            state = STATE_WRITE_START;
                            rrLostTries++;
                            break;
                        }
                        else
                        {
                            printf("Serious error - exiting the program\n\n");
                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;
                            return -3;
                        }
                    }
                }
                else
                {

                    if ((code == CTRL_REJ1) || (code == CTRL_RR0))
                    {
                        printf("Read correct command(0x%2x)\n", code);
                        state = STATE_WRITE_C_RCV;
                        break;
                    }
                    else if ((code == CTRL_REJ0) || (code == CTRL_RR1))
                    {
                        printf("Command read: out of sync!!\n");
                        if (rrLostTries < TRIES)
                        {
                            if (rrLostTries > RR_LOST_TRIES && code == CTRL_REJ0)
                            {
                                printf("Assumed rr lost, moving to next frame\n");
                                frame_num = !frame_num;
                                return -2;
                            }
                            printf("Retrying rr reception\n");
                            state = STATE_WRITE_START;
                            rrLostTries++;
                            break;
                        }
                        else
                        {
                            printf("Skipping to next frame\n");
                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;
                            return -2;
                        }
                    }
                    else
                    {

                        printf("didn't read the correct command:  0x%2x\n", code);
                        if (rrLostTries < TRIES)
                        {
                            printf("Retrying rr reception\n");
                            state = STATE_WRITE_START;
                            rrLostTries++;
                            break;
                        }
                        else
                        {
                            printf("Serious error - exiting the program\n\n");
                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;
                            return -3;
                        }
                    }
                }

                state = STATE_WRITE_START;
                break;

            case STATE_WRITE_C_RCV:
                if (bt == FLAG)
                {
                    printf("found flag instead of bcc\n");
                    state = STATE_WRITE_FLAG_RCV;
                    break;
                }
                if (bt == (ADDR_SX ^ code))
                {
                    state = STATE_WRITE_BCC_CORRECT;
                    printf("read correct bcc\n");
                    break;
                }
                printf("problem in error correction: %d read, %d expected\n", bt, ADDR_SX ^ code);
                state = STATE_WRITE_START;
                break;

            case STATE_WRITE_REPEAT_UA_RECEIVED:
                if (bt == FLAG)
                {
                    state = STATE_WRITE_FLAG_RCV;
                    break;
                }
                if (bt == (ADDR_SX ^ CTRL_UA))
                {
                    state = STATE_WRITE_BCC_CORRECT;
                    printf("read error correction (repeat ua)\n");
                    break;
                }
                state = STATE_WRITE_START;
                break;
            case STATE_WRITE_REPEAT_UA_BCC_CORRECT:
                state = STATE_WRITE_START;
                break;
            case STATE_WRITE_BCC_CORRECT:
                if (bt == FLAG)
                {
                    printf("read final flag\n");

                    if (frame_num == 0)
                    {
                        if (code == CTRL_REJ0)
                        {
                            printf("resend 0\n");
                            errors_read += 1;
                            state = STATE_WRITE_START;
                            alarmCount = 0;
                        }
                        else if (code == CTRL_RR1)
                        {

                            printf("send next 1\n\n");
                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;

                            return num_bytes;
                        }
                        printf("shouldn't happen without resend 0\n");
                        // WHAT HAPPENS IF RRO?
                    }
                    else
                    {
                        if (code == CTRL_REJ1)
                        {
                            printf("resend 1\n");
                            errors_read += 1;
                            state = STATE_WRITE_START;
                            alarmCount = 0;
                        }
                        else if (code == CTRL_RR0)
                        {

                            frame_num = !frame_num;
                            alarm(0);
                            alarmEnabled = FALSE;
                            printf("send next 0\n\n");

                            return num_bytes;
                        }
                        printf("shouldn't happen without resend 1\n");
                        // WHAT HAPPENS IF RR1?
                    }
                }
                else
                {
                    printf("Didn't read final flag - resetting\n");
                    state = STATE_WRITE_START;
                }
                break;
            }
        }
        else if (bytes == -1)
        {
            return -1;
        }
    }
    printf("write timeout\n");
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
enum RTERM_STATE
{
    RTERM_STATE_START = 0,
    RTERM_STATE_FLAG_RCV = 1,
    RTERM_STATE_A_RCV = 2,
    RTERM_STATE_C_RCV = 3,
    RTERM_STATE_BCC_OK = 4,
};

int terminate_reader()
{

    if (writeBytesSerialPort(DISC, SHORT_MESSAGE_SIZE) == -1)
    {

        return -1;
    }

    frame_num = 0;

    enum RTERM_STATE state = RTERM_STATE_START;
    int run = TRUE;
    unsigned char buf, expected_address_flag = ADDR_SX, expected_code = CTRL_UA;

    while (run)
    {
        if (alarmEnabled == FALSE)
        {
            alarmEnabled = TRUE;
            alarm(TIMEOUT);
        }
        if (alarmCount == MAX_ALARM_REPEATS)
        {
            run = FALSE;
        }
        int bytes = readByteSerialPort(&buf);
        if (bytes == 1)
        {
            alarmCount = 0;
            switch (state)
            {
            case RTERM_STATE_START:
                if (buf == FLAG)
                {
                    state = RTERM_STATE_FLAG_RCV;

                    printf("term read flag\n");
                    break;
                }
                printf("Should have been flag\n");
                break;

            case RTERM_STATE_FLAG_RCV:
                if (buf == FLAG)
                {
                    break;
                }
                if (buf == expected_address_flag)
                {
                    state = RTERM_STATE_A_RCV;
                    printf("term read address\n");
                    break;
                }
                state = RTERM_STATE_START;
                break;

            case RTERM_STATE_A_RCV:
                if (buf == FLAG)
                {
                    state = RTERM_STATE_FLAG_RCV;
                    break;
                }
                if (buf == expected_code)
                {
                    state = RTERM_STATE_C_RCV;
                    printf("term read code\n");
                    break;
                }
                state = RTERM_STATE_START;
                break;

            case RTERM_STATE_C_RCV:
                if (buf == FLAG)
                {
                    state = RTERM_STATE_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ expected_code))
                {
                    state = RTERM_STATE_BCC_OK;
                    printf("term read error correction\n");
                    break;
                }
                state = RTERM_STATE_START;
                break;

            case RTERM_STATE_BCC_OK:
                if (buf == FLAG)
                {
                    printf("term read final flag\n");
                    ;
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    open_port_called = FALSE;
                    closeSerialPort();
                    return 1;
                }
                else
                {
                    state = RTERM_STATE_START;
                }
                break;
            }
        }
        else if (bytes == -1)
        {
            return -1;
        }
    }

    return -1;
}

enum READ_STATE
{
    STATE_READ_START = 0,
    STATE_READ_FLAG_RCV = 1,
    STATE_READ_A_RCV = 2,
    STATE_READ_C_RCV = 3,
    STATE_READ_DATA = 4,
    STATE_READ_ESCAPED = 5,
    STATE_READ_DISC = 6,
    STATE_READ_DISC_BCC_OK = 7,
    STATE_READ_SET = 8,
    STATE_READ_SET_BCC_OK = 9
};

int data_is_correct(unsigned char *data, unsigned int data_length, unsigned char bbc2)
{
    unsigned char res = 0;
    for (size_t i = 0; i < data_length; i++)
    {
        res ^= data[i];
    }
    printf("bcc2 created: %d, bcc2 received with message:%d\n", res, bbc2);
    return res == bbc2;
}

int llread(unsigned char *packet) // buffer already instantiated
{
    printf("frame ordering: %d\n", frame_num ? 1 : 0);

    alarmCount = 0;
    enum READ_STATE state = 0;
    int run = TRUE;

    unsigned char buf, expected_address_flag = ADDR_SX, expected_code, expected_rej, expected_rr, out_of_order_frame_code, received_code, attemptCount = 0;
    unsigned int current_data_index = 0;

    if (frame_num == 0)
    {
        expected_code = CTRL_I0;
        out_of_order_frame_code = CTRL_I1;
        expected_rej = CTRL_REJ0;
        expected_rr = CTRL_RR1;
        received_code = expected_code;
    }
    else
    {
        expected_code = CTRL_I1;
        received_code = expected_code;
        out_of_order_frame_code = CTRL_I0;
        expected_rej = CTRL_REJ1;
        expected_rr = CTRL_RR0;
    }

    while (run)
    {
        if (alarmEnabled == FALSE)
        {
            alarmEnabled = TRUE;
            alarm(TIMEOUT);
        }
        if (alarmCount >= MAX_ALARM_REPEATS)
        {
            run = FALSE;
        }
        int bytes = readByteSerialPort(&buf);
        if (bytes == 1)
        {
            alarmCount = 0;

            if (current_data_index > MAX_PAYLOAD_SIZE)
            {
                printf("Overflow danger: end flag not found for too long!!!\n");
                return -1;
            }
            switch (state)
            {
            case STATE_READ_START:

                if (buf == FLAG)
                {
                    state = STATE_READ_FLAG_RCV;

                    printf("read flag 1\n");
                }
                break;

            case STATE_READ_FLAG_RCV:
                if (buf == FLAG)
                {
                    printf("found flag instead of address\n");
                    break;
                }
                if (buf == expected_address_flag)
                {
                    state = STATE_READ_A_RCV;
                    printf("read address\n");
                    break;
                }
                printf("read wrong address:  0x%2x\n", buf);
                state = STATE_READ_START;
                break;

            case STATE_READ_A_RCV:
                if (buf == FLAG)
                {
                    state = STATE_READ_FLAG_RCV;
                    printf("found flag instead of code\n");
                    break;
                }
                if (buf == CTRL_SET)
                {
                    received_code = CTRL_SET;
                    state = STATE_READ_SET;
                    printf("read set code\n");
                    break;
                }
                if (buf == CTRL_DISC)
                {
                    received_code = CTRL_DISC;
                    state = STATE_READ_DISC;
                    printf("read disc code\n");
                    break;
                }
                if (buf == expected_code)
                {
                    received_code = expected_code;
                    state = STATE_READ_C_RCV;
                    printf("read frame code\n");
                    break;
                }
                if (buf == out_of_order_frame_code)
                {
                    received_code = out_of_order_frame_code;
                    state = STATE_READ_C_RCV;
                    printf("read out of order frame code\n");
                    break;
                }
                printf("Read wrong code: 0x%2x\n", buf);
                received_code = buf;
                state = STATE_READ_C_RCV; // TODO: I don't like this, but it has to be, just in case it's a data frame. I don't want "arbitrary code execution" here at all
                break;
            case STATE_READ_SET:

                if (buf == FLAG)
                {

                    printf("read flag instead of bcc!!!\n");

                    state = STATE_READ_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ received_code))
                {
                    state = STATE_READ_SET_BCC_OK;
                    printf("read set bbc ok\n");
                    break;
                }
                printf("(set) bcc wrong\n");
                state = STATE_READ_DATA;
                break;
            case STATE_READ_SET_BCC_OK:
                if (buf == FLAG)
                {
                    frame_num = 0; //?Because Reset?
                    alarm(0);
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    writeBytesSerialPort(UA, SHORT_MESSAGE_SIZE);
                    printf("Had to send another UA!");
                    return -4; // also not a documented return value, but could be useful
                }
                printf("Didn't find the final flag of a set command!");

                state = STATE_READ_DATA; // TODO: I hate that I have to do this, but I have no alternative. Many errors could happen if I didn't
                break;

            case STATE_READ_DISC:

                if (buf == FLAG)
                {
                    state = STATE_READ_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ received_code))
                {
                    state = STATE_READ_DISC_BCC_OK;
                    printf("bbc ok\n");
                    break;
                }
                state = STATE_READ_START;
                break;

            case STATE_READ_DISC_BCC_OK:
                if (buf == FLAG)
                {
                    printf("terminating reader function called!\n");
                    return (terminate_reader() == 1) ? -2 : -3;
                }
                state = STATE_READ_START;

                break;

            case STATE_READ_C_RCV:
                if (buf == FLAG)
                {
                    printf("Found flag instead of bcc\n");
                    state = STATE_READ_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ received_code))
                {

                    state = STATE_READ_DATA;

                    printf("bcc correct\n");
                    break;
                }
                printf("bcc incorrect\n");
                if (received_code != CTRL_DISC && received_code != CTRL_SET)
                {
                    printf("Assuming data frame -> must not be induced in error due to possible frame content\n");
                    state = STATE_READ_DATA; // will this fix the problems?
                    break;
                }
                state = STATE_READ_START;
                break;

            case STATE_READ_DATA:
                if (buf == FLAG)
                {
                    printf("read final flag\n");

                    if (received_code == out_of_order_frame_code)
                    {
                        printf("out of order frame received\n");
                        current_data_index = 0;
                        int rs;
                        if (expected_rej == CTRL_REJ0)
                        {
                            printf("rej0\n");
                            rs = writeBytesSerialPort(REJ0, SHORT_MESSAGE_SIZE);
                        }
                        else
                        {
                            printf("rej1\n");
                            rs = writeBytesSerialPort(REJ1, SHORT_MESSAGE_SIZE);
                        }
                        // TODO: maybe don't send this?
                        if (rs == -1)
                        {
                            printf("Error sending REJ\n");
                            if (attemptCount < TRIES)
                            {
                                attemptCount++;
                                printf("Retrying reception\n");
                                state = STATE_READ_START;
                                break;
                            }
                            else
                            {
                                alarm(0);
                                alarmEnabled = FALSE;

                                printf("Irrecoverable send REJ error\n\n");

                                return -1;
                            }
                        }

                        if (attemptCount < TRIES)
                        {
                            attemptCount++;
                            printf("Retrying reception\n");
                            state = STATE_READ_START;
                            break;
                        }
                        else
                        {
                            alarm(0);
                            alarmEnabled = FALSE;

                            printf("Irrecoverable sync error\n\n");

                            return -1;
                        }
                    }

                    if (received_code != expected_code)
                    {
                        current_data_index = 0;

                        if (attemptCount < TRIES)
                        {
                            attemptCount++;
                            printf("Retrying reception due to error control\n");
                            state = STATE_READ_START;
                            break;
                        }
                        else
                        {
                            alarm(0);
                            alarmEnabled = FALSE;

                            printf("Irrecoverable command related error\n\n");

                            return -1;
                        }
                    }

                    current_data_index--;
                    if (data_is_correct(packet, current_data_index, packet[current_data_index]))
                    {
                        printf("data received!\n");
                        int res;
                        if (expected_rr == CTRL_RR0)
                        {
                            res = writeBytesSerialPort(RR0, SHORT_MESSAGE_SIZE);

                            printf("sent rr0\n");
                        }
                        else
                        {
                            res = writeBytesSerialPort(RR1, SHORT_MESSAGE_SIZE);
                            printf("sent rr1\n");
                        }
                        if (res != -1)
                        {
                            alarm(0);
                            alarmEnabled = FALSE;
                            frame_num = !frame_num;
                            return current_data_index;
                        }
                        alarm(0);
                        alarmEnabled = FALSE;
                        printf("error in sending rr");
                        return -1;
                    }
                    else
                    {
                        printf("bbc2 incorrect!\n");
                        if (expected_rej == CTRL_REJ0)
                        {
                            writeBytesSerialPort(REJ0, SHORT_MESSAGE_SIZE);
                        }
                        else
                        {
                            writeBytesSerialPort(REJ1, SHORT_MESSAGE_SIZE);
                        }

                        if (attemptCount < TRIES)
                        {
                            attemptCount++;
                            current_data_index = 0;
                            state = STATE_READ_START;
                            printf("--> Retrying reception: %d/%d\n", attemptCount, TRIES);
                            break;
                        }
                        else
                        {
                            attemptCount += 1;
                            alarm(0);
                            alarmEnabled = FALSE;

                            return -1;
                        }
                    }
                }
                else if (buf == ESCAPE)
                {
                    printf("escaped\n");
                    state = STATE_READ_ESCAPED;
                }
                else
                {
                    packet[current_data_index] = buf;
                    current_data_index++;
                    printf("byte: 0x%2x\n", packet[current_data_index - 1]);
                }
                break;

            case STATE_READ_ESCAPED:
                packet[current_data_index] = buf ^ SPECIAL_MASK;
                printf("byte: 0x%2x\n", packet[current_data_index]);
                current_data_index++;
                state = STATE_READ_DATA;
                break;
            default:
                printf("I quit,\n");
                return -1;
                break;
            }
        }
        else if (bytes == -1)
        {
            return -1;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
enum CLOSE_STATE
{
    CLOSE_STATE_START = 0,
    CLOSE_STATE_FLAG_RCV = 1,
    CLOSE_STATE_A_RCV = 2,
    CLOSE_STATE_C_RCV = 3,
    CLOSE_STATE_BCC_OK = 4,
};
int llclose(int showStatistics)
{

    if (showStatistics == TRUE)
    {
        printf("Wrote %u unique bytes(%u counting repeated sends), with %u calls to llwrite (repeated frames included).\n", bytes_sent,
               actual_bytes_sent, swrite_calls);
    }
    enum CLOSE_STATE state = CLOSE_STATE_START;
    int run = TRUE;

    unsigned char buf, expected_address_flag = ADDR_SX, expected_code = CTRL_DISC;
    alarmCount = 0;
    while (run)
    {
        if (alarmEnabled == FALSE)
        {
            printf("wrote disc\n");
            alarmEnabled = TRUE;
            if (writeBytesSerialPort(DISC, SHORT_MESSAGE_SIZE) == -1)
            {

                return -1;
            }
            alarm(TIMEOUT);
        }
        if (alarmCount == MAX_ALARM_REPEATS)
        {
            run = FALSE;
        }
        int bytes = readByteSerialPort(&buf);
        if (bytes == 1)
        {

            alarmCount = 0;
            switch (state)
            {
            case CLOSE_STATE_START:

                if (buf == FLAG)
                {
                    state = CLOSE_STATE_FLAG_RCV;

                    printf("read flag\n");
                }
                break;

            case CLOSE_STATE_FLAG_RCV:
                if (buf == FLAG)
                {
                    break;
                }
                if (buf == expected_address_flag)
                {
                    state = CLOSE_STATE_A_RCV;
                    printf("read address\n");
                    break;
                }
                state = CLOSE_STATE_START;
                break;

            case CLOSE_STATE_A_RCV:
                if (buf == FLAG)
                {
                    state = CLOSE_STATE_FLAG_RCV;
                    break;
                }
                if (buf == expected_code)
                {
                    state = CLOSE_STATE_C_RCV;
                    printf("read code\n");
                    break;
                }
                state = CLOSE_STATE_START;
                break;

            case CLOSE_STATE_C_RCV:
                if (buf == FLAG)
                {
                    state = CLOSE_STATE_FLAG_RCV;
                    break;
                }
                if (buf == (expected_address_flag ^ expected_code))
                {
                    state = CLOSE_STATE_BCC_OK;
                    printf("read error correction\n");
                    break;
                }
                state = CLOSE_STATE_START;
                break;

            case CLOSE_STATE_BCC_OK:
                if (buf == FLAG)
                {
                    printf("read final flag\n");

                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    writeBytesSerialPort(UA, SHORT_MESSAGE_SIZE);
                    writeBytesSerialPort(UA, SHORT_MESSAGE_SIZE); // just in case the first isn't read, so that the receive doesn't terminate with errors :/
                    writeBytesSerialPort(UA, SHORT_MESSAGE_SIZE);
                    run = 0;
                    open_port_called = FALSE;
                    int clstat = closeSerialPort();
                    return clstat != -1 ? 1 : -1;
                }
                else
                {
                    state = CLOSE_STATE_START;
                }
                break;
            }
        }
        else if (bytes == -1)
        {
            return -1;
        }
    }

    return -1;
}
