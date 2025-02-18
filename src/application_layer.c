// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define MIN(x,y) ((x)<(y)?(x):(y))

#define SLEEP_AMOUNT 0
#define SEND_BUFFER_SIZE 16
// do not change the num header bytes should be 4
#define NUM_HEADER_BYTES 4
#define RECEIVE_BUFFER_SIZE 2048
unsigned char buf[SEND_BUFFER_SIZE];
unsigned char receivedbuf[RECEIVE_BUFFER_SIZE]; // could be more
int bytes;
int bufindex = 0;
int S = 0; // number of current packet
FILE *fptr;
long filesize;

typedef struct
{
    unsigned char *pointer;
    int size;
} PointerIntPair;

void readFileSize()
{
    fseek(fptr, 0, SEEK_END);
    filesize = ftell(fptr);
    fseek(fptr, 0, SEEK_SET);
}

void splitFile()
{
    bytes = fread(buf, 1, SEND_BUFFER_SIZE, fptr);
}

PointerIntPair createDataPacket() // returns data packet
{
    S++;
    printf("packet number: %d (as a byte:%d)\n", S, S % 256);
    unsigned char *datapacket = (unsigned char *)malloc((bytes + NUM_HEADER_BYTES) * sizeof(unsigned char));
    datapacket[0] = 2;
    datapacket[1] = S;
    datapacket[2] = bytes / 256; // L2
    datapacket[3] = bytes % 256; // L3

    for (int i = 0; i < bytes; i++)
    {
        datapacket[4 + i] = buf[i];
    }

    PointerIntPair result;
    result.pointer = datapacket;
    result.size = 4 + bytes;
    return result;
}

PointerIntPair createControlPacket(int option, const char *filename) // option is 0 for start packet 1 for end packet
{
    unsigned char lenfilename = (unsigned char)strlen(filename);
    unsigned char *controlpacket = (unsigned char *)malloc((13 + lenfilename) * sizeof(unsigned char));
    if (option == 0) // start control packet
    {
        controlpacket[0] = 1;
    }
    else
        controlpacket[0] = 3;

    controlpacket[1] = 0; // file size
    controlpacket[2] = 8; // bytes of length (filesize)
    int tmp = 56;
    for (int i = 0; i < 8; i++)
    {
        controlpacket[i + 3] = (filesize >> tmp) & 0xFF;
        tmp -= 8;
    }
    controlpacket[11] = 1; // file name
    controlpacket[12] = lenfilename;
    memcpy(&controlpacket[13], filename, lenfilename);

    PointerIntPair result;
    result.pointer = controlpacket;
    result.size = 13 + lenfilename;

    return result;
}

int parsePacket(unsigned char *packet, int size)
{
    const static int TOO_SHORT_MAX_NUMBER = 4;
    static int too_short_counter = FALSE;
    static unsigned char lastPacketValue = 0;
    if (packet[0] == 1)
    {
        return 1;
    }
    else if (packet[0] == 2)
    {
        unsigned char current_packet = packet[1];
        if (size > SEND_BUFFER_SIZE + NUM_HEADER_BYTES)
        {
            printf("Too large packet received(size=%u)\n",size);
            return -2;
        }

        if ((unsigned char)current_packet != (unsigned char)(lastPacketValue + 1))
        {
            printf("Out of order(previous %u vs current %u) \n", lastPacketValue, current_packet);
            return -1;
        }

        // TODO: remove, maybe!!!
        if (size < SEND_BUFFER_SIZE + NUM_HEADER_BYTES)
        {
            if (too_short_counter < TOO_SHORT_MAX_NUMBER)
            {
                too_short_counter += 1;
                printf("Too short frame received!!!\n");
            }
            else
            {
                printf("Too many short frames in a row! This doesn't make sense.\n\n");
                exit(-1);
            }
        }
        else
        {
            too_short_counter = 0;
        }

        lastPacketValue = current_packet;

        int L1 = packet[3];
        int L2 = packet[2];
        fwrite(&packet[4], 1, MIN(L2 * 256 + L1,SEND_BUFFER_SIZE), fptr);
        printf("packet number: %u \n\n", packet[1]);
        return 2;
    }
    else if (packet[0] == 3)
        return 3;

    return 4;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    if (strcmp(role, "tx") == 0)
        connectionParameters.role = LlTx;
    else if (strcmp(role, "rx") == 0)
        connectionParameters.role = LlRx;

    // TODO: for safety, check if end info packet has the same information as the start info packet

    printf("llopen try loop called\n");

    if (llopen(connectionParameters) < 0)
    {
        exit(-1);
    }

    printf("connection established.\n\n");

    if (strcmp(role, "tx") == 0) // transmiter
    {
        fptr = fopen(filename, "rb");
        readFileSize();

        // sedning start control packet
        PointerIntPair controlpacketstart = createControlPacket(0, filename);

        int res;
        if (llwrite(controlpacketstart.pointer, controlpacketstart.size) < 0)
        {

            printf("Error in writing start control packet\n");
            exit(-1);
        }

        free(controlpacketstart.pointer);

        // sending data packets
        do
        {
            splitFile();
            PointerIntPair datapacket = createDataPacket();

            res = llwrite(datapacket.pointer, datapacket.size);
            if (res == -1)
            {
                printf("Error in llwrite\n");
                exit(-1);
            }
            else if (res == -2)
            { // the dirtiest thing so far in this code :/
                printf("Error: unexpected RR or REJ code -> skipping to next packet\n\n");
            }
            else if (res == -3)
            {
                printf("Randomly dissapearing bytes error :)\n");
                exit(-1);
            }

            usleep(SLEEP_AMOUNT);
            free(datapacket.pointer);
        } while (bytes > 0);

        // sending end control packet
        PointerIntPair controlpacketend = createControlPacket(1, filename);

        if (llwrite(controlpacketend.pointer, controlpacketend.size) < 0)
        {
            printf("Error in writing end control packet\n");
            exit(-1);
        }

        free(controlpacketend.pointer);
        fclose(fptr);

        if (llclose(1) < 0)
        {
            printf("Error in llclose.\n");
            exit(-1);
        }
    }
    else if (strcmp(role, "rx") == 0) // receiver
    {
        fptr = fopen(filename, "wb");
        int end = FALSE;
        while (!end)
        {

            int res = llread(receivedbuf);
            printf("bytes received: %d\n", res);
            switch (res)
            {
            case -2:
                printf("Terminated correctly.\n");
                end = TRUE;
                break;
            case -3:
                printf("terminated with errors!\n");
                break;
            case -4:
                printf("SET received: resetting the file.\n");
                rewind(fptr);
                break;
            case -1:
                printf("Error in llread.\n");
                exit(-1);
                break;
            default: // should be correct
            {
                int resParse = parsePacket(receivedbuf, res);
                if (resParse == -1)
                {
                    printf("Critical frame order error!\n");
                    exit(-1);
                    break;
                }
                  if (resParse == -2)
                {
                    printf("Assumed everything is fine\n");
                    break;
                }

                if (resParse == 3)
                {

                    printf("end packet received\n");

                    int res_ = llread(receivedbuf);
                    if (res_ == -2)
                    {
                        printf("should end correctly!\n");
                        end = TRUE;
                    }
                    else if (res_ == -3)
                    {
                        printf("terminated incorrectly!!!\n");
                        exit(-1);
                        break;
                    }
                }
            }
            break;
            }

            usleep(SLEEP_AMOUNT);
        }
        printf("llread ended\n");
        fclose(fptr);
    }
    printf("Terminating application layer!\n");
}