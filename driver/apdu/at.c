#include "at.h"
#include "at_common.h"

#include <euicc/hexutil.h>
#include <euicc/interface.h>
#include <lpac/utils.h>

#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

// Replace FILE_IO with RAW_IO
#define USE_RAW_IO          1   /* 1=RAW_IO,  0:FILE_IO */

// Replace AT+CCHO, AT+CCHC, AT+CGLA  with AT+CSIM
#define USE_AT_CSIM         1   /* 1=AT+CSIM, 0:AT+CCHO */

#if USE_RAW_IO
#include <fcntl.h>      /* File control definitions */
#include <errno.h>      /* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */
#endif

#define DEBUG_LOG_ENABLED   1   /* 1=enable,  0=disable */
static bool s_at_debug = false;

#if (DEBUG_LOG_ENABLED == 1)
  #define DEBUG_PRINT(x)          if (s_at_debug) printf x
#else
  #define DEBUG_PRINT(x)          ((void)0)
#endif

#ifndef LOBYTE
#define LOBYTE(uwData)              ((uint8_t)((uwData) & 0xFFu))
#endif
#ifndef HIBYTE
#define HIBYTE(uwData)              ((uint8_t)(((uwData) >> 8) & 0xFFu))
#endif

/* Length of status word */
#define SIZE_OF_SW                  0x02

#define ISO_STATUS_OK               0x9000

/* Read data from 'Buffer' Array and update 'Value' variable,
** Array is input parameter
** and value is output parameter */
#define READU16(Ppui1Buffer,Pui2Value)   {\
        Pui2Value = (((*(uint8_t *)Ppui1Buffer) << 8 ) |(*(uint8_t *)(Ppui1Buffer + 1)));}

#if USE_RAW_IO
#define BAUD_RATE               B115200
#define TC_GET_ATTR_VTIME       1   /* 0.1 sec */
#define SNPRINTF                snprintf
#define AT_CSIM_CMD_PADDED_LEN  24
#endif

static void print_bytes_helper( const char* str,
                                const uint8_t *p_ubArray,
                                int size)
{
    int i;
    if (str && *str) {
        DEBUG_PRINT(("%s", str));
    }

    for (i = 0; i < size; i++) {
        DEBUG_PRINT(("%02X", p_ubArray[i]));
    }
}

#if USE_RAW_IO
static int s_fd = -1; /* File descriptor for the port */
#else
static FILE *fuart;
#endif

static int logic_channel = 0;
static char *buffer;

#if USE_RAW_IO
static size_t PosixModem_ReadResponse(int fd,
                                      char *responseBuf_p,
                                      size_t maxResponseBufSize)
{
    size_t bytesRead;

    bytesRead = read(fd,
                     (uint8_t *)responseBuf_p,
                     maxResponseBufSize);

    return bytesRead;
}

static bool PosixModem_WriteCommand(int fd,
                                    const char* commandStr_p)
{
    bool result = false;
    size_t bytesWritten = 0;
    size_t commandStrSize = strlen(commandStr_p);

    bytesWritten = write(fd,
                         (const uint8_t *)commandStr_p,
                         commandStrSize);

    result = (commandStrSize == bytesWritten);
    if (result) {
        //DEBUG_PRINT(("AT_DEBUG: %s[%d] %s\n", __FUNCTION__, __LINE__, commandStr_p));
    }
    else {
        DEBUG_PRINT(("AT_DEBUG: %s[%d] FAILED! %s\n", __FUNCTION__, __LINE__, commandStr_p));
    }

    return result;
}

/* read until there is no more data */
static void FlushAll(int fd)
{
#define MAX_FLUSH_TRIES     50
#define MAX_FLUSH_BUF_SIZE  20

    char recvBuf[MAX_FLUSH_BUF_SIZE];
    size_t tries = MAX_FLUSH_TRIES;

    while (tries > 0) {
        size_t numBytesRead;

        numBytesRead = PosixModem_ReadResponse(fd,
                                               recvBuf,
                                               sizeof(recvBuf));

        if (numBytesRead == 0) {
            break;
        }
        else {
            recvBuf[numBytesRead] = '\0';
            DEBUG_PRINT(("AT_DEBUG: %s[%d] %s\n", __FUNCTION__, __LINE__, recvBuf));
        }

        tries--;
    }

#undef MAX_FLUSH_TRIES
#undef MAX_FLUSH_BUF_SIZE
}
#endif

static void enumerate_serial_device_linux(cJSON *data) {
    const char *dir_path = "/dev/serial/by-id";
    DIR *dir = opendir(dir_path);
    if (dir == NULL)
        return;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        size_t path_len = strlen(dir_path) + 1 /* SEP */ + strlen(entry->d_name) + 1 /* NUL */;
        _cleanup_free_ char *full_path = malloc(path_len);
        snprintf(full_path, path_len, "%s/%s", dir_path, entry->d_name);

        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "env", full_path);
        cJSON_AddStringToObject(item, "name", entry->d_name);
        cJSON_AddItemToArray(data, item);
    }
    closedir(dir);
}

static int at_expect(char **response, const char *expected) {
    memset(buffer, 0, AT_BUFFER_SIZE);

    if (response)
        *response = NULL;

    while (1) {
#if USE_RAW_IO
        size_t numBytesRead;
        numBytesRead = PosixModem_ReadResponse( s_fd,
                                                buffer,
                                                AT_BUFFER_SIZE);

        if (numBytesRead > 0) {
            buffer[numBytesRead] = '\0';
        }

        char* start_p = buffer;
        size_t lenRead;

        while (*start_p) {
            // analyze buffer for multiple '\r\n' seqments
            lenRead = strlen(start_p);
            size_t posCrLf = strcspn(start_p, "\r\n");

#if 0
            if (lenRead) {
                DEBUG_PRINT(("AT_DEBUG: lenRead: %ld, posCrLf: %ld\n",
                    lenRead, posCrLf));
            }
#endif

            start_p[posCrLf] = 0;

            if (lenRead) {
                DEBUG_PRINT(("AT_DEBUG: %s\n", start_p));
            }

            if (strstr(start_p, "ERROR") != NULL)
            {
                return -1;
            }
            else if (strcmp(start_p, "OK") == 0)
            {
                return 0;
            }
            else if (expected && strncmp(start_p, expected, strlen(expected)) == 0)
            {
                if (response)
                    *response = strdup(start_p + strlen(expected));
            }

            if (lenRead > (posCrLf + 1)) {
                start_p = &start_p[posCrLf + 1];
            }
            else {
                break;
            }
        }

#else
        fgets(buffer, AT_BUFFER_SIZE, fuart);
        buffer[strcspn(buffer, "\r\n")] = 0;
        if (getenv_or_default(ENV_AT_DEBUG, (bool)false))
            printf("AT_DEBUG: %s\n", buffer);
        if (strcmp(buffer, "ERROR") == 0) {
            return -1;
        } else if (strcmp(buffer, "OK") == 0) {
            return 0;
        } else if (expected && strncmp(buffer, expected, strlen(expected)) == 0) {
            if (response)
                *response = strdup(buffer + strlen(expected));
        }
#endif
    }
    return 0;
}

static int apdu_interface_connect(struct euicc_ctx *ctx) {
#if USE_RAW_IO
    struct termios options;
    s_at_debug = getenv("LPAC_APDU_AT_DEBUG");
#endif

    const char *device = getenv_or_default(ENV_AT_DEVICE, "/dev/ttyUSB0");

    logic_channel = 0;

    DEBUG_PRINT(("AT_DEBUG: %s[%d] %s\n", __FUNCTION__, __LINE__, device));

#if USE_RAW_IO
    /*
    The O_NOCTTY flag tells UNIX that this program doesn't want to be the "controlling terminal"
    for that port. If you don't specify this then any input (such as keyboard abort signals and
    so forth) will affect your process. Programs like getty(1M/8) use this feature when starting
    the login process, but normally a user program does not want this behavior.

    The O_NDELAY flag tells UNIX that this program doesn't care what state the DCD signal line is
    in - whether the other end of the port is up and running. If you do not specify this flag,
    your process will be put to sleep until the DCD signal line is the space voltage.
    */
    s_fd = open(device,
                O_RDWR | O_NOCTTY | O_NDELAY);

    if (s_fd < 0) {
        fprintf(stderr, "%s [%d]: Unable to open %s - check access permission\n",
                __FUNCTION__, __LINE__, device);
        return -1;
    }
    fcntl(s_fd, F_SETFL, 0);

    /* get the current options */
    tcgetattr(s_fd, &options);

    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

    /* set raw input, 1 second timeout */
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN]  = 0;  /* Minimum number of characters to read */
    options.c_cc[VTIME] = TC_GET_ATTR_VTIME; /* Time to wait for data (in 0.1 sec) */

    /* set the options */
    tcsetattr(s_fd, TCSANOW, &options);

#else
    fuart = fopen(device, "r+");
    if (fuart == NULL) {
        fprintf(stderr, "Failed to open device: %s\n", device);
        return -1;
    }
    setbuf(fuart, NULL);
#endif

#if USE_RAW_IO
    FlushAll(s_fd);
#endif

#if USE_AT_CSIM
#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "AT+CSIM=?\r\n");
#else
    fprintf(fuart, "AT+CSIM=?\r\n");
#endif

    if (at_expect(NULL, NULL))
    {
        fprintf(stderr, "Device missing AT+CSIM support\n");
        return -1;
    }

#else

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "AT+CCHO=?\r\n");
#else
    fprintf(fuart, "AT+CCHO=?\r\n");
#endif

    if (at_expect(NULL, NULL)) {
        fprintf(stderr, "Device missing AT+CCHO support\n");
        return -1;
    }

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "AT+CCHC=?\r\n");
#else
    fprintf(fuart, "AT+CCHC=?\r\n");
#endif

    if (at_expect(NULL, NULL)) {
        fprintf(stderr, "Device missing AT+CCHC support\n");
        return -1;
    }

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "AT+CGLA=?\r\n");
#else
    fprintf(fuart, "AT+CGLA=?\r\n");
#endif

    if (at_expect(NULL, NULL)) {
        fprintf(stderr, "Device missing AT+CGLA support\n");
        return -1;
    }

#endif

    return 0;
}

static void apdu_interface_disconnect(struct euicc_ctx *ctx) {
#if USE_RAW_IO
    close(s_fd);
    s_fd = -1;
#else
    fclose(fuart);
    fuart = NULL;
#endif
    logic_channel = 0;
}


#if USE_AT_CSIM
static int apdu_interface_transmit_atcsim(struct euicc_ctx *ctx,
    uint8_t **rx, uint32_t *rx_len, const uint8_t *tx, uint32_t tx_len)
{
#if USE_RAW_IO
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#endif

    int fret = 0;
    int ret;
    char *response = NULL;
    char *hexstr = NULL;


    *rx = NULL;
    *rx_len = 0;

    if (!logic_channel)
    {
        return -1;
    }

#if USE_RAW_IO
    SNPRINTF(strBuffer,
             sizeof(strBuffer),
             "AT+CSIM=%u,\"",
             tx_len * 2);

    PosixModem_WriteCommand(s_fd, strBuffer);
#else
    fprintf(fuart, "AT+CSIM=%u,\"", tx_len * 2);
#endif
    for (uint32_t i = 0; i < tx_len; i++)
    {
#if USE_RAW_IO
        SNPRINTF(strBuffer,
                 sizeof(strBuffer),
                 "%02X",
                 (uint8_t)(tx[i] & 0xFF));

        PosixModem_WriteCommand(s_fd, strBuffer);
#else
        fprintf(fuart, "%02X", (uint8_t)(tx[i] & 0xFF));
#endif
    }

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "\"\r\n");
#else
    fprintf(fuart, "\"\r\n");
#endif

    if (at_expect(&response, "+CSIM: "))
    {
        goto err;
    }
    if (response == NULL)
    {
        goto err;
    }

    strtok(response, ",");
    hexstr = strtok(NULL, ",");
    if (!hexstr)
    {
        goto err;
    }
    if (hexstr[0] == '"')
    {
        hexstr++;
    }
    hexstr[strcspn(hexstr, "\"")] = '\0';

    *rx_len = strlen(hexstr) / 2;
    *rx = malloc(*rx_len);
    if (!*rx)
    {
        goto err;
    }

    ret = euicc_hexutil_hex2bin_r(*rx, *rx_len, hexstr, strlen(hexstr));
    if (ret < 0)
    {
        goto err;
    }
    *rx_len = ret;

    goto exit;

err:
    fret = -1;
    free(*rx);
    *rx = NULL;
    *rx_len = 0;
exit:
    return fret;
}

#else
static int apdu_interface_transmit(struct euicc_ctx *ctx, uint8_t **rx, uint32_t *rx_len, const uint8_t *tx,
                                   uint32_t tx_len) {
#if USE_RAW_IO
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#endif
    int fret = 0;
    int ret;
    _cleanup_free_ char *response = NULL;
    char *hexstr = NULL;

    *rx = NULL;
    *rx_len = 0;

    if (!logic_channel) {
        return -1;
    }

#if USE_RAW_IO
    SNPRINTF(strBuffer,
             sizeof(strBuffer),
             "AT+CGLA=%d,%u,\"",
             logic_channel,
             tx_len * 2);

    PosixModem_WriteCommand(s_fd, strBuffer);
#else
    fprintf(fuart, "AT+CGLA=%d,%u,\"", logic_channel, tx_len * 2);
#endif

    for (uint32_t i = 0; i < tx_len; i++) {
#if USE_RAW_IO
        SNPRINTF(strBuffer,
                 sizeof(strBuffer),
                 "%02X",
                 (uint8_t)(tx[i] & 0xFF));

        PosixModem_WriteCommand(s_fd, strBuffer);
#else
        fprintf(fuart, "%02X", (uint8_t)(tx[i] & 0xFF));
#endif
    }

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "\"\r\n");
#else
    fprintf(fuart, "\"\r\n");
#endif
    if (at_expect(&response, "+CGLA:")) {
        goto err;
    }
    if (response == NULL) {
        goto err;
    }

    strtok(response, ",");
    hexstr = strtok(NULL, ",");
    if (!hexstr) {
        goto err;
    }
    if (hexstr[0] == '"') {
        hexstr++;
    }
    hexstr[strcspn(hexstr, "\"")] = '\0';

    *rx_len = strlen(hexstr) / 2;
    *rx = malloc(*rx_len);
    if (!*rx) {
        goto err;
    }

    ret = euicc_hexutil_hex2bin_r(*rx, *rx_len, hexstr, strlen(hexstr));
    if (ret < 0) {
        goto err;
    }
    *rx_len = ret;

    goto exit;

err:
    fret = -1;
    free(*rx);
    *rx = NULL;
    *rx_len = 0;
exit:
    return fret;
}
#endif


#if USE_AT_CSIM
static int apdu_interface_logic_channel_open_atcsim(struct euicc_ctx *ctx,
    const uint8_t *aid, uint8_t aid_len)
{
#if USE_RAW_IO
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#endif

    char *response;

    if (logic_channel)
    {
        return logic_channel;
    }

    // Manage Channel (Open)
    logic_channel = 1;

    int fret;
    uint8_t *rx_p = NULL;
    uint32_t rx_len = 0;
    uint8_t manageChannelOpen[] = {0x00, 0x70, 0x00, 0x00, 0x01};

    fret = apdu_interface_transmit_atcsim(ctx, &rx_p, &rx_len,
        manageChannelOpen, sizeof(manageChannelOpen));

    if (fret == 0) {
        if (rx_len > 0) {
            if (rx_p != NULL) {
                print_bytes_helper("ManageChannelOpen RSP:", rx_p, rx_len);
                DEBUG_PRINT(("\n"));

            }
        }
        free(rx_p);
    }

    // Select ISD-R
    rx_p = NULL;
    rx_len = 0;

    uint8_t selectIsdr[] = {logic_channel, 0xA4, 0x04, 0x00, 0x10, 0xA0, 0x00, 0x00,
                0x05, 0x59, 0x10, 0x10, 0xFF, 0xFF, 0xFF, 0xFF,
                0x89, 0x00, 0x00, 0x01, 0x00};

    fret = apdu_interface_transmit_atcsim(ctx, &rx_p, &rx_len,
        selectIsdr, sizeof(selectIsdr));

    uint16_t ui2StatusWord = 0;
    uint8_t ui1Sw1 = 0;
    uint8_t ui1Sw2 = 0;

    if (fret == 0) {
        if (rx_len >= 2) {
            if (rx_p != NULL) {
                print_bytes_helper("SelectIsdr RSP:", rx_p, rx_len);
                DEBUG_PRINT(("\n"));

                READU16(&rx_p[rx_len - SIZE_OF_SW], ui2StatusWord);
                ui1Sw1 = HIBYTE(ui2StatusWord);
                ui1Sw2 = LOBYTE(ui2StatusWord);

                if (ui1Sw1 == 0x61) {
                    do {
                        ui1Sw1 = 0;

                        // Get Data
                        uint8_t getData[] = {logic_channel, 0xC0, 0x00, 0x00, ui1Sw2};
                        uint8_t *rx2_p = NULL;
                        uint32_t rx2_len = 0;

                        fret = apdu_interface_transmit_atcsim(ctx, &rx2_p, &rx2_len,
                                    getData, sizeof(getData));

                        if (fret == 0) {
                            if (rx2_len >= 2) {
                                if (rx2_p != NULL) {
                                    print_bytes_helper("GetData RSP:", rx2_p, rx2_len);
                                    DEBUG_PRINT(("\n"));

                                    READU16(&rx2_p[rx2_len - SIZE_OF_SW], ui2StatusWord);
                                    ui1Sw1 = HIBYTE(ui2StatusWord);
                                    ui1Sw2 = LOBYTE(ui2StatusWord);
                                }
                            }
                            free(rx2_p);
                        }

                    } while (ui1Sw1 == 0x61);

                }

            }
        }
        free(rx_p);
    }

    if (!(ui1Sw1 == 0x90) && (ui1Sw2 == 0x00)) {
        logic_channel = -1;
    }

    return logic_channel;
}

#else
static int apdu_interface_logic_channel_open(struct euicc_ctx *ctx, const uint8_t *aid, uint8_t aid_len) {
#if USE_RAW_IO
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#endif
    char *response;

    if (logic_channel) {
        return logic_channel;
    }

    for (int i = 1; i <= 4; i++) {
#if USE_RAW_IO
        SNPRINTF(strBuffer,
                 sizeof(strBuffer),
                 "AT+CCHC=%d\r\n",
                 i);
        PosixModem_WriteCommand(s_fd, strBuffer);
#else
        fprintf(fuart, "AT+CCHC=%d\r\n", i);
#endif
        at_expect(NULL, NULL);
    }

#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "AT+CCHO=\"");
#else
    fprintf(fuart, "AT+CCHO=\"");
#endif
    for (int i = 0; i < aid_len; i++) {
#if USE_RAW_IO
        SNPRINTF(strBuffer,
                 sizeof(strBuffer),
                 "%02X",
                 (uint8_t)(aid[i] & 0xFF));
        PosixModem_WriteCommand(s_fd, strBuffer);
#else
        fprintf(fuart, "%02X", (uint8_t)(aid[i] & 0xFF));
#endif
    }
#if USE_RAW_IO
    PosixModem_WriteCommand(s_fd, "\"\r\n");
#else
    fprintf(fuart, "\"\r\n");
#endif
    if (at_expect(&response, "+CCHO: ")) {
        return -1;
    }
    if (response == NULL) {
        return -1;
    }
    logic_channel = atoi(response);

    return logic_channel;
}
#endif


#if USE_AT_CSIM
static void apdu_interface_logic_channel_close_atcsim(struct euicc_ctx *ctx,
    uint8_t channel)
{
    // do nothing for the AT+CSIM case
}

#else
static void apdu_interface_logic_channel_close(struct euicc_ctx *ctx, uint8_t channel) {
#if USE_RAW_IO
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#endif
    if (!logic_channel) {
        return;
    }
#if USE_RAW_IO
    SNPRINTF(strBuffer,
             sizeof(strBuffer),
             "AT+CCHC=%d\r\n",
             logic_channel);

    PosixModem_WriteCommand(s_fd, strBuffer);
#else
    fprintf(fuart, "AT+CCHC=%d\r\n", logic_channel);
#endif
    at_expect(NULL, NULL);
}
#endif

static int libapduinterface_init(struct euicc_apdu_interface *ifstruct) {
    set_deprecated_env_name(ENV_AT_DEBUG, "AT_DEBUG");
    set_deprecated_env_name(ENV_AT_DEVICE, "AT_DEVICE");

    memset(ifstruct, 0, sizeof(struct euicc_apdu_interface));

    ifstruct->connect = apdu_interface_connect;
    ifstruct->disconnect = apdu_interface_disconnect;
#if USE_AT_CSIM
    ifstruct->logic_channel_open = apdu_interface_logic_channel_open_atcsim;
    ifstruct->logic_channel_close = apdu_interface_logic_channel_close_atcsim;
    ifstruct->transmit = apdu_interface_transmit_atcsim;
#else
    ifstruct->logic_channel_open = apdu_interface_logic_channel_open;
    ifstruct->logic_channel_close = apdu_interface_logic_channel_close;
    ifstruct->transmit = apdu_interface_transmit;
#endif
    buffer = malloc(AT_BUFFER_SIZE);
    if (!buffer) {
        fprintf(stderr, "Failed to allocate memory\n");
        return -1;
    }

    return 0;
}

static int libapduinterface_main(const struct euicc_apdu_interface *ifstruct, int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <list>\n", argv[0]);
        return -1;
    }

    if (strcmp(argv[1], "list") == 0) {
        _cleanup_cjson_ cJSON *data = cJSON_CreateArray();

#ifdef __linux__
        enumerate_serial_device_linux(data);
#else
        fprintf(stderr, "Serial device enumeration not implemented on this platform.\n");
        fflush(stderr);
#endif

        jprint_enumerate_devices(data);
    }

    return 0;
}

static void libapduinterface_fini(struct euicc_apdu_interface *ifstruct) { free(buffer); }

const struct euicc_driver driver_apdu_at = {
    .type = DRIVER_APDU,
    .name = "at",
    .init = (int (*)(void *))libapduinterface_init,
    .main = (int (*)(void *, int, char **))libapduinterface_main,
    .fini = (void (*)(void *))libapduinterface_fini,
};
