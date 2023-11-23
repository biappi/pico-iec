#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <string.h>
#include <ctype.h>

enum PIN {
    PIN_SRQ  = 6,
    PIN_ATTN = 7,
    PIN_CLK  = 8,
    PIN_DATA = 9,
};

enum IECState {
    noFlags   = 0,
    eoiFlag   = (1 << 0),
    atnFlag   = (1 << 1), 
    errorFlag = (1 << 2), 
};

enum IECState m_state;

enum ATNCheck {
    ATN_IDLE = 0,       // Nothing recieved of our concern
    ATN_CMD = 1,        // A command is received
    ATN_CMD_LISTEN = 2, // A command is received and data is coming to us
    ATN_CMD_TALK = 3,   // A command is received and we must talk now
    ATN_ERROR = 4,      // A problem occoured, reset communication
    ATN_RESET = 5,      // The IEC bus is in a reset state (RESET line).
};


const char const* atn_check_names[] = {
    "ATN_IDLE",
    "ATN_CMD",
    "ATN_CMD_LISTEN",
    "ATN_CMD_TALK",
    "ATN_ERROR",
    "ATN_RESET",
};


#define ATN_CMD_MAX_LENGTH 40
#define CMD_CHANNEL 15

struct ATNCmd{
    uint8_t code;
    char str[ATN_CMD_MAX_LENGTH];
    int strLen;
};

enum ATNCommand {
    ATN_CODE_LISTEN   = 0x20,    
    ATN_CODE_TALK     = 0x40,
    ATN_CODE_DATA     = 0x60,
    ATN_CODE_CLOSE    = 0xE0,
    ATN_CODE_OPEN     = 0xF0,
    ATN_CODE_UNLISTEN = 0x3F,
    ATN_CODE_UNTALK   = 0x5F,
};

enum ATNCommandEnd {
    ATN_CODE_LISTEN_END = (ATN_CODE_LISTEN + 0x1f),
    ATN_CODE_TALK_END   = (ATN_CODE_TALK   + 0x1f),
    ATN_CODE_DATA_END   = (ATN_CODE_DATA   + 0x10),
    ATN_CODE_CLOSE_END  = (ATN_CODE_CLOSE  + 0x10),
    ATN_CODE_OPEN_END   = (ATN_CODE_OPEN   + 0x10),
};

const int TIMING_BIT          =  70;  // bit clock hi/lo time     (us)
const int TIMING_NO_EOI       =  20;  // delay before bits        (us)
const int TIMING_EOI_WAIT     = 200;  // delay to signal EOI      (us)
const int TIMING_EOI_THRESH   =  20;  // threshold for EOI detect (*10 us approx)
const int TIMING_STABLE_WAIT  =  20;  // line stabilization       (us)
const int TIMING_ATN_PREDELAY =  50;  // delay required in atn    (us)
const int TIMING_ATN_DELAY    = 100;  // delay required after atn (us)
const int TIMING_FNF_DELAY    = 100;  // delay after fnf?         (us)

const int TIMING_TIMEOUT      = 65000;

int m_deviceNumber = 8;
struct ATNCmd m_cmd;

bool pin_read(enum PIN pin) {
    gpio_set_dir(pin, GPIO_IN);
    return gpio_get(pin);
}

void pin_write(enum PIN pin, bool data) {
    gpio_set_dir(pin, data ? GPIO_OUT : GPIO_IN);
    gpio_put(pin, !data);
}

uint8_t timeout_wait(enum PIN pin, bool whileHigh)
{
    int t = 0;
    bool c;

    while (t < TIMING_TIMEOUT) {
        c = pin_read(pin);

        if(whileHigh)
            c = !c;

        if(c)
            return false;

        sleep_us(2); 
        t++;
    }

    pin_write(PIN_CLK, false);
    pin_write(PIN_DATA, false);

    m_state = errorFlag;

    while(!pin_read(PIN_ATTN));

    return true;
}

uint8_t receive_byte(void)
{
    m_state = noFlags;

    if(timeout_wait(PIN_CLK, false)) {
        return 0;
    }

    pin_write(PIN_DATA, false);

    uint8_t n = 0;
    while(pin_read(PIN_CLK) && (n < 20)) {
        sleep_us(10);
        n++;
    }

    if(n >= TIMING_EOI_THRESH) {
        m_state |= eoiFlag;

        pin_write(PIN_DATA, true);
        sleep_us(TIMING_BIT);
        pin_write(PIN_DATA, false);

        if(timeout_wait(PIN_CLK, true)) {
            return 0;
        }
    }

    if(false == pin_read(PIN_ATTN))
        m_state |= atnFlag;

    uint8_t data = 0;
    for(n = 0; n < 8; n++) {
        data >>= 1;
        if(timeout_wait(PIN_CLK, false)) {
            return 0;
        }
        data |= (pin_read(PIN_DATA) ? (1 << 7) : 0);
        if(timeout_wait(PIN_CLK, true)) {
            return 0;
        }
    }

    pin_write(PIN_DATA, true);

    return data;
}

bool send_byte(uint8_t data, bool signal_eoi)
{
    if(timeout_wait(PIN_DATA, true))
        return false;

    pin_write(PIN_CLK, false);

    if(timeout_wait(PIN_DATA, false))
        return false;

    if(signal_eoi) {
        sleep_us(TIMING_EOI_WAIT);

        if(timeout_wait(PIN_DATA, true))
            return false;

        if(timeout_wait(PIN_DATA, false))
            return false;
    }

    sleep_us(TIMING_NO_EOI);

    for(uint8_t n = 0; n < 8; n++) {
        // FIXME: Here check whether data pin goes low, if so end (enter cleanup)!

        pin_write(PIN_CLK, true);
        pin_write(PIN_DATA, (data & 1) ? false : true);

        sleep_us(TIMING_BIT);
        pin_write(PIN_CLK, false);
        sleep_us(TIMING_BIT);

        data >>= 1;
    }

    pin_write(PIN_CLK, true);
    pin_write(PIN_DATA, false);

    sleep_us(TIMING_STABLE_WAIT);

    if(timeout_wait(PIN_DATA, true))
        return false;

    return true;
}


bool turn_around(void)
{
    if(timeout_wait(PIN_CLK, false))
        return false;

    pin_write(PIN_DATA, false);
    sleep_us(TIMING_BIT);
    pin_write(PIN_CLK, true);
    sleep_us(TIMING_BIT);

    return true;
}


bool undo_turn_around(void)
{
    pin_write(PIN_DATA, true);
    sleep_us(TIMING_BIT);
    pin_write(PIN_CLK, false);
    sleep_us(TIMING_BIT);

    if(timeout_wait(PIN_CLK, true))
        return false;

    return true;
}

bool send_eoi(uint8_t data)
{
    if(send_byte(data, true)) {
        if(undo_turn_around())
            return true;
    }

    return false;
}

enum ATNCheck check_atn(struct ATNCmd *cmd)
{
    enum ATNCheck ret = ATN_IDLE;
    uint8_t i = 0;

    if (pin_read(PIN_ATTN)) {
        pin_write(PIN_DATA, false);
        pin_write(PIN_CLK, false);

        cmd->strLen = 0;
        return ATN_IDLE;
    }

    pin_write(PIN_DATA, true);
    pin_write(PIN_CLK, false);
    sleep_us(TIMING_ATN_PREDELAY);

    uint8_t c = receive_byte();
    if(m_state & errorFlag)
        return ATN_ERROR;

    if(c == (ATN_CODE_LISTEN | m_deviceNumber)) {
        c = receive_byte();
        if (m_state & errorFlag)
            return ATN_ERROR;

        cmd->code = c;

        if((c & 0xF0) == ATN_CODE_DATA && (c & 0xF) != CMD_CHANNEL) {
            ret = ATN_CMD_LISTEN;
        }
        else if(c != ATN_CODE_UNLISTEN) {
            for(;;) {
                c = receive_byte();
                if(m_state & errorFlag)
                    return ATN_ERROR;

                if((m_state & atnFlag) && (ATN_CODE_UNLISTEN == c))
                    break;

                if(i >= ATN_CMD_MAX_LENGTH) {
                    return ATN_ERROR;
                }
                cmd->str[i++] = c;
            }
            ret = ATN_CMD;
        }
    }
    else if (c == (ATN_CODE_TALK | m_deviceNumber)) {
        c = receive_byte();
        if(m_state & errorFlag)
            return ATN_ERROR;
        cmd->code = c;

        while(!pin_read(PIN_ATTN)) {
            if(pin_read(PIN_CLK)) {
                c = receive_byte();
                if(m_state & errorFlag)
                    return ATN_ERROR;

                if(i >= ATN_CMD_MAX_LENGTH) {
                    return ATN_ERROR;
                }
                cmd->str[i++] = c;
            }
        }

        if(!turn_around())
            return ATN_ERROR;

        ret = ATN_CMD_TALK;
    }
    else {
        sleep_us(TIMING_ATN_DELAY);
        pin_write(PIN_DATA, false);
        pin_write(PIN_CLK, false);

        while(!pin_read(PIN_ATTN));
    }

    sleep_us(TIMING_ATN_DELAY);

    cmd->strLen = i;
    return ret;
}

bool send_fnf()
{
    pin_write(PIN_DATA, false);
    pin_write(PIN_CLK, false);

    sleep_us(TIMING_FNF_DELAY);

    return true;
}

void send_line(char* text, int *addr)
{
    int len = strlen(text);
    *addr += len + 5 - 2;

    send_byte(*addr & 0xFF, false);
    send_byte(*addr >> 8, false);

    while (*text != 0)
        send_byte(*text, false);

    send_byte(0, false);
}


void listen_bus(void)
{
    enum ATNCheck retATN = check_atn(&m_cmd);

    if (retATN == ATN_ERROR) {
        printf("ATNCMD: IEC_ERROR!\n");
        return;
    }

    if (retATN == ATN_IDLE) {
        return;
    }

    m_cmd.str[m_cmd.strLen] = '\0';

    printf("ATN code:%2x cmd: %s (len: %d) retATN: %s\n", m_cmd.code, m_cmd.str, m_cmd.strLen, atn_check_names[retATN]);

    uint8_t chan = m_cmd.code & 0x0F;
    switch(m_cmd.code & 0xF0) {
        case ATN_CODE_OPEN:
        printf("%s", m_cmd.str);
        break;

        case ATN_CODE_DATA:
            if(retATN == ATN_CMD_TALK) {
                if(CMD_CHANNEL == chan) {
                    printf("handleATNCmdCodeOpen\n");
                }

                const int basic_addr = 0x0801;
                int addr = basic_addr;

                send_byte(0x01, false);
                send_byte(0x80, false);
                send_byte(0x10, false);
                send_byte(0x08, false);
                send_byte(0x0a, false);
                send_byte(0x00, false);
                send_byte(0x99, false);
                send_byte(0x22, false);
                send_byte(0x53, false);
                send_byte(0x55, false);
                send_byte(0x43, false);
                send_byte(0x43, false);
                send_byte(0x48, false);
                send_byte(0x49, false);
                send_byte(0x41, false);
                send_byte(0x22, false);
                send_byte(0x00, false);
                send_eoi(0x00);

            }
            else if(retATN == ATN_CMD_LISTEN) {

                printf("\nRECEIVING DATA:\n\n");

                do {
                    uint8_t byte = receive_byte();
                    printf("    0x%02x, // %c\n", byte, isprint(byte) ? byte : ' ');
                } while (!(m_state & errorFlag) && !(m_state & eoiFlag));
            }
            else if(retATN == ATN_CMD) 
                 printf("handleATNCmdCodeOpen\n");
            break;

        case ATN_CODE_CLOSE:
            // printf("handleATNCmdClose\n");
            break;

        case ATN_CODE_LISTEN:
            // printf("LISTEN\n");
            break;
        case ATN_CODE_TALK:
            // printf("TALK\n");
            break;
        case ATN_CODE_UNLISTEN:
            //printf("UNLISTEN");
            break;
        case ATN_CODE_UNTALK:
            //printf("UNTALK");
            break;
    }
}

int main() {
    stdio_init_all();

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);

    for (int i = PIN_SRQ; i <= PIN_DATA; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }

    gpio_put(LED_PIN, 1);

    for (int i = 0; i < 5; i++) {
        printf("wait... %d\n", i);
        sleep_ms(1000);
    }

    printf("\n\nREADY!\n\n");

    gpio_put(LED_PIN, true);

    while (true) {
       listen_bus();
    }

    return 0;
}

