#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define SSD1306_I2C_ADDRESS 0x3C
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define PAGE_COUNT (SCREEN_HEIGHT / 8)
#define LOG_FILE   "/home/ranch/programs/battery/battery_log.txt"
#define STATE_FILE "/home/ranch/programs/penv/ai_state.txt"
#define PI 3.14159265358979323846

/* Bottom status panel */
#define BAR_H 18
#define BAR_Y (SCREEN_HEIGHT - BAR_H)          /* 46 */
#define BAR_TEXT_Y (BAR_Y + (BAR_H - 8) / 2)   /* 51 */

/* X_OFFSET*/
#define X_OFFSET 6
#define CONTENT_W (SCREEN_WIDTH - X_OFFSET)

/* Face area above the bar */
#define MOUTH_CX (CONTENT_W / 2)
#define MOUTH_CY 23
#define MOUTH_W  64
#define MOUTH_H  18

int i2c_fd;
uint8_t frame_buffer[SCREEN_WIDTH * PAGE_COUNT] = {0};
uint8_t prev_buffer[SCREEN_WIDTH * PAGE_COUNT] = {0};

/* Simple 5x8 font */
const uint8_t font5x8[95][5] = {
    [0] = {0x00,0x00,0x00,0x00,0x00}, // space
    ['!' - 32] = {0x00,0x00,0x5F,0x00,0x00},
    ['%' - 32] = {0x62,0x64,0x08,0x13,0x23},
    [',' - 32] = {0x00,0x80,0x60,0x00,0x00},
    ['-' - 32] = {0x08,0x08,0x08,0x08,0x08},
    ['.' - 32] = {0x00,0x60,0x60,0x00,0x00},
    ['/' - 32] = {0x20,0x10,0x08,0x04,0x02},
    [':' - 32] = {0x00,0x36,0x36,0x00,0x00},
    [';' - 32] = {0x00,0x56,0x36,0x00,0x00},
    ['<' - 32] = {0x00,0x08,0x14,0x22,0x41},
    ['=' - 32] = {0x14,0x14,0x14,0x14,0x14},
    ['>' - 32] = {0x41,0x22,0x14,0x08,0x00},
    ['?' - 32] = {0x02,0x01,0x51,0x09,0x06},
    ['@' - 32] = {0x32,0x49,0x79,0x41,0x3E},
    ['0' - 32] = {0x3E,0x51,0x49,0x45,0x3E},
    ['1' - 32] = {0x00,0x42,0x7F,0x40,0x00},
    ['2' - 32] = {0x42,0x61,0x51,0x49,0x46},
    ['3' - 32] = {0x21,0x41,0x45,0x4B,0x31},
    ['4' - 32] = {0x18,0x14,0x12,0x7F,0x10},
    ['5' - 32] = {0x27,0x45,0x45,0x45,0x39},
    ['6' - 32] = {0x3C,0x4A,0x49,0x49,0x30},
    ['7' - 32] = {0x01,0x71,0x09,0x05,0x03},
    ['8' - 32] = {0x36,0x49,0x49,0x49,0x36},
    ['9' - 32] = {0x06,0x49,0x49,0x29,0x1E},
    ['A' - 32] = {0x7E,0x11,0x11,0x11,0x7E},
    ['B' - 32] = {0x7F,0x49,0x49,0x49,0x36},
    ['C' - 32] = {0x3E,0x41,0x41,0x41,0x22},
    ['D' - 32] = {0x7F,0x41,0x41,0x22,0x1C},
    ['E' - 32] = {0x7F,0x49,0x49,0x49,0x41},
    ['F' - 32] = {0x7F,0x09,0x09,0x01,0x01},
    ['G' - 32] = {0x3E,0x41,0x41,0x51,0x32},
    ['H' - 32] = {0x7F,0x08,0x08,0x08,0x7F},
    ['I' - 32] = {0x00,0x41,0x7F,0x41,0x00},
    ['J' - 32] = {0x20,0x40,0x41,0x3F,0x01},
    ['K' - 32] = {0x7F,0x08,0x14,0x22,0x41},
    ['L' - 32] = {0x7F,0x40,0x40,0x40,0x40},
    ['M' - 32] = {0x7F,0x02,0x04,0x02,0x7F},
    ['N' - 32] = {0x7F,0x04,0x08,0x10,0x7F},
    ['O' - 32] = {0x3E,0x41,0x41,0x41,0x3E},
    ['P' - 32] = {0x7F,0x09,0x09,0x09,0x06},
    ['Q' - 32] = {0x3E,0x41,0x51,0x21,0x5E},
    ['R' - 32] = {0x7F,0x09,0x19,0x29,0x46},
    ['S' - 32] = {0x46,0x49,0x49,0x49,0x31},
    ['T' - 32] = {0x01,0x01,0x7F,0x01,0x01},
    ['U' - 32] = {0x3F,0x40,0x40,0x40,0x3F},
    ['V' - 32] = {0x1F,0x20,0x40,0x20,0x1F},
    ['W' - 32] = {0x7F,0x20,0x18,0x20,0x7F},
    ['X' - 32] = {0x63,0x14,0x08,0x14,0x63},
    ['Y' - 32] = {0x03,0x04,0x78,0x04,0x03},
    ['Z' - 32] = {0x61,0x51,0x49,0x45,0x43},
    ['a' - 32] = {0x20,0x54,0x54,0x54,0x78},
    ['b' - 32] = {0x7F,0x50,0x50,0x50,0x3C},
    ['c' - 32] = {0x38,0x44,0x44,0x44,0x20},
    ['d' - 32] = {0x38,0x44,0x44,0x48,0x7F},
    ['e' - 32] = {0x38,0x54,0x54,0x54,0x18},
    ['f' - 32] = {0x08,0x7E,0x09,0x01,0x02},
    ['g' - 32] = {0x0C,0x52,0x52,0x52,0x3E},
    ['h' - 32] = {0x7F,0x08,0x04,0x04,0x78},
    ['i' - 32] = {0x00,0x44,0x7D,0x40,0x00},
    ['j' - 32] = {0x20,0x40,0x44,0x3D,0x00},
    ['k' - 32] = {0x7F,0x10,0x28,0x44,0x00},
    ['l' - 32] = {0x00,0x41,0x7F,0x40,0x00},
    ['m' - 32] = {0x7C,0x04,0x18,0x04,0x78},
    ['n' - 32] = {0x7C,0x08,0x04,0x04,0x78},
    ['o' - 32] = {0x38,0x44,0x44,0x44,0x38},
    ['p' - 32] = {0x7C,0x14,0x14,0x14,0x08},
    ['q' - 32] = {0x08,0x14,0x14,0x18,0x7C},
    ['r' - 32] = {0x7C,0x08,0x04,0x04,0x08},
    ['s' - 32] = {0x48,0x54,0x54,0x54,0x20},
    ['t' - 32] = {0x04,0x3F,0x44,0x40,0x20},
    ['u' - 32] = {0x3C,0x40,0x40,0x20,0x7C},
    ['v' - 32] = {0x1C,0x20,0x40,0x20,0x1C},
    ['w' - 32] = {0x3C,0x40,0x30,0x40,0x3C},
    ['x' - 32] = {0x44,0x28,0x10,0x28,0x44},
    ['y' - 32] = {0x0C,0x50,0x50,0x50,0x3C},
    ['z' - 32] = {0x44,0x64,0x54,0x4C,0x44}
};

/* --- SSD1306 low level --- */
void ssd1306_send_command(uint8_t command) {
    uint8_t buffer[2] = {0x00, command};
    if (write(i2c_fd, buffer, 2) != 2) { perror("Failed to send command"); exit(1); }
}

void ssd1306_init() {
    const char *i2c_device = "/dev/i2c-0";
    i2c_fd = open(i2c_device, O_RDWR);
    if (i2c_fd < 0) { perror("Open I2C"); exit(1); }
    if (ioctl(i2c_fd, I2C_SLAVE, SSD1306_I2C_ADDRESS) < 0) { perror("I2C ioctl"); exit(1); }

    ssd1306_send_command(0xAE);              // display off
    ssd1306_send_command(0x20);              // memory addressing mode
    ssd1306_send_command(0x00);              //   horizontal
    ssd1306_send_command(0xB0);
    ssd1306_send_command(0xC0);              // COM scan direction: NORMAL  
    ssd1306_send_command(0x00);
    ssd1306_send_command(0x10);
    ssd1306_send_command(0x40);
    ssd1306_send_command(0x81);
    ssd1306_send_command(0x7F);
    ssd1306_send_command(0xA0);              // segment remap: NORMAL     
    ssd1306_send_command(0xA6);
    ssd1306_send_command(0xA8);
    ssd1306_send_command(0x3F);
    ssd1306_send_command(0xA4);
    ssd1306_send_command(0xD3);
    ssd1306_send_command(0x00);
    ssd1306_send_command(0xD5);
    ssd1306_send_command(0xF0);
    ssd1306_send_command(0xD9);
    ssd1306_send_command(0x22);
    ssd1306_send_command(0xDA);
    ssd1306_send_command(0x12);
    ssd1306_send_command(0xDB);
    ssd1306_send_command(0x20);
    ssd1306_send_command(0x8D);
    ssd1306_send_command(0x14);
    ssd1306_send_command(0xAF);              // display on
}

void ssd1306_update_display_full() {
    ssd1306_send_command(0x21);
    ssd1306_send_command(0);
    ssd1306_send_command(SCREEN_WIDTH - 1);
    ssd1306_send_command(0x22);
    ssd1306_send_command(0);
    ssd1306_send_command(PAGE_COUNT - 1);

    uint8_t data[1 + SCREEN_WIDTH * PAGE_COUNT];
    data[0] = 0x40;
    memcpy(data + 1, frame_buffer, SCREEN_WIDTH * PAGE_COUNT);
    if (write(i2c_fd, data, sizeof(data)) != (ssize_t)sizeof(data)) {
        perror("Update display"); exit(1);
    }
    memcpy(prev_buffer, frame_buffer, sizeof(frame_buffer));
}

/* Only push pages that actually changed */
void ssd1306_update_display() {
    for (int page = 0; page < PAGE_COUNT; page++) {
        uint8_t *cur = frame_buffer + page * SCREEN_WIDTH;
        uint8_t *prv = prev_buffer  + page * SCREEN_WIDTH;
        if (memcmp(cur, prv, SCREEN_WIDTH) == 0) continue;

        int first = 0;
        while (first < SCREEN_WIDTH && cur[first] == prv[first]) first++;
        int last = SCREEN_WIDTH - 1;
        while (last > first && cur[last] == prv[last]) last--;
        int len = last - first + 1;

        ssd1306_send_command(0x21);
        ssd1306_send_command(first);
        ssd1306_send_command(last);
        ssd1306_send_command(0x22);
        ssd1306_send_command(page);
        ssd1306_send_command(page);

        uint8_t buf[1 + SCREEN_WIDTH];
        buf[0] = 0x40;
        memcpy(buf + 1, cur + first, len);
        if (write(i2c_fd, buf, 1 + len) != 1 + len) { perror("Update display"); exit(1); }
        memcpy(prv + first, cur + first, len);
    }
}

void ssd1306_set_pixel(int x, int y, uint8_t color) {
    x += X_OFFSET;
    if (x < 0 || y < 0 || x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) return;
    int index = x + (y / 8) * SCREEN_WIDTH;
    uint8_t bit = 1 << (y % 8);
    if (color) frame_buffer[index] |= bit;
    else       frame_buffer[index] &= ~bit;
}

void ssd1306_clear_display() { memset(frame_buffer, 0, sizeof(frame_buffer)); }

/* --- primitives --- */
void fill_rect(int x, int y, int w, int h, uint8_t color) {
    for (int i = 0; i < w; i++)
        for (int j = 0; j < h; j++)
            ssd1306_set_pixel(x + i, y + j, color);
}

void ssd1306_draw_line(int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        ssd1306_set_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void draw_rect(int x, int y, int w, int h, uint8_t color) {
    ssd1306_draw_line(x,         y,         x + w - 1, y,         color);
    ssd1306_draw_line(x,         y + h - 1, x + w - 1, y + h - 1, color);
    ssd1306_draw_line(x,         y,         x,         y + h - 1, color);
    ssd1306_draw_line(x + w - 1, y,         x + w - 1, y + h - 1, color);
}

void ssd1306_draw_filled_ellipse(int x0, int y0, int rx, int ry, uint8_t color) {
    int rx2 = rx * rx, ry2 = ry * ry;
    for (int dy = -ry; dy <= ry; dy++)
        for (int dx = -rx; dx <= rx; dx++)
            if ((dx * dx * ry2 + dy * dy * rx2) <= (rx2 * ry2))
                ssd1306_set_pixel(x0 + dx, y0 + dy, color);
}

void ssd1306_draw_char(int x, int y, char c) {
    if (c < 32 || c > 126) return;
    int idx = c - 32;
    for (int col = 0; col < 5; col++) {
        uint8_t data = font5x8[idx][col];
        for (int row = 0; row < 8; row++)
            ssd1306_set_pixel(x + col, y + row, (data >> row) & 1);
    }
}


void ssd1306_draw_text_clipped(int x, int y, const char *str, int max_x) {
    while (*str) {
        if (x + 5 > max_x) break;
        ssd1306_draw_char(x, y, *str++);
        x += 6;
    }
}

void ssd1306_draw_text(int x, int y, const char *str) {
    ssd1306_draw_text_clipped(x, y, str, CONTENT_W - 1);
}

/* --- mouth --- */
void draw_mouth(int cx, int cy, int w, int h, const char *emotion) {
    int half = w / 2;

    if (strcmp(emotion, "happy") == 0) {
        for (int i = 0; i <= w; i++) {
            float t = (float)i / (float)w;
            float s = sinf(PI * t);
            int x   = cx - half + i;
            int top = cy - (int)(4.0f * s);
            int bot = cy + (int)((float)h * s);
            int th  = bot - top;
            if (th < 4) th = 4;
            fill_rect(x, top, 1, th, 1);
        }
        for (int k = 0; k < 3; k++) {
            ssd1306_draw_line(cx - half,     cy + k, cx - half - 6, cy - 6 + k, 1);
            ssd1306_draw_line(cx + half,     cy + k, cx + half + 6, cy - 6 + k, 1);
        }

    } else if (strcmp(emotion, "sad") == 0) {
        int drop = (int)(h * 0.7f);
        for (int i = 0; i <= w; i++) {
            float t = (float)i / (float)w;
            float s = sinf(PI * t);
            int x = cx - half + i;
            int y = cy + (int)((float)drop * (1.0f - s));
            fill_rect(x, y, 1, 6, 1);
        }
        fill_rect(cx - half - 1, cy + drop + 5, 3, 5, 1);
        fill_rect(cx + half - 1, cy + drop + 5, 3, 5, 1);

    } else if (strcmp(emotion, "angry") == 0) {
        int bw = w - 6, bh = 12;
        int bx = cx - bw / 2, by = cy - bh / 2;
        fill_rect(bx, by, bw, bh, 1);
        for (int x = bx + 7; x < bx + bw - 4; x += 8)
            fill_rect(x, by, 2, bh, 0);                   /* tooth gaps */
        fill_rect(bx + 1, by + bh / 2 - 1, bw - 2, 2, 0); /* gum line */
        for (int k = 0; k < 3; k++) {                     /* flared corners */
            ssd1306_draw_line(bx,          by + k, bx - 7,      by - 6 + k, 1);
            ssd1306_draw_line(bx + bw - 1, by + k, bx + bw + 6, by - 6 + k, 1);
        }

    } else if (strcmp(emotion, "surprised") == 0) {
        int rx = w / 4, ry = h;
        ssd1306_draw_filled_ellipse(cx, cy, rx,     ry,     1);
        ssd1306_draw_filled_ellipse(cx, cy, rx - 4, ry - 4, 0);
        ssd1306_draw_filled_ellipse(cx, cy + ry - 7, rx - 7, 4, 1);

    } else {
        fill_rect(cx - half + 6, cy - 2, w - 12, 6, 1);
        fill_rect(cx - half + 1, cy - 5, 6,      6, 1);
        fill_rect(cx + half - 6, cy - 5, 6,      6, 1);
    }
}

/* --- bottom status panel --- */
void draw_battery_icon(int x, int y, int w, int h, int pct) {
    draw_rect(x, y, w, h, 1);
    fill_rect(x + w, y + h / 2 - 1, 2, 2, 1);            /* terminal nub */
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    int inner = w - 4;
    int fill  = (inner * pct) / 100;
    if (fill > 0) fill_rect(x + 2, y + 2, fill, h - 4, 1);
}

void draw_status_bar(const char *status, int battery) {
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", battery);
    int pct_w  = (int)strlen(pct) * 6;
    int pct_x  = CONTENT_W - 4 - pct_w;
    int icon_x = pct_x - 18;

    draw_rect(0, BAR_Y, CONTENT_W, BAR_H, 1);
    ssd1306_set_pixel(0,             BAR_Y,             0);
    ssd1306_set_pixel(CONTENT_W - 1, BAR_Y,             0);
    ssd1306_set_pixel(0,             SCREEN_HEIGHT - 1, 0);
    ssd1306_set_pixel(CONTENT_W - 1, SCREEN_HEIGHT - 1, 0);

    fill_rect(icon_x - 5, BAR_Y + 3, 1, BAR_H - 6, 1);  

    ssd1306_draw_text_clipped(4, BAR_TEXT_Y, status, icon_x - 8);
    draw_battery_icon(icon_x, BAR_TEXT_Y, 13, 8, battery);
    ssd1306_draw_text(pct_x, BAR_TEXT_Y, pct);
}

/* --- battery log --- */
int read_latest_battery_value(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) { perror("Log open"); return -1; }
    char line[256], last[256] = {0};
    while (fgets(line, sizeof(line), fp)) strcpy(last, line);
    fclose(fp);
    int battery = -1;
    sscanf(last, "[%*[^]]] Battery: %d%%", &battery);
    return battery;
}

const char* battery_emotion(int battery) {
    if (battery >= 80) return "happy";
    else if (battery >= 50) return "neutral";
    else if (battery >= 20) return "sad";
    else return "angry";
}

/* --- state reader --- */
void read_state(char *state, size_t n) {
    FILE *fp = fopen(STATE_FILE, "r");
    if (fp) {
        if (!fgets(state, (int)n, fp)) strcpy(state, "idle");
        state[strcspn(state, "\n")] = 0;
        fclose(fp);
    } else {
        strcpy(state, "idle");
    }
}

/* --- main --- */
int main(void) {
    ssd1306_init();
    ssd1306_clear_display();
    ssd1306_update_display_full();     

    time_t last_battery_time = time(NULL) - 60;
    int battery = 0;
    int frame = 0;

    while (1) {
        time_t now = time(NULL);
        if (now - last_battery_time >= 60) {
            battery = read_latest_battery_value(LOG_FILE);
            if (battery < 0) battery = 0;
            last_battery_time = now;
        }

        char state[20];
        read_state(state, sizeof(state));

        ssd1306_clear_display();
        const char *emotion = battery_emotion(battery);
        char display_state[24] = {0};

        if (strcmp(state, "idle") == 0) {
            strcpy(display_state, "Idle");
            draw_mouth(MOUTH_CX, MOUTH_CY, MOUTH_W, MOUTH_H, emotion);

        } else if (strcmp(state, "listening") == 0) {
            strcpy(display_state, "Listening");
            draw_mouth(MOUTH_CX, MOUTH_CY, MOUTH_W, MOUTH_H, "neutral");

        } else if (strcmp(state, "thinking") == 0) {
            strcpy(display_state, "Thinking");
            int dots = (frame / 3) % 4;                  
            for (int i = 0; i < dots; i++) strcat(display_state, ".");
            draw_mouth(MOUTH_CX, MOUTH_CY, MOUTH_W, MOUTH_H, "neutral");

        } else if (strcmp(state, "speaking") == 0) {
            strcpy(display_state, "Speaking");
            const char *m = (frame % 2 == 0) ? "surprised" : "neutral";
            draw_mouth(MOUTH_CX, MOUTH_CY, MOUTH_W, MOUTH_H, m);

        } else {
            strncpy(display_state, state, sizeof(display_state) - 1);
            draw_mouth(MOUTH_CX, MOUTH_CY, MOUTH_W, MOUTH_H, emotion);
        }

        draw_status_bar(display_state, battery);
        ssd1306_update_display();
        frame++;
        usleep(200000);
    }

    close(i2c_fd);
    return 0;
}