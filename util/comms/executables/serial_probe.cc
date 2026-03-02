/*
 * Minimal RS485 diagnostic tool for BLDC controller.
 * No boost, no threads — just POSIX serial I/O in a simple loop.
 *
 * Usage:
 *   serial_probe /dev/ttyUSB0          # boot + probe
 *   serial_probe /dev/ttyUSB0 --skip   # skip bootloader, just listen + probe
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

/* ── CRC-16/ARC (poly=0x8005, reflected) ──────────────────── */

static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ── Protocol constants ───────────────────────────────────── */

static constexpr uint8_t SYNC        = 0xFF;
static constexpr uint8_t VERSION     = 0xFE;
static constexpr uint8_t FC_NOP      = 0x00;
static constexpr uint8_t FC_ENUMERATE = 0xFF;
static constexpr uint8_t FC_CONFIRM  = 0xFE;
static constexpr uint8_t FC_JUMP     = 0x81;

/* ── Packet builder ───────────────────────────────────────── */

static std::vector<uint8_t> build_packet(uint8_t board_id, uint8_t func_code,
                                          const std::vector<uint8_t> &payload = {}) {
    /* Sub-message: [board_id, func_code, payload...] */
    std::vector<uint8_t> sub;
    sub.push_back(board_id);
    sub.push_back(func_code);
    sub.insert(sub.end(), payload.begin(), payload.end());

    /* Wire payload: [sub_len_lo, sub_len_hi, sub_msg...] */
    uint16_t sub_len = (uint16_t)sub.size();
    std::vector<uint8_t> wire;
    wire.push_back(sub_len & 0xFF);
    wire.push_back(sub_len >> 8);
    wire.insert(wire.end(), sub.begin(), sub.end());

    /* CRC over wire payload */
    uint16_t crc = crc16(wire.data(), wire.size());

    /* Full packet */
    uint16_t payload_len = (uint16_t)wire.size();
    std::vector<uint8_t> pkt;
    pkt.push_back(SYNC);
    pkt.push_back(VERSION);
    pkt.push_back(0x00); /* flags = host */
    pkt.push_back(payload_len & 0xFF);
    pkt.push_back(payload_len >> 8);
    pkt.insert(pkt.end(), wire.begin(), wire.end());
    pkt.push_back(crc & 0xFF);
    pkt.push_back(crc >> 8);

    return pkt;
}

/* ── Serial port ──────────────────────────────────────────── */

static int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);

    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);

    tty.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; /* non-blocking */

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    /* Clear O_NONBLOCK after setup so reads return immediately with 0
     * if no data, but writes block until complete */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Set VTIME for 100ms read timeout */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; /* 100ms */
    tcsetattr(fd, TCSANOW, &tty);

    return fd;
}

static void send_pkt(int fd, const std::vector<uint8_t> &pkt) {
    write(fd, pkt.data(), pkt.size());
    tcdrain(fd);
}

static int drain_and_print(int fd, int timeout_ms) {
    uint8_t buf[512];
    int total = 0;
    /* Read until absolute timeout expires */
    struct timeval start, now;
    gettimeofday(&start, nullptr);
    while (1) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (buf[i] >= 0x20 && buf[i] < 0x7F)
                    putchar(buf[i]);
                else if (buf[i] == '\n')
                    putchar('\n');
                else if (buf[i] == '\r')
                    {} /* skip CR */
                else
                    printf("\\x%02X", buf[i]);
            }
            fflush(stdout);
            total += n;
        }
        gettimeofday(&now, nullptr);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                      (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_ms)
            break;
    }
    return total;
}

/* ── Millisecond clock ────────────────────────────────────── */

static uint32_t millis(void) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

/* ── Main ─────────────────────────────────────────────────── */

int main(int argc, char **argv) {
    const char *port = "/dev/ttyUSB0";
    bool skip_boot = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--skip") == 0 || strcmp(argv[i], "-s") == 0)
            skip_boot = true;
        else
            port = argv[i];
    }

    printf("Opening %s at 1 Mbit/s...\n", port);
    int fd = open_serial(port);
    if (fd < 0)
        return 1;

    if (!skip_boot) {
        /* ── Bootloader sequence ──────────────────────────── */
        printf("--- Bootloader sequence ---\n");

        printf("[1] Enumerate board 1...\n");
        send_pkt(fd, build_packet(0, FC_ENUMERATE, {1}));
        drain_and_print(fd, 500);
        printf("\n");

        printf("[2] Confirm board 1...\n");
        send_pkt(fd, build_packet(1, FC_CONFIRM));
        drain_and_print(fd, 200);
        printf("\n");

        printf("[3] Jump to 0x08010000...\n");
        send_pkt(fd, build_packet(1, FC_JUMP, {0x00, 0x00, 0x01, 0x08}));
        printf("    Waiting for firmware to start...\n");
        drain_and_print(fd, 300);
        printf("\n");
    }

    printf("--- Listening (NOP every 2s, Ctrl+C to quit) ---\n\n");

    uint32_t last_nop = millis();
    uint32_t nop_count = 0;
    uint8_t buf[512];

    while (1) {
        /* Read and print any available data */
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (buf[i] >= 0x20 && buf[i] < 0x7F)
                    putchar(buf[i]);
                else if (buf[i] == '\n')
                    putchar('\n');
                else if (buf[i] == '\r')
                    {} /* skip CR */
                else
                    printf("\\x%02X", buf[i]);
            }
            fflush(stdout);
        }

        /* Send NOP every 2 seconds */
        uint32_t now = millis();
        if (now - last_nop >= 2000) {
            last_nop = now;
            nop_count++;
            printf("\n>>> NOP #%u <<<\n", nop_count);
            send_pkt(fd, build_packet(0, FC_NOP));  /* board_id=0 (broadcast) */
        }
    }

    close(fd);
    return 0;
}
