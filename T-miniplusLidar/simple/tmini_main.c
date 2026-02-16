#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#define TempLen_Max     1024
#define Lidar_HeaderLSB 0xAA
#define Lidar_HeaderMSB 0x55

// 사용자님 구조체 정의
typedef struct { uint16_t Intensity; uint16_t SI_dis; } DataSI_t;
typedef struct {
    uint16_t PH, FSA, LSA, CS;
    uint8_t CT, LSN;
    DataSI_t SI[360];
} TminiPlus_MsgData_t;

// 전역 변수
uint8_t g_recvbuf[TempLen_Max];
TminiPlus_MsgData_t timiplus_msg;

// 사용자님 거리/각도 변환 로직
int Get_Tminidis(uint16_t dis_temp) {
    return ((dis_temp >> 8) << 6) | ((dis_temp & 0x00FF) >> 2);
}

double Get_Start_Stop_Tminiangle(uint16_t S_angle) {
    return (double)((S_angle >> 1) / 64.0);
}

uint8_t Tmini_checkout(TminiPlus_MsgData_t *msg) {
    uint16_t result = 0;
    uint16_t second = (msg->LSN << 8) | msg->CT;
    uint8_t len_temp = msg->LSN;
    result = msg->PH ^ second ^ msg->FSA ^ msg->LSA;
    while(len_temp--) {
        result ^= msg->SI[len_temp].Intensity;
        result ^= msg->SI[len_temp].SI_dis;
    }
    return (result == msg->CS) ? 0 : 1;
}

void Deal_Radar() {
    TminiPlus_MsgData_t *p = &timiplus_msg;
    p->PH = g_recvbuf[1] << 8 | g_recvbuf[0];
    p->CT = g_recvbuf[2];
    p->LSN = g_recvbuf[3];
    p->FSA = g_recvbuf[5] << 8 | g_recvbuf[4];
    p->LSA = g_recvbuf[7] << 8 | g_recvbuf[6];
    p->CS  = g_recvbuf[9] << 8 | g_recvbuf[8];

    for (int i = 0; i < p->LSN; i++) {
        p->SI[i].Intensity = g_recvbuf[10 + i*3];
        p->SI[i].SI_dis = g_recvbuf[10 + i*3 + 2] << 8 | g_recvbuf[10 + i*3 + 1];
    }

    if (Tmini_checkout(p) == 0) {
        printf("[Valid] FSA: %.2f, LSA: %.2f, Dist: %dmm, LSN: %d\n", 
               Get_Start_Stop_Tminiangle(p->FSA), 
               Get_Start_Stop_Tminiangle(p->LSA), 
               Get_Tminidis(p->SI[0].SI_dis), p->LSN);
    }
}

int main() {
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd < 0) return 1;

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B230400); cfsetispeed(&tty, B230400);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    tty.c_lflag = 0; tty.c_oflag = 0; tty.c_iflag = 0;
    tty.c_cc[VMIN] = 1; tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);

    unsigned char start_cmd[] = {0xA5, 0x60};
    write(fd, start_cmd, 2);

    uint8_t rxtemp, step = 0;
    uint16_t si_len = 0, si_index = 0;

    while (read(fd, &rxtemp, 1) > 0) {
        switch (step) {
            case 0: if (rxtemp == Lidar_HeaderLSB) { g_recvbuf[0] = rxtemp; step = 1; } break;
            case 1: if (rxtemp == Lidar_HeaderMSB) { g_recvbuf[1] = rxtemp; step = 2; } else step = 0; break;
            case 2: g_recvbuf[2] = rxtemp; step = 3; break;
            case 3: g_recvbuf[3] = rxtemp; si_len = rxtemp * 3; step = 4; break;
            case 4: case 5: case 6: case 7: case 8: case 9: g_recvbuf[step++] = rxtemp; break;
            case 10:
                g_recvbuf[10 + si_index++] = rxtemp;
                if (si_index >= si_len) {
                    Deal_Radar();
                    step = 0; si_index = 0;
                }
                break;
        }
    }
    close(fd);
    return 0;
}
