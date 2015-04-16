#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>

#include "wav.h"

#define SILENCE 0
#define REPEAT_PREV 1
#define REPEAT_NEXT 2
#define REPEAT_BOTH 3
#define INTERPOLATE 4

#include "config.h"

void send_packet(int seqno, int len, char *data, FILE *ofile, int repair_strategy, int use_synthetic);
void transmit_packet(int seqno, int len, char *data, FILE* ofile, int strategy, int use_synthetic);
void recv_packet(int seqno, int len, char *data, FILE *ofile, int strategy);
int process_wav_header(FILE *ifile, FILE *ofile, int msPerPacket, int *bytesPerPacket);
void init_dct(int N_init);
int *dct(int16_t *idata, int num_samples);
int16_t *idct(int *idata, int num_samples);
