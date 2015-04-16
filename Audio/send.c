#include "interpolate.h"

/* simulate sending the packet */
void send_packet(int seqno, int len, char *data, FILE *ofile, int repair_strategy, int use_synthetic) {
  char *pktbuf = malloc(len);
  if (use_synthetic == 0)
    memcpy(pktbuf, data, len);
  transmit_packet(seqno, len, pktbuf, ofile, repair_strategy, use_synthetic);
};

/* simulate the packet traversing the net.  Sometimes it gets lost */
void transmit_packet(int seqno, int len, char *data, FILE* ofile, int repair_strategy, int use_synthetic) {
  long r = random();
  if (use_synthetic == 0) {
    if ( (r%1000000)/1000000.0 < LOSS_PROB) {
      /* ooops - the packet was lost */
      free(data);
      return;
    }
    recv_packet(seqno, len, data, ofile, repair_strategy);
  } else {
    /* we're in synthetic mode - generate synthetic test data */
    int i;
    int16_t* samples = (int16_t*)data;
    static int pitch = 0;
    static int gaplen = 0;
    static int skip = 0;
    if (gaplen > 4)
      return;
    if (skip > 0) {
      skip--;
      return;
    }
    if (pitch == 0) {
      for(i = 0; i < len/2; i++)
	samples[i] = 25000*sin(i/4.0);
    } else {
      for(i = 0; i < len/2; i++)
	samples[i] = 20000*sin(i/10.0);
    }  
    recv_packet(seqno, len, data, ofile, repair_strategy);
    pitch = 1 - pitch;
    gaplen++;
    skip = gaplen;
    seqno++;
  }
}

