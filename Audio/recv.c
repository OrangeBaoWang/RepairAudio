#include "interpolate.h"
#include "assert.h"
#include "inttypes.h"
/*
 General notes:
 1. The are working with 16bit, so every SAMPLE occupies 2byte
 */

#define SAMPLE_SIZE 2
#define STARTING_CUT_FREQ 80
#define SAMPLE_TO_MIX 80

/*
 This function mix 2 array. More precisely, it mixes each by each values. Initially, the source has more weight than destionation, finally the situation switches.
 About the steps: if length is 4, the value for a are 0;0.33..;0.66...;1
 Example: Source[1,2,3] and Destination[4,15,30] -->[1,8.5,30].
 */
void mix(int16_t* destination, int16_t* source,int length){
    float a=(float)1/(length-1);
    for(int i=0;i<length;i++){
        int16_t s=source[i];
        int16_t d=destination[i];
        float index=a*i;
        int aux1=(float)(1-index)*s;
        int aux2=(float)(index)*d;
        int finalD=aux1+aux2;
        destination[i]=finalD;
    }
}

/*
 This function copies a certain quantity of cells (specified with numberOfCellsToCopy) in the destination array. It doesn't do any control about buffer overflow, so pay attention.
 */
void copyInt16_t(int16_t* destination,int16_t* source, int destinationStart, int sourceStart,int numberOfCellsToCopy){
    for(int i=0;i<numberOfCellsToCopy;i++){
        destination[destinationStart+i]=source[sourceStart+i];
    }
}

/*
 This function manage the entire smoothing process. NB: newPacket will be modified!!
 INPUT:
    prev: is the previous packet. It must be already written onto the file.
    newPacket: contains the packet that will be modified. Finally it will be written onto the ofile.
    ofile: output file.
 */
void executeGlobalSmooth(int16_t* prev, int16_t* newPacket,FILE *ofile){
    //create HALF packet to smooth. It will be smaller than original size because we want to focus on the junction of the packets
    int16_t* packetToSmooth=malloc(320);
    copyInt16_t(packetToSmooth, prev, 0, 240, 80);
    copyInt16_t(packetToSmooth, newPacket, 80, 0, 80);
    int* coeffDCT=dct(packetToSmooth, 160);
    for(int i=STARTING_CUT_FREQ;i<160;i++){
        coeffDCT[i]=0;
    }
    int16_t* elab=idct(coeffDCT, 160);
    mix(elab,packetToSmooth,SAMPLE_TO_MIX);
    
    //rewrite the first half on already written data
    fseek(ofile, -160, SEEK_CUR);
    fwrite(elab, 2, 80, ofile);
    
    //write the new packet, but before I need to replace the just calculated values
    copyInt16_t(newPacket, elab, 0, 80, 80);
    
    fwrite(newPacket, 2, 320, ofile);
    free(elab);
    free(packetToSmooth);
}

/*
 Return an array of specified length init to 0.
 */
int16_t* zerosInt16_t(int length){
    int16_t* a=malloc(sizeof(int16_t)*length);
    for(int i=0;i<length;i++){
        a[i]=0;
    }
    return a;
}

/* write_silence writes num_samples of silent audio data to ofile */
void write_silence(FILE *ofile, int num_samples) {
    int16_t* missing_pkt=zerosInt16_t(num_samples);
    fwrite(missing_pkt,2 , num_samples, ofile);
}

/* recv_packet processes a received audio packet.  The packet has
   sequence number seqno, and consists of len bytes of data.  In this
   case, it's just PCM, encoded in 16-bit linear format.  The
   "strategy" parameter determines which of several possible loss
   concealment techniques to apply. One packet is composed by 320sample, length in byte 640
    INPUT:
        seqno: the identifier of the current packet
        len: length in byte of the current packet
        data: the array with samples
 */
void recv_packet(int seqno, int len, char *data, FILE *ofile, int strategy) {
  //NB: these 2 variables are STATIC: so this is init only the first time that I call the function
    static int prev_seqno = -1;
    static int16_t* prev_packet_samples = 0;
    int notAlreadyWritten=1;
    int16_t *samples = (int16_t*)data; /* we receive a bunch of bytes
					from the (simulated) net, but
					we know they're really 16 bit
					sample values.  In real life
					we'd convert them from network
					byte order to host byte order,
					but no need to do that here */
  int num_samples = len/SAMPLE_SIZE;  /* two bytes to a 16 bit integer */
  printf("recv_packet: seqno=%d\n", seqno);

    if (prev_seqno != -1 && (prev_seqno+1 != seqno)) {
        /* there was missing data - we need to replace it */
        int missing_seqno;

        switch(strategy) {
            case SILENCE:
                /* create as many packet containing silence as we need to fill the gap */
                missing_seqno = prev_seqno + 1;
                while (missing_seqno < seqno) {
                    write_silence(ofile, num_samples);
                    missing_seqno++;
                }
                break;
            case REPEAT_PREV:
	/* repeat the previous packet once to fill the gap.  If the
	   gap is longer than one packet, fill the remainder with
	   silence */
                fwrite(prev_packet_samples, 2, num_samples, ofile);
                missing_seqno = prev_seqno + 2;
                while (missing_seqno < seqno) {
                    write_silence(ofile, num_samples);
                    missing_seqno++;
                }
                break;
            case REPEAT_NEXT:
                /* play the next packet (twice) to fill the gap.  If the gap
                 is longer than one packet, first insert silence, then
                 insert the next packet */
                missing_seqno = prev_seqno + 1;
                while (missing_seqno < seqno) {
                    if (missing_seqno == seqno-1) {
                        /* only repeat back once */
                        /* write the current packet instead of the missing one */
                        fwrite(data, 2, num_samples, ofile);
                    } else {
                        write_silence(ofile, num_samples);
                    }
                    missing_seqno++;
                }
                break;
            case REPEAT_BOTH:
                /* we'll fill the gap with data from both before and after the
                 gap.  If the gap is one packet long, repeat the last half
                 of the previous packet and the first half of the next
                 packet.  If the gap is two packets long, repeat the whole
                 previous packet, then the whole next packet.  If the gap is
                 three of more packets long, fill the remainder with
                 silence. */
                missing_seqno = prev_seqno + 1;
                if (missing_seqno == seqno-1) {
                    /* the gap is only one packet long */
                    /* fill with last half of prev packet, first half of current packet */

                    int16_t* newPacket=malloc(640);
                    copyInt16_t(newPacket, prev_packet_samples, 0, 160, 160);
                    copyInt16_t(newPacket, samples, 160, 0, 160);
                    
                    //Smooth of the junction in the middle
                    int16_t* packetToSmooth=malloc(320);
                    copyInt16_t(packetToSmooth, newPacket, 0, 80, 160);
                    int* coeffDCT=dct(packetToSmooth, 160);
                    for(int i=STARTING_CUT_FREQ;i<160;i++){
                        coeffDCT[i]=0;
                    }
                    int16_t* elab=idct(coeffDCT, 160);
                    mix(elab,packetToSmooth,SAMPLE_TO_MIX);
                    copyInt16_t(newPacket, elab, 80, 0, 160);
                    
                    executeGlobalSmooth(prev_packet_samples, newPacket, ofile);
                    executeGlobalSmooth(newPacket, samples, ofile);
                    
                } else {
                    /* the gap is two or more packets long */
                    /* first write the prev packet a second time */
                    int16_t* newPacket=malloc(640);
                    copyInt16_t(newPacket, prev_packet_samples, 0, 0, 320);
                    executeGlobalSmooth(prev_packet_samples, newPacket, ofile);
                    
                    missing_seqno++;
                    int16_t* prev_packet=newPacket;
                    while (missing_seqno < seqno) {
                        int16_t* newPacket=malloc(640);
                        if (missing_seqno == seqno-1) {
                            /* write the current packet instead of the missing one */
                            newPacket=samples;
                            executeGlobalSmooth(prev_packet, newPacket, ofile);
                        } else {
                            newPacket=zerosInt16_t(320);
                            executeGlobalSmooth(prev_packet, newPacket, ofile);
                        }
                        prev_packet=newPacket;
                        missing_seqno++;
                    }
                    executeGlobalSmooth(prev_packet, samples, ofile);
                }
                notAlreadyWritten=0;
                break;
            /* We're going to interpolate a whole new packet (or several packets) in the frequency domain */
            case INTERPOLATE:
                printf("");
                int* prevDCT=dct(prev_packet_samples,num_samples);
                int* lastDCT=dct(samples,num_samples);
                
                //number of loss packet
                int lossPacketNumber=seqno-prev_seqno-1;
                
                int* avg=malloc(sizeof(int)*num_samples);
                //a is the single/basic step. For example: if I have only one loss packet, the step must be 0.5; if I have 2 loss packets I must have two steps: 0.3333.. and 0.6666..
                float a=(float)1/((float)(lossPacketNumber+1));
                //elaborate every loss packet
                for(int i=0;i<lossPacketNumber;i++){
                    //elaborate every coeff
                    printf("Elaboration packet number %d with interpolation coeff %f \n",prev_seqno+i+1,a*(i+1));
                    for(int j=0;j<num_samples;j++){
						float index=(float)a*(i+1);
						float aux0=(float)prevDCT[j];
                        float aux1=(float)lastDCT[j];
                        avg[j]=(1-index)*aux0+index*aux1;
                    }

                    int16_t* elaborated=idct(avg, num_samples);
                    
                    fwrite(elaborated, 2, num_samples, ofile);
                }
                break;
        }//switch closed
    }//Mega-if closed:
    
    
    
    //I execute this code even if the packet isn't missed
    /* finally, don't forget to write out the packet that arrived after the gap */
    if(notAlreadyWritten){
        fwrite(samples, 2, num_samples, ofile);
    }
    
    /* hold onto the last received packet - we may need it */
    if (prev_packet_samples != 0)
        free(prev_packet_samples);
    prev_packet_samples = samples;
    prev_seqno = seqno;
}
