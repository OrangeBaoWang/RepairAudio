#include "interpolate.h"

void usage() {
    printf("Usage:\n");
    printf("interpolate --help\n");
    printf("  prints this usage message\n");
    printf("interpolate [--synthetic] [--printhist] [--strategy <strategy name>] [--in <infilename>] [--out <outfilename>]\n");
    printf("  reads from file in.wav, emulates packet loss, and writes to file out.wav\n");
    printf("  Optional parameters:\n");
    printf("    --printhist\n");
    printf("        Prints a histogram of sample values in the output stream.\n");
    printf("        This should be similar to that in the input stream of your code probably has a bug\n");
    printf("    --synthetic\n");
    printf("        Instead of reading from in.wav, generate synthetic test data with 1, 2, 3 and 4 packet loses.\n");
    printf("        Alternate between high and low frequency data between each of these loss itervals.\n");
    printf("    --strategy <strategy name>\n");
    printf("        Specify the repair strategy to be used.\n");
    printf("        <strategy name> can be one of silence, repeat_prev, repeat_next, repeat_both, interpolate\n");
    printf("    --in <infilename>\n");
    printf("        Read audio from <infilename> instead of in.wav\n");
    printf("    --out <outfilename>\n");
    printf("        Write audio to <outfilename> instead of out.wav\n");
    exit(0);
}

int main(int argc, char **argv) {
    FILE *ifile;
    FILE *ofile;
    char *dbuf;
    char *ifilename = "in.wav";
    char *ofilename = "out.wav";
    int use_synthetic = 0; /* generate synthetic test data instead of reading input file */
    int printhist = 0; /* print a histogram of values in the input data - only intended for debugging */
    int msPerPacket, bytesPerPacket, i, seqno = 0, filesize, size;
    int samplesPerPacket;
    int hist[32], j;
    int16_t *samples;
    
    int strategy = REPAIR_STRATEGY;
    printf("Analyzing paramenter:\n");
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--synthetic") == 0) {
            /* use synthetic test data instead of read audio data - makes it
             much easier to see in audacity what's going on */
            use_synthetic = 1;
        } else if (strcmp(argv[i], "--printhist") == 0) {
            /* print input data histogram */
            printhist = 1;
        } else if (strcmp(argv[i], "--help") == 0) {
            usage();
        } else if (strcmp(argv[i], "--in") == 0) {
            if (argc < i+2) {
                usage();
            }
            ifilename = argv[i+1];
            i++;
        } else if (strcmp(argv[i], "--out") == 0) {
            if (argc < i+2) {
                usage();
            }
            ofilename = argv[i+1];
            i++;
        } else if (strcmp(argv[i], "--strategy") == 0) {
            printf("Strategy detected.\n");

            if (argc < i+2)
                usage();
            if (strcmp(argv[i+1], "silence") == 0) {
                printf("I'll use Silence.\n\n");
                strategy = SILENCE;
            } else if (strcmp(argv[i+1], "repeat_prev") == 0) {
                strategy = REPEAT_PREV;
                printf("I'll use Precedent Packet.\n\n");
            } else if (strcmp(argv[i+1], "repeat_next") == 0) {
                printf("I'll use Following Packet.\n\n");
                strategy = REPEAT_NEXT;
            } else if (strcmp(argv[i+1], "repeat_both") == 0) {
                printf("I'll repeat both.\n\n");
                strategy = REPEAT_BOTH;
            } else if (strcmp(argv[i+1], "interpolate") == 0) {
                printf("I'll interpolate.\n\n");
                strategy = INTERPOLATE;
            } else {
                fprintf(stderr, "Unknown strategy: %s\n", argv[i+1]);
                exit(1);
            }
            i++;
        } else {
            usage();
        }
    }
    if (use_synthetic && printhist) {
        fprintf(stderr, "Can't specify both --printhist and --synthetic\n");
        exit(1);
    }
    
    ofile = fopen(ofilename, "w+");
    if (ofile == 0) {
        fprintf(stderr, "Output file %s creation failed\n", ofilename);
        exit(1);
    }
    
    msPerPacket = MS_PER_PACKET;
    if (use_synthetic == 1) {
        /* synthetic data still needs a wav file header */
        ifilename = "/Users/Fabiuz/Desktop/Multimedia System/dummy.wav";
    }
    
    ifile = fopen(ifilename, "r");
    if (ifile == 0) {
        fprintf(stderr, "Input file %s not found\n", ifilename);
        exit(1);
    }
    filesize = process_wav_header(ifile, ofile, msPerPacket, &bytesPerPacket);
    
    /* Assume 16 bit linear data */
    samplesPerPacket = bytesPerPacket/2;
    printf("samplesPerPacket: %d\n", samplesPerPacket);
    /* initialize the DCT for later usage.  If you want to run it on
     less than a whole packet, you'll need to change this */
    init_dct(samplesPerPacket);
    
    if (printhist) {
        for (j=0; j < 32; j++) {
            hist[j]=0;
        }
    }
    
    dbuf = malloc(bytesPerPacket);
    for (i=0; i < filesize / bytesPerPacket; i++) {
        printf("%d ", i);
        fflush(stdout);
        if (use_synthetic == 0) {
            size = fread(dbuf, 1, bytesPerPacket, ifile);
            if (size < bytesPerPacket) {
                printf("EOF\n");
                exit(0);
            }
        } else {
            /* we'll generate synthetic data in send_packet() */
            size = bytesPerPacket;
        }
        
        if (printhist) {
            samples = (int16_t*)dbuf;
            for (j=0; j < bytesPerPacket/2; j++) {
                hist[(samples[j]/2048) + 16]++;
            }
        }
        send_packet(seqno++, bytesPerPacket, dbuf, ofile, strategy, use_synthetic);
    }
    
    if (printhist) {
        printf("Input data histogram: \n");
        for (j=0; j < 32; j++) {
            printf("%3d: %10d\n", j-16, hist[j]);
        }
    }
}

