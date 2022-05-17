#include    "dosbox.h"
#include    "inout.h"
#include    "logging.h"
#include    "np2glue.h"
//#include	"compiler.h"
//#include	"pccore.h"
//#include	"iocore.h"
#include	"sound.h"
#include	"fmboard.h"

static IO_ReadHandleObject ReadHandler86[6];

static Bitu pcm86_ia460(Bitu port, Bitu /*iolen*/) {
    LOG_MSG("read port a460h: Always return 0x43");
    return 0x43;
}

static Bitu pcm86_ia466(Bitu port, Bitu /*iolen*/) {
    // Port 0xa466 Read: FIFO status
    LOG_MSG("read port a466h: Always return 0x80 (FIFO Full)");
    return 0x80;
}

static Bitu pcm86_ia468(Bitu port, Bitu /*iolen*/) {
    // Port 0xa468 Read: FIFO control
    LOG_MSG("read port a468h: Always return 0");
    return 0;
}

static Bitu pcm86_ia46a(Bitu port, Bitu /*iolen*/) {
    // Port 0xa46a Read: D/A converter control
    LOG_MSG("read port a468h: Always return 0xB2");
    return 0xB2;
}

static Bitu pcm86_ia46c(Bitu port, Bitu /*iolen*/) {
    // Port 0xa46c Read: Read FIFO
    LOG_MSG("read port a46ch: Always return 0");
    return 0;
}

static Bitu pcm86_ia66e(Bitu port, Bitu /*iolen*/) {
    // Port 0xa66e Read: Mute control
    LOG_MSG("read port a66eh: Always return 0");
    return 0;
}

void set_86pcmio(void) {
    ReadHandler86[0].Uninstall();
    ReadHandler86[1].Uninstall();
    ReadHandler86[2].Uninstall();
    ReadHandler86[3].Uninstall();
    ReadHandler86[4].Uninstall();
    ReadHandler86[5].Uninstall();
    ReadHandler86[0].Install(0xa460, pcm86_ia460, IO_MB);
    ReadHandler86[1].Install(0xa466, pcm86_ia466, IO_MB);
    ReadHandler86[2].Install(0xa468, pcm86_ia468, IO_MB);
    ReadHandler86[3].Install(0xa46a, pcm86_ia46a, IO_MB);
    ReadHandler86[4].Install(0xa46c, pcm86_ia46c, IO_MB);
    ReadHandler86[5].Install(0xa66e, pcm86_ia66e, IO_MB);
}
