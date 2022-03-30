/*
 *  Copyright (C) 2002-2021  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include <time.h>
#include <math.h>

#include "dosbox.h"
#include "timer.h"
#include "cpu.h"
#include "pic.h"
#include "inout.h"
#include "mem.h"
#include "bios_disk.h"
#include "setup.h"
#include "cross.h" //fmod on certain platforms
#include "control.h"
bool date_host_forced=false;
#if defined (WIN32) && !defined (__MINGW32__)
#include "sys/timeb.h"
#else
#include "sys/time.h"
#endif

// sigh... Windows doesn't know gettimeofday
#if defined (WIN32) && !defined (__MINGW32__)
typedef Bitu suseconds_t;

struct timeval {
    time_t tv_sec;
    suseconds_t tv_usec;
};

static void gettimeofday (timeval* ptime, void* pdummy) {
    struct _timeb thetime;
    _ftime(&thetime);

    ptime->tv_sec = thetime.time;
    ptime->tv_usec = (Bitu)thetime.millitm;
}

#endif

static struct {
    uint8_t regs[0x40];
    bool nmi;
    bool bcd;
    bool ampm;                  // am/pm mode (false = 24h mode)
    bool lock;                  // lock bit set (no updates)
    uint8_t reg;
    struct {
        bool enabled;
        uint8_t rs; // RS0-3 of Status Register A
        uint8_t dv; // DV0-2 of Status Register A
        float delay;
        bool acknowledged;
    } timer;
    struct {
        double timer;
        double ended;
        double alarm;
    } last;
    bool update_ended;
    time_t time_diff;           // difference between real UTC and DOSbox UTC
    struct timeval locktime;    // UTC time of setting lock bit
    struct timeval safetime;    // UTC time of last safe time
} cmos;

static bool read_regc = false;
static bool update_reset = false; // reset flag of 5-7th bit of Status register A
static bool update_start = true;
static double error = 0;  // accumulated error between target and actual time of firing IRQ
static double error1 = 0; // error between target and actual time of firing IRQ
#define DIV 4

static void cmos_timerevent(Bitu val) {
    (void)val;//UNUSED
    double index = PIC_FullIndex();
    double remd = cmos.last.timer + cmos.timer.delay - index; // remaining time to fire IRQ8
    if(update_reset) update_start = false;
    if(index <= cmos.last.timer) LOG_MSG("CMOS: index=%f, cmos.last.timer=%f", index, cmos.last.timer);
    if(!(cmos.regs[0x0a] & 0x80) && (index >= cmos.last.ended + 1000 - 1.984 - 0.224 - 0.001)) {
        if(!update_reset) {
            if(update_start) cmos.regs[0x0a] |= 0x80; //set UIP flag (registers under update)
        }
        else {
            update_start = true; // skip update flag once after reset flag is gone (flag starts 1.5sec after reset) 
        }
        //LOG_MSG("CMOS_timerevent: UIP flag update %d index %f", update_start, index);
    }
    
    if(remd >= cmos.timer.delay / DIV) {
        //PIC_RemoveEvents(cmos_timerevent);
        if(read_regc) {
            read_regc = false;
            cmos.regs[0x0c] = 0; // Hack: set time lag to reset Status Register C (cmos.timer.delay / DIV)
        }
        if((remd >= (double)cmos.timer.delay / DIV * 2) || (-error < (double)(cmos.timer.delay / DIV))) {
            // goto next step
            // skip if error is larger than next step
            PIC_AddEvent(cmos_timerevent, (float)(cmos.timer.delay / DIV));
            return;
        }
    }
    /* else if(remd >= 0.01) {
        PIC_AddEvent(cmos_timerevent, (float)(remd));
        return;
    } */
    cmos.timer.acknowledged = false;
    error1 = index - cmos.last.timer - cmos.timer.delay;
    error += error1;
    bool fired_IRQ8 = false;
    if(cmos.timer.enabled && cmos.timer.rs) {
        PIC_ActivateIRQ(8); //Fire IRQ only when timer is enabled
        fired_IRQ8 = true;
        //LOG_MSG("CMOS: IRQ8 (PI) interval = %f delay=%f error=%f error1=%f", index - cmos.last.timer, cmos.timer.delay, error, error1);
    }
    cmos.last.timer = index;
    if (cmos.timer.rs) cmos.regs[0xc] |= 0x40;    // set Periodic Interrupt Flag (PF)
    if((index >= (cmos.last.ended + 1000 - (cmos.timer.delay / DIV))) && update_start) { // only when DV flag != reset (update_start = true) 
        if(!fired_IRQ8 && (cmos.regs[0x0b] & 0x10u)) { // don't fire IRQ twice on same time
            PIC_ActivateIRQ(8); // Fire IRQ if UI enabled and PI not enabled
            //LOG_MSG("CMOS: IRQ8 (UI) interval = %f delay=%f error=%f error1=%f", index - cmos.last.timer, cmos.timer.delay, error, error1);
        }
        //LOG_MSG("CMOS: IRQ8 (UF) index = %f interval = %f", index, index - cmos.last.ended);
        cmos.last.ended = index;
        cmos.regs[0xc] |= 0x10;    // set Update-Ended Interrupt Flag (UF)
        cmos.regs[0x0a] &= 0x7f;   // reset UIP flag
    }
    double next_delay = cmos.timer.delay / DIV - error; //set next step considering time errors
    if(next_delay <= 0) {
        next_delay = 0.001;
        LOG_MSG("CMOS: next_delay negative, set 0.001");
    }
    PIC_AddEvent(cmos_timerevent, (float)next_delay);
    return;
}

static void cmos_checktimer(void) {
    PIC_RemoveEvents(cmos_timerevent);
    cmos.timer.enabled = (cmos.regs[0x0b] & 0x40u); // check PI flag (fire IRQ8 or not)
    if (cmos.timer.rs<=2) cmos.timer.rs+=7; // Special care for rs=1(3.0625ms same as rs=8) 2(7.8125ms same as rs=9)
    cmos.timer.delay=(1000.0f/(32768.0f / (1 << (cmos.timer.rs - 1))));
    LOG(LOG_PIT,LOG_NORMAL)("RTC Timer at %.2f hz",1000.0/cmos.timer.delay);
    /* A rtc is always running */
    error = 0; // reset accumulated errors 
    PIC_AddEvent(cmos_timerevent,(float)(cmos.timer.delay / DIV)); // Time steps are defined by rs, but run steps in shorter steps
}

void cmos_selreg(Bitu port,Bitu val,Bitu iolen) {
    (void)port;//UNUSED
    (void)iolen;//UNUSED
    if (machine != MCH_PCJR) {
        /* bit 7 also controls NMI masking, if set, NMI is disabled */
        CPU_NMI_gate = (val&0x80) ? false : true;
    }

    cmos.reg=val & 0x3f;
    cmos.nmi=(val & 0x80)>0;
}

static void cmos_writereg(Bitu port,Bitu val,Bitu iolen) {
    (void)port;//UNUSED
    (void)iolen;//UNUSED
    if (date_host_forced && (cmos.reg <= 0x09 || cmos.reg == 0x32)) {   // date/time related registers
        if (cmos.bcd)           // values supplied are BCD, convert to binary values
        {
            if ((val & 0xf0) > 0x90 || (val & 0x0f) > 0x09) return;     // invalid BCD value
            // other checks for valid values are done in case-switch

            // convert pm hours differently (bcd 81-92 corresponds to hex 81-8c)
            if (cmos.reg == 0x04 && val >= 0x80)
            {
                val = (val < 90) ? 0x80 : 0x8a + (val & 0x0f);
            }
            else
            {
                val = ((val >> 4) * 10) + (val & 0x0f);
            }
        }

        struct tm *loctime;         // local dosbox time (based on dosbox UTC)

        if (cmos.lock)              // if locked, use locktime instead of current time
        {
            loctime = localtime((const time_t*)&cmos.locktime.tv_sec);
        }
        else                        // not locked, use current time
        {
            struct timeval curtime;
            gettimeofday(&curtime, NULL);
            curtime.tv_sec += cmos.time_diff;
            loctime = localtime((const time_t*)&curtime.tv_sec);
        }

        switch (cmos.reg)
        {
        case 0x00:      /* Seconds */
            if (val > 59) return;       // invalid seconds value
            loctime->tm_sec = (int)val;
            break;

        case 0x02:      /* Minutes */
            if (val > 59) return;       // invalid minutes value
            loctime->tm_min = (int)val;
            break;

        case 0x04:      /* Hours */
            if (cmos.ampm)              // 12h am/pm mode
            {
                if ((val > 12 && val < 0x81) || val > 0x8c) return; // invalid hour value
                if (val > 12) val -= (0x80-12);         // convert pm to 24h
            }
            else                        // 24h mode
            {
                if (val > 23) return;                               // invalid hour value
            }

            loctime->tm_hour = (int)val;         
            break;

        case 0x06:      /* Day of week */
            // seems silly to set this, as it is calculated? ignore for now
            break;

        case 0x07:      /* Date of month */
            if (val > 31) return;               // invalid date value (mktime() should catch the rest)
            loctime->tm_mday = (int)val;
            break;

        case 0x08:      /* Month */
            if (val > 12) return;               // invalid month value
            loctime->tm_mon = (int)val;
            break;

        case 0x09:      /* Year */
            loctime->tm_year = (int)val;
            break;

        case 0x32:      /* Century */
            if (val < 19) return;               // invalid century value?
            loctime->tm_year += (int)((val * 100) - 1900);
            break;

        case 0x01:      /* Seconds Alarm */
        case 0x03:      /* Minutes Alarm */
        case 0x05:      /* Hours Alarm */
            LOG(LOG_BIOS,LOG_NORMAL)("CMOS:Writing to an alarm register");
            cmos.regs[cmos.reg] = (uint8_t)val;
            return;     // done
        }

        time_t newtime = mktime(loctime);       // convert new local time back to dosbox UTC

        if (newtime != (time_t)-1)
        {
            if (!cmos.lock)         // no lock, takes immediate effect
            {
                cmos.time_diff = newtime - time(NULL);  // calculate new diff
            }
            else
            {
                cmos.locktime.tv_sec = newtime;         // store for later use
                // no need to set usec, we don't use it
            }
        }
        return;
    }

    switch (cmos.reg) {
    case 0x00:      /* Seconds */
    case 0x02:      /* Minutes */
    case 0x04:      /* Hours */
    case 0x06:      /* Day of week */
    case 0x07:      /* Date of month */
    case 0x08:      /* Month */
    case 0x09:      /* Year */
    case 0x32:      /* Century */
        /* Ignore writes to change alarm */
        break;
    case 0x01:      /* Seconds Alarm */
    case 0x03:      /* Minutes Alarm */
    case 0x05:      /* Hours Alarm */
        LOG(LOG_BIOS,LOG_NORMAL)("CMOS:Writing to an alarm register");
        cmos.regs[cmos.reg]=(uint8_t)val;
        break;
    case 0x0a:      /* Status reg A */
        cmos.regs[cmos.reg]=val & 0x7f;
        if ((val & 0x70)!=0x20) LOG(LOG_BIOS,LOG_ERROR)("CMOS:Illegal 22 stage divider value");
        cmos.timer.rs=(val & 0xf);
        if(((val & 0x7f) >> 4) >= 6) {
            update_reset = true;
            //LOG_MSG("CMOS: regA count up reset");
        }
        else update_reset = false;
        cmos_checktimer();
        break;
    case 0x0b:      /* Status reg B */
        if(date_host_forced) {
            bool waslocked = cmos.lock;
            cmos.ampm = !(val & 0x02);
            cmos.bcd = !(val & 0x04);
            cmos.lock = (val & 0x80) != 0;

            if (cmos.lock)              // if locked, set locktime for later use
            {
                if (!waslocked)         // if already locked, no further action
                {
                    // locked for the first time, calculate dosbox UTC
                    gettimeofday(&cmos.locktime, NULL);
                    cmos.locktime.tv_sec += cmos.time_diff;
                }
            }
            else if (waslocked)         // time was locked, now unlock
            {
                // calculate new diff between real UTC and dosbox UTC
                cmos.time_diff = cmos.locktime.tv_sec - time(NULL);
            }
            cmos.regs[cmos.reg] = (uint8_t)val;
            cmos.timer.enabled = cmos.regs[0x0b] & 0x40u;
            LOG_MSG("CMOS timer.enabled val: %d 0x40: %d 0x10: %d", val, val & 0x40, val & 0x10);
            cmos_checktimer();
        } else {
            cmos.bcd=!(val & 0x4);
            cmos.regs[cmos.reg]=(uint8_t)val;
            cmos.timer.enabled = cmos.regs[0x0b] & 0x40u;
            LOG_MSG("CMOS timer.enabled val: %d 0x40: %d 0x10: %d", val, val & 0x40, val & 0x10);
            cmos_checktimer();
        }
        break;
    case 0x0c:      /* Status reg C */
        break;
    case 0x0d:      /* Status reg D */
        if(!date_host_forced) {
            cmos.regs[cmos.reg]=val & 0x80; /*Bit 7=1:RTC Power on*/
        }
        break;
    case 0x0f:      /* Shutdown status byte */
        cmos.regs[cmos.reg]=val & 0x7f;
        break;
    default:
        LOG(LOG_BIOS, LOG_NORMAL)("CMOS:Writing to register %x", cmos.reg);
        cmos.regs[cmos.reg]=val & 0x7f;
    }
}

unsigned char CMOS_GetShutdownByte() {
    return cmos.regs[0x0F];
}

#define MAKE_RETURN(_VAL) ((unsigned char)(cmos.bcd ? (((((unsigned int)_VAL) / 10U) << 4U) | (((unsigned int)_VAL) % 10U)) : ((unsigned int)_VAL)))

static Bitu cmos_readreg(Bitu port,Bitu iolen) {
    (void)port;//UNUSED
    (void)iolen;//UNUSED
    if (cmos.reg>0x3f) {
        LOG(LOG_BIOS,LOG_ERROR)("CMOS:Read attempted from illegal register %x",cmos.reg);
        return 0xff;
    }

    // JAL_20060817 - rewrote most of the date/time part
    if (date_host_forced && (cmos.reg <= 0x09 || cmos.reg == 0x32)) {       // date/time related registers
        struct tm* loctime;

        if (cmos.lock)              // if locked, use locktime instead of current time
        {
            loctime = localtime((const time_t*)&cmos.locktime.tv_sec);
        }
        else                        // not locked, get current time
        {
            struct timeval curtime;
            gettimeofday(&curtime, NULL);
    
            // allow a little more leeway (1 sec) than the .244 sec officially given
            if (curtime.tv_sec - cmos.safetime.tv_sec == 1 &&
                curtime.tv_usec < cmos.safetime.tv_usec)
            {
                curtime = cmos.safetime;        // within safe range, use safetime instead of current time
            }

            curtime.tv_sec += cmos.time_diff;
            loctime = localtime((const time_t*)&curtime.tv_sec);
        }

        switch (cmos.reg)
        {
        case 0x00:      // seconds
            return MAKE_RETURN(loctime->tm_sec);
        case 0x02:      // minutes
            return MAKE_RETURN(loctime->tm_min);
        case 0x04:      // hours
            if (cmos.ampm && loctime->tm_hour > 12)     // time pm, convert
            {
                loctime->tm_hour -= 12;
                loctime->tm_hour += (cmos.bcd) ? 80 : 0x80;
            }
            return MAKE_RETURN(loctime->tm_hour);
        case 0x06:      /* Day of week */
            return MAKE_RETURN(loctime->tm_wday + 1);
        case 0x07:      /* Date of month */
            return MAKE_RETURN(loctime->tm_mday);
        case 0x08:      /* Month */
            return MAKE_RETURN(loctime->tm_mon + 1);
        case 0x09:      /* Year */
            return MAKE_RETURN(loctime->tm_year % 100);
        case 0x32:      /* Century */
            return MAKE_RETURN(loctime->tm_year / 100 + 19);

        case 0x01:      /* Seconds Alarm */
        case 0x03:      /* Minutes Alarm */
        case 0x05:      /* Hours Alarm */
            return MAKE_RETURN(cmos.regs[cmos.reg]);
        }
    }

    Bitu drive_a, drive_b;
    uint8_t hdparm;
    time_t curtime;
    struct tm *loctime;
    /* Get the current time. */
    curtime = time (NULL);

    /* Convert it to local time representation. */
    loctime = localtime (&curtime);

    switch (cmos.reg) {
    case 0x00:      /* Seconds */
        return    MAKE_RETURN(loctime->tm_sec);
    case 0x02:      /* Minutes */
        return    MAKE_RETURN(loctime->tm_min);
    case 0x04:      /* Hours */
        return    MAKE_RETURN(loctime->tm_hour);
    case 0x06:      /* Day of week */
        return    MAKE_RETURN(loctime->tm_wday + 1);
    case 0x07:      /* Date of month */
        return    MAKE_RETURN(loctime->tm_mday);
    case 0x08:      /* Month */
        return    MAKE_RETURN(loctime->tm_mon + 1);
    case 0x09:      /* Year */
        return    MAKE_RETURN(loctime->tm_year % 100);
    case 0x32:      /* Century */
        return    MAKE_RETURN(loctime->tm_year / 100 + 19);
    case 0x01:      /* Seconds Alarm */
    case 0x03:      /* Minutes Alarm */
    case 0x05:      /* Hours Alarm */
        return cmos.regs[cmos.reg];
    case 0x0a:      /* Status register A */
        if(date_host_forced) {
            // take bit 7 of reg b into account (if set, never updates)
            gettimeofday (&cmos.safetime, NULL);        // get current UTC time
            if (cmos.lock ||                            // if lock then never updated, so reading safe
                cmos.safetime.tv_usec < (1000-244)) {   // if 0, at least 244 usec should be available
                return cmos.regs[0x0a];                 // reading safe
            } else {
                return cmos.regs[0x0a] | 0x80;          // reading not safe!
            }
        } else {
            if (PIC_TickIndex()<0.002) {
                return (cmos.regs[0x0a]&0x7f) | 0x80;
            } else {
                return (cmos.regs[0x0a]&0x7f);
            }
        }
    case 0x0c:      /* Status register C */
    {
        cmos.timer.acknowledged=true;
        const uint8_t val_b = cmos.regs[0xb];
        uint8_t val_c = cmos.regs[0xc];
        if (val_b & val_c & 0x40u) { // If both PF and PIE are 1
            val_c |= 0x80; // Set Interrupt Request Flag (IRQF) to 1
        }
        else if(val_b & val_c & 0x10u) { // If both UF and UIE are 1
            val_c |= 0x80; // Set Interrupt Request Flag (IRQF) to 1
        }
        else if(val_b & val_c & 0x20u) { // If both AF and AIE are 1
            val_c |= 0x80; // Set Interrupt Request Flag (IRQF) to 1
        }
        read_regc = true;
        /* else {
            cmos.regs[0xc] = 0; // All flags are cleared by reading the register
            //Hack: skip clear status register C despite it should be cleared after RESET or software read
            //LOG_MSG("CMOS: Read Register C");
        } */
        return val_c;
    }
    case 0x10:      /* Floppy size */
        drive_a = 0;
        drive_b = 0;
        if(imageDiskList[0] != NULL) drive_a = imageDiskList[0]->GetBiosType();
        if(imageDiskList[1] != NULL) drive_b = imageDiskList[1]->GetBiosType();
        return ((drive_a << 4) | (drive_b));
    /* First harddrive info */
    case 0x12:
        /* NTS: DOSBox 0.74 mainline has these backwards: the upper nibble is the first hard disk,
           the lower nibble is the second hard disk. It makes a big difference to stupid OS's like
           Windows 95. */
        hdparm = 0;
        if(imageDiskList[3] != NULL) hdparm |= 0xf;
        if(imageDiskList[2] != NULL) hdparm |= 0xf0;
//      hdparm = 0;
        return hdparm;
    case 0x19:
        if(imageDiskList[2] != NULL) return 47; /* User defined type */
        return 0;
    case 0x1b:
        if(imageDiskList[2] != NULL) return (imageDiskList[2]->cylinders & 0xff);
        return 0;
    case 0x1c:
        if(imageDiskList[2] != NULL) return ((imageDiskList[2]->cylinders & 0xff00)>>8);
        return 0;
    case 0x1d:
        if(imageDiskList[2] != NULL) return (imageDiskList[2]->heads);
        return 0;
    case 0x1e:
        if(imageDiskList[2] != NULL) return 0xff;
        return 0;
    case 0x1f:
        if(imageDiskList[2] != NULL) return 0xff;
        return 0;
    case 0x20:
        if(imageDiskList[2] != NULL) return (0xc0 | (((imageDiskList[2]->heads) > 8) << 3));
        return 0;
    case 0x21:
        if(imageDiskList[2] != NULL) return (imageDiskList[2]->cylinders & 0xff);
        return 0;
    case 0x22:
        if(imageDiskList[2] != NULL) return ((imageDiskList[2]->cylinders & 0xff00)>>8);
        return 0;
    case 0x23:
        if(imageDiskList[2] != NULL) return (imageDiskList[2]->sectors);
        return 0;
    /* Second harddrive info */
    case 0x1a:
        if(imageDiskList[3] != NULL) return 47; /* User defined type */
        return 0;
    case 0x24:
        if(imageDiskList[3] != NULL) return (imageDiskList[3]->cylinders & 0xff);
        return 0;
    case 0x25:
        if(imageDiskList[3] != NULL) return ((imageDiskList[3]->cylinders & 0xff00)>>8);
        return 0;
    case 0x26:
        if(imageDiskList[3] != NULL) return (imageDiskList[3]->heads);
        return 0;
    case 0x27:
        if(imageDiskList[3] != NULL) return 0xff;
        return 0;
    case 0x28:
        if(imageDiskList[3] != NULL) return 0xff;
        return 0;
    case 0x29:
        if(imageDiskList[3] != NULL) return (0xc0 | (((imageDiskList[3]->heads) > 8) << 3));
        return 0;
    case 0x2a:
        if(imageDiskList[3] != NULL) return (imageDiskList[3]->cylinders & 0xff);
        return 0;
    case 0x2b:
        if(imageDiskList[3] != NULL) return ((imageDiskList[3]->cylinders & 0xff00)>>8);
        return 0;
    case 0x2c:
        if(imageDiskList[3] != NULL) return (imageDiskList[3]->sectors);
        return 0;
    case 0x39:
        return 0;
    case 0x3a:
        return 0;


    case 0x0b:      /* Status register B */
    case 0x0d:      /* Status register D */
    case 0x0f:      /* Shutdown status byte */
    case 0x14:      /* Equipment */
    case 0x15:      /* Base Memory KB Low Byte */
    case 0x16:      /* Base Memory KB High Byte */
    case 0x17:      /* Extended memory in KB Low Byte */
    case 0x18:      /* Extended memory in KB High Byte */
    case 0x30:      /* Extended memory in KB Low Byte */
    case 0x31:      /* Extended memory in KB High Byte */
        //LOG_MSG("CMOS:Read from reg %X : %04X",cmos.reg,cmos.regs[cmos.reg]);
        return cmos.regs[cmos.reg];
    case 0x2F:
        extern bool PS1AudioCard;
        if( PS1AudioCard )
            return 0xFF;
    default:
        LOG(LOG_BIOS,LOG_NORMAL)("CMOS:Reading from register %X",cmos.reg);
        return cmos.regs[cmos.reg];
    }
}

void CMOS_SetRegister(Bitu regNr, uint8_t val) {
    cmos.regs[regNr] = val;
    LOG_MSG("CMOS: set_register %x = %x", regNr, val);
}


static IO_ReadHandleObject ReadHandler[2];
static IO_WriteHandleObject WriteHandler[2];

void CMOS_Destroy(Section* sec) {
    (void)sec;//UNUSED
}

void CMOS_Reset(Section* sec) {
    (void)sec;//UNUSED
    LOG(LOG_MISC,LOG_DEBUG)("CMOS_Reset(): reinitializing CMOS/RTC controller");

    WriteHandler[0].Uninstall();
    WriteHandler[1].Uninstall();
    ReadHandler[0].Uninstall();
    ReadHandler[1].Uninstall();

    if (IS_PC98_ARCH)
        return;

    WriteHandler[0].Install(0x70,cmos_selreg,IO_MB);
    WriteHandler[1].Install(0x71,cmos_writereg,IO_MB);
    ReadHandler[0].Install(0x71,cmos_readreg,IO_MB);
    cmos.timer.enabled=false;
    cmos.timer.acknowledged=true;
    cmos.reg=0xa;
    cmos_writereg(0x71,0x26,1);
    cmos.reg=0xb;
    cmos_writereg(0x71,0x02,1);  //Struct tm *loctime is of 24 hour format,
    cmos.regs[0x0c] = 0;
    if(date_host_forced) {
        cmos.regs[0x0d]=(uint8_t)0x80;
    } else {
        cmos.reg=0xd;
        cmos_writereg(0x71,0x80,1); /* RTC power on */
    }
    // Equipment is updated from bios.cpp and bios_disk.cpp
    /* Fill in base memory size, it is 640K always */
    cmos.regs[0x15]=(uint8_t)0x80;
    cmos.regs[0x16]=(uint8_t)0x02;
    /* Fill in extended memory size */
    Bitu exsize=MEM_TotalPages()*4;
    if (exsize >= 1024) exsize -= 1024;
    else exsize = 0;
    if (exsize > 65535) exsize = 65535; /* cap at 64MB. this value is returned as-is by INT 15H AH=0x88 in a 16-bit register */
    cmos.regs[0x17]=(uint8_t)exsize;
    cmos.regs[0x18]=(uint8_t)(exsize >> 8);
    cmos.regs[0x30]=(uint8_t)exsize;
    cmos.regs[0x31]=(uint8_t)(exsize >> 8);
    if (date_host_forced) {
        cmos.time_diff = 0;
        cmos.locktime.tv_sec = 0;
    }
}

void CMOS_Init() {
    LOG(LOG_MISC,LOG_DEBUG)("Initializing CMOS/RTC");

    if (control->opt_date_host_forced) {
        LOG_MSG("Synchronize date with host: Forced");
        date_host_forced=true;
    }

    AddExitFunction(AddExitFunctionFuncPair(CMOS_Destroy),true);
    AddVMEventFunction(VM_EVENT_RESET,AddVMEventFunctionFuncPair(CMOS_Reset));
}

// save state support
void *cmos_timerevent_PIC_Event = (void*)((uintptr_t)cmos_timerevent);

namespace
{
class SerializeCmos : public SerializeGlobalPOD
{
public:
    SerializeCmos() : SerializeGlobalPOD("CMOS")
    {
        registerPOD(cmos.regs);
        registerPOD(cmos.nmi);
        registerPOD(cmos.reg);
        registerPOD(cmos.timer.enabled);
        registerPOD(cmos.timer.rs);
        registerPOD(cmos.timer.delay);
        registerPOD(cmos.timer.acknowledged);
        registerPOD(cmos.last.timer);
        registerPOD(cmos.last.ended);
        registerPOD(cmos.last.alarm);
        registerPOD(cmos.update_ended);
    }
} dummy;
}
