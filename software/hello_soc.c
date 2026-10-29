/*
 * hello_soc.c - Full SoC test firmware for MES50HP (Cortex-M0 + FIR + DMA)
 *
 * Tests over UART0:
 *   A. FIR register access   (COEF_NUM == 82)
 *   B. DMA register access   (VERSION == "DMA1")
 *   C. DMA mem2mem transfer  (SRAM -> SRAM, verified)
 *   D. FIR DC response       (push 160 samples of 1000, check steady-state)
 *   E. UART echo             (board interactive)
 *
 * Memory map:
 *   0x0000_0000  FLASH (boot)
 *   0x2000_0000  SRAM (64KB)
 *   0x4000_4000  UART0
 *   0x4001_0000  GPIO0 (LEDs)
 *   0x5000_0000  FIR filter
 *   0x5001_0000  DMA controller
 */
#include "CMSDK_CM0.h"
#include "core_cm0.h"
#include <stdio.h>

/* ---------------- peripheral addresses ---------------- */
#define FIR_BASE   0x50000000u
#define DMA_BASE   0x50010000u

#define FIR_CTRL      (*(volatile unsigned int*)(FIR_BASE  + 0x00))
#define FIR_STATUS    (*(volatile unsigned int*)(FIR_BASE  + 0x04))
#define FIR_DATA_IN   (*(volatile unsigned int*)(FIR_BASE  + 0x08))
#define FIR_DATA_OUT  (*(volatile unsigned int*)(FIR_BASE  + 0x0C))
#define FIR_IRQ_STAT  (*(volatile unsigned int*)(FIR_BASE  + 0x10))
#define FIR_IRQ_EN    (*(volatile unsigned int*)(FIR_BASE  + 0x14))
#define FIR_COEF_IDX  (*(volatile unsigned int*)(FIR_BASE  + 0x18))
#define FIR_COEF_DATA (*(volatile unsigned int*)(FIR_BASE  + 0x1C))
#define FIR_COEF_NUM  (*(volatile unsigned int*)(FIR_BASE  + 0x20))
#define FIR_IN_CNT    (*(volatile unsigned int*)(FIR_BASE  + 0x24))
#define FIR_OUT_CNT   (*(volatile unsigned int*)(FIR_BASE  + 0x28))

#define DMA_CH_CTRL(n) (*(volatile unsigned int*)(DMA_BASE + (n)*0x100 + 0x00))
#define DMA_CH_SRC(n)  (*(volatile unsigned int*)(DMA_BASE + (n)*0x100 + 0x04))
#define DMA_CH_DST(n)  (*(volatile unsigned int*)(DMA_BASE + (n)*0x100 + 0x08))
#define DMA_CH_LEN(n)  (*(volatile unsigned int*)(DMA_BASE + (n)*0x100 + 0x0C))
#define DMA_CH_CFG(n)  (*(volatile unsigned int*)(DMA_BASE + (n)*0x100 + 0x10))
#define DMA_IRQ_STAT   (*(volatile unsigned int*)(DMA_BASE + 0x600))
#define DMA_IRQ_EN     (*(volatile unsigned int*)(DMA_BASE + 0x604))
#define DMA_VERSION    (*(volatile unsigned int*)(DMA_BASE + 0x60C))

/* DMA CFG bits */
#define DMA_BURST16    0x3
#define DMA_SRC_INCR   (1u<<2)
#define DMA_DST_INCR   (1u<<4)
#define DMA_MEM2MEM    (0u<<6)

/* DMA CTRL bits */
#define DMA_EN         (1u<<0)
#define DMA_START      (1u<<1)
#define DMA_CTRL_IRQ_EN (1u<<2)

/* FIR CTRL bits */
#define FIR_EN_BIT     (1u<<0)
#define FIR_CLR_BIT    (1u<<1)
#define FIR_IRQ_EN_BIT (1u<<2)

extern void UartStdOutInit(void);
extern unsigned char UartPutc(unsigned char);
extern unsigned char UartGetc(void);

static int pass_cnt = 0;
static int fail_cnt = 0;

static void check(const char *name, unsigned int got, unsigned int exp)
{
    if (got == exp) {
        pass_cnt++;
        printf("[PASS] %s: 0x%08X\n", name, got);
    } else {
        fail_cnt++;
        printf("[FAIL] %s: got 0x%08X exp 0x%08X\n", name, got, exp);
    }
}

/* bounded poll: returns 1 on success, 0 on timeout */
static int poll_eq(volatile unsigned int *reg, unsigned int mask,
                   unsigned int exp, unsigned int timeout)
{
    while (timeout--) {
        if (((*reg) & mask) == exp) return 1;
    }
    return 0;
}

int main(void)
{
    unsigned int i;
    unsigned int v;
    signed   int s;
    int err;

    UartStdOutInit();
    setvbuf(stdout, NULL, _IONBF, 0);   /* newlib: flush every char, no 1KB buffer */

    printf("\n========================================\n");
    printf("  MES50HP SoC - FIR + DMA Test\n");
    printf("  Cortex-M0 @ 50MHz\n");
    printf("========================================\n\n");

    /* ---- Test A: FIR COEF_NUM ---- */
    printf("Test A: FIR COEF_NUM\n");
    check("FIR COEF_NUM", FIR_COEF_NUM, 82);

    /* ---- Test B: DMA VERSION ---- */
    printf("Test B: DMA VERSION\n");
    check("DMA VERSION", DMA_VERSION, 0x444D4131);   /* "DMA1" */

    /* ---- Test C: DMA mem2mem ---- */
    printf("Test C: DMA mem2mem\n");
    {
        volatile unsigned int *src = (volatile unsigned int*)0x20000100u;
        volatile unsigned int *dst = (volatile unsigned int*)0x20000400u;

        for (i = 0; i < 16; i++) src[i] = 0xA0000000u + i;
        for (i = 0; i < 16; i++) dst[i] = 0xDEADBEEFu;

        DMA_IRQ_EN    = 0x1;                 /* enable ch0 irq */
        DMA_CH_SRC(0) = 0x20000100u;
        DMA_CH_DST(0) = 0x20000400u;
        DMA_CH_LEN(0) = 64;                  /* 64 bytes = 16 words */
        DMA_CH_CFG(0) = DMA_BURST16 | DMA_SRC_INCR | DMA_DST_INCR | DMA_MEM2MEM;
        DMA_CH_CTRL(0) = DMA_EN | DMA_START | DMA_CTRL_IRQ_EN;

        err = poll_eq(&DMA_CH_CTRL(0), (1u<<4), (1u<<4), 200000);   /* DONE */
        if (!err) fail_cnt++;
        else      pass_cnt++;
        printf("[%s] DMA CH0 DONE\n", err ? "PASS" : "FAIL");

        check("DMA IRQ_STAT", DMA_IRQ_STAT & 0x1, 0x1);
        DMA_IRQ_STAT = 0x1;                  /* w1c clear */

        err = 0;
        for (i = 0; i < 16; i++) {
            if (dst[i] != 0xA0000000u + i) err++;
        }
        if (err == 0) { pass_cnt++; printf("[PASS] mem2mem data 16/16\n"); }
        else          { fail_cnt++; printf("[FAIL] mem2mem data err=%d\n", err); }
    }

    /* ---- Test D: FIR DC response ---- */
    printf("Test D: FIR DC response\n");
    {
        /* flush FIFOs, enable */
        FIR_CTRL = FIR_CLR_BIT;
        FIR_CTRL = 0;
        FIR_CTRL = FIR_EN_BIT;

        /* push 160 samples of 1000 */
        for (i = 0; i < 160; i++) {
            FIR_DATA_IN = 1000;
        }

        /* wait until 160 outputs are available (OUT_CNT field [23:16]) */
        err = poll_eq(&FIR_STATUS, (0xFFu<<16), (160u<<16), 500000);
        if (!err) fail_cnt++;
        else      pass_cnt++;
        printf("[%s] FIR 160 outputs ready (STATUS=0x%08X)\n",
               err ? "PASS" : "FAIL", FIR_STATUS);

        /* pop all 160, check steady-state samples (index >= 120) */
        err = 0;
        for (i = 0; i < 160; i++) {
            v = FIR_DATA_OUT;
            if (i >= 120) {
                s = (signed int)(v & 0xFFFF);
                if (s > 32767) s -= 65536;     /* sign extend */
                if (s != 1000) err++;
            }
        }
        if (err == 0) { pass_cnt++; printf("[PASS] FIR DC steady-state 40/40 == 1000\n"); }
        else          { fail_cnt++; printf("[FAIL] FIR DC err=%d\n", err); }
    }

    /* ---- Test F: 82-tap asymmetric coefficient golden regression ---- */
    printf("Test F: FIR asymmetric 82-coef golden (1024 samples)\n");
    {
        static unsigned int in_words[1024];
        static unsigned int out_words[1024];
        signed short coef[82];
        signed short exp_s;
        unsigned int n, k;
        unsigned int src, dst;
        long long acc;
        long long sh;
        int mismatch = 0;
        int batch;

        /* 82 arbitrary asymmetric coefficients (same pattern as TB). */
        for (k = 0; k < 82; k++) {
            coef[k] = (signed short)((int)(k * 37 + 11) % 2001 - 1000);
            FIR_COEF_IDX  = k;
            FIR_COEF_DATA = (unsigned int)(unsigned short)coef[k];
        }

        /* 1024 input samples in SRAM, 16-bit signed in the low half. */
        for (n = 0; n < 1024; n++) {
            int x = (int)(n * 97 + 23) % 4001 - 2000;
            in_words[n]  = (unsigned int)(unsigned short)(signed short)x;
            out_words[n] = 0xDEADBEEFu;
        }

        /* Clear FIR core + FIFOs, then enable. */
        FIR_CTRL = FIR_CLR_BIT;
        FIR_CTRL = 0;
        FIR_CTRL = FIR_EN_BIT;

        for (batch = 0; batch < 2; batch++) {
            src = (unsigned int)&in_words[batch * 512];
            dst = (unsigned int)&out_words[batch * 512];

            /* CH0: SRAM -> FIR DATA_IN, 512 words, 16-beat bursts. */
            DMA_CH_SRC(0) = src;
            DMA_CH_DST(0) = FIR_DATA_IN;
            DMA_CH_LEN(0) = 512u * 4u;
            DMA_CH_CFG(0) = DMA_BURST16 | (1u << 6);   /* mem2fir */
            DMA_CH_CTRL(0) = DMA_EN | DMA_START;
            err = poll_eq(&DMA_CH_CTRL(0), (1u << 4), (1u << 4), 500000);
            if (!err) {
                fail_cnt++;
                printf("[FAIL] DMA ch0 batch %d timeout\n", batch);
                break;
            }
            pass_cnt++;

            /* CH1: FIR DATA_OUT -> SRAM, 512 words, 16-beat bursts. */
            DMA_CH_SRC(1) = FIR_DATA_OUT;
            DMA_CH_DST(1) = dst;
            DMA_CH_LEN(1) = 512u * 4u;
            DMA_CH_CFG(1) = DMA_BURST16 | (2u << 6);   /* fir2mem */
            DMA_CH_CTRL(1) = DMA_EN | DMA_START;
            err = poll_eq(&DMA_CH_CTRL(1), (1u << 4), (1u << 4), 500000);
            if (!err) {
                fail_cnt++;
                printf("[FAIL] DMA ch1 batch %d timeout\n", batch);
                break;
            }
            pass_cnt++;
        }

        if (batch == 2) {
            /* Golden model on the CPU: 82-tap, Q1.15, round + saturate. */
            for (n = 0; n < 1024; n++) {
                acc = 0;
                for (k = 0; k < 82; k++) {
                    if (n >= k) {
                        signed short x =
                            (signed short)(in_words[n - k] & 0xFFFF);
                        acc += (long long)x * (long long)coef[k];
                    }
                }
                sh = (acc + 16384) >> 15;
                if (sh > 32767)       exp_s = 32767;
                else if (sh < -32768) exp_s = -32768;
                else                  exp_s = (signed short)sh;

                {
                    signed short got =
                        (signed short)(out_words[n] & 0xFFFF);
                    if (got != exp_s) {
                        mismatch++;
                        if (mismatch <= 5)
                            printf("  [FAIL] n=%u got=%d exp=%d\n",
                                   n, (int)got, (int)exp_s);
                    }
                }
            }

            if (mismatch == 0) {
                pass_cnt++;
                printf("[PASS] FIR asymmetric golden 1024/1024, coef h[0..81]\n");
            } else {
                fail_cnt++;
                printf("[FAIL] FIR asymmetric golden mismatch=%d\n", mismatch);
            }
        }
    }

    /* ---- Summary ---- */
    printf("\n========================================\n");
    printf("Test Summary: %d PASS, %d FAIL\n", pass_cnt, fail_cnt);
    printf("========================================\n");
    if (fail_cnt == 0) printf("*** ALL TESTS PASSED ***\n");
    else               printf("*** %d TESTS FAILED ***\n", fail_cnt);

#ifdef SIMULATION
    UartPutc(0x04);      /* end simulation */
    while (1);
#else
    printf("UART echo ready...\n");
    for (;;) {
        if (CMSDK_UART0->STATE & 0x02) {
            unsigned char ch = (unsigned char)CMSDK_UART0->DATA;
            printf("echo: 0x%02X '%c'\n", ch, ch);
            UartPutc(ch);
        }
    }
#endif
    return 0;
}
