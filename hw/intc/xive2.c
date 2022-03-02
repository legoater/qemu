/*
 * QEMU PowerPC XIVE2 interrupt controller model (POWER10)
 *
 * Copyright (c) 2019-2022, IBM Corporation..
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "target/ppc/cpu.h"
#include "sysemu/cpus.h"
#include "sysemu/dma.h"
#include "hw/qdev-properties.h"
#include "monitor/monitor.h"
#include "hw/ppc/xive.h"
#include "hw/ppc/xive2.h"
#include "hw/ppc/xive2_regs.h"

void xive2_eas_pic_print_info(Xive2Eas *eas, uint32_t lisn, Monitor *mon)
{
    if (!xive2_eas_is_valid(eas)) {
        return;
    }

    monitor_printf(mon, "  %08x %s end:%02x/%04x data:%08x\n",
                   lisn, xive2_eas_is_masked(eas) ? "M" : " ",
                   (uint8_t)  xive_get_field64(EAS2_END_BLOCK, eas->w),
                   (uint32_t) xive_get_field64(EAS2_END_INDEX, eas->w),
                   (uint32_t) xive_get_field64(EAS2_END_DATA, eas->w));
}

void xive2_end_queue_pic_print_info(Xive2End *end, uint32_t width,
                                    Monitor *mon)
{
    uint64_t qaddr_base = xive2_end_qaddr(end);
    uint32_t qsize = xive_get_field32(END2_W3_QSIZE, end->w3);
    uint32_t qindex = xive_get_field32(END2_W1_PAGE_OFF, end->w1);
    uint32_t qentries = 1 << (qsize + 10);
    int i;

    /*
     * print out the [ (qindex - (width - 1)) .. (qindex + 1)] window
     */
    monitor_printf(mon, " [ ");
    qindex = (qindex - (width - 1)) & (qentries - 1);
    for (i = 0; i < width; i++) {
        uint64_t qaddr = qaddr_base + (qindex << 2);
        uint32_t qdata = -1;

        if (dma_memory_read(&address_space_memory, qaddr, &qdata,
                            sizeof(qdata), MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "XIVE: failed to read EQ @0x%"
                          HWADDR_PRIx "\n", qaddr);
            return;
        }
        monitor_printf(mon, "%s%08x ", i == width - 1 ? "^" : "",
                       be32_to_cpu(qdata));
        qindex = (qindex + 1) & (qentries - 1);
    }
    monitor_printf(mon, "]");
}

void xive2_end_pic_print_info(Xive2End *end, uint32_t end_idx, Monitor *mon)
{
    uint64_t qaddr_base = xive2_end_qaddr(end);
    uint32_t qindex = xive_get_field32(END2_W1_PAGE_OFF, end->w1);
    uint32_t qgen = xive_get_field32(END2_W1_GENERATION, end->w1);
    uint32_t qsize = xive_get_field32(END2_W3_QSIZE, end->w3);
    uint32_t qentries = 1 << (qsize + 10);

    uint32_t nvp_blk = xive_get_field32(END2_W6_VP_BLOCK, end->w6);
    uint32_t nvp_idx = xive_get_field32(END2_W6_VP_OFFSET, end->w6);
    uint8_t priority = xive_get_field32(END2_W7_F0_PRIORITY, end->w7);
    uint8_t pq;

    if (!xive2_end_is_valid(end)) {
        return;
    }

    pq = xive_get_field32(END2_W1_ESn, end->w1);

    monitor_printf(mon,
                   "  %08x %c%c %c%c%c%c%c%c%c%c%c%c prio:%d nvp:%02x/%04x",
                   end_idx,
                   pq & XIVE_ESB_VAL_P ? 'P' : '-',
                   pq & XIVE_ESB_VAL_Q ? 'Q' : '-',
                   xive2_end_is_valid(end)    ? 'v' : '-',
                   xive2_end_is_enqueue(end)  ? 'q' : '-',
                   xive2_end_is_notify(end)   ? 'n' : '-',
                   xive2_end_is_backlog(end)  ? 'b' : '-',
                   xive2_end_is_escalate(end) ? 'e' : '-',
                   xive2_end_is_escalate_end(end) ? 'N' : '-',
                   xive2_end_is_uncond_escalation(end)   ? 'u' : '-',
                   xive2_end_is_silent_escalation(end)   ? 's' : '-',
                   xive2_end_is_firmware1(end)   ? 'f' : '-',
                   xive2_end_is_firmware2(end)   ? 'F' : '-',
                   priority, nvp_blk, nvp_idx);

    if (qaddr_base) {
        monitor_printf(mon, " eq:@%08"PRIx64"% 6d/%5d ^%d",
                       qaddr_base, qindex, qentries, qgen);
        xive2_end_queue_pic_print_info(end, 6, mon);
    }
    monitor_printf(mon, "\n");
}

void xive2_end_eas_pic_print_info(Xive2End *end, uint32_t end_idx,
                                  Monitor *mon)
{
    Xive2Eas *eas = (Xive2Eas *) &end->w4;
    uint8_t pq;

    if (!xive2_end_is_escalate(end)) {
        return;
    }

    pq = xive_get_field32(END2_W1_ESe, end->w1);

    monitor_printf(mon, "  %08x %c%c %c%c end:%02x/%04x data:%08x\n",
                   end_idx,
                   pq & XIVE_ESB_VAL_P ? 'P' : '-',
                   pq & XIVE_ESB_VAL_Q ? 'Q' : '-',
                   xive2_eas_is_valid(eas) ? 'v' : ' ',
                   xive2_eas_is_masked(eas) ? 'M' : ' ',
                   (uint8_t)  xive_get_field64(EAS2_END_BLOCK, eas->w),
                   (uint32_t) xive_get_field64(EAS2_END_INDEX, eas->w),
                   (uint32_t) xive_get_field64(EAS2_END_DATA, eas->w));
}

static void xive2_end_enqueue(Xive2End *end, uint32_t data)
{
    uint64_t qaddr_base = xive2_end_qaddr(end);
    uint32_t qsize = xive_get_field32(END2_W3_QSIZE, end->w3);
    uint32_t qindex = xive_get_field32(END2_W1_PAGE_OFF, end->w1);
    uint32_t qgen = xive_get_field32(END2_W1_GENERATION, end->w1);

    uint64_t qaddr = qaddr_base + (qindex << 2);
    uint32_t qdata = cpu_to_be32((qgen << 31) | (data & 0x7fffffff));
    uint32_t qentries = 1 << (qsize + 10);

    if (dma_memory_write(&address_space_memory, qaddr, &qdata, sizeof(qdata),
                         MEMTXATTRS_UNSPECIFIED)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: failed to write END data @0x%"
                      HWADDR_PRIx "\n", qaddr);
        return;
    }

    qindex = (qindex + 1) & (qentries - 1);
    if (qindex == 0) {
        qgen ^= 1;
        end->w1 = xive_set_field32(END2_W1_GENERATION, end->w1, qgen);

        /* TODO(PowerNV): reset GF bit on a cache watch operation */
        end->w1 = xive_set_field32(END2_W1_GEN_FLIPPED, end->w1, qgen);
    }
    end->w1 = xive_set_field32(END2_W1_PAGE_OFF, end->w1, qindex);
}
/*
 * XIVE Router (aka. Virtualization Controller or IVRE)
 */

int xive2_router_get_eas(Xive2Router *xrtr, uint8_t eas_blk, uint32_t eas_idx,
                         Xive2Eas *eas)
{
    Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

    return xrc->get_eas(xrtr, eas_blk, eas_idx, eas);
}

int xive2_router_get_end(Xive2Router *xrtr, uint8_t end_blk, uint32_t end_idx,
                         Xive2End *end)
{
   Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

   return xrc->get_end(xrtr, end_blk, end_idx, end);
}

int xive2_router_write_end(Xive2Router *xrtr, uint8_t end_blk, uint32_t end_idx,
                           Xive2End *end, uint8_t word_number)
{
   Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

   return xrc->write_end(xrtr, end_blk, end_idx, end, word_number);
}

int xive2_router_get_nvp(Xive2Router *xrtr, uint8_t nvp_blk, uint32_t nvp_idx,
                         Xive2Nvp *nvp)
{
   Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

   return xrc->get_nvp(xrtr, nvp_blk, nvp_idx, nvp);
}

int xive2_router_write_nvp(Xive2Router *xrtr, uint8_t nvp_blk, uint32_t nvp_idx,
                           Xive2Nvp *nvp, uint8_t word_number)
{
   Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

   return xrc->write_nvp(xrtr, nvp_blk, nvp_idx, nvp, word_number);
}

static int xive2_router_get_block_id(Xive2Router *xrtr)
{
   Xive2RouterClass *xrc = XIVE2_ROUTER_GET_CLASS(xrtr);

   return xrc->get_block_id(xrtr);
}

/*
 * Encode the HW CAM line with 7bit or 8bit thread id. The thread id
 * width and block id width is configurable at the IC level.
 *
 *    chipid << 24 | 0000 0000 0000 0000 1 threadid (7Bit)
 *    chipid << 24 | 0000 0000 0000 0001 threadid   (8Bit)
 */
static uint32_t xive2_tctx_hw_cam_line(XivePresenter *xptr, XiveTCTX *tctx)
{
    Xive2Router *xrtr = XIVE2_ROUTER(xptr);
    CPUPPCState *env = &POWERPC_CPU(tctx->cs)->env;
    uint32_t pir = env->spr_cb[SPR_PIR].default_value;
    uint8_t blk = xive2_router_get_block_id(xrtr);
    uint8_t tid_shift = 7;
    uint8_t tid_mask = (1 << tid_shift) - 1;

    return xive2_nvp_cam_line(blk, 1 << tid_shift | (pir & tid_mask));
}

/*
 * The thread context register words are in big-endian format.
 */
int xive2_presenter_tctx_match(XivePresenter *xptr, XiveTCTX *tctx,
                               uint8_t format,
                               uint8_t nvt_blk, uint32_t nvt_idx,
                               bool cam_ignore, uint32_t logic_serv)
{
    uint32_t cam =   xive2_nvp_cam_line(nvt_blk, nvt_idx);
    uint32_t qw3w2 = xive_tctx_word2(&tctx->regs[TM_QW3_HV_PHYS]);
    uint32_t qw2w2 = xive_tctx_word2(&tctx->regs[TM_QW2_HV_POOL]);
    uint32_t qw1w2 = xive_tctx_word2(&tctx->regs[TM_QW1_OS]);
    uint32_t qw0w2 = xive_tctx_word2(&tctx->regs[TM_QW0_USER]);

    /*
     * TODO (PowerNV): ignore mode. The low order bits of the NVT
     * identifier are ignored in the "CAM" match.
     */

    if (format == 0) {
        if (cam_ignore == true) {
            /*
             * F=0 & i=1: Logical server notification (bits ignored at
             * the end of the NVT identifier)
             */
            qemu_log_mask(LOG_UNIMP, "XIVE: no support for LS NVT %x/%x\n",
                          nvt_blk, nvt_idx);
            return -1;
        }

        /* F=0 & i=0: Specific NVT notification */

        /* PHYS ring */
        if ((be32_to_cpu(qw3w2) & TM2_QW3W2_VT) &&
            cam == xive2_tctx_hw_cam_line(xptr, tctx)) {
            return TM_QW3_HV_PHYS;
        }

        /* HV POOL ring */
        if ((be32_to_cpu(qw2w2) & TM2_QW2W2_VP) &&
            cam == xive_get_field32(TM2_QW2W2_POOL_CAM, qw2w2)) {
            return TM_QW2_HV_POOL;
        }

        /* OS ring */
        if ((be32_to_cpu(qw1w2) & TM2_QW1W2_VO) &&
            cam == xive_get_field32(TM2_QW1W2_OS_CAM, qw1w2)) {
            return TM_QW1_OS;
        }
    } else {
        /* F=1 : User level Event-Based Branch (EBB) notification */

        /* USER ring */
        if  ((be32_to_cpu(qw1w2) & TM2_QW1W2_VO) &&
             (cam == xive_get_field32(TM2_QW1W2_OS_CAM, qw1w2)) &&
             (be32_to_cpu(qw0w2) & TM2_QW0W2_VU) &&
             (logic_serv == xive_get_field32(TM2_QW0W2_LOGIC_SERV, qw0w2))) {
            return TM_QW0_USER;
        }
    }
    return -1;
}

static void xive2_router_realize(DeviceState *dev, Error **errp)
{
    Xive2Router *xrtr = XIVE2_ROUTER(dev);

    assert(xrtr->xfb);
}

/*
 * Notification using the END ESe/ESn bit (Event State Buffer for
 * escalation and notification). Profide futher coalescing in the
 * Router.
 */
static bool xive2_router_end_es_notify(Xive2Router *xrtr, uint8_t end_blk,
                                       uint32_t end_idx, Xive2End *end,
                                       uint32_t end_esmask)
{
    uint8_t pq = xive_get_field32(end_esmask, end->w1);
    bool notify = xive_esb_trigger(&pq);

    if (pq != xive_get_field32(end_esmask, end->w1)) {
        end->w1 = xive_set_field32(end_esmask, end->w1, pq);
        xive2_router_write_end(xrtr, end_blk, end_idx, end, 1);
    }

    /* ESe/n[Q]=1 : end of notification */
    return notify;
}

/*
 * An END trigger can come from an event trigger (IPI or HW) or from
 * another chip. We don't model the PowerBus but the END trigger
 * message has the same parameters than in the function below.
 */
static void xive2_router_end_notify(Xive2Router *xrtr, uint8_t end_blk,
                                    uint32_t end_idx, uint32_t end_data)
{
    Xive2End end;
    uint8_t priority;
    uint8_t format;
    bool found;
    Xive2Nvp nvp;
    uint8_t nvp_blk;
    uint32_t nvp_idx;

    /* END cache lookup */
    if (xive2_router_get_end(xrtr, end_blk, end_idx, &end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: No END %x/%x\n", end_blk,
                      end_idx);
        return;
    }

    if (!xive2_end_is_valid(&end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: END %x/%x is invalid\n",
                      end_blk, end_idx);
        return;
    }

    if (xive2_end_is_enqueue(&end)) {
        xive2_end_enqueue(&end, end_data);
        /* Enqueuing event data modifies the EQ toggle and index */
        xive2_router_write_end(xrtr, end_blk, end_idx, &end, 1);
    }

    /*
     * When the END is silent, we skip the notification part.
     */
    if (xive2_end_is_silent_escalation(&end)) {
        goto do_escalation;
    }

    /*
     * The W7 format depends on the F bit in W6. It defines the type
     * of the notification :
     *
     *   F=0 : single or multiple NVP notification
     *   F=1 : User level Event-Based Branch (EBB) notification, no
     *         priority
     */
    format = xive_get_field32(END2_W6_FORMAT_BIT, end.w6);
    priority = xive_get_field32(END2_W7_F0_PRIORITY, end.w7);

    /* The END is masked */
    if (format == 0 && priority == 0xff) {
        return;
    }

    /*
     * Check the END ESn (Event State Buffer for notification) for
     * even futher coalescing in the Router
     */
    if (!xive2_end_is_notify(&end)) {
        /* ESn[Q]=1 : end of notification */
        if (!xive2_router_end_es_notify(xrtr, end_blk, end_idx,
                                       &end, END2_W1_ESn)) {
            return;
        }
    }

    /*
     * Follows IVPE notification
     */
    nvp_blk = xive_get_field32(END2_W6_VP_BLOCK, end.w6);
    nvp_idx = xive_get_field32(END2_W6_VP_OFFSET, end.w6);

    /* NVP cache lookup */
    if (xive2_router_get_nvp(xrtr, nvp_blk, nvp_idx, &nvp)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: no NVP %x/%x\n",
                      nvp_blk, nvp_idx);
        return;
    }

    if (!xive2_nvp_is_valid(&nvp)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: NVP %x/%x is invalid\n",
                      nvp_blk, nvp_idx);
        return;
    }

    found = xive_presenter_notify(xrtr->xfb, format, nvp_blk, nvp_idx,
                          xive_get_field32(END2_W6_IGNORE, end.w7),
                          priority,
                          xive_get_field32(END2_W7_F1_LOG_SERVER_ID, end.w7));

    /* TODO: Auto EOI. */

    if (found) {
        return;
    }

    /*
     * If no matching NVP is dispatched on a HW thread :
     * - specific VP: update the NVP structure if backlog is activated
     * - logical server : forward request to IVPE (not supported)
     */
    if (xive2_end_is_backlog(&end)) {
        uint8_t ipb;

        if (format == 1) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "XIVE: END %x/%x invalid config: F1 & backlog\n",
                          end_blk, end_idx);
            return;
        }

        /*
         * Record the IPB in the associated NVP structure for later
         * use. The presenter will resend the interrupt when the vCPU
         * is dispatched again on a HW thread.
         */
        ipb = xive_get_field32(NVP2_W2_IPB, nvp.w2) |
            xive_priority_to_ipb(priority);
        nvp.w2 = xive_set_field32(NVP2_W2_IPB, nvp.w2, ipb);
        xive2_router_write_nvp(xrtr, nvp_blk, nvp_idx, &nvp, 2);

        /*
         * On HW, follows a "Broadcast Backlog" to IVPEs
         */
    }

do_escalation:
    /*
     * If activated, escalate notification using the ESe PQ bits and
     * the EAS in w4-5
     */
    if (!xive2_end_is_escalate(&end)) {
        return;
    }

    /*
     * Check the END ESe (Event State Buffer for escalation) for even
     * futher coalescing in the Router
     */
    if (!xive2_end_is_uncond_escalation(&end)) {
        /* ESe[Q]=1 : end of escalation notification */
        if (!xive2_router_end_es_notify(xrtr, end_blk, end_idx,
                                       &end, END2_W1_ESe)) {
            return;
        }
    }

    /*
     * The END trigger becomes an Escalation trigger
     */
    xive2_router_end_notify(xrtr,
                           xive_get_field32(END2_W4_END_BLOCK,     end.w4),
                           xive_get_field32(END2_W4_ESC_END_INDEX, end.w4),
                           xive_get_field32(END2_W5_ESC_END_DATA,  end.w5));
}

void xive2_router_notify(XiveNotifier *xn, uint32_t lisn)
{
    Xive2Router *xrtr = XIVE2_ROUTER(xn);
    uint8_t eas_blk = XIVE_EAS_BLOCK(lisn);
    uint32_t eas_idx = XIVE_EAS_INDEX(lisn);
    Xive2Eas eas;

    /* EAS cache lookup */
    if (xive2_router_get_eas(xrtr, eas_blk, eas_idx, &eas)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: Unknown LISN %x\n", lisn);
        return;
    }

    if (!xive2_eas_is_valid(&eas)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: Invalid LISN %x\n", lisn);
        return;
    }

    if (xive2_eas_is_masked(&eas)) {
        /* Notification completed */
        return;
    }

    /*
     * The event trigger becomes an END trigger
     */
    xive2_router_end_notify(xrtr,
                             xive_get_field64(EAS2_END_BLOCK, eas.w),
                             xive_get_field64(EAS2_END_INDEX, eas.w),
                             xive_get_field64(EAS2_END_DATA,  eas.w));
}

static Property xive2_router_properties[] = {
    DEFINE_PROP_LINK("xive-fabric", Xive2Router, xfb,
                     TYPE_XIVE_FABRIC, XiveFabric *),
    DEFINE_PROP_END_OF_LIST(),
};

static void xive2_router_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XiveNotifierClass *xnc = XIVE_NOTIFIER_CLASS(klass);

    dc->desc    = "XIVE2 Router Engine";
    device_class_set_props(dc, xive2_router_properties);
    /* Parent is SysBusDeviceClass. No need to call its realize hook */
    dc->realize = xive2_router_realize;
    xnc->notify = xive2_router_notify;
}

static const TypeInfo xive2_router_info = {
    .name          = TYPE_XIVE2_ROUTER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .abstract      = true,
    .instance_size = sizeof(Xive2Router),
    .class_size    = sizeof(Xive2RouterClass),
    .class_init    = xive2_router_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_XIVE_NOTIFIER },
        { TYPE_XIVE_PRESENTER },
        { }
    }
};

static inline bool addr_is_even(hwaddr addr, uint32_t shift)
{
    return !((addr >> shift) & 1);
}

static uint64_t xive2_end_source_read(void *opaque, hwaddr addr, unsigned size)
{
    Xive2EndSource *xsrc = XIVE2_END_SOURCE(opaque);
    uint32_t offset = addr & 0xFFF;
    uint8_t end_blk;
    uint32_t end_idx;
    Xive2End end;
    uint32_t end_esmask;
    uint8_t pq;
    uint64_t ret;

    /*
     * The block id should be deduced from the load address on the END
     * ESB MMIO but our model only supports a single block per XIVE chip.
     */
    end_blk = xive2_router_get_block_id(xsrc->xrtr);
    end_idx = addr >> (xsrc->esb_shift + 1);

    if (xive2_router_get_end(xsrc->xrtr, end_blk, end_idx, &end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: No END %x/%x\n", end_blk,
                      end_idx);
        return -1;
    }

    if (!xive2_end_is_valid(&end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: END %x/%x is invalid\n",
                      end_blk, end_idx);
        return -1;
    }

    end_esmask = addr_is_even(addr, xsrc->esb_shift) ? END2_W1_ESn :
        END2_W1_ESe;
    pq = xive_get_field32(end_esmask, end.w1);

    switch (offset) {
    case XIVE_ESB_LOAD_EOI ... XIVE_ESB_LOAD_EOI + 0x7FF:
        ret = xive_esb_eoi(&pq);

        /* Forward the source event notification for routing ?? */
        break;

    case XIVE_ESB_GET ... XIVE_ESB_GET + 0x3FF:
        ret = pq;
        break;

    case XIVE_ESB_SET_PQ_00 ... XIVE_ESB_SET_PQ_00 + 0x0FF:
    case XIVE_ESB_SET_PQ_01 ... XIVE_ESB_SET_PQ_01 + 0x0FF:
    case XIVE_ESB_SET_PQ_10 ... XIVE_ESB_SET_PQ_10 + 0x0FF:
    case XIVE_ESB_SET_PQ_11 ... XIVE_ESB_SET_PQ_11 + 0x0FF:
        ret = xive_esb_set(&pq, (offset >> 8) & 0x3);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: invalid END ESB load addr %d\n",
                      offset);
        return -1;
    }

    if (pq != xive_get_field32(end_esmask, end.w1)) {
        end.w1 = xive_set_field32(end_esmask, end.w1, pq);
        xive2_router_write_end(xsrc->xrtr, end_blk, end_idx, &end, 1);
    }

    return ret;
}

static void xive2_end_source_write(void *opaque, hwaddr addr,
                                   uint64_t value, unsigned size)
{
    Xive2EndSource *xsrc = XIVE2_END_SOURCE(opaque);
    uint32_t offset = addr & 0xFFF;
    uint8_t end_blk;
    uint32_t end_idx;
    Xive2End end;
    uint32_t end_esmask;
    uint8_t pq;
    bool notify = false;

    /*
     * The block id should be deduced from the load address on the END
     * ESB MMIO but our model only supports a single block per XIVE chip.
     */
    end_blk = xive2_router_get_block_id(xsrc->xrtr);
    end_idx = addr >> (xsrc->esb_shift + 1);

    if (xive2_router_get_end(xsrc->xrtr, end_blk, end_idx, &end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: No END %x/%x\n", end_blk,
                      end_idx);
        return;
    }

    if (!xive2_end_is_valid(&end)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: END %x/%x is invalid\n",
                      end_blk, end_idx);
        return;
    }

    end_esmask = addr_is_even(addr, xsrc->esb_shift) ? END2_W1_ESn :
        END2_W1_ESe;
    pq = xive_get_field32(end_esmask, end.w1);

    switch (offset) {
    case 0 ... 0x3FF:
        notify = xive_esb_trigger(&pq);
        break;

    case XIVE_ESB_STORE_EOI ... XIVE_ESB_STORE_EOI + 0x3FF:
        /* TODO: can we check StoreEOI availability from the router ? */
        notify = xive_esb_eoi(&pq);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: invalid END ESB write addr %d\n",
                      offset);
        return;
    }

    if (pq != xive_get_field32(end_esmask, end.w1)) {
        end.w1 = xive_set_field32(end_esmask, end.w1, pq);
        xive2_router_write_end(xsrc->xrtr, end_blk, end_idx, &end, 1);
    }

    /* TODO: Forward the source event notification for routing */
    if (notify) {
        ;
    }
}

static const MemoryRegionOps xive2_end_source_ops = {
    .read = xive2_end_source_read,
    .write = xive2_end_source_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void xive2_end_source_realize(DeviceState *dev, Error **errp)
{
    Xive2EndSource *xsrc = XIVE2_END_SOURCE(dev);

    assert(xsrc->xrtr);

    if (!xsrc->nr_ends) {
        error_setg(errp, "Number of interrupt needs to be greater than 0");
        return;
    }

    if (xsrc->esb_shift != XIVE_ESB_4K &&
        xsrc->esb_shift != XIVE_ESB_64K) {
        error_setg(errp, "Invalid ESB shift setting");
        return;
    }

    /*
     * Each END is assigned an even/odd pair of MMIO pages, the even page
     * manages the ESn field while the odd page manages the ESe field.
     */
    memory_region_init_io(&xsrc->esb_mmio, OBJECT(xsrc),
                          &xive2_end_source_ops, xsrc, "xive.end",
                          (1ull << (xsrc->esb_shift + 1)) * xsrc->nr_ends);
}

static Property xive2_end_source_properties[] = {
    DEFINE_PROP_UINT32("nr-ends", Xive2EndSource, nr_ends, 0),
    DEFINE_PROP_UINT32("shift", Xive2EndSource, esb_shift, XIVE_ESB_64K),
    DEFINE_PROP_LINK("xive", Xive2EndSource, xrtr, TYPE_XIVE2_ROUTER,
                     Xive2Router *),
    DEFINE_PROP_END_OF_LIST(),
};

static void xive2_end_source_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc    = "XIVE END Source";
    device_class_set_props(dc, xive2_end_source_properties);
    dc->realize = xive2_end_source_realize;
}

static const TypeInfo xive2_end_source_info = {
    .name          = TYPE_XIVE2_END_SOURCE,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(Xive2EndSource),
    .class_init    = xive2_end_source_class_init,
};

static void xive2_register_types(void)
{
    type_register_static(&xive2_router_info);
    type_register_static(&xive2_end_source_info);
}

type_init(xive2_register_types)