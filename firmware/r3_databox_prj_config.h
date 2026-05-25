/**
 ******************************************************************************
 * @brief   r3_databox project config — SPI-relevant excerpts
 *
 * NOTE: This is a reference excerpt from the POSSUMIC SDK showing the key
 * SPI configuration macros. For the complete project, use the original SDK.
 *
 * Source: psdf_sdk/project/mmwave/r3_databox/app/inc/prj_config.h
 ******************************************************************************
 */

#ifndef _PRJ_CONFIG_H
#define _PRJ_CONFIG_H

/*
 * MRS6130-P1806 will use uart0 as HIF and uart1 as printf com port.
 * MRS6130-P1812 / MRS6240-P2512 will use uart0 as printf & shell,
 * SPI or UART1 as HIF.
 */

/* ── HIF Communication: SPI enabled ─────────────────────────── */
#define CONFIG_HIF_DEVICE_COM_SPI         1   /* SPI as HIF transport */
#define CONFIG_HIF_DEVICE_WAKEUP_SPI      1   /* SPI wakeup support   */

/* ── HIF Communication: UART also available ─────────────────── */
#define CONFIG_HIF_DEVICE_COM_UART        1
#define CONFIG_HIF_DEVICE_WAKEUP_UART     1

/* ── SPI DMA: enables zero-copy TX via DMA linked list ──────── */
#define CONFIG_HIF_SEND_DMA               1

/* ── Micro point cloud: enables sub-mm micro-motion detection ── */
#define CONFIG_MMW_MICRO_POINT_CLOUD      1

/* ── PM disabled (simplifies host-side communication) ───────── */
#define CONFIG_PM                         0

/* ── Heap sizing ────────────────────────────────────────────── */
#if CONFIG_MMW_MICRO_POINT_CLOUD
#define HEAP_SZIE_MiCRO                   (1024 * 78)
#else
#define HEAP_SZIE_MiCRO                   (1024 * 0)
#endif

#define HEAP_SZIE_BASIC                   (1024 * 55)
#define CAL_HEAP_SZIE_TOTAL               (HEAP_SZIE_DATA_CUBE + HEAP_SZIE_MiCRO + HEAP_SZIE_1D_CUBE)
#define CONFIG_HEAP_SIZE                  (CAL_HEAP_SZIE_TOTAL + HEAP_SZIE_BASIC)

#endif /* _PRJ_CONFIG_H */
