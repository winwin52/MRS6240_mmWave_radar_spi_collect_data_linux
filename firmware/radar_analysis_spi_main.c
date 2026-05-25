/**
 ******************************************************************************
 * @brief   radar_analysis_spi — main.c (SPI auto-start excerpt)
 *
 * This firmware starts radar data reporting over SPI immediately at boot
 * WITHOUT requiring UART shell commands ("comtype 1", "report points", "start").
 *
 * Key SPI init sequence (lines 208-258 in original):
 *   1. Set report type to POINTS
 *   2. Init HIF over SPI at 56 MHz
 *   3. Init micro point cloud library
 *   4. Register point cloud data callback
 *   5. Start radar (20 fps, Cartesian coordinates)
 *
 * NOTE: This is a reference excerpt from the POSSUMIC SDK.
 * Source: psdf_sdk/project/mmwave/radar_analysis_spi/app/src/main.c
 ******************************************************************************
 */

#include <common.h>
#include "mmw_hif.h"
#include "mmw_ctrl.h"
#include "mmw_report.h"
#include "mmw_app_micro_pointcloud.h"

int human_motion_default_config(void)
{
#if (CONFIG_SOC_RS6240 || CONFIG_SOC_RS7241)
    int ret = mmw_mode_cfg(MMW_MIMO_2T4R, MMW_WORK_MODE_2DFFT);
#elif CONFIG_SOC_RS6130
    int ret = mmw_mode_cfg(MMW_MIMO_1T3R, MMW_WORK_MODE_2DFFT);
#endif

    if (ret) { LOG_PRINT("mode cfg error! %d\n", ret); return ret; }
    ret = mmw_range_cfg(80*256, 80);      /* resol=8cm, max=20.48m */
    if (ret) { LOG_PRINT("range cfg error! %d\n", ret); return ret; }
    ret = mmw_velocity_cfg(200 * 16, 200); /* resol=0.2m/s, max=32m/s */
    if (ret) { LOG_PRINT("velocity cfg error! %d\n", ret); return ret; }
    ret = mmw_frame_cfg(50, 0);            /* 50ms = 20fps */
    if (ret) { LOG_PRINT("frame cfg error! %d\n", ret); return ret; }
    return 0;
}

int main(void)
{
    uint32_t status = 0;
    int cpuid = csi_get_cpu_id();

    LOG_PRINT("Radar Analysis SPI Project\n");

    status = mmw_ctrl_open(true, false, true);
    if (status != 0) {
        LOG_PRINT("mmw_ctrl_open fail %d\n", status);
    }

    status = human_motion_default_config();
    if (status != 0) {
        LOG_PRINT("human_motion_default_config fail %d\n", status);
    }

    /* ==================================================================
     * SPI Mode Auto-Start
     * Replaces the manual shell sequence:
     *   comtype 1 → report points → start
     * ================================================================== */
    {
        extern uint8_t gui_data_report;

        /* Step 1: Set report type = points */
        gui_data_report = GUI_DATA_TYPE_POINTS;
        mmw_report_gui(gui_data_report, 2048, 0);

        /* Step 2: Init HIF over SPI at 56 MHz for point cloud reporting */
        mmw_report_param_get();
        mmw_data_report_hif_init(HIF_COM_TYPE_SPI, 56000000, 0);

        /* Step 3: Init micro point cloud */
#if (CONFIG_MMW_MICRO_POINT_CLOUD)
        mmw_micro_point_init();
#endif

        /* Step 4: Register point cloud data callback */
        {
            uint8_t txrx, work;
            mmw_mode_get(&txrx, &work);
            uint8_t datatype = (work == MMW_WORK_MODE_1DFFT) ?
                               MMW_DATA_TYPE_1DFFT : MMW_DATA_TYPE_2DFFT;
            mmw_ctrl_callback_cfg(&mmw_ctrl_data_point_report_cb, datatype, NULL);
        }

        /* Step 5: Start radar (Cartesian coords, 20 fps) */
        mmw_coordinate_config(MMW_COORDINATE_TYPE_CART);
        mmw_set_ant_align(1);
        mmw_point_cloud_init();
        mmw_fft_autogain_set(1);
#if (CONFIG_MMW_MICRO_POINT_CLOUD)
        mmw_micro_point_restart();
#endif
        status = mmw_ctrl_start();
        if (status != 0) {
            LOG_PRINT("mmw_ctrl_start fail %d\n", status);
        }

        LOG_PRINT("radar_analysis_spi: SPI point cloud active\n");
    }

    return 0;
}
