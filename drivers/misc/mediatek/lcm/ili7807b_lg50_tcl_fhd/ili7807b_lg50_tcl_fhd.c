/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
#define LCM_ID_ILI7807 (0x78)



// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void lcm_init_power(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif
}

static void lcm_suspend_power(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif
}

static void lcm_resume_power(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif
}

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xff, 3,{0x78,0x07,0x01}},
{0x42, 1,{0x11}},
{0x43, 1,{0xb5}},
{0x44, 1,{0xbf}},
{0x45, 1,{0x2f}},
{0x46, 1,{0x3f}},
{0x50, 1,{0x44}},
{0x51, 1,{0x44}},
{0x59, 1,{0x34}},
{0xa2, 1,{0x01}},
{0xa3, 1,{0x2d}},
{0xff, 3,{0x78,0x07,0x01}},
{0x22, 1,{0x06}},
{0x36, 1,{0x00}},
{0x64, 1,{0x10}},
{0x65, 1,{0x07}},
{0x66, 1,{0x08}},
{0x6d, 1,{0x11}},
{0xff, 3,{0x78,0x07,0x08}},
{0x0b, 1,{0x80}},
{0xff, 3,{0x78,0x07,0x06}},
{0x00, 1,{0x41}},
{0x01, 1,{0x04}},
{0x02, 1,{0x00}},
{0x03, 1,{0x00}},
{0x04, 1,{0x00}},
{0x05, 1,{0x00}},
{0x06, 1,{0x00}},
{0x07, 1,{0x00}},
{0x08, 1,{0x81}},
{0x09, 1,{0x04}},
{0x0a, 1,{0x30}},
{0x0b, 1,{0x00}},
{0x0c, 1,{0x00}},
{0x0d, 1,{0x00}},
{0x0e, 1,{0x08}},
{0x0f, 1,{0x08}},
{0x10, 1,{0x00}},
{0x11, 1,{0x00}},
{0x12, 1,{0x00}},
{0x13, 1,{0x00}},
{0x31, 1,{0x22}},
{0x32, 1,{0x07}},
{0x33, 1,{0x07}},
{0x34, 1,{0x07}},
{0x35, 1,{0x01}},
{0x36, 1,{0x00}},
{0x37, 1,{0x07}},
{0x38, 1,{0x07}},
{0x39, 1,{0x07}},
{0x3a, 1,{0x07}},
{0x3b, 1,{0x10}},
{0x3c, 1,{0x12}},
{0x3d, 1,{0x08}},
{0x3e, 1,{0x28}},
{0x3f, 1,{0x29}},
{0x40, 1,{0x2a}},
{0x41, 1,{0x22}},
{0x42, 1,{0x07}},
{0x43, 1,{0x07}},
{0x44, 1,{0x07}},
{0x45, 1,{0x01}},
{0x46, 1,{0x00}},
{0x47, 1,{0x07}},
{0x48, 1,{0x07}},
{0x49, 1,{0x07}},
{0x4a, 1,{0x07}},
{0x4b, 1,{0x11}},
{0x4c, 1,{0x13}},
{0x4d, 1,{0x09}},
{0x4e, 1,{0x28}},
{0x4f, 1,{0x29}},
{0x50, 1,{0x2a}},
{0x61, 1,{0x22}},
{0x62, 1,{0x07}},
{0x63, 1,{0x07}},
{0x64, 1,{0x07}},
{0x65, 1,{0x01}},
{0x66, 1,{0x00}},
{0x67, 1,{0x07}},
{0x68, 1,{0x07}},
{0x69, 1,{0x07}},
{0x6a, 1,{0x07}},
{0x6b, 1,{0x13}},
{0x6c, 1,{0x11}},
{0x6d, 1,{0x09}},
{0x6e, 1,{0x28}},
{0x6f, 1,{0x29}},
{0x70, 1,{0x2a}},
{0x71, 1,{0x22}},
{0x72, 1,{0x07}},
{0x73, 1,{0x07}},
{0x74, 1,{0x07}},
{0x75, 1,{0x01}},
{0x76, 1,{0x00}},
{0x77, 1,{0x07}},
{0x78, 1,{0x07}},
{0x79, 1,{0x07}},
{0x7a, 1,{0x07}},
{0x7b, 1,{0x12}},
{0x7c, 1,{0x10}},
{0x7d, 1,{0x08}},
{0x7e, 1,{0x28}},
{0x7f, 1,{0x29}},
{0x80, 1,{0x2a}},
{0xd0, 1,{0x01}},
{0xdb, 1,{0x40}},
{0xdc, 1,{0x00}},
{0xa0, 1,{0x09}},
{0xa1, 1,{0x00}},
{0xa2, 1,{0x04}},
{0xa3, 1,{0x0d}},
{0xa6, 1,{0x04}},
{0xff, 3,{0x78,0x07,0x08}},
{0x08, 1,{0xc2}},
{0x06, 1,{0x10}},
{0xff, 3,{0x78,0x07,0x07}},
{0x28, 1,{0x0c}},
{0xff, 3,{0x78,0x07,0x02}},
{0x00, 1,{0x00}},
{0x01, 1,{0x00}},
{0x02, 1,{0x00}},
{0x03, 1,{0x08}},
{0x04, 1,{0x00}},
{0x05, 1,{0x18}},
{0x06, 1,{0x00}},
{0x07, 1,{0x28}},
{0x08, 1,{0x00}},
{0x09, 1,{0x37}},
{0x0a, 1,{0x00}},
{0x0b, 1,{0x46}},
{0x0c, 1,{0x00}},
{0x0d, 1,{0x53}},
{0x0e, 1,{0x00}},
{0x0f, 1,{0x5F}},
{0x10, 1,{0x00}},
{0x11, 1,{0x6B}},
{0x12, 1,{0x00}},
{0x13, 1,{0x94}},
{0x14, 1,{0x00}},
{0x15, 1,{0xB5}},
{0x16, 1,{0x00}},
{0x17, 1,{0xEB}},
{0x18, 1,{0x01}},
{0x19, 1,{0x17}},
{0x1a, 1,{0x01}},
{0x1b, 1,{0x5B}},
{0x1c, 1,{0x01}},
{0x1d, 1,{0x92}},
{0x1e, 1,{0x01}},
{0x1f, 1,{0x94}},
{0x20, 1,{0x01}},
{0x21, 1,{0xC9}},
{0x22, 1,{0x02}},
{0x23, 1,{0x09}},
{0x24, 1,{0x02}},
{0x25, 1,{0x35}},
{0x26, 1,{0x02}},
{0x27, 1,{0x74}},
{0x28, 1,{0x02}},
{0x29, 1,{0x9F}},
{0x2a, 1,{0x02}},
{0x2b, 1,{0xDA}},
{0x2c, 1,{0x02}},
{0x2d, 1,{0xEC}},
{0x2e, 1,{0x02}},
{0x2f, 1,{0xFF}},
{0x30, 1,{0x03}},
{0x31, 1,{0x17}},
{0x32, 1,{0x03}},
{0x33, 1,{0x33}},
{0x34, 1,{0x03}},
{0x35, 1,{0x54}},
{0x36, 1,{0x03}},
{0x37, 1,{0x7F}},
{0x38, 1,{0x03}},
{0x39, 1,{0xB6}},
{0x3a, 1,{0x03}},
{0x3b, 1,{0xCF}},
{0x3c, 1,{0x00}},
{0x3d, 1,{0x00}},
{0x3e, 1,{0x00}},
{0x3f, 1,{0x08}},
{0x40, 1,{0x00}},
{0x41, 1,{0x18}},
{0x42, 1,{0x00}},
{0x43, 1,{0x28}},
{0x44, 1,{0x00}},
{0x45, 1,{0x37}},
{0x46, 1,{0x00}},
{0x47, 1,{0x46}},
{0x48, 1,{0x00}},
{0x49, 1,{0x53}},
{0x4a, 1,{0x00}},
{0x4b, 1,{0x5F}},
{0x4c, 1,{0x00}},
{0x4d, 1,{0x6B}},
{0x4e, 1,{0x00}},
{0x4f, 1,{0x94}},
{0x50, 1,{0x00}},
{0x51, 1,{0xB5}},
{0x52, 1,{0x00}},
{0x53, 1,{0xEB}},
{0x54, 1,{0x01}},
{0x55, 1,{0x17}},
{0x56, 1,{0x01}},
{0x57, 1,{0x5B}},
{0x58, 1,{0x01}},
{0x59, 1,{0x92}},
{0x5a, 1,{0x01}},
{0x5b, 1,{0x94}},
{0x5c, 1,{0x01}},
{0x5d, 1,{0xC9}},
{0x5e, 1,{0x02}},
{0x5f, 1,{0x09}},
{0x60, 1,{0x02}},
{0x61, 1,{0x35}},
{0x62, 1,{0x02}},
{0x63, 1,{0x74}},
{0x64, 1,{0x02}},
{0x65, 1,{0x9F}},
{0x66, 1,{0x02}},
{0x67, 1,{0xDA}},
{0x68, 1,{0x02}},
{0x69, 1,{0xEC}},
{0x6a, 1,{0x02}},
{0x6b, 1,{0xFF}},
{0x6c, 1,{0x03}},
{0x6d, 1,{0x17}},
{0x6e, 1,{0x03}},
{0x6f, 1,{0x33}},
{0x70, 1,{0x03}},
{0x71, 1,{0x54}},
{0x72, 1,{0x03}},
{0x73, 1,{0x7F}},
{0x74, 1,{0x03}},
{0x75, 1,{0xB6}},
{0x76, 1,{0x03}},
{0x77, 1,{0xCF}},
{0x78, 1,{0x01}},
{0x79, 1,{0x01}},
{0xff, 3,{0x78,0x07,0x03}},
{0x00, 1,{0x00}},
{0x01, 1,{0x09}},
{0x02, 1,{0x00}},
{0x03, 1,{0x08}},
{0x04, 1,{0x00}},
{0x05, 1,{0x17}},
{0x06, 1,{0x00}},
{0x07, 1,{0x27}},
{0x08, 1,{0x00}},
{0x09, 1,{0x37}},
{0x0a, 1,{0x00}},
{0x0b, 1,{0x46}},
{0x0c, 1,{0x00}},
{0x0d, 1,{0x53}},
{0x0e, 1,{0x00}},
{0x0f, 1,{0x5F}},
{0x10, 1,{0x00}},
{0x11, 1,{0x6A}},
{0x12, 1,{0x00}},
{0x13, 1,{0x93}},
{0x14, 1,{0x00}},
{0x15, 1,{0xB5}},
{0x16, 1,{0x00}},
{0x17, 1,{0xEB}},
{0x18, 1,{0x01}},
{0x19, 1,{0x16}},
{0x1a, 1,{0x01}},
{0x1b, 1,{0x5A}},
{0x1c, 1,{0x01}},
{0x1d, 1,{0x91}},
{0x1e, 1,{0x01}},
{0x1f, 1,{0x93}},
{0x20, 1,{0x01}},
{0x21, 1,{0xC8}},
{0x22, 1,{0x02}},
{0x23, 1,{0x09}},
{0x24, 1,{0x02}},
{0x25, 1,{0x35}},
{0x26, 1,{0x02}},
{0x27, 1,{0x74}},
{0x28, 1,{0x02}},
{0x29, 1,{0xA0}},
{0x2a, 1,{0x02}},
{0x2b, 1,{0xDC}},
{0x2c, 1,{0x02}},
{0x2d, 1,{0xEE}},
{0x2e, 1,{0x03}},
{0x2f, 1,{0x03}},
{0x30, 1,{0x03}},
{0x31, 1,{0x1B}},
{0x32, 1,{0x03}},
{0x33, 1,{0x36}},
{0x34, 1,{0x03}},
{0x35, 1,{0x57}},
{0x36, 1,{0x03}},
{0x37, 1,{0x80}},
{0x38, 1,{0x03}},
{0x39, 1,{0xB9}},
{0x3a, 1,{0x03}},
{0x3b, 1,{0xCF}},
{0x3c, 1,{0x00}},
{0x3d, 1,{0x09}},
{0x3e, 1,{0x00}},
{0x3f, 1,{0x08}},
{0x40, 1,{0x00}},
{0x41, 1,{0x17}},
{0x42, 1,{0x00}},
{0x43, 1,{0x27}},
{0x44, 1,{0x00}},
{0x45, 1,{0x37}},
{0x46, 1,{0x00}},
{0x47, 1,{0x46}},
{0x48, 1,{0x00}},
{0x49, 1,{0x53}},
{0x4a, 1,{0x00}},
{0x4b, 1,{0x5F}},
{0x4c, 1,{0x00}},
{0x4d, 1,{0x6A}},
{0x4e, 1,{0x00}},
{0x4f, 1,{0x93}},
{0x50, 1,{0x00}},
{0x51, 1,{0xB5}},
{0x52, 1,{0x00}},
{0x53, 1,{0xEB}},
{0x54, 1,{0x01}},
{0x55, 1,{0x16}},
{0x56, 1,{0x01}},
{0x57, 1,{0x5A}},
{0x58, 1,{0x01}},
{0x59, 1,{0x91}},
{0x5a, 1,{0x01}},
{0x5b, 1,{0x93}},
{0x5c, 1,{0x01}},
{0x5d, 1,{0xC8}},
{0x5e, 1,{0x02}},
{0x5f, 1,{0x09}},
{0x60, 1,{0x02}},
{0x61, 1,{0x35}},
{0x62, 1,{0x02}},
{0x63, 1,{0x74}},
{0x64, 1,{0x02}},
{0x65, 1,{0xA0}},
{0x66, 1,{0x02}},
{0x67, 1,{0xDC}},
{0x68, 1,{0x02}},
{0x69, 1,{0xEE}},
{0x6a, 1,{0x03}},
{0x6b, 1,{0x03}},
{0x6c, 1,{0x03}},
{0x6d, 1,{0x1B}},
{0x6e, 1,{0x03}},
{0x6f, 1,{0x36}},
{0x70, 1,{0x03}},
{0x71, 1,{0x57}},
{0x72, 1,{0x03}},
{0x73, 1,{0x80}},
{0x74, 1,{0x03}},
{0x75, 1,{0xB9}},
{0x76, 1,{0x03}},
{0x77, 1,{0xCF}},
{0x78, 1,{0x01}},
{0x79, 1,{0x01}},
{0xff, 3,{0x78,0x07,0x04}},
{0x00, 1,{0x00}},
{0x01, 1,{0x00}},
{0x02, 1,{0x00}},
{0x03, 1,{0x09}},
{0x04, 1,{0x00}},
{0x05, 1,{0x1A}},
{0x06, 1,{0x00}},
{0x07, 1,{0x2C}},
{0x08, 1,{0x00}},
{0x09, 1,{0x3D}},
{0x0a, 1,{0x00}},
{0x0b, 1,{0x4C}},
{0x0c, 1,{0x00}},
{0x0d, 1,{0x59}},
{0x0e, 1,{0x00}},
{0x0f, 1,{0x65}},
{0x10, 1,{0x00}},
{0x11, 1,{0x71}},
{0x12, 1,{0x00}},
{0x13, 1,{0x9B}},
{0x14, 1,{0x00}},
{0x15, 1,{0xBC}},
{0x16, 1,{0x00}},
{0x17, 1,{0xF3}},
{0x18, 1,{0x01}},
{0x19, 1,{0x1D}},
{0x1a, 1,{0x01}},
{0x1b, 1,{0x5F}},
{0x1c, 1,{0x01}},
{0x1d, 1,{0x96}},
{0x1e, 1,{0x01}},
{0x1f, 1,{0x98}},
{0x20, 1,{0x01}},
{0x21, 1,{0xCD}},
{0x22, 1,{0x02}},
{0x23, 1,{0x0C}},
{0x24, 1,{0x02}},
{0x25, 1,{0x37}},
{0x26, 1,{0x02}},
{0x27, 1,{0x78}},
{0x28, 1,{0x02}},
{0x29, 1,{0xA5}},
{0x2a, 1,{0x02}},
{0x2b, 1,{0xE7}},
{0x2c, 1,{0x02}},
{0x2d, 1,{0xFD}},
{0x2e, 1,{0x03}},
{0x2f, 1,{0x18}},
{0x30, 1,{0x03}},
{0x31, 1,{0x2E}},
{0x32, 1,{0x03}},
{0x33, 1,{0x47}},
{0x34, 1,{0x03}},
{0x35, 1,{0x65}},
{0x36, 1,{0x03}},
{0x37, 1,{0x8B}},
{0x38, 1,{0x03}},
{0x39, 1,{0xBA}},
{0x3a, 1,{0x03}},
{0x3b, 1,{0xCF}},
{0x3c, 1,{0x00}},
{0x3d, 1,{0x00}},
{0x3e, 1,{0x00}},
{0x3f, 1,{0x09}},
{0x40, 1,{0x00}},
{0x41, 1,{0x1A}},
{0x42, 1,{0x00}},
{0x43, 1,{0x2C}},
{0x44, 1,{0x00}},
{0x45, 1,{0x3D}},
{0x46, 1,{0x00}},
{0x47, 1,{0x4C}},
{0x48, 1,{0x00}},
{0x49, 1,{0x59}},
{0x4a, 1,{0x00}},
{0x4b, 1,{0x65}},
{0x4c, 1,{0x00}},
{0x4d, 1,{0x71}},
{0x4e, 1,{0x00}},
{0x4f, 1,{0x9B}},
{0x50, 1,{0x00}},
{0x51, 1,{0xBC}},
{0x52, 1,{0x00}},
{0x53, 1,{0xF3}},
{0x54, 1,{0x01}},
{0x55, 1,{0x1D}},
{0x56, 1,{0x01}},
{0x57, 1,{0x5F}},
{0x58, 1,{0x01}},
{0x59, 1,{0x96}},
{0x5a, 1,{0x01}},
{0x5b, 1,{0x98}},
{0x5c, 1,{0x01}},
{0x5d, 1,{0xCD}},
{0x5e, 1,{0x02}},
{0x5f, 1,{0x0C}},
{0x60, 1,{0x02}},
{0x61, 1,{0x37}},
{0x62, 1,{0x02}},
{0x63, 1,{0x78}},
{0x64, 1,{0x02}},
{0x65, 1,{0xA5}},
{0x66, 1,{0x02}},
{0x67, 1,{0xE7}},
{0x68, 1,{0x02}},
{0x69, 1,{0xFD}},
{0x6a, 1,{0x03}},
{0x6b, 1,{0x18}},
{0x6c, 1,{0x03}},
{0x6d, 1,{0x2E}},
{0x6e, 1,{0x03}},
{0x6f, 1,{0x47}},
{0x70, 1,{0x03}},
{0x71, 1,{0x65}},
{0x72, 1,{0x03}},
{0x73, 1,{0x8B}},
{0x74, 1,{0x03}},
{0x75, 1,{0xBA}},
{0x76, 1,{0x03}},
{0x77, 1,{0xCF}},
{0x78, 1,{0x01}},
{0x79, 1,{0x01}},
{0xff, 3,{0x78,0x07,0x00}},
{0x35, 1,{0x00}},


	//CMD_Page 0
	{0xFF,3,{0x98,0x81,0x00}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0,	5,	{0x55, 0xaa, 0x52,0x08,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(2);
       	}
    }

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count=720*3;

    params->dsi.vertical_sync_active                = 6;// 3    2
    params->dsi.vertical_backporch                  = 18;// 20   1
    params->dsi.vertical_frontporch                 = 20; // 1  12
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 20; //60;// 50  2
    params->dsi.horizontal_backporch                = 80; //140;//90
    params->dsi.horizontal_frontporch               = 80; //90
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    params->dsi.ssc_disable                         = 1;
    params->dsi.ssc_range                         = 4;
    params->dsi.HS_TRAIL                             = 15;
/*    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
*/
    params->dsi.PLL_CLOCK =470; //220;
}

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(50);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(50);
  SET_RESET_PIN(1);
  MDELAY(120);
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}
static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
	lcm_init();
	//lcm_compare_id();
}

static unsigned int lcm_compare_id(void)
{

	int array[4];
	char buffer[3];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(100);

	//{0x39, 0xFF, 5, { 0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1 CMD
	array[0] = 0x00043902;
	array[1] = 0x018178FF;
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x00, &buffer[0], 1);  //0x98

	id = buffer[0];
#ifdef BUILD_LK
  printf("%s, LK ili7807 debug: nt35590 id = 0x%08x\n", __func__, id);
#else
  printk("%s, kernel ili7807 horse debug: nt35590 id = 0x%08x\n", __func__, id);
#endif

  return (LCM_ID_ILI7807 == id) ? 1 : 0;
}

LCM_DRIVER ili7807b_lg50_tcl_fhd_lcm_drv =
{
	.name			= "ili7807b_lg50_tcl_fhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
};

