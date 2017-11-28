/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/lm.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/am_eth_pinmux.h>
#include <mach/nand.h>
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <mach/power_gate.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>

#ifdef CONFIG_AM_UART_WITH_S_CORE 
#include <linux/uart-aml.h>
#endif
#include <mach/card_io.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <mach/clk_set.h>
#include "board-8726m-refb14.h"

#if defined(CONFIG_TOUCHSCREEN_ADS7846)
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/ads7846.h>
#endif

#ifdef CONFIG_ANDROID_PMEM
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#endif

#ifdef CONFIG_SENSORS_MXC622X
#include <linux/mxc622x.h>
#endif

#ifdef CONFIG_SENSORS_MMC31XX
#include <linux/mmc31xx.h>
#endif

#ifdef CONFIG_SN7325
#include <linux/sn7325.h>
#endif

#ifdef CONFIG_TWX_TC101
#include <linux/twx_tc101.h>
#endif

#ifdef CONFIG_AMLOGIC_PM
#include <linux/power_supply.h>
#include <linux/aml_power.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#ifdef CONFIG_TOUCH_KEY_PAD_HA2605
#include <linux/i2c/ha2605.h>
#endif

#ifdef CONFIG_SND_AML_M1_MID_CS42L52
#include <sound/cs42l52.h>
#endif

#ifdef CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN
#include <linux/goodix_touch.h>
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
#include <media/amlogic/aml_camera.h>
#endif

#ifdef CONFIG_HAPTIC_ISA1200
#include <linux/i2c/isa1200.h>
#endif

int board_ver = 2;

#if defined(CONFIG_JPEGLOGO)
static struct resource jpeglogo_resources[] = {
    [0] = {
        .start = CONFIG_JPEGLOGO_ADDR,
        .end   = CONFIG_JPEGLOGO_ADDR + CONFIG_JPEGLOGO_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device jpeglogo_device = {
    .name = "jpeglogo-dev",
    .id   = 0,
    .num_resources = ARRAY_SIZE(jpeglogo_resources),
    .resource      = jpeglogo_resources,
};
#endif

#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_KEYPADS_AM_MODULE)
static struct resource intput_resources[] = {
    {
        .start = 0x0,
        .end = 0x0,
        .name="8726",
        .flags = IORESOURCE_IO,
    },
};

static struct platform_device input_device = {
    .name = "m1-kp",
    .id = 0,
    .num_resources = ARRAY_SIZE(intput_resources),
    .resource = intput_resources,
    
};
#endif

#ifdef CONFIG_SARADC_AM
#include <linux/saradc.h>
static struct platform_device saradc_device = {
    .name = "saradc",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};
#endif

#ifdef CONFIG_ADC_TOUCHSCREEN_AM
#include <linux/adc_ts.h>

static struct adc_ts_platform_data adc_ts_pdata = {
    .irq = -1,  //INT_SAR_ADC
    .x_plate_ohms = 400,
};

static struct platform_device adc_ts_device = {
    .name = "adc_ts",
    .id = 0,
    .dev = {
        .platform_data = &adc_ts_pdata,
    },
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static int adc_kp_led_control(int *param)
{
    if(param[0] == 0){//led off
        printk("%s : led off\n", __FUNCTION__);
        return 0;
    }else if(param[0] == 1) {//led on
    	if ((param[1]!=KEY_PAGEUP) && (param[1]!=KEY_PAGEDOWN)){
            printk("%s : led on\n", __FUNCTION__);
            return 0;
        }
    }
    else if(param[0] == 2) {//start counting
	return 1;
        }
    return 0;
}

static struct adc_key adc_kp_key[] = {
    {KEY_HOME,              "home", CHAN_4, 0, 60},     //0v    
    {KEY_PAGEDOWN,          "vol-", CHAN_4, 480, 60},   //1.6
    {KEY_PAGEUP,            "vol+", CHAN_4, 670, 60},   //2.2
};

static struct adc_kp_platform_data adc_kp_pdata = {
    .led_control = adc_kp_led_control,
    .led_control_param_num =2,
    .key = &adc_kp_key[0],
    .key_num = ARRAY_SIZE(adc_kp_key),
};

static struct platform_device adc_kp_device = {
    .name = "m1-adckp",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
    .platform_data = &adc_kp_pdata,
    }
};
#endif

#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

int _key_code_list[] = {KEY_POWER};

static inline int key_input_init_func(void)
{
    WRITE_CBUS_REG(0x21d0/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d0/*RTC_ADDR0*/) &~(1<<11)));
    WRITE_CBUS_REG(0x21d1/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d1/*RTC_ADDR0*/) &~(1<<3)));
    return 0;
}
static inline int key_scan(int *key_state_list)
{
    int ret = 0;
    key_state_list[0] = ((READ_CBUS_REG(0x21d1/*RTC_ADDR1*/) >> 2) & 1) ? 0 : 1;
    return ret;
}

static  struct key_input_platform_data  key_input_pdata = {
    .scan_period = 20,
    .fuzz_time = 60,
    .key_code_list = &_key_code_list[0],
    .key_num = ARRAY_SIZE(_key_code_list),
    .scan_func = key_scan,
    .init_func = key_input_init_func,
    .config = 0,
};

static struct platform_device input_device_key = {
    .name = "m1-keyinput",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &key_input_pdata,
    }
};
#endif

#ifdef CONFIG_SN7325

static int sn7325_pwr_rst(void)
{
    //reset GPIOD_20 (AA5)
    set_gpio_val(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), 0); //low
    set_gpio_mode(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), GPIO_OUTPUT_MODE);

    udelay(2); //delay 2us

    set_gpio_val(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), 1); //high
    set_gpio_mode(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), GPIO_OUTPUT_MODE);
    //end

    return 0;
}

static struct sn7325_platform_data sn7325_pdata = {
    .pwr_rst = &sn7325_pwr_rst,
};
#endif

#if defined(CONFIG_FB_AM)
static struct resource fb_device_resources[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#if defined(CONFIG_FB_OSD2_ENABLE)
    [1] = {
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#endif
};

static struct platform_device fb_device = {
    .name       = "mesonfb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(fb_device_resources),
    .resource      = fb_device_resources,
};
#endif
#ifdef CONFIG_USB_DWC_OTG_HCD
static void set_usb_a_vbus_power(char is_power_on)
{  /*GPIOA_26 (D1)*/
#define USB_A_POW_GPIO          PREG_EGPIO
#define USB_A_POW_GPIO_BIT      3
#define USB_A_POW_GPIO_BIT_ON   1
#define USB_A_POW_GPIO_BIT_OFF  0
    if(is_power_on) {
        printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_ON);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
    }
    else {
        printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_OFF);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
    }
}
//usb_a is OTG port
static struct lm_device usb_ld_a = {
    .type = LM_DEVICE_TYPE_USB,
    .id = 0,
    .irq = INT_USB_A,
    .resource.start = IO_USB_A_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_OTG,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE,
    .set_vbus_power = set_usb_a_vbus_power,
};
#endif
#ifdef CONFIG_SATA_DWC_AHCI
static struct lm_device sata_ld = {
    .type = LM_DEVICE_TYPE_SATA,
    .id = 2,
    .irq = INT_SATA,
    .dma_mask_room = DMA_BIT_MASK(32),
    .resource.start = IO_SATA_BASE,
    .resource.end = -1,
};
#endif

#if defined(CONFIG_AM_STREAMING)
static struct resource codec_resources[] = {
    [0] = {
        .start =  CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
	 .end = STREAMBUF_ADDR_END,
	 .flags = IORESOURCE_MEM,
    },
};

static struct platform_device codec_device = {
    .name       = "amstream",
    .id         = 0,
    .num_resources = ARRAY_SIZE(codec_resources),
    .resource      = codec_resources,
};
#endif

#if defined(CONFIG_AM_VIDEO)
static struct resource deinterlace_resources[] = {
    [0] = {
        .start =  DI_ADDR_START,
        .end   = DI_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device deinterlace_device = {
    .name       = "deinterlace",
    .id         = 0,
    .num_resources = ARRAY_SIZE(deinterlace_resources),
    .resource      = deinterlace_resources,
};
#endif

#if defined(CONFIG_TVIN_VDIN)
static struct resource vdin_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,  //pbufAddr
        .end   = VDIN_ADDR_END,     //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = VDIN_ADDR_START,
        .end   = VDIN_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
    [3] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vdin_device = {
    .name       = "vdin",
    .id         = -1,
    .num_resources = ARRAY_SIZE(vdin_resources),
    .resource      = vdin_resources,
};
#endif

#ifdef CONFIG_TVIN_BT656IN
//add pin mux info for bt656 input
#if 0
static struct resource bt656in_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,      //pbufAddr
        .end   = VDIN_ADDR_END,             //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {     //bt656/camera/bt601 input resource pin mux setting
        .start =  0x3000,       //mask--mux gpioD 15 to bt656 clk;  mux gpioD 16:23 to be bt656 dt_in
        .end   = PERIPHS_PIN_MUX_5 + 0x3000,
        .flags = IORESOURCE_MEM,
    },

    [2] = {         //camera/bt601 input resource pin mux setting
        .start =  0x1c000,      //mask--mux gpioD 12 to bt601 FIQ; mux gpioD 13 to bt601HS; mux gpioD 14 to bt601 VS;
        .end   = PERIPHS_PIN_MUX_5 + 0x1c000,
        .flags = IORESOURCE_MEM,
    },

    [3] = {         //bt601 input resource pin mux setting
        .start =  0x800,        //mask--mux gpioD 24 to bt601 IDQ;;
        .end   = PERIPHS_PIN_MUX_5 + 0x800,
        .flags = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,
//    .num_resources = ARRAY_SIZE(bt656in_resources),
//    .resource      = bt656in_resources,
};
#endif

#if defined(CONFIG_CARDREADER)
static struct resource amlogic_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};

void extern_wifi_reset(int is_on)
{
    if(is_on){
        /*high*/
#ifdef CONFIG_SN7325
        /*WL_RST_N PP4*/
        configIO(1, 0);
        setIO_level(1, 1, 4);
#endif
    }
    else{
        /*low*/
#ifdef CONFIG_SN7325
        /*WL_RST_N PP4*/
        configIO(1, 0);
        setIO_level(1, 0, 4);
#endif
    }
    printk("extern_wifi_reset %d\n", is_on);
}
EXPORT_SYMBOL(extern_wifi_reset);

void extern_wifi_power(int is_power)
{
  /* WIFI/BT REGON OD5 */
    if (0 == is_power)
    {
        #ifdef CONFIG_SN7325
        /*WIFI/BT_EN OD5*/
        configIO(0, 0);
        setIO_level(0, 0, 5);
        #endif
    }
    else
    {
        #ifdef CONFIG_SN7325
        /*WIFI/BT_EN OD5*/
        configIO(0, 0);
        setIO_level(0, 1, 5);
        #endif
    }
    printk("extern_wifi_power %d\n", is_power);
}

EXPORT_SYMBOL(extern_wifi_power);

#define GPIO_WIFI_HOSTWAKE  ((GPIOD_bank_bit2_24(13)<<16) |GPIOD_bit_bit2_24(13))

void sdio_extern_init(void)
{
    //set clk for wifi
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<18));
    CLEAR_CBUS_REG_MASK(PREG_EGPIO_EN_N, (1<<4));

#if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)/* Jone add */
    gpio_direction_input(GPIO_WIFI_HOSTWAKE);
    gpio_enable_level_int(63, 0, 4);
    gpio_enable_edge_int(63, 0, 4);
#endif /* (CONFIG_BCM4329_HW_OOB) || (CONFIG_BCM4329_OOB_INTR_ONLY) Jone add */

    extern_wifi_power(1);
    extern_wifi_reset(0);
    msleep(100);
    extern_wifi_reset(1);
}

void inand_extern_init(void)
{
	CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_1,( (1<<29) | (1<<27) | (1<<25) | (1<<23)|0x3f));
	CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_6,0x7fff);
	/*attension, PINMUX 7 not 0x3d<<24 */
	SET_CBUS_REG_MASK(CARD_PIN_MUX_7,(0x3f<<24));
}
static struct mtd_partition inand_partition_info[] = 
{
    {
        .name = "charge_logo",
        .offset = 16*SZ_1M,
        .size = 16*SZ_1M,
    },
	{
		.name = "logo",
		.offset = 32*SZ_1M,
		.size = 16*SZ_1M,
	},
	{
		.name = "aml_logo",
		.offset = 48*SZ_1M,
		.size = 16*SZ_1M,
	},
	{
		.name = "recovery",
		.offset = 64*SZ_1M,
		.size = 32*SZ_1M,
	},
	{
		.name = "boot",
		.offset = 96*SZ_1M,
		.size = 32*SZ_1M,
	},
	{
		.name = "system",
		.offset = 128*SZ_1M,
		.size = 256*SZ_1M,
	},
	{
		.name = "cache",
		.offset = 384*SZ_1M,
		.size = 128*SZ_1M,
	},
	{
		.name = "userdata",
		.offset = 512*SZ_1M,
		.size = 512*SZ_1M,
	},
	{
		.name = "media",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
};

static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_GPIOA_9_14,
        .card_ins_en_reg = EGPIO_GPIOC_ENABLE,
        .card_ins_en_mask = PREG_IO_0_MASK,
        .card_ins_input_reg = EGPIO_GPIOC_INPUT,
        .card_ins_input_mask = PREG_IO_0_MASK,
        .card_power_en_reg = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev = 0,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = 0,
    },
    [1] = {
        .name = "sdio_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_GPIOB_2_7,
        .card_ins_en_reg = 0,
        .card_ins_en_mask = 0,
        .card_ins_input_reg = 0,
        .card_ins_input_mask = 0,
        .card_power_en_reg = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev = 1,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = sdio_extern_init,
    },
    [2] = {
	.name = "inand_card",
	.work_mode = CARD_HW_MODE,
	.io_pad_type = SDIO_GPIOE_6_11,
	.card_ins_en_reg = 0,
	.card_ins_en_mask = 0,
	.card_ins_input_reg = 0,
	.card_ins_input_mask = 0,
	.card_power_en_reg = 0,
	.card_power_en_mask = 0,
	.card_power_output_reg = 0,
	.card_power_output_mask = 0,
	.card_power_en_lev = 0,
	.card_wp_en_reg = 0,
	.card_wp_en_mask = 0,
	.card_wp_input_reg = 0,
	.card_wp_input_mask = 0,
	.card_extern_init = inand_extern_init,
	.partitions = inand_partition_info,
	.nr_partitions = ARRAY_SIZE(inand_partition_info),
	},
};

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = { 
    .name = "AMLOGIC_CARD", 
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};

#endif

#if defined (CONFIG_AMLOGIC_VIDEOIN_MANAGER)
static struct resource vm_resources[] = {
    [0] = {
        .start =  VM_ADDR_START,
        .end   = VM_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vm_device =
{
	.name = "vm",
	.id = 0,
    .num_resources = ARRAY_SIZE(vm_resources),
    .resource      = vm_resources,
};
#endif /* AMLOGIC_VIDEOIN_MANAGER */

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308)
static int gc0308_v4l2_init(void)
{
    udelay(1000);
    WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,0x30f);// 24M XTAL
    //WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,0x31e);// 12M XTAL
    WRITE_CBUS_REG(HHI_DEMOD_PLL_CNTL,0x232);// 24M XTAL
    udelay(1000);

    eth_set_pinmux(ETH_BANK0_GPIOC3_C12,ETH_CLK_OUT_GPIOC12_REG3_1, 1);
#ifdef CONFIG_SN7325
    printk( "amlogic camera driver: init gc0308_v4l2_init. \n");
    configIO(1, 0);
    setIO_level(1, 0, 0);//CAMR_PWDN set high into Work mode
    configIO(0, 0);
    setIO_level(0, 0, 7);//CAME_EN set low
    setIO_level(0, 0, 3);//SENSOR_RST1 set low
    msleep(30);
    setIO_level(0, 1, 7);//CAME_EN set high
    msleep(30);
    setIO_level(0, 1, 3);//SENSOR_RST1 set high
    msleep(30);
#endif
    return 0;
}
static int gc0308_v4l2_uninit(void)
{
#ifdef CONFIG_SN7325
    printk( "amlogic camera driver: uninit gc0308_v4l2_uninit. \n");
    configIO(0, 0);
    setIO_level(0, 0, 3);//SENSOR_RST1 set low
    setIO_level(0, 0, 7);//CAME_EN set low
    configIO(1, 0);
    setIO_level(1, 1, 0);//CAMR_PWDN set high into Power Down mode
    msleep(300); 
#endif
    return 0;
}

static struct aml_camera_i2c_fig1_s gc0308_custom_init_script[] = {
	{0xff,0xff},
};

aml_plat_cam_data_t video_gc0308_data = {
	.name="video-gc0308",
	.video_nr=1,
	.device_init= gc0308_v4l2_init,
	.device_uninit=gc0308_v4l2_uninit,
	.custom_init_script = gc0308_custom_init_script,
};
#endif /* CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308 */

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)
//#include <media/amlogic/aml_camera.h>

static int gt2005_v4l2_init(void)
{
    udelay(1000);
    WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,0x30f);// 24M XTAL
    WRITE_CBUS_REG(HHI_DEMOD_PLL_CNTL,0x232);// 24M XTAL
	  udelay(1000);

    eth_set_pinmux(ETH_BANK0_GPIOC3_C12,ETH_CLK_OUT_GPIOC12_REG3_1, 1);
    
    //set_gpio_val(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), 1); //low
    //set_gpio_mode(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), GPIO_OUTPUT_MODE);
    set_gpio_val(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), 1); //low
    set_gpio_mode(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), GPIO_OUTPUT_MODE);
    msleep(300);

}
static int gt2005_v4l2_uninit(void)
{
    //set_gpio_val(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), 0); //low
    //set_gpio_mode(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), GPIO_OUTPUT_MODE);
    set_gpio_val(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), 0); //low
    set_gpio_mode(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), GPIO_OUTPUT_MODE);
    msleep(300);
}

aml_plat_cam_data_t video_gt2005_data = {
	.name="video-gt2005",
	.video_nr=0,
	.device_init= gt2005_v4l2_init,
	.device_uninit=gt2005_v4l2_uninit,
};
#endif /* VIDEO_AMLOGIC_CAPTURE_GT2005 */

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device audiodsp_device = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(audiodsp_resources),
    .resource      = audiodsp_resources,
};
#endif

static struct resource aml_m1_audio_resource[]={
        [0] =   {
                .start  =   0,
                .end        =   0,
                .flags  =   IORESOURCE_MEM,
        },
};

static struct platform_device aml_audio={
#ifdef CONFIG_SND_AML_M1_MID_CS42L52
        .name 				= "aml_m1_audio_cs42l52",
#endif
		.id 					= -1,
		.resource 		=	aml_m1_audio_resource,
		.num_resources	=	ARRAY_SIZE(aml_m1_audio_resource),
};
#ifdef CONFIG_SND_AML_M1_MID_CS42L52
static int cs42l52_pwr_rst(void)
{
    //reset GPIOA_3 (A17)
    set_gpio_val(GPIOA_bank_bit0_14(3), GPIOA_bit_bit0_14(3), 0); //low
    set_gpio_mode(GPIOA_bank_bit0_14(3), GPIOA_bit_bit0_14(3), GPIO_OUTPUT_MODE);

    udelay(20); //delay 2us

    set_gpio_val(GPIOA_bank_bit0_14(3), GPIOA_bit_bit0_14(3), 1); //high
    set_gpio_mode(GPIOA_bank_bit0_14(3), GPIOA_bit_bit0_14(3), GPIO_OUTPUT_MODE);
    //end

    return 0;
}
static struct cs42l52_platform_data cs42l52_pdata = {
    .cs42l52_pwr_rst = &cs42l52_pwr_rst,
};
#endif

#ifdef CONFIG_HX8520_CAPACITIVE_TOUCHSCREEN
#include <linux/capts.h>
/* GPIOD_24 */
#define TS_IRQ_GPIO  ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24))
#define TS_IRQ_IDX     (GPIOD_IDX + 24)
#define TS_RESET_GPIO  ((GPIOD_bank_bit2_24(23)<<16) |GPIOD_bit_bit2_24(23))

static int ts_init_irq(void);
static int ts_get_irq_level(void);
static struct ts_platform_data ts_pdata = {
    .mode = TS_MODE_INT_FALLING,
    .irq = INT_GPIO_0,
    .init_irq = ts_init_irq,
    .get_irq_level = ts_get_irq_level,
    .poll_period = 16*1000000, //16msecs
    .cache_enable = 1,
    .info = {
        .xmin = 0,
        .xmax = 1280,
        .ymin = 0,
        .ymax = 768,
        .zmin = 0,
        .zmax = 1,
        .wmin = 0,
        .wmax = 1,
        .swap_xy = 0,
        .x_pol = 0,
        .y_pol = 1,
    },
    .data = (void *)(TS_RESET_GPIO+1),
};

static int ts_init_irq(void)
{
    int group = ts_pdata.irq - INT_GPIO_0;
    int mode =  ts_pdata.mode;
    
    if (mode < TS_MODE_TIMER_READ) {
        gpio_direction_input(TS_IRQ_GPIO);
        if (mode == TS_MODE_INT_FALLING) {
            gpio_enable_edge_int(TS_IRQ_IDX, 1, group);
        }
        else if (mode == TS_MODE_INT_RISING) {
            gpio_enable_edge_int(TS_IRQ_IDX, 0, group);
        }
        else if (mode == TS_MODE_INT_LOW) {
            gpio_enable_level_int(TS_IRQ_IDX, 1, group);
        }
        else if (mode == TS_MODE_INT_HIGH) {
            gpio_enable_level_int(TS_IRQ_IDX, 0, group);
        }
    }
    return 0;
}

static int ts_get_irq_level(void)
{
    return gpio_get_value(TS_IRQ_GPIO);
}
#endif


#ifdef CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN
u8 ts_config_data[] = {
//    0x30,0x19,0x05,0x06,0x28,0x02,0x14,0x14,0x10,0x1E,
//    0x70,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,
//    0xAB,0xCD,0xE0,0x00,0x00,0x00,0x00,0x4D,0xC7,0x20,
//    0x03,0x00,0x00,0x50,0x3C,0x1E,0xB4,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x01
//0x30, 0x19,0x05,0x04,0x28,0x02,0x14,0x40,0x10,0x3C,0xB0,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xE0,0x00,0x00,0x00,0x00,0x4D,0xC0,0x20,0x03,0x00,0x00,0x50,0x3C,0x1E,0xB4,0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
//0x30, 0x19,0x05,0x04,0x28,0x02,0x14,0x60,0x10,0x3C,0xB0,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xE0,0x00,0x00,0x00,0x00,0x4D,0xC4,0x20,0x01,0x01,0x03,0x50,0x3C,0x1E,0xB4,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
//0x30,0x19,0x05,0x04,0x28,0x02,0x14,0x60,0x10,0x3C,0xB0,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xE0,0x00,0x00,0x32,0x28,0x4D,0xC4,0x20,0x01,0x01,0x03,0x50,0x3C,0x1E,0xB4,0x00,0x2B,0x27,0x01,0xB4,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
0x30,0x19,0x05,0x05,0x28,0x02,0x14,0x14,0x10,0x2D,0xF2,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xE0,0x00,0x00,0x37,0x2E,0x4D,0xC1,0x20,0x01,0x00,0xA0,0x3C,0x3C,0x1E,0xB4,0x10,0x35,0x2C,0x01,0xEC,0x28,0x37,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
};
   
static struct goodix_i2c_rmi_platform_data ts_pdata = {
    .gpio_shutdown = ((GPIOA_bank_bit(4)<<16) | GPIOA_bit_bit0_14(4)),
    .gpio_irq = ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24)),
    .irq_edge = 1, /* 0:rising edge, 1:falling edge */
    .swap_xy = 1,
    .xpol = 1,//0
    .ypol = 0,//1
    .xmax = 7680,
    .ymax = 5120,
    .config_info_len = ARRAY_SIZE(ts_config_data),
    .config_info = ts_config_data,
};
#endif

#ifdef CONFIG_EETI_CAPACITIVE_TOUCHSCREEN
#include <linux/i2c/eeti.h>

//GPIOD_24
#define GPIO_EETI_PENIRQ ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24)) 
#define GPIO_EETI_RST

static int eeti_init_irq(void)
{
/* memson
    Bit(s)  Description
    256-105 Unused
    104     JTAG_TDO
    103     JTAG_TDI
    102     JTAG_TMS
    101     JTAG_TCK
    100     gpioA_23
    99      gpioA_24
    98      gpioA_25
    97      gpioA_26
    98-76    gpioE[21:0]
    75-50   gpioD[24:0]
    49-23   gpioC[26:0]
    22-15   gpioB[22;15]
    14-0    gpioA[14:0]
 */

    /* set input mode */
    gpio_direction_input(GPIO_EETI_PENIRQ);
    /* set gpio interrupt #0 source=GPIOD_24, and triggered by falling edge(=1) */
    gpio_enable_edge_int(50+24, 1, 0);

    return 0;
}
static int eeti_get_irq_level(void)
{
    return gpio_get_value(GPIO_EETI_PENIRQ);
}

static struct eeti_platform_data eeti_pdata = {
    .init_irq = &eeti_init_irq,
    .get_irq_level = &eeti_get_irq_level,
    .tp_max_width = 32752,
    .tp_max_height = 32752,
    .lcd_max_width = 800,
    .lcd_max_height = 600,
};
#endif


#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
#include <linux/i2c/pixcir_i2c_ts.h>
static struct pixcir_i2c_ts_platform_data pixcir_pdata = {
     .gpio_shutdown = ((GPIOA_bank_bit(4)<<16) | GPIOA_bit_bit0_14(4)),
	.gpio_irq = (GPIOD_bank_bit2_24(24)<<16) | GPIOD_bit_bit2_24(24),
	.xmin = 0,
	.xmax = 1280,
	.ymin = 0,
	.ymax = 768,
  .swap_xy = 1,
  .xpol = 1,
  .ypol = 1,
  .point_id_available = 0,	
};
#endif

#ifdef CONFIG_TOUCH_KEY_PAD_HA2605
#include <linux/input.h>
//GPIOC_21
#define GPIO_HA2605_ATTN	((GPIOC_bank_bit0_26(21)<<16) |GPIOC_bit_bit0_26(21)) 
static int ha2605_init_irq(void)
{
	/* set input mode */
	gpio_direction_input(GPIO_HA2605_ATTN);
	/* set gpio interrupt #1 source=GPIOC_21, and triggered by rising edge(=0) */
	gpio_enable_edge_int(23+21, 0, 1);
	return 0;
}

static int ha2605_get_irq_level(void)
{
	return !gpio_get_value(GPIO_HA2605_ATTN);
}

static struct cap_key ha2605_keys[] = {
	{ 125,		1,	"menu"},
	{ 15,		2,	"back"},
};

static struct ha2605_platform_data ha2605_pdata = {
	.init_irq = ha2605_init_irq,
	.get_irq_level = ha2605_get_irq_level,
	.key = ha2605_keys,
	.key_num = ARRAY_SIZE(ha2605_keys),
};
#endif

#ifdef CONFIG_HAPTIC_ISA1200

void isa1200_setup_pin()
{
    /* MOTOR_HEN PP2 */
#ifdef CONFIG_SN7325
    configIO(1, 0);
    setIO_level(1, 1, 2);
#endif

//    msleep(200);

    /* MOTOR_LEN PP1 */
#ifdef CONFIG_SN7325
    configIO(1, 0);
    setIO_level(1, 1, 1);
#endif
}

int isa1200_power_switch(int onoff)
{
//	printk("%s: %d\n", __func__, onoff);

    if(onoff)
    {
    /* MOTOR_HEN PP1 */
#ifdef CONFIG_SN7325
        configIO(1, 0);
        setIO_level(1, 1, 2);
#endif
    }
    else
    {
    /* MOTOR_HEN PP1 */
#ifdef CONFIG_SN7325
        configIO(1, 0);
        setIO_level(1, 0, 2);
#endif
    }

    return 0;
}

int isa1200_pwm_config(int period, int duty)
{
    int val, val_low;

//    printk("isa1200 pwm set to period=%d duty=%d\n", period, duty);

    val = 24000000/period;
    val_low = val * (128 + duty) / 256;

    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<21));
    SET_CBUS_REG_MASK(PWM_MISC_REG_CD, (1 << 0));

    WRITE_CBUS_REG_BITS(PWM_PWM_C,val_low,0,16);  //low
    WRITE_CBUS_REG_BITS(PWM_PWM_C,val - val_low,16,16);  //hi

    return 0;
}

static struct isa1200_platform_data isa1200_pdata = {
    .name = "vibrator",
    .max_timeout = 30000,
    .setup_pin = isa1200_setup_pin,
    .power_on = isa1200_power_switch,
    .pwm_config = isa1200_pwm_config,
    .ldo_level = 0x2,
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_data =
{
    .name = "pmem",
    .start = PMEM_START,
    .size = PMEM_SIZE,
    .no_allocator = 1,
    .cached = 0,
};

static struct platform_device android_pmem_device =
{
    .name = "android_pmem",
    .id = 0,
    .dev = {
        .platform_data = &pmem_data,
    },
};
#endif

#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#ifdef CONFIG_TWX_TC101
static struct platform_device twx_device = {
    .name       = "twx",
    .id         = -1,
};
#endif

#if defined(CONFIG_SUSPEND)
typedef struct {
	char name[32];
	unsigned bank;
	unsigned bit;
	gpio_mode_t mode;
	unsigned value;
	unsigned enable;
} gpio_data_t;

#define MAX_GPIO 21
static gpio_data_t gpio_data[MAX_GPIO] = {
//  {"GPIOA_7 -- BL_PWM",		   	 GPIOA_bank_bit0_14(7),		GPIOA_bit_bit0_14(7),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOA_6 -- VCCx2_EN",		 GPIOA_bank_bit0_14(6),		GPIOA_bit_bit0_14(6),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOA_3 -- DAC_RST",		 GPIOA_bank_bit0_14(3),		GPIOA_bit_bit0_14(3),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOA_5 -- LCD_CLK",		 GPIOA_bank_bit0_14(5),		GPIOA_bit_bit0_14(5),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOA_2 -- OEH",			 GPIOA_bank_bit0_14(2),		GPIOA_bit_bit0_14(2),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_2 -- WIFI_SD_CMD",		 GPIOB_bank_bit0_7(2),		GPIOB_bit_bit0_7(2),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_3 -- WIFI_SD_CLK",	 	 GPIOB_bank_bit0_7(3),		GPIOB_bit_bit0_7(3),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_4 -- WIFI_SD_D0",	 	 GPIOB_bank_bit0_7(4),		GPIOB_bit_bit0_7(4),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_5 -- WIFI_SD_D1",	 	 GPIOB_bank_bit0_7(5),		GPIOB_bit_bit0_7(5),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_6 -- WIFI_SD_D2",	 	 GPIOB_bank_bit0_7(6),		GPIOB_bit_bit0_7(6),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOB_7 -- WIFI_SD_D3",	 	 GPIOB_bank_bit0_7(7),		GPIOB_bit_bit0_7(7),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOD_18 -- UART_CTS_N",		 GPIOD_bank_bit2_24(18), 	GPIOD_bit_bit2_24(18), 	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOC_3 -- camera PCLK",	 	 GPIOC_bank_bit0_26(3),		GPIOC_bit_bit0_26(3),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_18 -- MOTOR_CLK",	 	 GPIOE_bank_bit16_21(18),	GPIOE_bit_bit16_21(18),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_19 -- RF_L_OUT",	 	 GPIOE_bank_bit16_21(19),	GPIOE_bit_bit16_21(19),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_7 -- iNAND_SD_CMD",		 GPIOE_bank_bit0_15(7),		GPIOE_bit_bit0_15(7),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_6 -- iNAND_SD_CLK",	 	 GPIOE_bank_bit0_15(6),		GPIOE_bit_bit0_15(6),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_8 -- iNAND_SD_D0",	 	 GPIOE_bank_bit0_15(8),		GPIOE_bit_bit0_15(8),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_9 -- iNAND_SD_D1",	 	 GPIOE_bank_bit0_15(9),		GPIOE_bit_bit0_15(9),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_10 -- iNAND_SD_D2",	 	 GPIOE_bank_bit0_15(10),	GPIOE_bit_bit0_15(10),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOE_11 -- iNAND_SD_D3",	 	 GPIOE_bank_bit0_15(11),	GPIOE_bit_bit0_15(11),	GPIO_OUTPUT_MODE, 1, 1},
  {"GPIOC_13 -- Linux_TX",	 	 GPIOC_bank_bit0_26(13),	GPIOC_bit_bit0_26(13), 	GPIO_OUTPUT_MODE, 1, 1},
 // {"TEST_N -- I2S_DOUT",		 GPIOJTAG_bank_bit(16),		GPIOJTAG_bit_bit16(16),	GPIO_OUTPUT_MODE, 1, 1},
};	

static void save_gpio(int port) 
{
	gpio_data[port].mode = get_gpio_mode(gpio_data[port].bank, gpio_data[port].bit);
	if (gpio_data[port].mode==GPIO_OUTPUT_MODE)
	{
		if (gpio_data[port].enable){
			printk("change %s output %d to input\n", gpio_data[port].name, gpio_data[port].value); 
			gpio_data[port].value = get_gpio_val(gpio_data[port].bank, gpio_data[port].bit);
			set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_INPUT_MODE);
		}
		else{
			printk("no change %s output %d\n", gpio_data[port].name, gpio_data[port].value); 
		}
	}
}

static void restore_gpio(int port)
{
	if ((gpio_data[port].mode==GPIO_OUTPUT_MODE)&&(gpio_data[port].enable))
	{
		set_gpio_val(gpio_data[port].bank, gpio_data[port].bit, gpio_data[port].value);
		set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_OUTPUT_MODE);
		printk("%s output %d\n", gpio_data[port].name, gpio_data[port].value); 
	}
}

typedef struct {
	char name[32];
	unsigned reg;
	unsigned bits;
	unsigned enable;
} pinmux_data_t;


#define MAX_PINMUX	13

pinmux_data_t pinmux_data[MAX_PINMUX] = {
	{"HDMI", 	0, (1<<2)|(1<<1)|(1<<0), 					1},
	{"TCON", 	0, (1<<14)|(1<<11), 						1},
	{"I2S_OUT",	0, (1<<18),			 				1},
	{"I2S_CLK",	1, (1<<19)|(1<<15)|(1<<11),		 			1},
	{"SPI",		1, (1<<29)|(1<<27)|(1<<25)|(1<<23),				1},
	{"I2C",		2, (1<<5)|(1<<2),						1},
	{"SD",		2, (1<<15)|(1<<14)|(1<<13)|(1<<12)|(1<<8),			1},
	{"PWM",		0, (1<<22)|(1<<21),						1},
	{"UART_A",	3, (1<<24)|(1<23),						0},
	{"RGB",		4, (1<<4)|(1<<2)|(1<<0),					1},
	{"UART_B",	3, (1<<27)|(1<30),						0},
	{"REMOTE",	5, (1<<31),							1},
	{"CAMERA",	3, (1<<13),							1},
};

static unsigned pinmux_backup[6];

static void save_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		pinmux_backup[i] = READ_CBUS_REG(PERIPHS_PIN_MUX_0+i);
	for (i=0;i<MAX_PINMUX;i++){
		if (pinmux_data[i].enable){
			printk("%s %x\n", pinmux_data[i].name, pinmux_data[i].bits);
			clear_mio_mux(pinmux_data[i].reg, pinmux_data[i].bits);
		}
	}
}

static void restore_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		 WRITE_CBUS_REG(PERIPHS_PIN_MUX_0+i, pinmux_backup[i]);
}

static void set_vccx2(int power_on)
{
	  int i=0;
    if(power_on){
        printk(KERN_INFO "set_vccx2 power up\n");
        for (i=0;i<MAX_GPIO;i++){
        	restore_gpio(i);
        }
        restore_pinmux();
        set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 1);
        set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE); 
        //touch enable
        set_gpio_val(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), 0);
        set_gpio_mode(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), GPIO_OUTPUT_MODE);
        //set clk for wifi
        SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<18));
        CLEAR_CBUS_REG_MASK(PREG_EGPIO_EN_N, (1<<4));	        
    }
    else{
        printk(KERN_INFO "set_vccx2 power down\n");   
		    		
        set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 0);
        set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);   
        //touch disable
        set_gpio_val(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), 1);
        set_gpio_mode(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), GPIO_OUTPUT_MODE);
        //disable wifi clk
        CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<18));
        SET_CBUS_REG_MASK(PREG_EGPIO_EN_N, (1<<4));
		
        save_pinmux();  
        for (i=0;i<MAX_GPIO;i++){
        	save_gpio(i);
        }   	        
    }
}

#ifdef CONFIG_EXGPIO
typedef struct {
	char name[32];
	unsigned bank;
	unsigned bit;
	gpio_mode_t mode;
	unsigned s_value;
	unsigned r_value;
	unsigned enable;
} exgpio_data_t;

#define MAX_EXGPIO 7
static exgpio_data_t exgpio_data[MAX_EXGPIO] = {
    {"OD2 -- KEY_LED",	 	EXGPIO_BANK0,	2,	GPIO_OUTPUT_MODE, 0, 1, 1},
    {"OD4 -- VCCx3_EN",	 	EXGPIO_BANK0,	4,	GPIO_OUTPUT_MODE, 0, 1, 1},
    {"PP1 -- MOTOR_LEN",	EXGPIO_BANK1,	1,	GPIO_OUTPUT_MODE, 1, 1, 1},
    {"PP2 -- MOTOR_HEN",	EXGPIO_BANK1,	2,	GPIO_OUTPUT_MODE, 0, 1, 1},
    {"PP6 -- MOTOR_EN",		EXGPIO_BANK1,	6,	GPIO_OUTPUT_MODE, 0, 0, 1},
    {"OD7 -- CAME_EN",	 	EXGPIO_BANK0,	7,	GPIO_OUTPUT_MODE, 1, 1, 1},
    {"PP0 -- CAMR_PWDN",	EXGPIO_BANK1,	0,	GPIO_OUTPUT_MODE, 1, 1, 1},
    {"PP5 -- BT_RST_N",		EXGPIO_BANK1,	5,	GPIO_OUTPUT_MODE, 0, 1, 1},
    {"PP4 -- WL_RST_N",		EXGPIO_BANK1,	4,	GPIO_OUTPUT_MODE, 1, 1, 1},
};	

static void save_exgpio(int port) 
{
    exgpio_data[port].mode = get_gpio_mode(exgpio_data[port].bank, exgpio_data[port].bit);
    if((exgpio_data[port].mode==GPIO_OUTPUT_MODE) && exgpio_data[port].enable){
        exgpio_data[port].r_value = get_gpio_val(exgpio_data[port].bank, exgpio_data[port].bit);
        set_gpio_val(exgpio_data[port].bank, exgpio_data[port].bit, exgpio_data[port].s_value);
        printk("name:%s, write:%d\n", exgpio_data[port].name, exgpio_data[port].s_value);
	}
    else{
        printk("no change %s output %d\n", exgpio_data[port].name, exgpio_data[port].s_value); 
        }
}

static void restore_exgpio(int port)
{
    if((exgpio_data[port].mode==GPIO_OUTPUT_MODE) && exgpio_data[port].enable){
        set_gpio_val(exgpio_data[port].bank, exgpio_data[port].bit, exgpio_data[port].r_value);
        printk("name:%s, write:%d\n", exgpio_data[port].name, exgpio_data[port].r_value);
        }
    else{
        printk("no change %s output %d\n", exgpio_data[port].name, exgpio_data[port].r_value); 
    }
}

static void set_exgpio_on_early_suspend(int power_on)
{
    int i;

    if(power_on){
        for(i=0;i<MAX_EXGPIO;i++)
            restore_exgpio(i);
        }
    else{
        for(i=0;i<MAX_EXGPIO;i++)
            save_exgpio(i);
        }
}
#else
#define set_exgpio_on_early_suspend NULL
#endif

static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = IO_APB_BUS_BASE,
    .mmc_reg_base = APB_REG_ADDR(0x1000),
    .hiu_reg_base = CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = set_vccx2,
    .core_voltage_adjust = 8,
    .set_exgpio_early_suspend = set_exgpio_on_early_suspend,
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif

#if defined(CONFIG_I2C_SW_AML)

static struct aml_sw_i2c_platform aml_sw_i2c_plat = {
    .sw_pins = {
        .scl_reg_out        = MESON_I2C_PREG_GPIOB_OUTLVL,
        .scl_reg_in         = MESON_I2C_PREG_GPIOB_INLVL,
        .scl_bit            = 0,    /*MESON_I2C_MASTER_B_GPIOB_0_REG*/
        .scl_oe             = MESON_I2C_PREG_GPIOB_OE,
        .sda_reg_out        = MESON_I2C_PREG_GPIOB_OUTLVL,
        .sda_reg_in         = MESON_I2C_PREG_GPIOB_INLVL,
        .sda_bit            = 1,    /*MESON_I2C_MASTER_B_GPIOB_1_REG*/
        .sda_oe             = MESON_I2C_PREG_GPIOB_OE,
    },  
    .udelay         = 2,
    .timeout            = 100,
};

static struct platform_device aml_sw_i2c_device = {
    .name         = "aml-sw-i2c",
    .id       = -1,
    .dev = {
        .platform_data = &aml_sw_i2c_plat,
    },
};

#endif

#if defined(CONFIG_I2C_AML)
static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count     = 1000000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_b_pinmux = {
        .scl_reg    = MESON_I2C_MASTER_B_GPIOB_0_REG,
        .scl_bit    = MESON_I2C_MASTER_B_GPIOB_0_BIT,
        .sda_reg    = MESON_I2C_MASTER_B_GPIOB_1_REG,
        .sda_bit    = MESON_I2C_MASTER_B_GPIOB_1_BIT,
    }
};

static struct resource aml_i2c_resource[] = {
    [0] = {/*master a*/
        .start =    MESON_I2C_MASTER_A_START,
        .end   =    MESON_I2C_MASTER_A_END,
        .flags =    IORESOURCE_MEM,
    },
    [1] = {/*master b*/
        .start =    MESON_I2C_MASTER_B_START,
        .end   =    MESON_I2C_MASTER_B_END,
        .flags =    IORESOURCE_MEM,
    },
    [2] = {/*slave*/
        .start =    MESON_I2C_SLAVE_START,
        .end   =    MESON_I2C_SLAVE_END,
        .flags =    IORESOURCE_MEM,
    },
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = -1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};
#endif

#ifdef CONFIG_AMLOGIC_PM
static int is_ac_connected(void)
{
    return (READ_CBUS_REG(ASSIST_HW_REV)&(1<<9))? 1:0;
}

#ifdef CONFIG_SARADC_AM
extern int get_adc_sample(int chan);
#endif
static int get_bat_vol(void)
{
#ifdef CONFIG_SARADC_AM
    return get_adc_sample(5);
#else
        return 0;
#endif
}

static int get_charge_status(void)
{
    return (READ_CBUS_REG(ASSIST_HW_REV)&(1<<8))? 1:0;
}


static void set_bat_off(void)
{

    //BL_PWM power off
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<31));
    CLEAR_CBUS_REG_MASK(PWM_MISC_REG_AB, (1 << 0));
    set_gpio_val(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), 0);
    set_gpio_mode(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), GPIO_OUTPUT_MODE);

    //VCCx2 power down
    //set_vccx2(0);
    set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 0);
    set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);        

    if(is_ac_connected()){ //AC in after power off press
        kernel_restart("reboot");
    }

    //Power hold down
    set_gpio_val(GPIOA_bank_bit(8), GPIOA_bit_bit0_14(8), 0);
    set_gpio_mode(GPIOA_bank_bit(8), GPIOA_bit_bit0_14(8), GPIO_OUTPUT_MODE);


}

static int bat_value_table[37]={
0,  //0    
700,//0
705,//4
712,//10
717,//15
720,//16
722,//18
724,//20
727,//23
730,//26
733,//29
736,//32
739,//35
742,//37
745,//40
750,//43
755,//46
760,//49
763,//51
768,//54
773,//57
776,//60
781,//63
786,//66
791,//68
794,//71
796,//74
799,//77
801,//80
803,//83
806,//85
808,//88
811,//91
815,//95
820,//97
829,//100
829 //100
};

static int bat_charge_value_table[37]={
0,  //0    
730,//750,//0
735,//755,//4
740,//760,//10
745,//765,//15
750,//769,//16
755,//773,//18
760,//776,//20
765,//778,//23
772,//780,//26599,//29
775,//782,//32
777,//784,//35
779,//786,//37
781,//788,//40
783,//791,//43
786,//793,//46
789,//796,//49
792,//799,//51
794,//802,//54
797,//805,//57
800,//808,//60
803,//811,//63
806,//814,//66
809,//818,//68
812,//821,//71
816,//826,//74
820,//829,//77
826,//835,//80
829,//838,//83
833,//840,//85
836,//843,//88
839,//846,//91
841,//848,//95
843,//849,//97
845,//851,//100
845,//851 //100
};


static int bat_level_table[37]={
0,
0,
4,
10,
15,
16,
18,
20,
23,
26,
29,
32,
35,
37,
40,
43,
46,
49,
51,
54,
57,
60,
63,
66,
68,
71,
74,
77,
80,
83,
85,
88,
91,
95,
97,
100,
100  
};

static struct aml_power_pdata power_pdata = {
	.is_ac_online	= is_ac_connected,
	//.is_usb_online	= is_usb_connected,
	//.set_charge = set_charge,
	.get_bat_vol = get_bat_vol,
	.get_charge_status = get_charge_status,
	.set_bat_off = set_bat_off,
	.bat_value_table = bat_value_table,
	.bat_charge_value_table = bat_charge_value_table,
	.bat_level_table = bat_level_table,
	.bat_table_len = 37,		
	//.supplied_to = supplicants,
	//.num_supplicants = ARRAY_SIZE(supplicants),
};

static struct platform_device power_dev = {
    .name       = "aml-power",
    .id     = -1,
    .dev = {
        .platform_data  = &power_pdata,
    },
};
#endif

#define PINMUX_UART_A   UART_A_GPIO_D21_D22
#define PINMUX_UART_B   UART_B_GPIO_C13_C14

#if defined(CONFIG_AM_UART_WITH_S_CORE)

#if defined(CONFIG_AM_UART0_SET_PORT_A)
#define UART_0_PORT     UART_A
#define UART_1_PORT     UART_B
#elif defined(CONFIG_AM_UART0_SET_PORT_B)
#define UART_0_PORT     UART_B
#define UART_1_PORT     UART_A
#endif

static struct aml_uart_platform aml_uart_plat = {
    .uart_line[0]       =   UART_0_PORT,
    .uart_line[1]       =   UART_1_PORT
};

static struct platform_device aml_uart_device = {
    .name         = "am_uart",  
    .id       = -1, 
    .num_resources    = 0,  
    .resource     = NULL,   
    .dev = {        
                .platform_data = &aml_uart_plat,
           },
};
#endif

#ifdef CONFIG_AM_NAND
static struct mtd_partition multi_partition_info[] = 
{
	{
		.name = "logo",
		.offset = 32*SZ_1M,
		.size = 16*SZ_1M,
	},
	{
		.name = "aml_logo",
		.offset = 48*SZ_1M,
		.size = 16*SZ_1M,
	},
	{
		.name = "recovery",
		.offset = 64*SZ_1M,
		.size = 32*SZ_1M,
	},
	{
		.name = "boot",
		.offset = 96*SZ_1M,
		.size = 32*SZ_1M,
	},
	{
		.name = "system",
		.offset = 128*SZ_1M,
		.size = 256*SZ_1M,
	},
	{
		.name = "cache",
		.offset = 384*SZ_1M,
		.size = 128*SZ_1M,
	},
	{
		.name = "userdata",
		.offset = 512*SZ_1M,
		.size = 512*SZ_1M,
	},
	{
		.name = "NFTL_Part",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
};


static struct aml_nand_platform aml_nand_mid_platform[] = {
{
		.name = NAND_BOOT_NAME,
		.chip_enable_pad = AML_NAND_CE0,
		.ready_busy_pad = AML_NAND_CE0,
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 1,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH16_MODE),
			},
    	},
		.T_REA = 20,
		.T_RHOH = 15,
	},
{
		.name = NAND_MULTI_NAME,
		.chip_enable_pad = (AML_NAND_CE0 | (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)),
		.ready_busy_pad = (AML_NAND_CE0 | (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)),
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 4,
				.nr_partitions = ARRAY_SIZE(multi_partition_info),
				.partitions = multi_partition_info,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH16_MODE | NAND_TWO_PLANE_MODE),
			},
    	},
		.T_REA = 20,
		.T_RHOH = 15,
	}
};

struct aml_nand_device aml_nand_mid_device = {
	.aml_nand_platform = aml_nand_mid_platform,
	.dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_nand_device = {
	.name = "aml_m1_nand",
	.id = 0,
	.num_resources = ARRAY_SIZE(aml_nand_resources),
	.resource = aml_nand_resources,
	.dev = {
		.platform_data = &aml_nand_mid_device,
	},
};
#endif

#if defined(CONFIG_AMLOGIC_BACKLIGHT)
#include <linux/aml_bl.h>

#define PWM_TCNT        (600-1)
#define PWM_MAX_VAL    (420)

static void aml_8726m_bl_init(void)
{
    SET_CBUS_REG_MASK(PWM_MISC_REG_AB, (1 << 0));
    msleep(20);
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<31));
    msleep(20);
    printk("\n\nBacklight init.\n\n");
}

static unsigned bl_level;

static unsigned aml_8726m_get_bl_level(void)
{
    return bl_level;
}

#define BL_MAX_NUMBER 255
#define BL_MAX_LEVEL 960
static int pre_level = 0;
static int first_time = 0;

static void aml_8726m_set_bl_level(unsigned level)
{
	
    unsigned cs_level, hi, low;

    if ((first_time == 0)||(first_time == 1)){
        first_time++;
        return;
        }
    if (level != pre_level){
        pre_level = level;

        int step = BL_MAX_LEVEL/BL_MAX_NUMBER;
        cs_level = level*step;
        if(level<30)
            cs_level = 90;
        else if(level <=218 && level >=30)
            cs_level = (level-29)*step + 90;
        else
            cs_level = BL_MAX_LEVEL;
        printk("cs_level = %d level = %d\n",cs_level, level);
        hi = cs_level;
        low = BL_MAX_LEVEL - hi;
        WRITE_CBUS_REG_BITS(PWM_PWM_A,low,0,16);  //low
        WRITE_CBUS_REG_BITS(PWM_PWM_A,hi,16,16);  //hi
        }
}

static void aml_8726m_power_on_bl(void)
{ 
    printk("backlight on\n");
}

static void aml_8726m_power_off_bl(void)
{
    printk("backlight off\n");
}

struct aml_bl_platform_data aml_bl_platform =
{
    .bl_init = aml_8726m_bl_init,
    .power_on_bl = aml_8726m_power_on_bl,
    .power_off_bl = aml_8726m_power_off_bl,
    .get_bl_level = aml_8726m_get_bl_level,
    .set_bl_level = aml_8726m_set_bl_level,
};

static struct platform_device aml_bl_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &aml_bl_platform,
    },
};
#endif
#if  defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
static struct resource vout_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vout_device = {
    .name       = "mesonvout",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout_device_resources),
    .resource      = vout_device_resources,
};
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns = 2,
       .vendor = "AMLOGIC",
       .product = "Android MID",
       .release = 0x0100,
};
static struct platform_device usb_mass_storage_device = {
       .name = "usb_mass_storage",
       .id = -1,
       .dev = {
               .platform_data = &mass_storage_pdata,
               },
};
#endif
static char *usb_functions[] = { "usb_mass_storage" };
static char *usb_functions_adb[] = { 
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
"usb_mass_storage", 
#endif

#ifdef CONFIG_USB_ANDROID_ADB
"adb" 
#endif
};
static struct android_usb_product usb_products[] = {
       {
               .product_id     = 0x0c01,
               .num_functions  = ARRAY_SIZE(usb_functions),
               .functions      = usb_functions,
       },
       {
               .product_id     = 0x0c02,
               .num_functions  = ARRAY_SIZE(usb_functions_adb),
               .functions      = usb_functions_adb,
       },
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id      = 0x0bb4,
       .product_id     = 0x0c01,
       .version        = 0x0100,
       .product_name   = "Android MID",
       .manufacturer_name = "AMLOGIC",
       .num_products = ARRAY_SIZE(usb_products),
       .products = usb_products,
       .num_functions = ARRAY_SIZE(usb_functions_adb),
       .functions = usb_functions_adb,
};

static struct platform_device android_usb_device = {
       .name   = "android_usb",
       .id             = -1,
       .dev            = {
               .platform_data = &android_usb_pdata,
       },
};
#endif

#ifdef CONFIG_POST_PROCESS_MANAGER
static struct resource ppmgr_resources[] = {
    [0] = {
        .start =  PPMGR_ADDR_START,
        .end   = PPMGR_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device ppmgr_device = {
    .name       = "ppmgr",
    .id         = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif

#ifdef CONFIG_BT_DEVICE
#include <linux/bt-device.h>

static struct platform_device bt_device = {
	.name             = "bt-dev",
	.id               = -1,
};

static void bt_device_init(void)
{
    printk("bt_device_init\n");
    /* UART_RTS_N(BT) */
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<26));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<17));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<17));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<8));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<24));

    /* UART_CTS_N(BT) */    
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<25));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<16));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<17));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<7));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<25));

    /* WIFI/BT_REGON OD5 */
#ifdef CONFIG_SN7325
    configIO(0, 0);
    setIO_level(0, 1, 5);
#endif
	
    /* BT_RST_N PP5 */
#ifdef CONFIG_SN7325
    configIO(1, 0);
    setIO_level(1, 0, 5);
#endif
	
    /* UART_RTS_N(BT) */
    SET_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<15));
    
    /* UART_CTS_N(BT) */
    CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<16));
    CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<16));
		
    /* BT_WAKE OD6 */
#ifdef CONFIG_SN7325
    //configIO(0, 0);
    //setIO_level(0, 1, 6);
#endif 
}

static void bt_device_on(void)
{
    printk("bt_device_on\n");
    /* BT_RST_N PP5 */
    msleep(200);
#ifdef CONFIG_SN7325
    configIO(1, 0);
    setIO_level(1, 1, 5);
#endif
}

static void bt_device_off(void)
{
    printk("bt_device_off\n");
    /* BT_RST_N PP5 */
#ifdef CONFIG_SN7325
    configIO(1, 0);
    setIO_level(1, 0, 5);
#endif
    msleep(200);
}

struct bt_dev_data bt_dev = {
    .bt_dev_init    = bt_device_init,
    .bt_dev_on      = bt_device_on,
    .bt_dev_off     = bt_device_off,
};
#endif

#if defined(CONFIG_AMLOGIC_SPI_NOR)
static struct mtd_partition spi_partition_info[] = {
/* Hide uboot partition
        {
                .name = "uboot",
                .offset = 0,
                .size = 0x3e000,
        },
*/
	{
		.name = "ubootenv",
		.offset = 0x3e000,
		.size = 0x2000,
	},
};

static struct flash_platform_data amlogic_spi_platform = {
	.parts = spi_partition_info,
	.nr_parts = ARRAY_SIZE(spi_partition_info),
};

static struct resource amlogic_spi_nor_resources[] = {
	{
		.start = 0xc1800000,
		.end = 0xc1ffffff,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device amlogic_spi_nor_device = {
	.name = "AMLOGIC_SPI_NOR",
	.id = -1,
	.num_resources = ARRAY_SIZE(amlogic_spi_nor_resources),
	.resource = amlogic_spi_nor_resources,
	.dev = {
		.platform_data = &amlogic_spi_platform,
	},
};
#endif


static struct platform_device __initdata *platform_devs[] = {
    #if defined(CONFIG_JPEGLOGO)
        &jpeglogo_device,
    #endif
    #if defined (CONFIG_AMLOGIC_PM)
        &power_dev,
    #endif  
    #if defined(CONFIG_FB_AM)
        &fb_device,
    #endif
    #if defined(CONFIG_AM_STREAMING)
        &codec_device,
    #endif
    #if defined(CONFIG_AM_VIDEO)
        &deinterlace_device,
    #endif
    #if defined(CONFIG_TVIN_VDIN)
        &vdin_device,
    #endif
    #if defined(CONFIG_AML_AUDIO_DSP)
        &audiodsp_device,
    #endif
        &aml_audio,
    #if defined(CONFIG_CARDREADER)
        &amlogic_card_device,
    #endif
    #if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_VIRTUAL_REMOTE)||defined(CONFIG_KEYPADS_AM_MODULE)
        &input_device,
    #endif
    #ifdef CONFIG_SARADC_AM
    &saradc_device,
    #endif
    #ifdef CONFIG_ADC_TOUCHSCREEN_AM
        &adc_ts_device,
    #endif
    #if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
        &adc_kp_device,
    #endif
    #if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
        &input_device_key,  //changed by Elvis
    #endif
    #if defined(CONFIG_TOUCHSCREEN_ADS7846)
        &spi_gpio,
    #endif
    #ifdef CONFIG_AM_NAND
        &aml_nand_device,
    #endif
    #if defined(CONFIG_NAND_FLASH_DRIVER_MULTIPLANE_CE)
        &aml_nand_device,
    #endif
    #if defined(CONFIG_AML_RTC)
        &aml_rtc_device,
    #endif
    #if defined(CONFIG_SUSPEND)
        &aml_pm_device,
    #endif
    #if defined(CONFIG_ANDROID_PMEM)
        &android_pmem_device,
    #endif
    #if defined(CONFIG_I2C_SW_AML)
        &aml_sw_i2c_device,
    #endif
    #if defined(CONFIG_I2C_AML)
        &aml_i2c_device,
    #endif
    #if defined(CONFIG_AM_UART_WITH_S_CORE)
        &aml_uart_device,
    #endif
    #if defined(CONFIG_AMLOGIC_BACKLIGHT)
        &aml_bl_device,
    #endif
    #ifdef CONFIG_TWX_TC101
        &twx_device,
    #endif
    #if defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
        &vout_device,   
    #endif
    #ifdef CONFIG_USB_ANDROID
        &android_usb_device,
        #ifdef CONFIG_USB_ANDROID_MASS_STORAGE
            &usb_mass_storage_device,
        #endif
    #endif
    #ifdef CONFIG_AMLOGIC_VIDEOIN_MANAGER
	&vm_device,
    #endif    
    #if defined(CONFIG_TVIN_BT656IN)
        &bt656in_device,
    #endif
    #ifdef CONFIG_BT_DEVICE
        &bt_device,
    #endif
    #ifdef CONFIG_POST_PROCESS_MANAGER
        &ppmgr_device,
    #endif
    #if defined(CONFIG_AMLOGIC_SPI_NOR)
    	&amlogic_spi_nor_device,
    #endif
};
static struct i2c_board_info __initdata aml_i2c_bus_info[] = {

#ifdef CONFIG_TWX_TC101
    {
        I2C_BOARD_INFO(TWX_TC101_I2C_NAME,  TWX_TC101_I2C_ADDR),
    },
#endif

#ifdef CONFIG_SENSORS_MMC31XX
    {
        I2C_BOARD_INFO(MMC31XX_I2C_NAME,  MMC31XX_I2C_ADDR),
    },
#endif

#ifdef CONFIG_SENSORS_MXC622X
    {
        I2C_BOARD_INFO(MXC622X_I2C_NAME,  MXC622X_I2C_ADDR),
    },
#endif

#ifdef CONFIG_SND_AML_M1_MID_CS42L52
    {
        I2C_BOARD_INFO("cs42l52", 0x4A),
	 .platform_data = (void *)&cs42l52_pdata,
    },
#endif

#ifdef CONFIG_SN7325
    {
        I2C_BOARD_INFO("sn7325", 0x59),		//IMPORTANT!!! EIO_A0 1   EIO_A1 0
        .platform_data = (void *)&sn7325_pdata,
    },
#endif

#ifdef CONFIG_EETI_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("eeti", 0x04),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&eeti_pdata,
    },
#endif
#ifdef CONFIG_HX8520_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("hx8520", 0x4b),
        .platform_data = (void *)&ts_pdata,
    },
#endif

#ifdef CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO(GOODIX_I2C_NAME, GOODIX_I2C_ADDR),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&ts_pdata,
    },
#endif

#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("pixcir168", 0x5c),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&pixcir_pdata,
    },
#endif

#ifdef CONFIG_TOUCH_KEY_PAD_HA2605
	{
		I2C_BOARD_INFO("ha2605", 0x62),
		.irq = INT_GPIO_1,
		.platform_data = (void *)&ha2605_pdata,
	},
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308
        {
        /*gc0308 i2c address is 0x42/0x43*/
                I2C_BOARD_INFO("gc0308_i2c",  0x42 >> 1),
                .platform_data = (void *)&video_gc0308_data,
        },
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
    {
    	/*gt2005 i2c address is 0x78/0x79*/
    	I2C_BOARD_INFO("gt2005_i2c",  0x78 >> 1 ),
    	.platform_data = (void *)&video_gt2005_data
    },
#endif

#ifdef CONFIG_HAPTIC_ISA1200
    {
        I2C_BOARD_INFO("isa1200", 0x48),
        .platform_data = (void *)&isa1200_pdata,
    },
#endif
};


static int __init aml_i2c_init(void)
{

    i2c_register_board_info(0, aml_i2c_bus_info,
        ARRAY_SIZE(aml_i2c_bus_info));
    return 0;
}

#if defined(CONFIG_TVIN_BT656IN)
static void __init bt656in_pinmux_init(void)
{
    set_mio_mux(3, 0xf000);   //mask--mux gpio_c3 to bt656 clk;  mux gpioc[4:11] to be bt656 dt_in
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, 0x0f000000);
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, 0x01be07fc);
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, 0x0c000000);
}
#endif

static void __init device_pinmux_init(void )
{
    clearall_pinmux();
    /* TCON control pins pinmux */
    /* GPIOA_5 -> LCD_Clk, GPIOA_0 -> TCON_STH1, GPIOA_1 -> TCON_STV1, GPIOA_2 -> TCON_OEH, */
    set_mio_mux(0, ((1<<11)|(1<<14)|(1<<15)|(1<<16)));    
    //set_mio_mux(4,(3<<0)|(3<<2)|(3<<4));   //For 8bits
    set_mio_mux(4,(1<<0)|(1<<2)|(1<<4));   //For 6bits
    
    //power off spi GPIOE_5:0
    set_gpio_val(GPIOE_bank_bit16_21(5), GPIOE_bit_bit16_21(5), 0); //low
    set_gpio_mode(GPIOE_bank_bit16_21(5), GPIOE_bit_bit16_21(5), GPIO_OUTPUT_MODE);
    
    /*other deivce power on*/
    /*GPIOA_200e_bit4..usb/eth/YUV power on*/
    //set_gpio_val(PREG_EGPIO,1<<4,1);
    //set_gpio_mode(PREG_EGPIO,1<<4,GPIO_OUTPUT_MODE);
    uart_set_pinmux(UART_PORT_A,PINMUX_UART_A);
    uart_set_pinmux(UART_PORT_B,PINMUX_UART_B);
    aml_i2c_init();
    #if defined(CONFIG_TVIN_BT656IN)
    bt656in_pinmux_init();
    #endif
    set_audio_pinmux(AUDIO_OUT_TEST_N);
    set_audio_pinmux(AUDIO_IN_JTAG);	
}

static void __init  device_clk_setting(void)
{
    /*Demod CLK for eth and sata*/
    demod_apll_setting(0,1200*CLK_1M);
    /*eth clk*/
    eth_clk_set(ETH_CLKSRC_APLL_CLK,400*CLK_1M,50*CLK_1M);
}

static void disable_unused_model(void)
{
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
    CLK_GATE_OFF(ETHERNET);
    CLK_GATE_OFF(SATA);
    CLK_GATE_OFF(WIFI);
    video_dac_disable();
    //audio_internal_dac_disable();
     //disable wifi
    SET_CBUS_REG_MASK(HHI_GCLK_MPEG2, (1<<5)); 
    SET_CBUS_REG_MASK(HHI_WIFI_CLK_CNTL, (1<<0));
    __raw_writel(0xCFF,0xC9320ED8);
    __raw_writel((__raw_readl(0xC9320EF0))&0xF9FFFFFF,0xC9320EF0);
    CLEAR_CBUS_REG_MASK(HHI_GCLK_MPEG2, (1<<5)); 
    CLEAR_CBUS_REG_MASK(HHI_WIFI_CLK_CNTL, (1<<0));
    ///disable demod
    SET_CBUS_REG_MASK(HHI_DEMOD_CLK_CNTL, (1<<8));//enable demod core digital clock
    SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL, (1<<15));//enable demod adc clock
    CLEAR_APB_REG_MASK(0x4004,(1<<31));  //disable analog demod adc
    CLEAR_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL, (1<<15));//disable demod adc clock  
    CLEAR_CBUS_REG_MASK(HHI_DEMOD_CLK_CNTL, (1<<8));//disable demod core digital clock
}

static void __init power_hold(void)
{
    printk(KERN_INFO "power hold set high!\n");
    set_gpio_val(GPIOA_bank_bit(8), GPIOA_bit_bit0_14(8), 1);
    set_gpio_mode(GPIOA_bank_bit(8), GPIOA_bit_bit0_14(8), GPIO_OUTPUT_MODE);

    // BL_PWM on
    SET_CBUS_REG_MASK(PWM_MISC_REG_AB, (1 << 0));
    mdelay(20);
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<31));
    mdelay(20);
    //BL_PWM -> GPIOA_7: 1
	set_gpio_val(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), 1);
	set_gpio_mode(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), GPIO_OUTPUT_MODE);
    
    //VCCx2 power up
    //set_vccx2(1);
    set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 1);
    set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);  

}

static void init_VCCK_SAVE(void)
{
    // Enable VBG_EN
    WRITE_CBUS_REG_BITS(PREG_AM_ANALOG_ADDR, 1, 0, 1);
    // wire pm_gpioA_7_led_pwm = pin_mux_reg0[22];
    WRITE_CBUS_REG(LED_PWM_REG0,(0 << 31)   |       /* disable the overall circuit */
                                (0 << 30)   |       /* 1:Closed Loop  0:Open Loop */
                                (0 << 16)   |       /* PWM total count */
                                (0 << 13)   |       /* Enable */
                                (1 << 12)   |       /* enable */
                                (0 << 10)   |       /* test */
                                (7 << 7)    |       /* CS0 REF, Voltage FeedBack: about 0.505V */
                                (7 << 4)    |       /* CS1 REF, Current FeedBack: about 0.505V */
                                (READ_CBUS_REG(LED_PWM_REG0)&0x0f));           /* DIMCTL Analog dimmer */
}

static __init void m1_init_machine(void)
{
    meson_cache_init();

    power_hold();
    pm_power_off = set_bat_off;
    device_clk_setting();
    
    device_pinmux_init();
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
    //camera_power_on_init();
#endif
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

#ifdef CONFIG_USB_DWC_OTG_HCD
    set_usb_phy_clk(USB_PHY_CLOCK_SEL_XTAL_DIV2);
    lm_device_register(&usb_ld_a);
#endif
#ifdef CONFIG_SATA_DWC_AHCI
    set_sata_phy_clk(SATA_PHY_CLOCK_SEL_DEMOD_PLL);
    lm_device_register(&sata_ld);
#endif
#if defined(CONFIG_TOUCHSCREEN_ADS7846)
    ads7846_init_gpio();
    spi_register_board_info(spi_board_info_list, ARRAY_SIZE(spi_board_info_list));
#endif
    disable_unused_model();
	init_VCCK_SAVE();
}

/*VIDEO MEMORY MAPING*/
static __initdata struct map_desc meson_video_mem_desc[] = {
    {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END-RESERVED_MEM_START+1,
        .type       = MT_DEVICE,
    },
};

static __init void m1_map_io(void)
{
    meson_map_io();
    iotable_init(meson_video_mem_desc, ARRAY_SIZE(meson_video_mem_desc));
}

static __init void m1_irq_init(void)
{
    meson_init_irq();
}

static __init void m1_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    m->nr_banks = 0;
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
    m->nr_banks++;
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END+1);
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
    pbank->node  = PHYS_TO_NID(RESERVED_MEM_END+1);
    m->nr_banks++;
}

MACHINE_START(MESON_8726M, "AMLOGIC MESON-M1 8726M SZ")
    .phys_io        = MESON_PERIPHS1_PHYS_BASE,
    .io_pg_offst    = (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = m1_map_io,
    .init_irq       = m1_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = m1_init_machine,
    .fixup          = m1_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END

static  int __init board_ver_setup(char *s)
{
    if(strncmp(s, "v2", 2)==0)
        board_ver = 2;
    else if(strncmp(s, "V2", 2)==0)
        board_ver = 2;
    else if(strncmp(s, "v1", 2)==0)
        board_ver = 1;
    else if(strncmp(s, "V1", 2)==0)
        board_ver = 1;
    printk("board_ver = %s",s);      
    return 0;
}
__setup("board_ver=",board_ver_setup) ;

