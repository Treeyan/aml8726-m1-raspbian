/*****************************************************************
**                                                              **
**  Copyright (C) 2004 Amlogic,Inc.                             **
**  All rights reserved                                         **
**        Filename : sd.c /Project:AVOS  driver                 **
**        Revision : 1.0                                        **
**                                                              **
*****************************************************************/

#include <linux/err.h>
#include <linux/device.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/am_regs.h>
#include <mach/irqs.h>
//#include <mach/card_io.h>
#include <mach/old/card_io.h>
#include <mach/power_gate.h>
//#include <linux/cardreader/card_block.h>
//#include <linux/cardreader/cardreader.h>
//#include <linux/cardreader/sdio.h>
//#include <linux/aml_uevent_msg.h>
#include <mach/old/card_block.h>
#include <mach/old/cardreader.h>
#include <mach/old/sdio.h>
#include <mach/old/aml_uevent_msg.h>

#include "sd_misc.h"
#include "sd_protocol.h"

static unsigned sd_backup_input_val = 0;
static unsigned sd_backup_output_val = 0;
static unsigned SD_BAKUP_INPUT_REG = (unsigned)&sd_backup_input_val;
static unsigned SD_BAKUP_OUTPUT_REG = (unsigned)&sd_backup_output_val;

unsigned SD_CMD_OUTPUT_EN_REG;
unsigned SD_CMD_OUTPUT_EN_MASK;
unsigned SD_CMD_INPUT_REG;
unsigned SD_CMD_INPUT_MASK;
unsigned SD_CMD_OUTPUT_REG;
unsigned SD_CMD_OUTPUT_MASK;

unsigned SD_CLK_OUTPUT_EN_REG;
unsigned SD_CLK_OUTPUT_EN_MASK;
unsigned SD_CLK_OUTPUT_REG;
unsigned SD_CLK_OUTPUT_MASK;

unsigned SD_DAT_OUTPUT_EN_REG;
unsigned SD_DAT0_OUTPUT_EN_MASK;
unsigned SD_DAT0_3_OUTPUT_EN_MASK;
unsigned SD_DAT_INPUT_REG;
unsigned SD_DAT_OUTPUT_REG;
unsigned SD_DAT0_INPUT_MASK;
unsigned SD_DAT0_OUTPUT_MASK;
unsigned SD_DAT0_3_INPUT_MASK;
unsigned SD_DAT0_3_OUTPUT_MASK;
unsigned SD_DAT_INPUT_OFFSET;
unsigned SD_DAT_OUTPUT_OFFSET;

unsigned SD_INS_OUTPUT_EN_REG;
unsigned SD_INS_OUTPUT_EN_MASK;
unsigned SD_INS_INPUT_REG;
unsigned SD_INS_INPUT_MASK;

unsigned SD_WP_OUTPUT_EN_REG;
unsigned SD_WP_OUTPUT_EN_MASK;
unsigned SD_WP_INPUT_REG;
unsigned SD_WP_INPUT_MASK;

unsigned SD_PWR_OUTPUT_EN_REG;
unsigned SD_PWR_OUTPUT_EN_MASK;
unsigned SD_PWR_OUTPUT_REG;
unsigned SD_PWR_OUTPUT_MASK;
unsigned SD_PWR_EN_LEVEL;

unsigned SD_WORK_MODE;

struct memory_card *card_find_card(struct card_host *host, u8 card_type); 

void sd_insert_detector(struct memory_card *card)
{
	SD_MMC_Card_Info_t *sd_mmc_info = (SD_MMC_Card_Info_t *)card->card_info;

	int ret = sd_mmc_check_insert(sd_mmc_info);
	if(ret)
        card->card_status = CARD_INSERTED;
    else
        card->card_status = CARD_REMOVED;

	return;
}

void sd_open(struct memory_card *card)
{
	int ret;
	struct aml_card_info *aml_card_info = card->card_plat_info;
	SD_MMC_Card_Info_t *sd_mmc_info = (SD_MMC_Card_Info_t *)card->card_info;

	if (aml_card_info->card_extern_init)
		aml_card_info->card_extern_init();
	ret = sd_mmc_init(sd_mmc_info);

	if(ret)
		ret = sd_mmc_init(sd_mmc_info);

	if(ret)
		ret = sd_mmc_init(sd_mmc_info);

	card->capacity = sd_mmc_info->blk_nums;
	card->sdio_funcs  = sd_mmc_info->sdio_function_nums;
	memcpy(card->raw_cid, &(sd_mmc_info->raw_cid), sizeof(card->raw_cid));

	if(ret)
	{
		card->unit_state = CARD_UNIT_READY;
		aml_send_msg("IDENTIFY", 1);
	}
	else
	{
		card->unit_state = CARD_UNIT_PROCESSED;
		aml_send_msg("IDENTIFY", 0);
	}

	return;
}

void sd_close(struct memory_card *card)
{
	SD_MMC_Card_Info_t *sd_mmc_info = (SD_MMC_Card_Info_t *)card->card_info;

	if(!sd_mmc_info)
	{
		printk("error: no card to exit\n");
		return;
	}
	sd_mmc_exit(sd_mmc_info);
	sd_mmc_free(sd_mmc_info);
	sd_mmc_info = NULL;
	card->card_info = NULL;
	card->unit_state =  CARD_UNIT_PROCESSED;

	return;
}

void sd_suspend(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;
	
	SD_MMC_Card_Info_t *sd_mmc_info = (SD_MMC_Card_Info_t *)card->card_info;

	printk("***Entered %s:%s\n", __FILE__,__func__);	
	
	CLK_GATE_OFF(SDIO);  
	 
	if(card->card_type == CARD_SDIO)
	{
		return;
	}
		        
	memset(sd_mmc_info, 0, sizeof(SD_MMC_Card_Info_t));
	if (card->host->dma_buf != NULL) {
		sd_mmc_info->sd_mmc_buf = card->host->dma_buf;
		sd_mmc_info->sd_mmc_phy_buf = card->host->dma_phy_buf;
	}	
	card->card_io_init(card);
	sd_mmc_info->io_pad_type = aml_card_info->io_pad_type;
	sd_mmc_info->bus_width = SD_BUS_SINGLE;
	sd_mmc_info->sdio_clk_unit = 3000;
	sd_mmc_info->clks_nac = SD_MMC_TIME_NAC_DEFAULT;
	sd_mmc_info->max_blk_count = card->host->max_blk_count;
	
}

void sd_resume(struct memory_card *card)
{
	printk("***Entered %s:%s\n", __FILE__,__func__);
	CLK_GATE_ON(SDIO);
}

void sd_io_init(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;
	SD_WORK_MODE = aml_card_info->work_mode;

	switch (aml_card_info->io_pad_type) {

		case SDIO_GPIOA_0_5:
			SD_CMD_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_9_MASK;
			SD_CMD_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_9_MASK;
			SD_CMD_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_9_MASK;

			SD_CLK_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_8_MASK;
			SD_CLK_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_8_MASK;

			SD_DAT_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_4_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_4_7_MASK;
			SD_DAT_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_DAT_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_4_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_4_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_4_7_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_4_7_MASK;
			SD_DAT_INPUT_OFFSET = 4;
			SD_DAT_OUTPUT_OFFSET = 4;
			break;

		case SDIO_GPIOA_9_14:
			SD_CMD_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_18_MASK;
			SD_CMD_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_18_MASK;
			SD_CMD_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_18_MASK;

			SD_CLK_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_17_MASK;
			SD_CLK_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_17_MASK;

			SD_DAT_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_13_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_13_16_MASK;
			SD_DAT_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_DAT_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_13_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_13_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_13_16_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_13_16_MASK;
			SD_DAT_INPUT_OFFSET = 13;
			SD_DAT_OUTPUT_OFFSET = 13;
			break;

		case SDIO_GPIOB_2_7:
			SD_CMD_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_21_MASK;
			SD_CMD_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_21_MASK;
			SD_CMD_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_21_MASK;

			SD_CLK_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_22_MASK;
			SD_CLK_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_22_MASK;

			SD_DAT_OUTPUT_EN_REG = EGPIO_GPIOA_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_REG = EGPIO_GPIOA_INPUT;
			SD_DAT_OUTPUT_REG = EGPIO_GPIOA_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_OFFSET = 23;
			SD_DAT_OUTPUT_OFFSET = 23;
			break;

		case SDIO_GPIOE_6_11:
			SD_CMD_OUTPUT_EN_REG = EGPIO_GPIOE_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_7_MASK;
			SD_CMD_INPUT_REG = EGPIO_GPIOE_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_7_MASK;
			SD_CMD_OUTPUT_REG = EGPIO_GPIOE_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_7_MASK;

			SD_CLK_OUTPUT_EN_REG = EGPIO_GPIOE_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_6_MASK;
			SD_CLK_OUTPUT_REG = EGPIO_GPIOE_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_6_MASK;

			SD_DAT_OUTPUT_EN_REG = EGPIO_GPIOE_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_8_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_8_11_MASK;
			SD_DAT_INPUT_REG = EGPIO_GPIOE_INPUT;
			SD_DAT_OUTPUT_REG = EGPIO_GPIOE_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_8_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_8_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_8_11_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_8_11_MASK;
			SD_DAT_INPUT_OFFSET = 8;
			SD_DAT_OUTPUT_OFFSET = 8;
			break;

        default:
			printk("Warning couldn`t find any valid hw io pad!!!\n");
            break;
	}

	if (aml_card_info->card_ins_en_reg) {
		SD_INS_OUTPUT_EN_REG = aml_card_info->card_ins_en_reg;
		SD_INS_OUTPUT_EN_MASK = aml_card_info->card_ins_en_mask;
		SD_INS_INPUT_REG = aml_card_info->card_ins_input_reg;
		SD_INS_INPUT_MASK = aml_card_info->card_ins_input_mask;
	}
	else {
		SD_INS_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_INS_OUTPUT_EN_MASK = 1;
		SD_INS_INPUT_REG = SD_BAKUP_INPUT_REG;
		SD_INS_INPUT_MASK =
		SD_WP_INPUT_MASK = 1;
	}

	if (aml_card_info->card_power_en_reg) {
		SD_PWR_OUTPUT_EN_REG = aml_card_info->card_power_en_reg;
		SD_PWR_OUTPUT_EN_MASK = aml_card_info->card_power_en_mask;
		SD_PWR_OUTPUT_REG = aml_card_info->card_power_output_reg;
		SD_PWR_OUTPUT_MASK = aml_card_info->card_power_output_mask;
		SD_PWR_EN_LEVEL = aml_card_info->card_power_en_lev;
	}
	else {
		SD_PWR_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_PWR_OUTPUT_EN_MASK = 1;
		SD_PWR_OUTPUT_REG = SD_BAKUP_OUTPUT_REG;
		SD_PWR_OUTPUT_MASK = 1;
		SD_PWR_EN_LEVEL = 0;	
	}

	if (aml_card_info->card_wp_en_reg) {
		SD_WP_OUTPUT_EN_REG = aml_card_info->card_wp_en_reg;
		SD_WP_OUTPUT_EN_MASK = aml_card_info->card_wp_en_mask;
		SD_WP_INPUT_REG = aml_card_info->card_wp_input_reg;
		SD_WP_INPUT_MASK = aml_card_info->card_wp_input_mask;
	}
	else {
		SD_WP_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_WP_OUTPUT_EN_MASK = 1;
		SD_WP_INPUT_REG = SD_BAKUP_INPUT_REG;
		SD_WP_INPUT_MASK = 1;
	}

	return;
}

static int sd_request(struct memory_card *card, struct card_blk_request *brq)
{
	SD_MMC_Card_Info_t *sd_mmc_info = (SD_MMC_Card_Info_t *)card->card_info;
	unsigned int lba, byte_cnt;
	unsigned char *data_buf;
	struct card_host *host = card->host;
	struct memory_card *sdio_card;
	
	lba = brq->card_data.lba;
	byte_cnt = brq->card_data.blk_size * brq->card_data.blk_nums;
	data_buf = brq->crq.buf;

	if(sd_mmc_info == NULL){
		brq->card_data.error = SD_MMC_ERROR_NO_CARD_INS;
		printk("[sd_request] sd_mmc_info == NULL, return SD_MMC_ERROR_NO_CARD_INS\n");
		return 0;
	}

	sdio_close_host_interrupt(SDIO_IF_INT);
	sd_sdio_enable(sd_mmc_info->io_pad_type);
	if(brq->crq.cmd == READ) {
		brq->card_data.error = sd_mmc_read_data(sd_mmc_info, lba, byte_cnt, data_buf);
	}
	else if(brq->crq.cmd == WRITE) {
		brq->card_data.error = sd_mmc_write_data(sd_mmc_info, lba, byte_cnt, data_buf);
	}

	sd_gpio_enable(sd_mmc_info->io_pad_type);

	sdio_card = card_find_card(host, CARD_SDIO);
	if(sdio_card)
	{
		sd_mmc_info = (SD_MMC_Card_Info_t *)sdio_card->card_info;
		sd_sdio_enable(sd_mmc_info->io_pad_type);
		sdio_open_host_interrupt(SDIO_IF_INT);
		if (sd_mmc_info->sd_save_hw_io_flag) {
	    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
	      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
    		}
	}
	return 0;
}

static int sdio_request(struct memory_card *card, struct card_blk_request *brq)
{
	SD_MMC_Card_Info_t *sdio_info = (SD_MMC_Card_Info_t *)card->card_info;
	int incr_addr, err;
	unsigned addr, blocks, blksz, fn, read_after_write;
	u8 *in, *out, *buf;

	sd_sdio_enable(sdio_info->io_pad_type);
	if (brq->crq.cmd & SDIO_OPS_REG) {

		WARN_ON(brq->card_data.blk_size != 1);
		WARN_ON(brq->card_data.blk_nums != 1);
	
		in = brq->crq.buf;
		addr = brq->card_data.lba;
		fn = brq->card_data.flags;
		out = brq->crq.back_buf;

		if (brq->crq.cmd & READ_AFTER_WRITE)
			read_after_write = 1;
		else
			read_after_write = 0;

		if ((brq->crq.cmd & 0x1 )== WRITE) {
			err = sdio_write_reg(sdio_info, fn, addr, in, read_after_write);
			if (err) {
				printk("sdio card write_reg failed %d at addr: %x \n", err, addr);
				brq->card_data.error = err;
				goto err;
			}
		}
		else {
			err = sdio_read_reg(sdio_info, fn, addr, out);
			if (err) {
				printk("sdio card read_reg failed %d at addr: %x  \n", err, addr);
				brq->card_data.error = err;
				goto err;
			}
		}
	}
	else {

		if (brq->crq.cmd & SDIO_FIFO_ADDR)
			incr_addr = 1;
		else
			incr_addr = 0;

		buf = brq->crq.buf;
		addr = brq->card_data.lba;
		blksz = brq->card_data.blk_size;
		blocks = brq->card_data.blk_nums;
		fn = brq->card_data.flags;
		sdio_info->sdio_blk_len[fn] = card->sdio_func[fn-1]->cur_blksize;

		if ((brq->crq.cmd & 0x1)== WRITE) {
			err = sdio_write_data(sdio_info, fn, incr_addr, addr, blocks*blksz, buf);
			if (err) {
				printk("sdio card write_data failed %d at addr: %x, function: %d \n", err, addr, fn);
				brq->card_data.error = err;
				goto err;
			}
		}
		else {
			err = sdio_read_data(sdio_info, fn, incr_addr, addr, blocks*blksz, buf);
			if (err) {
				printk("sdio card read_data failed %d at addr: %x, function: %d\n", err, addr, fn);
				brq->card_data.error = err;
				goto err;
			}
		}
	}

	//sd_gpio_enable(sdio_info->io_pad_type);
	brq->card_data.error = 0;
	return 0;

err:
	//sd_gpio_enable(sdio_info->io_pad_type);
	return err;
}

int sd_mmc_probe(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;

	SD_MMC_Card_Info_t *sd_mmc_info = sd_mmc_malloc(sizeof(SD_MMC_Card_Info_t), GFP_KERNEL);
	if (sd_mmc_info == NULL)
		return -ENOMEM;

	if (card->host->dma_buf != NULL) {
		sd_mmc_info->sd_mmc_buf = card->host->dma_buf;
		sd_mmc_info->sd_mmc_phy_buf = card->host->dma_phy_buf;
	}

	card->card_info = sd_mmc_info;
	card->card_io_init = sd_io_init;
	card->card_detector = sd_insert_detector;
	card->card_insert_process = sd_open;
	card->card_remove_process = sd_close;
	card->card_request_process = sd_request;
	card->card_suspend = sd_suspend;
	card->card_resume = sd_resume;
	if (aml_card_info->card_extern_init)
		aml_card_info->card_extern_init();
	card->card_io_init(card);
	sd_mmc_prepare_init(sd_mmc_info);
	sd_mmc_info->io_pad_type = aml_card_info->io_pad_type;
	sd_mmc_info->bus_width = SD_BUS_SINGLE;
	sd_mmc_info->sdio_clk_unit = 3000;
	sd_mmc_info->clks_nac = SD_MMC_TIME_NAC_DEFAULT;
	sd_mmc_info->max_blk_count = card->host->max_blk_count;

	return 0;
}

int sdio_probe(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;

	SD_MMC_Card_Info_t *sdio_info = sd_mmc_malloc(sizeof(SD_MMC_Card_Info_t), GFP_KERNEL);
	if (sdio_info == NULL)
		return -ENOMEM;

	if (card->host->dma_buf != NULL) {
		sdio_info->sd_mmc_buf = card->host->dma_buf;
		sdio_info->sd_mmc_phy_buf = card->host->dma_phy_buf;
	}

	card->card_info = sdio_info;
	card->card_io_init = sd_io_init;
	card->card_detector = sd_insert_detector;
	card->card_insert_process = sd_open;
	card->card_remove_process = sd_close;
	card->card_request_process = sdio_request;
        card->card_suspend = sd_suspend;
        card->card_resume = sd_resume;
	if (aml_card_info->card_extern_init)
		aml_card_info->card_extern_init();
	card->card_io_init(card);
	sd_mmc_prepare_init(sdio_info);
	sdio_info->io_pad_type = aml_card_info->io_pad_type;
	sdio_info->bus_width = SD_BUS_SINGLE;
	sdio_info->sdio_clk_unit = 3000;
	sdio_info->clks_nac = SD_MMC_TIME_NAC_DEFAULT;
	sdio_info->max_blk_count = card->host->max_blk_count;

	return 0;
}

#ifdef CONFIG_INAND



int inand_probe(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;

	SD_MMC_Card_Info_t *sdio_info = sd_mmc_malloc(sizeof(SD_MMC_Card_Info_t), GFP_KERNEL);
	if (sdio_info == NULL)
		return -ENOMEM;

	if (card->host->dma_buf != NULL) {
		sdio_info->sd_mmc_buf = card->host->dma_buf;
		sdio_info->sd_mmc_phy_buf = card->host->dma_phy_buf;
	}

	card->card_info = sdio_info;
	card->card_io_init = sd_io_init;
	card->card_detector = sd_insert_detector;
	card->card_insert_process = sd_open;
	card->card_remove_process = sd_close;
	card->card_request_process = sd_request;

	if (aml_card_info->card_extern_init)
		aml_card_info->card_extern_init();
	card->card_io_init(card);
	sd_mmc_prepare_init(sdio_info);
	sdio_info->io_pad_type = aml_card_info->io_pad_type;
	sdio_info->bus_width = SD_BUS_SINGLE;
	sdio_info->sdio_clk_unit = 3000;
	sdio_info->clks_nac = SD_MMC_TIME_NAC_DEFAULT;
	sdio_info->max_blk_count = card->host->max_blk_count;

	return 0;
}

#endif

MODULE_DESCRIPTION("Amlogic sd card Interface driver");

MODULE_LICENSE("GPL");

