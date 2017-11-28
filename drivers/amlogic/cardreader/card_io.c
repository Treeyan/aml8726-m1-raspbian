#include <linux/slab.h>
//#include <linux/cardreader/cardreader.h>
//#include <linux/cardreader/card_block.h>
//#include <linux/cardreader/sdio_hw.h>
#include <mach/old/cardreader.h>
#include <mach/old/card_block.h>
#include <mach/old/sdio_hw.h>
#include <mach/am_regs.h>
#include <mach/irqs.h>
//#include <mach/card_io.h>
#include <mach/old/card_io.h>
    
int i_GPIO_timer;
int ATA_MASTER_DISABLED = 0;
int ATA_SLAVE_ENABLED = 0;
unsigned ATA_EIGHT_BIT_ENABLED = 1;
unsigned char card_share_ins_pwr_flag[MAX_CARD_UNIT];
unsigned sdio_timeout_int_times = 1;
unsigned sdio_timeout_int_num = 0;

struct completion sdio_int_complete;

/* */ void (*sd_mmc_power_register) (int power_on) = NULL;
/* */ int (*sd_mmc_ins_register) (void) = NULL;
/* */ int (*sd_mmc_wp_register) (void) = NULL;
/* */ void (*sd_mmc_io_release_register) (void) = NULL;
/**/ void (*cf_power_register) (int power_on) = NULL;
/**/ void (*cf_reset_register) (int reset_high) = NULL;
/**/ int (*cf_ins_register) (void) = NULL;
/**/ void (*cf_io_release_register) (void) = NULL;
/* */ void (*ms_mspro_power_register) (int power_on) = NULL;
/* */ int (*ms_mspro_ins_register) (void) = NULL;
/* */ int (*ms_mspro_wp_register) (void) = NULL;
/* */ void (*ms_mspro_io_release_register) (void) = NULL;

void sd_sdio_enable(SDIO_Pad_Type_t io_pad_type)
{
	switch (io_pad_type) {

		case SDIO_GPIOA_0_5:
			SET_CBUS_REG_MASK(CARD_PIN_MUX_1, 0x3F);
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDIO_GPIOA_9_14:
			//CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_2, ((0xF<<16) | (1<<8) | (1<<12)));
			SET_CBUS_REG_MASK(CARD_PIN_MUX_0, (0x3F<<23));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		case SDIO_GPIOB_2_7:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_0, (0x3F<<23));
			//CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_5, ((1<<5)));
			SET_CBUS_REG_MASK(CARD_PIN_MUX_2, ((0xF<<16) | (1<<8) | (1<<12)));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDIO_GPIOE_6_11:
			SET_CBUS_REG_MASK(CARD_PIN_MUX_7, ((0xF<<26) | (1<<24)));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		default :
			printk("invalid hw io pad!!!\n");
			break;
	}
	
	return;
}

void sd_gpio_enable(SDIO_Pad_Type_t io_pad_type)
{
	switch (io_pad_type) {

		case SDIO_GPIOA_0_5:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_1, 0x3F);
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDIO_GPIOA_9_14:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_0, (0x3F<<23));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			//SET_CBUS_REG_MASK(CARD_PIN_MUX_2, ((0xF<<16) | (1<<8) | (1<<12)));
			break;

		case SDIO_GPIOB_2_7:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_2, ((0xF<<16) | (1<<8) | (1<<12)));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDIO_GPIOE_6_11:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_7, ((0xF<<26) | (1<<24)));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		default :
			printk("invalid hw io pad!!!\n");
			break;
	}
	
	return;
}

void sdio_open_host_interrupt(unsigned int_resource) 
{	
	unsigned long irq_config, status_irq;	
	MSHW_IRQ_Config_Reg_t * irq_config_reg;	
	SDIO_Status_IRQ_Reg_t * status_irq_reg;	
	irq_config = READ_CBUS_REG(SDIO_IRQ_CONFIG);	
	irq_config_reg = (void *)&irq_config;
	status_irq = READ_CBUS_REG(SDIO_STATUS_IRQ);
	status_irq_reg = (void *)&status_irq;

	switch (int_resource) {
		case SDIO_IF_INT:	
			irq_config_reg->arc_if_int_en = 1;	
			status_irq_reg->if_int = 1;	
			break;

		case SDIO_CMD_INT:
			irq_config_reg->arc_cmd_int_en = 1;	
			status_irq_reg->cmd_int = 1;	
			break;

		case SDIO_SOFT_INT:	
			irq_config_reg->arc_soft_int_en = 1;	
			status_irq_reg->soft_int = 1;
			break;

		case SDIO_TIMEOUT_INT:	
			status_irq_reg->arc_timing_out_int_en = 1;	
			status_irq_reg->timing_out_int = 1;	
			break;

		default:	
			break;	
	}

	WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);	
	WRITE_CBUS_REG(SDIO_IRQ_CONFIG, irq_config);
}

void sdio_clear_host_interrupt(unsigned int_resource) 
{
	unsigned long status_irq;
	SDIO_Status_IRQ_Reg_t * status_irq_reg;
	status_irq = READ_CBUS_REG(SDIO_STATUS_IRQ);
	status_irq_reg = (void *)&status_irq;

	switch (int_resource) {
		case SDIO_IF_INT:	
			status_irq_reg->if_int = 1;	
			break;
	

		case SDIO_CMD_INT:	
			status_irq_reg->cmd_int = 1;	
			break;
	
		case SDIO_SOFT_INT:	
			status_irq_reg->soft_int = 1;	
			break;
	

		case SDIO_TIMEOUT_INT:
			status_irq_reg->timing_out_int = 1;
			status_irq_reg->timing_out_count = 0x1FFF;	
			break;
	
		default:	
			break;	
	}
	
	WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);
}

void sdio_close_host_interrupt(unsigned int_resource) 
{
	unsigned long irq_config, status_irq;
	MSHW_IRQ_Config_Reg_t * irq_config_reg;
	SDIO_Status_IRQ_Reg_t * status_irq_reg;
	irq_config = READ_CBUS_REG(SDIO_IRQ_CONFIG);
	status_irq = READ_CBUS_REG(SDIO_STATUS_IRQ);
	irq_config_reg = (void *)&irq_config;
	status_irq_reg = (void *)&status_irq;

	switch (int_resource) {
		case SDIO_IF_INT:	
			irq_config_reg->arc_if_int_en = 0;		
			status_irq_reg->if_int = 1;		
			break;
	
		case SDIO_CMD_INT:
			irq_config_reg->arc_cmd_int_en = 0;
			status_irq_reg->cmd_int = 1;	
			break;
	
		case SDIO_SOFT_INT:
			irq_config_reg->arc_soft_int_en = 0;
			status_irq_reg->soft_int = 1;
			break;
	
		case SDIO_TIMEOUT_INT:	
			status_irq_reg->arc_timing_out_int_en = 0;	
			status_irq_reg->timing_out_int = 1;	
			break;
	
		default:	
			break;	
	}

	WRITE_CBUS_REG(SDIO_IRQ_CONFIG, irq_config);
	WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);
}

unsigned sdio_check_interrupt(void) 
{
	unsigned long status_irq;
	SDIO_Status_IRQ_Reg_t * status_irq_reg;
	status_irq = READ_CBUS_REG(SDIO_STATUS_IRQ);
	status_irq_reg = (void *)&status_irq;
	
	if (status_irq_reg->cmd_int) {
		status_irq_reg->cmd_int = 1;
		status_irq_reg->arc_timing_out_int_en = 0;	
		status_irq_reg->timing_out_int = 1;	
		status_irq_reg->timing_out_count = 0;
		WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);	
		return SDIO_CMD_INT;	
	}
	else if (status_irq_reg->timing_out_int) {
		status_irq_reg->timing_out_int = 1;	
		status_irq_reg->timing_out_count = 0x1FFF;	
		WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);	
		return SDIO_TIMEOUT_INT;	
	}
	else if (status_irq_reg->if_int) {	
		/*close IF INT before clear if int, avoid IF INT twice*/
		sdio_close_host_interrupt(SDIO_IF_INT);
		status_irq_reg->if_int = 1;		
		WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);		
		return SDIO_IF_INT;		
	}
	else if (status_irq_reg->soft_int)
		return SDIO_SOFT_INT;
	else	
		return SDIO_NO_INT;
}

void sdio_if_int_handler(struct card_host *host) 
{
	sdio_irq_handled = 0;
	if (host->caps & CARD_CAP_SDIO_IRQ){
		//sdio_close_host_interrupt(SDIO_IF_INT);
		if(host->sdio_irq_thread)
			wake_up_process(host->sdio_irq_thread);
	}
	return;
} 

void sdio_cmd_int_handle(struct memory_card *card) 
{
	sdio_timeout_int_num = 0;
	complete(&sdio_int_complete);
	return;
} 

void sdio_timeout_int_handle(struct memory_card *card) 
{
	if(card){
	card->card_io_init(card);
	card->card_detector(card);
//	if(sdio_timeout_int_num && ((sdio_timeout_int_num%10000)==0))
//		printk("[sdio_timeout_int_handle] sdio_timeout_int_num = %d\n", sdio_timeout_int_num);
	if ((card->card_status == CARD_REMOVED) || (++sdio_timeout_int_num >= sdio_timeout_int_times)) {
		sdio_close_host_interrupt(SDIO_TIMEOUT_INT);
		sdio_timeout_int_num = 0;
		sdio_timeout_int_times = 0;
		complete(&sdio_int_complete);		
	}
	}else{
			printk("%s%s Null Card point\n", __FILE__, __func__);
			sdio_close_host_interrupt(SDIO_TIMEOUT_INT);
			sdio_timeout_int_num = 0;
			sdio_timeout_int_times = 0;
			complete(&sdio_int_complete);		
	}
	return;
}


