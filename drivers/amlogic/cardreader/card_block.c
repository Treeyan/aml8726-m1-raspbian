/*
 * Block driver for media (i.e., flash cards)
 */

#include <mach/io.h>

#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/proc_fs.h>
#include <linux/genhd.h>
#include <linux/kthread.h>
//#include <linux/cardreader/card_block.h>
//#include <linux/cardreader/cardreader.h>
#include <mach/old/card_block.h>
#include <mach/old/cardreader.h>
//#include <linux/aml_uevent_msg.h>
#include <mach/old/aml_uevent_msg.h>
//#include <asm/system.h>
//#include <mach/system.h>
#include <asm/uaccess.h>

#include <linux/semaphore.h>

#ifndef MAX_MTD_DEVICES
#define MAX_MTD_DEVICES     32
#endif // MAX_MTD_DEVICES

#ifndef PAGE_CACHE_SIZE
#define PAGE_CACHE_SIZE     PAGE_SIZE
#endif // PAGE_CACHE_SIZE


static int major;
#define CARD_SHIFT	4
#define CARD_QUEUE_EXIT		(1 << 0)
#define CARD_QUEUE_SUSPENDED	(1 << 1)

#define CARD_QUEUE_BOUNCESZ	(512*256)

#define CARD_NUM_MINORS	(256 >> CARD_SHIFT)
//static unsigned long dev_use[CARD_NUM_MINORS / (8 * sizeof(unsigned long))];
static unsigned long dev_use[1];

#define CARD_INAND_START_MINOR		40

static int card_blk_issue_rq(struct card_queue *cq, struct request *req);
static int card_blk_probe(struct memory_card *card);
static int card_blk_prep_rq(struct card_queue *cq, struct request *req);

struct card_blk_data {
	spinlock_t lock;
	struct gendisk *disk;
	struct card_queue queue;

	unsigned int usage;
	unsigned int block_bits;
	unsigned int read_only;
};

static DEFINE_MUTEX(open_lock);

static unsigned card_thread_sleep_flag = 0;
static struct completion card_thread_complete;
static wait_queue_head_t card_thread_wq;
static struct semaphore card_thread_sem;
/*wait device delete*/
struct completion card_devdel_comp;

/*sdio irq flag*/
unsigned char sdio_irq_handled=0;

struct card_queue_list {
	int cq_num;
	unsigned cq_flag;
	struct card_queue *cq;
	struct card_queue_list *cq_next;
};
static struct card_queue_list *card_queue_head = NULL;

void card_cleanup_queue(struct card_queue *cq)
{
	struct card_queue_list *cq_node_current = card_queue_head;
	struct card_queue_list *cq_node_prev = NULL;
	struct request_queue *q = cq->queue;
	unsigned long flags;
	
	while (cq_node_current != NULL){
		if (cq_node_current->cq == cq)
			break;

		cq_node_prev = cq_node_current;
		cq_node_current = cq_node_current->cq_next;
	}

	if (cq_node_current == card_queue_head) {
		if (cq_node_current->cq_next == NULL) {
			cq->flags |= CARD_QUEUE_EXIT;
			wake_up_interruptible(&card_thread_wq);
			up(&card_thread_sem);
			wait_for_completion(&card_thread_complete);
		} else {
			card_queue_head->cq_num = 0;
		}
		card_queue_head = card_queue_head->cq_next;
	} else {
		cq_node_prev->cq_next = cq_node_current->cq_next;
	}

	kfree(cq->sg);
	cq->sg = NULL;
	
	/* Empty the queue */   
	spin_lock_irqsave(q->queue_lock, flags);
	q->queuedata = NULL;
	blk_start_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
	
	//blk_cleanup_queue(cq->queue);

	cq->card = NULL;

	if (cq_node_current == card_queue_head)
		card_queue_head = NULL;
	kfree(cq_node_current);
	cq_node_current = NULL;
}

static struct card_blk_data *card_blk_get(struct gendisk *disk)
{
	struct card_blk_data *card_data;

	mutex_lock(&open_lock);
	card_data = disk->private_data;
	if (card_data && card_data->usage == 0)
		card_data = NULL;
	if (card_data)
		card_data->usage++;
	mutex_unlock(&open_lock);

	return card_data;
}

static void card_blk_put(struct card_blk_data *card_data)
{
	mutex_lock(&open_lock);
	card_data->usage--;
	if (card_data->usage == 0) {
		put_disk(card_data->disk);
		//card_cleanup_queue(&card_data->queue);
		blk_cleanup_queue(card_data->queue.queue);
		card_data->disk->queue = NULL;
		kfree(card_data);
		complete(&card_devdel_comp);
	}
	mutex_unlock(&open_lock);
}

static int card_blk_open(struct block_device *bdev, fmode_t mode)
{
	struct card_blk_data *card_data;
	int ret = -ENXIO;

	card_data = card_blk_get(bdev->bd_disk);
	if (card_data) {
		if (card_data->usage == 2)
			check_disk_change(bdev);
		ret = 0;

		if ((mode & FMODE_WRITE) && card_data->read_only)
			ret = -EROFS;
	}

	return ret;
}

//static int card_blk_release(struct gendisk *disk, fmode_t mode)
static void card_blk_release(struct gendisk *disk, fmode_t mode)
{
	struct card_blk_data *card_data = disk->private_data;
	
    card_blk_put(card_data);
//	return 0;
}

static int card_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->cylinders = get_capacity(bdev->bd_disk) / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static struct block_device_operations card_ops = {
	.open = card_blk_open,
	.release = card_blk_release,
	.getgeo = card_blk_getgeo,
	.owner = THIS_MODULE,
};

static inline int card_claim_card(struct memory_card *card)
{
	if(card->card_status == CARD_REMOVED)
		return -ENODEV;
	return __card_claim_host(card->host, card);
}

static int card_prep_request(struct request_queue *q, struct request *req)
{
	struct card_queue *cq = q->queuedata;
	int ret = BLKPREP_KILL;

	if (!cq) {
		printk(KERN_ERR "[card_prep_request] %s: killing request - no device/host\n", req->rq_disk->disk_name);
		return BLKPREP_KILL;
	}
	
    if( REQ_TYPE_DRV_PRIV == req->cmd_type ) {
//    if( REQ_TYPE_SPECIAL == req->cmd_type ) {
//	if (blk_special_request(req)) {
		/*
		 * Special commands already have the command
		 * blocks already setup in req->special.
		 */
		BUG_ON(!req->special);

		ret = BLKPREP_OK;
//	} else if (blk_fs_request(req) || blk_pc_request(req)) {
	} else if (( REQ_TYPE_FS == req->cmd_type ) || ( REQ_TYPE_BLOCK_PC == req->cmd_type)) {
		/*
		 * Block I/O requests need translating according
		 * to the protocol.
		 */
		ret = cq->prep_fn(cq, req);
	} else {
		/*
		 * Everything else is invalid.
		 */
		blk_dump_rq_flags(req, "CARD bad request");
	}

	if (ret == BLKPREP_OK)
		req->cmd_flags |= REQ_DONTPREP;

	return ret;
}

static void card_request(struct request_queue *q)
{
	struct card_queue *cq = q->queuedata;
	struct card_queue_list *cq_node_current = card_queue_head;
    struct request* req; 

	if (!cq) {
		while ((req = blk_fetch_request(q)) != NULL) {
			req->cmd_flags |= REQ_QUIET;
			__blk_end_request_all(req, -EIO);
		}
		return;
	}
	
	WARN_ON(!cq);
	WARN_ON(!cq_node_current);
	WARN_ON(!cq_node_current->cq);

	while (cq_node_current != NULL){
		if (cq && cq_node_current->cq == cq) {
			cq_node_current->cq_flag = 1;
			break;
		}

		cq_node_current = cq_node_current->cq_next;
	}

	if (card_thread_sleep_flag) {
		card_thread_sleep_flag = 0;
		wake_up_interruptible(&card_thread_wq);
	}
}

void card_queue_suspend(struct card_queue *cq)
{
	struct request_queue *q = cq->queue;
	unsigned long flags;

	if (!(cq->flags & CARD_QUEUE_SUSPENDED)) {
		cq->flags |= CARD_QUEUE_SUSPENDED;

		spin_lock_irqsave(q->queue_lock, flags);
		blk_stop_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);

		down(&card_thread_sem);
	}
}

void card_queue_resume(struct card_queue *cq)
{
	struct request_queue *q = cq->queue;
	unsigned long flags;

	if (cq->flags & CARD_QUEUE_SUSPENDED) {
		cq->flags &= ~CARD_QUEUE_SUSPENDED;

		up(&card_thread_sem);

		spin_lock_irqsave(q->queue_lock, flags);
		blk_start_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);
	}
}

static int card_queue_thread(void *d)
{
	struct card_queue *cq = d;
	struct request_queue *q = cq->queue;
	struct card_queue_list *cq_node_current;
	unsigned char rewait;
	DECLARE_WAITQUEUE(wait, current);

//	daemonize("card_queue_thread");
	/*
	 * Set iothread to ensure that we aren't put to sleep by
	 * the process freezing.  We handle suspension ourselves.
	 */
	current->flags |= PF_MEMALLOC;

	complete(&card_thread_complete);

	down(&card_thread_sem);
	add_wait_queue(&card_thread_wq, &wait);
	do {
		struct request *req = NULL;

		//spin_lock_irq(q->queue_lock);
		cq_node_current = card_queue_head;
		WARN_ON(!card_queue_head);
		while (cq_node_current != NULL) {
			cq = cq_node_current->cq;
			q = cq->queue;
			if (cq_node_current->cq_flag) {
				/*wait sdio handle irq & xfer data*/
				for(rewait=3;(!sdio_irq_handled)&&(rewait--);)
					schedule();
//				if (!blk_queue_plugged(q)) {
				if ( !blk_queue_tagged(q) ) {
					spin_lock_irq(q->queue_lock);
					req = blk_fetch_request(q);
					spin_unlock_irq(q->queue_lock);
				}
				if (req)
					break;

				cq_node_current->cq_flag = 0;
			}

			cq_node_current = cq_node_current->cq_next;
		}

		cq->req = req;
		//spin_unlock_irq(q->queue_lock);

		if (cq->flags & CARD_QUEUE_EXIT)
			break;
		
		if (!req) {

			if (cq_node_current == NULL) {
				up(&card_thread_sem);
				card_thread_sleep_flag = 1;
				//interruptible_sleep_on(&card_thread_wq);
                //
                // ddd
			    wait_event_interruptible( card_thread_wq, 0 == card_thread_sleep_flag );
				//schedule();
				#pragma GCC diagnostic push
                #pragma GCC diagnostic ignored "-Wunused-result"
                down_interruptible(&card_thread_sem);
                #pragma GCC diagnostic pop

			}
			continue;
		}

		cq->issue_fn(cq, req);
		/*yield*/
		cond_resched();
	} while (1);
	remove_wait_queue(&card_thread_wq, &wait);
	up(&card_thread_sem);

	complete_and_exit(&card_thread_complete, 0);

	return 0;
}

#define CONFIG_CARD_BLOCK_BOUNCE 1

#ifdef CONFIG_CARD_BLOCK_BOUNCE
/*
 * Prepare the sg list(s) to be handed of to the host driver
 */
static unsigned int card_queue_map_sg(struct card_queue *cq)
{
	unsigned int sg_len;
	size_t buflen;
	struct scatterlist *sg;
	int i;

	if (!cq->bounce_buf)
		return blk_rq_map_sg(cq->queue, cq->req, cq->sg);

	BUG_ON(!cq->bounce_sg);

	sg_len = blk_rq_map_sg(cq->queue, cq->req, cq->bounce_sg);

	cq->bounce_sg_len = sg_len;

	buflen = 0;
	for_each_sg(cq->bounce_sg, sg, sg_len, i)
		buflen += sg->length;

	sg_init_one(cq->sg, cq->bounce_buf, buflen);

	return 1;
}

/*
 * If writing, bounce the data to the buffer before the request
 * is sent to the host driver
 */
static void card_queue_bounce_pre(struct card_queue *cq)
{
	unsigned long flags;

	if (!cq->bounce_buf)
		return;

	if (rq_data_dir(cq->req) != WRITE)
		return;
	
	local_irq_save(flags);
		
	sg_copy_to_buffer(cq->bounce_sg, cq->bounce_sg_len,
		cq->bounce_buf, cq->sg[0].length);
	
	local_irq_restore(flags);	
}

/*
 * If reading, bounce the data from the buffer after the request
 * has been handled by the host driver
 */
static void card_queue_bounce_post(struct card_queue *cq)
{
	unsigned long flags;

	if (!cq->bounce_buf)
		return;

	if (rq_data_dir(cq->req) != READ)
		return;
	
	local_irq_save(flags);
	
	sg_copy_from_buffer(cq->bounce_sg, cq->bounce_sg_len,
		cq->bounce_buf, cq->sg[0].length);

	local_irq_restore(flags);
	
	bio_flush_dcache_pages(cq->req->bio);
}

/*
 * Alloc bounce buf for read/write numbers of pages in one request
 */
static int card_init_bounce_buf(struct card_queue *cq, 
			struct memory_card *card)
{
	int ret=0;
	struct card_host *host = card->host;
	unsigned int bouncesz;

	bouncesz = CARD_QUEUE_BOUNCESZ;

	if (bouncesz > host->max_req_size)
		bouncesz = host->max_req_size;

	if (bouncesz >= PAGE_CACHE_SIZE) {
		//cq->bounce_buf = kmalloc(bouncesz, GFP_KERNEL);
		cq->bounce_buf = host->dma_buf;
		if (!cq->bounce_buf) {
			printk(KERN_WARNING "%s: unable to "
				"allocate bounce buffer\n", card->name);
		}
	}

	if (cq->bounce_buf) {
		blk_queue_bounce_limit(cq->queue, BLK_BOUNCE_HIGH);
		blk_queue_max_hw_sectors(cq->queue, bouncesz / 512);
		blk_queue_physical_block_size(cq->queue, bouncesz);
		blk_queue_max_segments(cq->queue, bouncesz / PAGE_CACHE_SIZE);
		blk_queue_max_segment_size(cq->queue, bouncesz);

		cq->queue->queuedata = cq;
		cq->req = NULL;
	
		cq->sg = kmalloc(sizeof(struct scatterlist),
			GFP_KERNEL);
		if (!cq->sg) {
			ret = -ENOMEM;
			blk_cleanup_queue(cq->queue);
			return ret;
		}
		sg_init_table(cq->sg, 1);

		cq->bounce_sg = kmalloc(sizeof(struct scatterlist) *
			bouncesz / PAGE_CACHE_SIZE, GFP_KERNEL);
		if (!cq->bounce_sg) {
			ret = -ENOMEM;
			kfree(cq->sg);
			cq->sg = NULL;
			blk_cleanup_queue(cq->queue);
			return ret;
		}
		sg_init_table(cq->bounce_sg, bouncesz / PAGE_CACHE_SIZE);
	}

	return 0;
}

#else

static unsigned int card_queue_map_sg(struct card_queue *cq)
{
}

static void card_queue_bounce_pre(struct card_queue *cq)
{
}

static void card_queue_bounce_post(struct card_queue *cq)
{
}

static int card_init_bounce_buf(struct card_queue *cq, 
			struct memory_card *card)
{
}

#endif

int card_init_queue(struct card_queue *cq, struct memory_card *card,
		    spinlock_t * lock)
{
	struct card_host *host = card->host;
	u64 limit = BLK_BOUNCE_HIGH;
	int ret=0, card_quene_num;
	struct card_queue_list *cq_node_current;
	struct card_queue_list *cq_node_prev = NULL;

	if (host->parent->dma_mask && *host->parent->dma_mask)
		limit = *host->parent->dma_mask;

	cq->card = card;
	cq->queue = blk_init_queue(card_request, lock);
	if (!cq->queue)
		return -ENOMEM;

	blk_queue_prep_rq(cq->queue, card_prep_request);
	card_init_bounce_buf(cq, card);
	
	if(!cq->bounce_buf){
		blk_queue_bounce_limit(cq->queue, limit);
		blk_queue_max_hw_sectors(cq->queue, host->max_sectors);
		//blk_queue_max_hw_phys_segments(cq->queue, host->max_phys_segs);
		blk_queue_max_segments(cq->queue, host->max_hw_segs);
		blk_queue_max_segment_size(cq->queue, host->max_seg_size);

		cq->queue->queuedata = cq;
		cq->req = NULL;

		cq->sg = kmalloc(sizeof(struct scatterlist) * host->max_phys_segs, GFP_KERNEL);
		if (!cq->sg) {
			ret = -ENOMEM;
			blk_cleanup_queue(cq->queue);
			return ret;
		}
	}

	/*change card io scheduler from cfq to deadline*/
	cq->queue->queuedata = cq;
	elevator_exit(cq->queue->elevator);
	cq->queue->elevator = NULL;
	ret = elevator_init(cq->queue, "deadline");
	if (ret) {
             printk("[card_init_queue] elevator_init deadline fail\n");
		blk_cleanup_queue(cq->queue);
		return ret;
	}

	if (card_queue_head == NULL)
	{
		card_queue_head = kmalloc(sizeof(struct card_queue_list), GFP_KERNEL);
		if (card_queue_head == NULL) 
		{
			ret = -ENOMEM;
			kfree(card_queue_head);
			card_queue_head = NULL;
			return ret;
		}
		card_queue_head->cq = cq;
		card_queue_head->cq_num = 0;
		card_queue_head->cq_flag = 0;
		card_queue_head->cq_next = NULL;

		init_completion(&card_thread_complete);
		init_waitqueue_head(&card_thread_wq);
		//init_MUTEX(&card_thread_sem);
		sema_init(&card_thread_sem, 1);
		host->queue_task = kthread_run(card_queue_thread, cq, "card_queue");
		if (host->queue_task)
		{
			wait_for_completion(&card_thread_complete);
			init_completion(&card_thread_complete);
			ret = 0;
			return ret;
		}
	} 
	else
	{
		card_quene_num = 0;
		cq_node_current = card_queue_head;
		do
		{
			card_quene_num = cq_node_current->cq_num;
			cq_node_prev = cq_node_current;
			cq_node_current = cq_node_current->cq_next;
		} while (cq_node_current != NULL);

		cq_node_current = kmalloc(sizeof(struct card_queue_list), GFP_KERNEL);
		if (cq_node_current == NULL)
		{
			ret = -ENOMEM;
			kfree(cq_node_current);
			cq_node_current = NULL;
			return ret;
		}
		cq_node_prev->cq_next = cq_node_current;
		cq_node_current->cq = cq;
		cq_node_current->cq_next = NULL;
		cq_node_current->cq_num = (++card_quene_num);
		cq_node_current->cq_flag = 0;

		ret = 0;
		return ret;
	}

	return ret;
}

static struct card_blk_data *card_blk_alloc(struct memory_card *card)
{
	struct card_blk_data *card_data;
	int devidx, ret;

	devidx = find_first_zero_bit(dev_use, CARD_NUM_MINORS);

	if(card->card_type == CARD_INAND)
		devidx = CARD_INAND_START_MINOR>>CARD_SHIFT;
	
	if (devidx >= CARD_NUM_MINORS)
		return ERR_PTR(-ENOSPC);
	__set_bit(devidx, dev_use);

	card_data = kmalloc(sizeof(struct card_blk_data), GFP_KERNEL);
	if (!card_data) {
		ret = -ENOMEM;
		return ERR_PTR(ret);
	}

	memset(card_data, 0, sizeof(struct card_blk_data));

	card_data->block_bits = 9;

	card_data->disk = alloc_disk(1 << CARD_SHIFT);
	if (card_data->disk == NULL) {
		ret = -ENOMEM;
		kfree(card_data);
		return ERR_PTR(ret);
	}

	spin_lock_init(&card_data->lock);
	card_data->usage = 1;

	ret = card_init_queue(&card_data->queue, card, &card_data->lock);
	if (ret) {
		put_disk(card_data->disk);
		return ERR_PTR(ret);
	}

	card_data->queue.prep_fn = card_blk_prep_rq;
	card_data->queue.issue_fn = card_blk_issue_rq;
	card_data->queue.data = card_data;

	card_data->disk->major = major;
	card_data->disk->minors = 1 << CARD_SHIFT;
	card_data->disk->first_minor = devidx << CARD_SHIFT;
	card_data->disk->fops = &card_ops;
	card_data->disk->private_data = card_data;
	card_data->disk->queue = card_data->queue.queue;
    //
    // ddd
    // 
//	card_data->disk->driverfs_dev = &card->dev;
//	sprintf(card_data->disk->disk_name, "cardblk%s", card->name);
	sprintf( card_data->disk->disk_name, 
             "mmcblk%u%s",
             card->host->index,
             card->name );

	blk_queue_logical_block_size(card_data->queue.queue, 1 << card_data->block_bits);

	set_capacity(card_data->disk, card->capacity);

	return card_data;
}

static int card_blk_prep_rq(struct card_queue *cq, struct request *req)
{
	struct card_blk_data *card_data = cq->data;
	int stat = BLKPREP_OK;

	WARN_ON(!cq->queue->queuedata);
	/*
	 * If we have no device, we haven't finished initialising.
	 */
	if (!card_data || !cq->card || !cq->queue->queuedata) {
		printk(KERN_ERR "%s: killing request - no device/host\n", req->rq_disk->disk_name);
		stat = BLKPREP_KILL;
	}

	return stat;
}

static int card_blk_issue_rq(struct card_queue *cq, struct request *req)
{
	struct card_blk_data *card_data = cq->data;
	struct memory_card *card = card_data->queue.card;
	struct card_blk_request brq;
	int ret;

	if (card_claim_card(card)) {
		spin_lock_irq(&card_data->lock);
		ret = 1;
		while (ret) {
			ret = __blk_end_request(req, -EIO, (1 << card_data->block_bits));
		}
		spin_unlock_irq(&card_data->lock);
		return 0;
	}

	do {
		brq.crq.cmd = rq_data_dir(req);
		brq.crq.buf = cq->bounce_buf;
		//	brq.crq.buf = req->buffer;

		brq.card_data.lba = blk_rq_pos(req);
		brq.card_data.blk_size = 1 << card_data->block_bits;
		brq.card_data.blk_nums = blk_rq_sectors(req);

		brq.card_data.sg = cq->sg;

		brq.card_data.sg_len = card_queue_map_sg(cq);
		//brq.card_data.sg_len = blk_rq_map_sg(req->q, req, brq.card_data.sg);

		card->host->card_type = card->card_type;
		
		card_queue_bounce_pre(cq);

		card_wait_for_req(card->host, &brq);
		
		card_queue_bounce_post(cq);
			
		/*
		 *the request issue failed
		 */
		if (brq.card_data.error) {
			card_release_host(card->host);

			spin_lock_irq(&card_data->lock);
			ret = 1;
			while (ret) {
				ret = __blk_end_request(req, -EIO, (1 << card_data->block_bits));
			}
			spin_unlock_irq(&card_data->lock);

			/*add_disk_randomness(req->rq_disk);
			   blkdev_dequeue_request(req);
			   end_that_request_last(req, 0);
			   spin_unlock_irq(&card_data->lock); */

			return 0;
		}
		/*
		 * A block was successfully transferred.
		 */
		spin_lock_irq(&card_data->lock);
		brq.card_data.bytes_xfered = brq.card_data.blk_size * brq.card_data.blk_nums;
		ret = __blk_end_request(req, 0, brq.card_data.bytes_xfered);
		//if(!ret) 
		//{
		/*
		 * The whole request completed successfully.
		 */
		/*add_disk_randomness(req->rq_disk);
		   blkdev_dequeue_request(req);
		   end_that_request_last(req, 1);
		   } */
		spin_unlock_irq(&card_data->lock);
	} while (ret);

	card_release_host(card->host);
	//printk("card request completely %d sector num: %d communiction dir %d\n", brq.card_data.lba, brq.card_data.blk_nums, brq.crq.cmd);
	return 1;
}

static void card_blk_remove(struct memory_card *card)
{
	struct card_blk_data *card_data = card_get_drvdata(card);

	if (card_data) {
		int devidx;

		del_gendisk(card_data->disk);

		/*
		 * I think this is needed.
		 */
		
		queue_flag_set_unlocked(QUEUE_FLAG_DEAD, card_data->queue.queue);
		queue_flag_set_unlocked(QUEUE_FLAG_STOPPED, card_data->queue.queue);
		card_data->queue.queue->queuedata = NULL;
		card_cleanup_queue(&card_data->queue);
		//card_data->disk->queue = NULL;

		devidx = card_data->disk->first_minor >> CARD_SHIFT;
		__clear_bit(devidx, dev_use);
		card_blk_put(card_data);
	}
	card_set_drvdata(card, NULL);
}

#ifdef CONFIG_PM
static int card_blk_suspend(struct memory_card *card, pm_message_t state)
{
	struct card_blk_data *card_data = card_get_drvdata(card);
	struct card_host *host = card->host;
	
	printk("Enter %s suspend\n",card->name);
	printk("***Entered %s:%s\n", __FILE__,__func__);


	if (card_data) 
	{
		card_queue_suspend(&card_data->queue);
	}
	if(!host->sdio_task_state)
	{
		host->sdio_task_state = 1;
	}
	if(!host->card_task_state)
	{
		host->card_task_state = 1;
	}
	if(card->card_suspend)
	{
		card->card_suspend(card);
	}
	if(card->card_type == CARD_SDIO)
		return 0;
	//card->unit_state = CARD_UNIT_NOT_READY;
	//host->slot_detector = CARD_REMOVED;
	card->unit_state = CARD_UNIT_RESUMED;
	return 0;
}

static int card_blk_resume(struct memory_card *card)
{
	struct card_blk_data *card_data = card_get_drvdata(card);
	struct card_host *host = card->host;
	
	printk("***Entered %s:%s\n", __FILE__,__func__);
	
	printk("Enter %s resume\n",card->name);

	if(card->card_resume)
	{
		card->card_resume(card);
	}
	if(host->card_task_state)
	{
		host->card_task_state = 0;
		if(host->card_task)
			wake_up_process(host->card_task);
	}
	if(host->sdio_task_state)
	{
		host->sdio_task_state = 0;
		if(host->sdio_irq_thread)
			wake_up_process(host->sdio_irq_thread);
	}
	if (card_data) {
		//mmc_blk_set_blksize(md, card);
		card_queue_resume(&card_data->queue);
	}
	return 0;
}
#else
#define	card_blk_suspend	NULL
#define card_blk_resume		NULL
#endif

#if 0 //def CONFIG_PROC_FS

/*====================================================================*/
/* Support for /proc/mtd */

static struct proc_dir_entry *proc_card;
struct mtd_partition *card_table[MAX_MTD_DEVICES];

static inline int card_proc_info (char *buf, char* dev_name, int i)
{
	struct mtd_partition *this = card_table[i];

	if (!this)
		return 0;

	return sprintf(buf, "%s%d: %8.8llx %8.8x \"%s\"\n", dev_name,
		        i+1,(unsigned long long)this->size,
		       CARD_QUEUE_BOUNCESZ, this->name);
}

static int card_read_proc (char *page, char **start, off_t off, int count,
			  int *eof, void *data_unused)
{
	int len, l, i;
        off_t   begin = 0;

	len = sprintf(page, "dev:    size   erasesize  name\n");
        for (i=0; i< MAX_MTD_DEVICES; i++) {

                l = card_proc_info(page + len, "inand", i);
                len += l;
                if (len+begin > off+count)
                        goto done;
                if (len+begin < off) {
                        begin += len;
                        len = 0;
                }
        }

        *eof = 1;

done:
        if (off >= len+begin)
                return 0;
        *start = page + (off-begin);
        return ((count < begin+len-off) ? count : begin+len-off);
}

#endif /* CONFIG_PROC_FS */

/**
 * add_card_partition : add card partition , refer to 
 * board-****.c  inand_partition_info[]
 * @disk: add partitions in which disk
 * @part: partition table
 * @nr_part: partition numbers
 */
int add_card_partition(struct gendisk * disk, struct mtd_partition * part, 
				unsigned int nr_part)
{
	unsigned int i;
	struct hd_struct * ret;
	uint64_t cur_offset=0;
	uint64_t offset, size;
	
	if(!part)
		return 0;

	for(i=0; i<nr_part; i++){
		offset = part[i].offset>>9;
		size = part[i].size>>9;
		if (part[i].offset== MTDPART_OFS_APPEND)
			offset = cur_offset;
		if (part[i].size == MTDPART_SIZ_FULL)
			size = disk->part0.nr_sects - offset;
//		ret = add_partition(disk, 1+i, offset, size, 0);
		ret = add_partition(disk, 1+i, offset, size, 0, NULL);
		printk("[%s] %20s  offset 0x%012llx, len 0x%012llx %s\n", 
				disk->disk_name, part[i].name, offset<<9, size<<9, 
				IS_ERR(ret) ? "add fail":"");
		//if(IS_ERR(ret)){
		//	printk("errno = %d, offset = %x, size = %x, disk->part0.nr_sects = %x\n", ret, offset, size);
		//	return ERR_PTR(ret);
		//}
		cur_offset = offset + size;
		
#if 0 // def CONFIG_PROC_FS

		card_table[i] = &part[i];
		card_table[i]->offset = offset<<9;
		card_table[i]->size = size<<9;
#endif

	}


#if 0 // def CONFIG_PROC_FS
    //
    // Lazy...lazy
    //
	if (!proc_card && (proc_card = create_proc_entry( "inand", 0, NULL )))
		proc_card->read_proc = card_read_proc;
#endif /* CONFIG_PROC_FS */



	return 0;
}


static int card_blk_probe(struct memory_card *card)
{
	struct card_blk_data *card_data;
	struct aml_card_info *pinfo = card->card_plat_info;

	card_data = card_blk_alloc(card);
	if (IS_ERR(card_data))
	{
		aml_send_msg("PROBE", 1);
		return PTR_ERR(card_data);
	}
	else
	{
		aml_send_msg("PROBE", 0);
	}

	card_set_drvdata(card, card_data);

	add_disk(card_data->disk);
	add_card_partition(card_data->disk, pinfo->partitions,
			pinfo->nr_partitions);

	return 0;
}

static struct card_driver card_driver = {
	.drv = {
//		.name = "cardblk",
		.name = "mmcblk",
		},
	.probe = card_blk_probe,
	.remove = card_blk_remove,
	.suspend = card_blk_suspend,
	.resume = card_blk_resume,
};

static int __init card_blk_init(void)
{
	int res = -ENOMEM;

	res = register_blkdev(major, "memorycard");
	if (res < 0) {
		printk(KERN_WARNING
		       "Unable to get major %d for Memory Card media: %d\n",
		       major, res);
		return res;
	}
	if (major == 0)
		major = res;
    printk(KERN_WARNING
		       "Memory Card media Major: %d\n",
		       major);
	return card_register_driver(&card_driver);
}

static void __exit card_blk_exit(void)
{
	card_unregister_driver(&card_driver);
	unregister_blkdev(major, "memorycard");
}

module_init(card_blk_init);
module_exit(card_blk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Memory Card block device driver");

module_param(major, int, 0444);
MODULE_PARM_DESC(major, "specify the major device number for Memory block driver");
