#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>
#include <linux/kthread.h>

#include <asm/mach-jz4770/jz4770msc.h>

#include "include/jz_mmc_dma.h"
#include "include/jz_mmc_host.h"


#ifdef JZ_MSC_USE_PIO

static int jz_mmc_pio_read(struct jz_mmc_host *host, unsigned int *buf, int len) {
	int i = 0;

	for (i = 0; i < len; i++) {
		while ((REG_MSC_STAT(host->pdev_id) & (MSC_STAT_DATA_FIFO_EMPTY | MSC_STAT_TIME_OUT_READ)) &&
		       !host->eject)
			;
		if (host->eject)
			return -ENOMEDIUM;

		if (REG_MSC_STAT(host->pdev_id) & MSC_STAT_TIME_OUT_READ)
			return -ETIMEDOUT;

		*buf = REG_MSC_RXFIFO(host->pdev_id);
		buf++;
	}

	return 0;
}

static int jz_mmc_pio_write(struct jz_mmc_host *host, unsigned int *buf, int len) {
	int i = 0;

	for (i = 0; i < len; i++) {
		while ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject)
			;
		if (host->eject)
			return -ENOMEDIUM;

		REG_MSC_TXFIFO(host->pdev_id) = *buf;
		buf++;
	}

	return 0;
}

void jz_mmc_stop_pio(struct jz_mmc_host *host) {
	while(!host->transfer_end);
}

static int jz_mmc_data_transfer(void *arg) {
	struct jz_mmc_host *host = (struct jz_mmc_host *)arg;
	struct mmc_data *data = host->curr_mrq->data;
	int is_write = data->flags & MMC_DATA_WRITE;
	int i = 0;
	int ret = 0;
	struct scatterlist *sgentry = NULL;
	unsigned int *buf = NULL;
	int len = 0;

	for_each_sg(data->sg, sgentry, data->sg_len, i) {
		buf = sg_virt(sgentry);
		len = (sg_dma_len(sgentry)+3) >> 2; /* divide by 4 */

		if (is_write)
			ret = jz_mmc_pio_write(host, buf, len);
		else
			ret = jz_mmc_pio_read(host, buf, len);
		if (ret) {
			data->error = ret;
			break;
		}
	}

	if (is_write) {
		if (ret) {
			host->data_ack = 0;
			wake_up_interruptible(&host->data_wait_queue);
		}
		/* else, DATA_TRANS_DONE interrupt will raise */
	} else {
		if (!ret)
			host->data_ack = 1;
		else
			host->data_ack = 0;
		wake_up_interruptible(&host->data_wait_queue);
	}

	host->transfer_end = 1;

	return 0;
}

void jz_mmc_start_pio(struct jz_mmc_host *host) {
	REG_MSC_CMDAT(host->pdev_id) &= ~MSC_CMDAT_DMA_EN;
	host->transfer_end = 0;
	kthread_run(jz_mmc_data_transfer, (void *)host, "msc pio transfer");
}

#endif /* JZ_MSC_USE_PIO */
