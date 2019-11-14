/*
 *	WKIC Ltd.
 * By  Xu XunWei Tech  
 *	DEMO Version :1.01 Data:2014-12-25
 *
 *
 *
 * 	1. compiler warnings all changes
 */
#include<linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include<linux/timer.h>

#include<linux/tty_flip.h>
#include <linux/gpio.h> 

#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "wk2xxx.h"

//#include <mach/map.h>
#include <linux/gpio.h>
//#include <mach/regs-clock.h>
//#include <mach/regs-gpio.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
//#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/syscalls.h>

MODULE_LICENSE("Dual BSD/GPL");

#define SPI_BUFSIZ      max(32,SMP_CACHE_BYTES)
//#define _DEBUG_WK2XXX
//#define _DEBUG_WK2XXX1
//#define _DEBUG_WK2XXX2
//#define _DEBUG_WK2XXX3

#define CONFIG_DEVFS_FS


#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0


#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8

#define  GPA1CON  0xE0200020

static DEFINE_MUTEX(wk2xxxs_lock);                /* race on probe */
static DEFINE_MUTEX(wk2xxxs_wr_lock); 
static DEFINE_MUTEX(wk2xxs_work_lock);                /* work on probe */

struct wk2xxx_port 
{
	//struct timer_list mytimer;	

	struct uart_port port;//[NR_PORTS];
	spinlock_t conf_lock;	/* shared data */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int suspending;
	void (*wk2xxx_hw_suspend) (int suspend);
	int tx_done;

	int force_end_work;
	int irq;
	int minor;		/* minor number */
	int tx_empty; 
	int tx_empty_flag;

	//int start_tx;
	int start_tx_flag;
	int stop_tx_flag;
	int stop_rx_flag; 
	int irq_flag;
	int conf_flag;

	int tx_empty_fail;
	int start_tx_fail;
	int stop_tx_fail;
	int stop_rx_fail;
	int irq_fail;
	int conf_fail;

	uint8_t new_lcr;
	uint8_t new_scr; 
	/*set baud 0f register*/
	uint8_t new_baud1;
	uint8_t new_baud0;
	uint8_t new_pres;

};

static struct wk2xxx_port wk2xxxs[NR_PORTS]; /* the chips */
///////////////////////////gai/////////////////////////////
//struct resource *s5pv210uart_resource;
//***************************** 函数声明 ********************************
void wk_s5pv210uart_InitIO(void);  //初始化IO端口的函数

/***************************** 函数定义 ********************************
描述   : 初始化IO端口
参数   : 无
返回值 : 无
***********************************************************************/
static void serial2002_tty_read_poll_wait(struct file *f, int timeout)
{                                                                     
	struct poll_wqueues table;                                        
	ktime_t start, now;                                               

	start = ktime_get();                                              
	poll_initwait(&table);                                            
	while (1) {                                                       
		long elapsed;                                                 
		int mask;                                                     

		mask = f->f_op->poll(f, &table.pt);                           
		if (mask & (POLLRDNORM | POLLRDBAND | POLLIN |                
					POLLHUP | POLLERR)) {                                 
			break;                                                    
		}                                                             
		now = ktime_get();                                            
		elapsed = ktime_us_delta(now, start);                         
		if (elapsed > timeout)                                        
			break;                                                    
		set_current_state(TASK_INTERRUPTIBLE);                        
		schedule_timeout(((timeout - elapsed) * HZ) / 10000);         
	}                                                                 
	poll_freewait(&table);                                            
}                                                                     

static long serial2002_tty_ioctl(struct file *f, unsigned int op,
		unsigned long param)
{
	if (f->f_op->unlocked_ioctl)
		return f->f_op->unlocked_ioctl(f, op, param);

	return -ENOTTY;
}


static void serial2002_tty_setspeed(struct file *f, int speed)
{
	struct termios termios;
	struct serial_struct serial;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	/* Set speed */
	serial2002_tty_ioctl(f, TCGETS, (unsigned long)&termios);
	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_lflag = 0;
	termios.c_cflag = CLOCAL | CS8 | CREAD;
	termios.c_cc[VMIN] = 0;
	termios.c_cc[VTIME] = 0;
	switch (speed) {
		case 2400:
			termios.c_cflag |= B2400;
			break;
		case 4800:
			termios.c_cflag |= B4800;
			break;
		case 9600:
			termios.c_cflag |= B9600;
			break;
		case 19200:
			termios.c_cflag |= B19200;
			break;
		case 38400:
			termios.c_cflag |= B38400;
			break;
		case 57600:
			termios.c_cflag |= B57600;
			break;
		case 115200:
			termios.c_cflag |= B115200;
			break;                                                    
		case 460800:
			termios.c_cflag |= B460800;
			break;                                                    
		default:                                                      
			termios.c_cflag |= B9600;                                 
			break;                                                    
	}                                                             
	serial2002_tty_ioctl(f, TCSETS, (unsigned long)&termios);     

	/* Set low latency */ 
	serial2002_tty_ioctl(f, TIOCGSERIAL, (unsigned long)&serial); 
	serial.flags |= ASYNC_LOW_LATENCY;                            
	serial2002_tty_ioctl(f, TIOCSSERIAL, (unsigned long)&serial); 

	set_fs(oldfs); 
} 


//static int fd;
struct file *filp;
mm_segment_t old_fs;
void wk_s5pv210uart_InitIO(void)
{
	printk(KERN_INFO"%s\n",__func__);
	
	filp = filp_open("/dev/ttymxc2",  O_RDWR, 0);
	if(IS_ERR(filp))
	{                                                          
		printk(KERN_INFO"open filp error\n");                  
	} else {
		serial2002_tty_setspeed(filp,460800);		
	} 
}

void s5pv_uart_wr(char c)
{
	loff_t pos = 0;

	old_fs = get_fs();                           
	set_fs(KERNEL_DS);                           

	//filp->f_op->write(filp, &c, 1, &filp->f_pos);
	kernel_write(filp, &c, 1, &pos);
	
	set_fs(old_fs);      
	udelay(150);
}
static volatile char read_fifo_flag = 0;

char s5pv_uart_rd(void)
{
	char c = 0, d = 0;
	loff_t pos = 0;
	int ret;
	
	old_fs = get_fs();                              
	set_fs(KERNEL_DS);                              

	//内核io带缓存
	if (read_fifo_flag == 1)
	{
		do {
			serial2002_tty_read_poll_wait(filp, 100);
			ret = kernel_read(filp, &c, 1, &pos);
			if(ret == 1)
			{
				d = c;
				break;
			} 
		} while(ret != 1);
	} 
	else 
	{
		do {
			serial2002_tty_read_poll_wait(filp, 100);
			ret = kernel_read(filp, &c, 1, &pos);
			if(ret == 1)
			{
				d = c;
			} 
		} while(ret == 1);
	}
	//ret = filp->f_op->read(filp, &c, 1, &filp->f_pos);
	/*ret = kernel_read(filp, &c, 1, &pos);
	if(ret != 1)
	{
		printk(KERN_INFO"read_ret=%d\n",ret);
	}*/

	set_fs(old_fs);                                 
	
	//udelay(150);
	return d;
}

static int wk2xxx_read_reg(uint8_t port,uint8_t reg,uint8_t *dat)
{
	uint8_t wk_command;


	mutex_lock(&wk2xxxs_wr_lock);
	wk_command=0x40|(((port-1)<<4)|reg);
	s5pv_uart_wr(wk_command);
	*dat=s5pv_uart_rd();
	mutex_unlock(&wk2xxxs_wr_lock);
	
	return 0;
}

static int wk2xxx_write_reg(uint8_t port,uint8_t reg,uint8_t dat)
{
	uint8_t wk_command;
	mutex_lock(&wk2xxxs_wr_lock);
	wk_command= (((port-1)<<4)|reg);
	s5pv_uart_wr(wk_command);
	s5pv_uart_wr(dat);
	udelay(2);
	mutex_unlock(&wk2xxxs_wr_lock);
	return 0;
}
static int wk2xxx_read_fifo(uint8_t port,uint8_t fifolen,uint8_t *dat)
{
	uint8_t wk_command, i;
		
	mutex_lock(&wk2xxxs_wr_lock);
	read_fifo_flag = 1;
	if(fifolen>0)
	{
		wk_command=0xc0|(((port-1)<<4)|(fifolen-1));
		s5pv_uart_wr(wk_command);
		for(i=0;i<fifolen;i++)
			*(dat+i)=s5pv_uart_rd();
	}
	read_fifo_flag = 0;
	mutex_unlock(&wk2xxxs_wr_lock);

	return 0;
}
static int wk2xxx_write_fifo(uint8_t port,uint8_t fifolen,uint8_t *dat)
{
	uint8_t wk_command,i;
	mutex_lock(&wk2xxxs_wr_lock);
	if(fifolen>0)
	{
		wk_command= (0x80|((port-1)<<4)|(fifolen-1));
		s5pv_uart_wr(wk_command);
		for(i=0;i<fifolen;i++)
			s5pv_uart_wr(*(dat+i));
		udelay(2);
	}
	mutex_unlock(&wk2xxxs_wr_lock);
	return 0;
}


static void wk2xxxirq_app(struct uart_port *port);//
static void conf_wk2xxx_subport(struct uart_port *port);//
static void wk2xxx_work(struct work_struct *w);
static void wk2xxx_stop_tx(struct uart_port *port);//
static u_int wk2xxx_tx_empty(struct uart_port *port);

static int wk2xxx_dowork(struct wk2xxx_port *s)
{    
#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_dowork---in---\n");
#endif

	if (!s->force_end_work && !work_pending(&s->work) && !freezing(current) && !s->suspending)
	{
		queue_work(s->workqueue, &s->work);//
#ifdef _DEBUG_WK2XXX
		printk("--queue_work---ok---\n");
#endif
		return 1;	
	}
	else
	{
#ifdef _DEBUG_WK2XXX
		printk("--queue_work---error---\n");
#endif
		return 0;
	}

}

static void wk2xxx_work(struct work_struct *w)
{  
#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_work---in---\n");
#endif

	struct wk2xxx_port *s = container_of(w, struct wk2xxx_port, work);
	uint8_t rx;


	//	int work_tx_empty_flag;
	int work_start_tx_flag; 

	int work_stop_rx_flag;
	//	int work_stop_tx_flag;

	int work_irq_flag;
	//int work_irq_fail;
	int work_conf_flag;
	do {

		mutex_lock(&wk2xxs_work_lock);
		//spin_lock(&s->conf_lock);
		/*work_tx_empty_flag = s->tx_empty_flag;
		  if(work_tx_empty_flag)
		  s->tx_empty_flag = 0;*/
		work_start_tx_flag = s->start_tx_flag;
		if(work_start_tx_flag)
			s->start_tx_flag = 0;
		/*work_stop_tx_flag = s->stop_tx_flag;
		  if(work_stop_tx_flag)
		  s->stop_tx_flag = 0;*/
		work_stop_rx_flag = s->stop_rx_flag;
		if(work_stop_rx_flag)
			s->stop_rx_flag = 0;
		work_conf_flag = s->conf_flag;
		/*if(work_conf_flag)
		  s->conf_flag = 0;*/



		work_irq_flag = s->irq_flag;
		if(work_irq_flag)
			s->irq_flag = 0;

		//work_irq_fail = s->irq_fail;
		//if(work_irq_fail)
		//s->irq_fail = 0;

		//spin_unlock(&s->conf_lock);
		mutex_unlock(&wk2xxs_work_lock);

		/*	if(work_conf_flag)
			{
			conf_wk2xxx_subport(&s->port);
			}*/
		/*if(work_tx_empty_flag)
		  {
		  wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,&rx);
		  s->tx_empty = (rx & WK2XXX_TDAT)<=0;
		  }*/
		if(work_start_tx_flag)
		{
			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
			rx |= WK2XXX_TFTRIG_IEN; 
			wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);
#ifdef _DEBUG_WK2XXX
			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
			printk(KERN_ALERT "wk2xxx_work1()----port:%d--sier:0x%x----\n",s->port.iobase,rx);
#endif
		}
		/*if(work_stop_tx_flag)
		  {
		  wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
		  rx &=~WK2XXX_TFTRIG_IEN;
		  wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);
		  wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,&rx);
		  rx &= ~WK2XXX_TFTRIG_INT;
		  wk2xxx_write_reg(s->port.iobase,WK2XXX_SIFR,rx);
		  }*/
		if(work_stop_rx_flag)
		{
			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
			rx &=~WK2XXX_RFTRIG_IEN;
			rx &=~WK2XXX_RXOUT_IEN;
			wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);
			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,&rx);
			rx &= ~WK2XXX_RXOVT_INT;
			rx &= ~WK2XXX_RFTRIG_INT;
			wk2xxx_write_reg(s->port.iobase,WK2XXX_SIFR,rx);
		}

		if(work_irq_flag)
		{
			wk2xxxirq_app(&s->port);
			s->irq_fail = 1;
		}



	}while (!s->force_end_work && !freezing(current) && \
			(work_irq_flag || work_stop_rx_flag)); 
	/*work_stop_tx_flag || work_tx_empty_flag || work_conf_flag));*/


#ifdef _DEBUG_WK2XXX
	printk("-----exit------- work ------\n");
#endif



	if(s->start_tx_fail)
	{
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
		rx |= WK2XXX_TFTRIG_IEN;
		wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);
		s->start_tx_fail =0;

#ifdef _DEBUG_WK2XXX
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
		printk(KERN_ALERT "wk2xxx_work()----port:%d--sier:0x%x----\n",s->port.iobase,rx);
#endif
	}

	/*if(s->stop_tx_fail)
	  {

	  wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
	  rx &=~WK2XXX_TFTRIG_IEN;
	  wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);
	  wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,&rx);
	  rx &= ~WK2XXX_TFTRIG_INT;
	  wk2xxx_write_reg(s->port.iobase,WK2XXX_SIFR,rx);
	  s->stop_tx_fail =0;

	  }*/

	if(s->stop_rx_fail)
	{
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,&rx);
		rx &=~WK2XXX_RFTRIG_IEN;
		rx &=~WK2XXX_RXOUT_IEN;
		wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,rx);

		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,&rx);
		rx &= ~WK2XXX_RFTRIG_INT;
		rx &= ~WK2XXX_RXOVT_INT;
		wk2xxx_write_reg(s->port.iobase,WK2XXX_SIFR,rx);
		s->stop_rx_fail =0;
	}
	if(s->irq_fail)
	{
		s->irq_fail = 0;
		enable_irq(s->port.irq);
		//s->irq_fail = 0;
	}

#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_work---exit---\n");
#endif






}


static void wk2xxx_rx_chars(struct uart_port *port)//vk32xx_port *port)
{
#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "wk2xxx_rx_chars()---------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t fsr,lsr,dat[1],rx_fifo[16]={0},rx_dat[256]={0};
	unsigned int ch, flg,sifr, ignored=0,status = 0,rx_count=0;
	int rfcnt=0,rx_num=0,i;
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];
	wk2xxx_read_reg(s->port.iobase,WK2XXX_LSR,dat);
	lsr = dat[0];
	wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,dat);
	sifr = dat[0];
#ifdef _DEBUG_WK2XXX1 
	printk(KERN_ALERT "wk2xxx_rx_chars()----port:%d--fsr:%d--lsr:%d--\n",s->port.iobase,fsr,lsr);
#endif
	if(!(sifr&0x80))//no error
	{ 
		flg = TTY_NORMAL;
		if (fsr& WK2XXX_RDAT)
		{
			wk2xxx_read_reg(s->port.iobase,WK2XXX_RFCNT,dat);
			rfcnt=dat[0];
			if(rfcnt==0)
			{
				rfcnt=255;	
			}
#ifdef _DEBUG_WK2XXX

			printk(KERN_ALERT "1wk2xxx_rx_chars()----port:%d--RFCNT:0x%x----\n",s->port.iobase,rfcnt);
#endif

			while(rfcnt>0)
			{
				if(rfcnt>16)
				{
					wk2xxx_read_fifo(s->port.iobase, 16,rx_fifo);
					for(i=0;i<16;i++)
					{
						rx_dat[rx_num]=rx_fifo[i];
						rx_num++;
					}
					rfcnt=rfcnt-16;
				}
				else
				{
					wk2xxx_read_fifo(s->port.iobase, rfcnt,rx_fifo);
					for(i=0;i<rfcnt;i++)
					{
						rx_dat[rx_num]=rx_fifo[i];
						rx_num++;
					}
					rfcnt=0;
				}
			}

			s->port.icount.rx+=rx_num;

			for(i=0;i<rx_num;i++)
			{
				if (uart_handle_sysrq_char(&s->port,rx_dat[rx_num]))//.state, ch))
					break;

#ifdef _DEBUG_WK2XXX 

				printk(KERN_ALERT "rx_chars:0x%x----\n",rx_dat[i]);
#endif
				uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, rx_dat[i], flg);
				rx_count++;

				if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL))
				{
					//tty_flip_buffer_push(s->port.state->port.tty);
					tty_flip_buffer_push(&s->port.state->port);
					rx_count = 0;
				}

			}



			if((rx_count > 0)&&(s->port.state->port.tty != NULL))
			{
#ifdef _DEBUG_WK2XXX
				printk(KERN_ALERT  "push buffer tty flip port = :%d count = :%d\n",s->port.iobase,rx_count);
#endif
				//tty_flip_buffer_push(s->port.state->port.tty);
				tty_flip_buffer_push(&s->port.state->port);
				rx_count = 0;
			}

		}
	}//ifm
	else//error
	{
		while (fsr& WK2XXX_RDAT)/**/
		{
			wk2xxx_read_reg(s->port.iobase,WK2XXX_FDAT,dat);
			ch = (int)dat[0];
#ifdef _DEBUG_WK2XXX1 

			printk(KERN_ALERT "wk2xxx_rx_chars()----port:%d--RXDAT:0x%x----\n",s->port.iobase,ch);
#endif

			s->port.icount.rx++;
			//rx_count++;
#ifdef _DEBUG_WK2XXX1
			printk(KERN_ALERT "icount.rx:%d\n",s->port.icount.rx);
#endif
			flg = TTY_NORMAL;
			if (lsr&(WK2XXX_OE |WK2XXX_FE|WK2XXX_PE|WK2XXX_BI))
			{
				printk(KERN_ALERT "wk2xxx_rx_chars()----port:%lx error,lsr:%x!!!!!!!!!!!!!!!!!\n",s->port.iobase,lsr);
				//goto handle_error;
				if (lsr & WK2XXX_PE)
				{
					s->port.icount.parity++;
					status |= WK2XXX_STATUS_PE;
					flg = TTY_PARITY;
				}
				if (lsr & WK2XXX_FE)
				{
					s->port.icount.frame++;
					status |= WK2XXX_STATUS_FE;
					flg = TTY_FRAME;
				}
				if (lsr & WK2XXX_OE)
				{
					s->port.icount.overrun++;
					status |= WK2XXX_STATUS_OE;
					flg = TTY_OVERRUN;
				}
				if(lsr&fsr & WK2XXX_BI)
				{
					s->port.icount.brk++;
					status |= WK2XXX_STATUS_BRK;
					flg = TTY_BREAK;
				}

				if (++ignored > 100) 
					goto out;

				goto ignore_char;       
			}

error_return:
			if (uart_handle_sysrq_char(&s->port,ch))//.state, ch))
				goto ignore_char;

			uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, ch, flg);
			rx_count++;

			if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL)) 
			{
				//tty_flip_buffer_push(s->port.state->port.tty);
				tty_flip_buffer_push(&s->port.state->port);
				rx_count = 0;
			} 
#ifdef _DEBUG_WK2XXX1
			printk(KERN_ALERT  " s->port.icount.rx = 0x%X char = 0x%X flg = 0x%X port = %d rx_count = %d\n",s->port.icount.rx,ch,flg,s->port.iobase,rx_count);
#endif
ignore_char:

			wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,dat);
			fsr = dat[0];
			wk2xxx_read_reg(s->port.iobase,WK2XXX_LSR,dat);
			lsr = dat[0];
		}
out:
		if((rx_count > 0)&&(s->port.state->port.tty != NULL))
		{
#ifdef _DEBUG_WK2XXX1
			printk(KERN_ALERT  "push buffer tty flip port = :%d count = :%d\n",s->port.iobase,rx_count);
#endif
			//tty_flip_buffer_push(s->port.state->port.tty);
			tty_flip_buffer_push(&s->port.state->port);
			rx_count = 0;
		}

	}//if()else

#if 0
	printk(KERN_ALERT  " rx_num = :%d\n",s->port.icount.rx);
#endif

#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_rx_chars()---------out---\n");
#endif

	return;
#ifdef SUPPORT_SYSRQ
	s->port.state->sysrq = 0;
#endif
	goto error_return;

#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_rx_chars---exit---\n");
#endif

}

static void wk2xxx_tx_chars(struct uart_port *port)//
{
#ifdef _DEBUG_WK2XXX3
	printk("--wk2xxx_tx_chars---in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t uart_empty_flag=0, fsr,tfcnt,dat[1],rx_fifo[16]={0};//txbuf[255]={0};
	int count,tx_count,i;
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	if (s->port.x_char) 
	{
		wk2xxx_write_reg(s->port.iobase,WK2XXX_FDAT,s->port.x_char);
		s->port.icount.tx++;
		s->port.x_char = 0;
		goto out;

	}

	if(uart_circ_empty(&s->port.state->xmit) || uart_tx_stopped(&s->port))
	{

		goto out;

	}

	/*
	 * Tried using FIFO (not checking TNF) for fifo fill:
	 * still had the '1 bytes repeated' problem.
	 */
	wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];

	wk2xxx_read_reg(s->port.iobase,WK2XXX_TFCNT,dat);	
	tfcnt= dat[0];
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_tx_chars   fsr:0x%x,rfcnt:0x%x,port = %x\n",fsr,tfcnt,s->port.iobase);
#endif

	if(tfcnt==0)
	{
		if(fsr & WK2XXX_TFULL)
		{
			tfcnt=255;
			tx_count=0;
		}
		else 
		{
			tfcnt=0;
			tx_count=255;
		}
	}
	else
	{
		tx_count=255-tfcnt;
#ifdef _DEBUG_WK2XXX
		printk(KERN_ALERT "wk2xxx_tx_chars2   tx_count:%x,port = %x\n",tx_count,s->port.iobase);
#endif 
	}



	count = tx_count;
	while(count>0)
	{   

		if(uart_empty_flag)
		{ 
			uart_empty_flag=0;
			break;
		}
		do
		{

			if(uart_circ_empty(&s->port.state->xmit))
			{
				uart_empty_flag=1;
				break;
			}
			rx_fifo[i]=s->port.state->xmit.buf[s->port.state->xmit.tail];
			i++;
			tx_count--;
			s->port.state->xmit.tail = (s->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
			s->port.icount.tx++;
			if(i>=16)
			{ 
				break;
			}
#ifdef _DEBUG_WK2XXX
			printk(KERN_ALERT "xmit.head:%d,xmit.tail:%d,char:%d,fsr:0x%X,port = %d\n",s->port.state->xmit.head,s->port.state->xmit.tail,s->port.state->xmit.buf[s->port.state->xmit.tail],fsr,s->port.iobase);
#endif
		}while(tx_count>0);
		
		//printk(KERN_INFO"write fifo\n");
		wk2xxx_write_fifo(s->port.iobase,i,rx_fifo);
		count=count-i;
		i=0;
	}


#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_tx_chars2   count:%x,port = %x\n",count,s->port.iobase);
#endif



out:wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];
	if(((fsr&WK2XXX_TDAT)==0)&&((fsr&WK2XXX_TBUSY)==0))
	{
		if (uart_circ_chars_pending(&s->port.state->xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);
		if (uart_circ_empty(&s->port.state->xmit))
		{
			wk2xxx_stop_tx(&s->port);
		}
	}
#ifdef _DEBUG_WK2XXX3
	printk("--wk2xxx_tx_chars---exit---\n");
#endif


}

static irqreturn_t wk2xxx_irq(int irq, void *dev_id)//
{
	struct wk2xxx_port *s = dev_id;

#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_irq---in---\n");
#endif

	disable_irq_nosync(s->port.irq);

	s->irq_flag = 1;
	if(wk2xxx_dowork(s))
	{

		//s->irq_flag = 1;

	}
	else
	{
		s->irq_flag = 0;
		s->irq_fail = 1;
	}

#ifdef _DEBUG_WK2XXX
	printk("--wk2xxx_irq---exit---\n");
#endif

	return IRQ_HANDLED;
}

static void wk2xxxirq_app(struct uart_port *port)//
{

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "wk2xxxirq_app()------port:%d--------------\n",s->port.iobase);
#endif
	unsigned int  pass_counter = 0;
	uint8_t sifr,gifr,sier,dat[1];
#ifdef _DEBUG_WK2XXX1		
	uint8_t  gier,sifr0,sifr1,sifr2,sifr3,sier1,sier0,sier2,sier3;
#endif				
	wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GIFR ,dat);
	gifr = dat[0];		
#ifdef _DEBUG_WK2XXX1
	wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GIER ,dat);
	gier = dat[0];
	wk2xxx_write_reg(1,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	wk2xxx_write_reg(2,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	wk2xxx_write_reg(3,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
	wk2xxx_write_reg(4,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0

	wk2xxx_read_reg(1,WK2XXX_SIFR,&sifr0);
	wk2xxx_read_reg(2,WK2XXX_SIFR,&sifr1);
	wk2xxx_read_reg(3,WK2XXX_SIFR,&sifr2);
	wk2xxx_read_reg(4,WK2XXX_SIFR,&sifr3);

	wk2xxx_read_reg(1,WK2XXX_SIER,&sier0);
	wk2xxx_read_reg(2,WK2XXX_SIER,&sier1);
	wk2xxx_read_reg(3,WK2XXX_SIER,&sier2);
	wk2xxx_read_reg(4,WK2XXX_SIER,&sier3);
#endif 		


#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "irq_app....gifr:%x  gier:%x  sier1:%x  sier2:%x sier3:%x sier4:%x   sifr1:%x sifr2:%x sifr3:%x sifr4:%x \n",gifr,gier,sier0,sier1,sier2,sier3,sifr0,sifr1,sifr2,sifr3);
#endif
	switch(s->port.iobase)
	{
		case 1 :
			if(!(gifr & WK2XXX_UT1INT))
			{
				return;
			}
			break;
		case 2 :
			if(!(gifr & WK2XXX_UT2INT))
			{ 			 
				return;
			} 									  
			break;
		case 3 :
			if(!(gifr & WK2XXX_UT3INT))
			{ 			 
				return;
			}
			break;
		case 4 :
			if(!(gifr & WK2XXX_UT4INT))
			{				
				return;
			}
			break;
		default:
			break;

	}

	wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,dat);
	sifr = dat[0];
	wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,dat);
	sier = dat[0];
#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "irq_app..........sifr:%x sier:%x \n",sifr,sier);
#endif
	do {
		if ((sifr&WK2XXX_RFTRIG_INT)||(sifr&WK2XXX_RXOVT_INT))
		{
			wk2xxx_rx_chars(&s->port);
		}

		if ((sifr & WK2XXX_TFTRIG_INT)&&(sier & WK2XXX_TFTRIG_IEN ))
		{
			wk2xxx_tx_chars(&s->port);
			return;
		}
		if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
			break;
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,dat);
		sifr = dat[0];				  
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,dat);
		sier = dat[0];
#ifdef _DEBUG_WK2XXX1
		printk(KERN_ALERT "irq_app...........rx............tx  sifr:%x sier:%x port:%x\n",sifr,sier,s->port.iobase);
#endif
	} while ((sifr & WK2XXX_RXOVT_INT)||(sifr & WK2XXX_RFTRIG_INT)||((sifr & WK2XXX_TFTRIG_INT)&&(sier & WK2XXX_TFTRIG_IEN)));
#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "sifr:%d\n",sifr);
#endif
#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "wk2xxxirq_app()---------exit---\n");
#endif

}


/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */

static u_int wk2xxx_tx_empty(struct uart_port *port)// or query the tx fifo is not empty?
{
	uint8_t rx;


	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX3
	printk(KERN_ALERT "wk2xxx_tx_empty()---------in---\n");
#endif    
	mutex_lock(&wk2xxxs_lock);
	if(!(s->tx_empty_flag || s->tx_empty_fail))
	{
		wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,&rx);

		while((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))
		{
			wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,&rx);
			msleep(1);
		}
		s->tx_empty = ((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))<=0;

		if(s->tx_empty)  
		{
			s->tx_empty_flag =0;
			s->tx_empty_fail=0;
		}
		else
		{
			s->tx_empty_fail=0;
			s->tx_empty_flag =0;
		}
	}
	mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK2XXX3
	printk(KERN_ALERT "wk2xxx_tx_empty----------exit---\n");
	printk(KERN_ALERT "k2xxx_tx_empty----s->tx_empty:0x%X\n",s->tx_empty);
#endif

	return s->tx_empty;


}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)//nothing
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_set_mctrl---------exit---\n");
#endif

}
static u_int wk2xxx_get_mctrl(struct uart_port *port)// since no modem control line
{       
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_get_mctrl---------exit---\n");
#endif

	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}


/*
 *  interrupts disabled on entry
 */

static void wk2xxx_stop_tx(struct uart_port *port)//
{

#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_stop_tx------in---\n");
#endif
	uint8_t dat[1],sier,sifr;
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

	mutex_lock(&wk2xxxs_lock);

	if(!(s->stop_tx_flag||s->stop_tx_fail))
	{
		wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,dat);
		sier=dat[0];
		s->stop_tx_fail=(sier&WK2XXX_TFTRIG_IEN)>0;
		if(s->stop_tx_fail)
		{

			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,dat);
			sier=dat[0];
			sier&=~WK2XXX_TFTRIG_IEN;
			wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,sier);
			wk2xxx_read_reg(s->port.iobase,WK2XXX_SIFR,dat);
			sifr=dat[0];
			sifr&= ~WK2XXX_TFTRIG_INT;
			wk2xxx_write_reg(s->port.iobase,WK2XXX_SIFR,sifr);
			s->stop_tx_fail =0;
			s->stop_tx_flag=0;

		}
		else
		{
			s->stop_tx_fail =0;
			s->stop_tx_flag=0;


		}



	}
	mutex_unlock(&wk2xxxs_lock); 
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_stop_tx------exit---\n");
#endif

}

/*
 *  * interrupts may not be disabled on entry
 */
static void wk2xxx_start_tx(struct uart_port *port)
{	

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);       

#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_start_tx------in---%d,%d\n",s->start_tx_flag,s->start_tx_fail);
#endif

	if(!(s->start_tx_flag||s->start_tx_fail))
	{    s->start_tx_flag = 1;
		if(wk2xxx_dowork(s))
		{
			//s->start_tx_flag = 1;
		}
		else
		{
			s->start_tx_fail = 1;
			s->start_tx_flag = 0;
		}
	}
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_start_tx------exit---\n");
#endif

}

/*
 *  * Interrupts enabled
 */

static void wk2xxx_stop_rx(struct uart_port *port)
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_stop_rx------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

	if(!(s->stop_rx_flag ||s->stop_rx_fail ))
	{
		s->stop_rx_flag = 1;
		if(wk2xxx_dowork(s))
		{
			//s->stop_rx_flag = 1;
		}
		else
		{
			s->stop_rx_flag = 0;
			s->stop_rx_fail = 1;
		}
	}
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_stop_rx------exit---\n");
#endif

}


/*
 *  * No modem control lines
 *   */
static void wk2xxx_enable_ms(struct uart_port *port)    //nothing
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_enable_ms------exit---\n");
#endif

}
/*
 *  * Interrupts always disabled.
 */   
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_break_ctl------exit---\n");
#endif

	//break operation, but there  seems no such operation in vk32  
}


static int wk2xxx_startup(struct uart_port *port)//i
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_startup------in---\n");
#endif


	uint8_t gena,grst,gier,sier,scr,dat[1];
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	char b[12];
	if (s->suspending)
		return 0;

	s->force_end_work = 0;
	sprintf(b, "wk2xxx-%d", (uint8_t)s->port.iobase);
	s->workqueue = create_singlethread_workqueue(b);

	if (!s->workqueue) 
	{
		//dev_warn(&s->spi_wk->dev, "cannot create workqueue\n");
		return -EBUSY;
	}

	INIT_WORK(&s->work, wk2xxx_work);

	if (s->wk2xxx_hw_suspend)
		s->wk2xxx_hw_suspend(0);

	wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GENA,dat);
	gena=dat[0];

	switch (s->port.iobase)
	{
		case 1:
			gena|=WK2XXX_UT1EN;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		case 2:
			gena|=WK2XXX_UT2EN;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		case 3:
			gena|=WK2XXX_UT3EN;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		case 4:
			gena|=WK2XXX_UT4EN;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GENA,gena);
			break;
		default:
			printk(KERN_ALERT ":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
	}

	wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GRST,dat);
	grst=dat[0];
	switch (s->port.iobase)
	{
		case 1:
			grst|=WK2XXX_UT1RST;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		case 2:
			grst|=WK2XXX_UT2RST;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		case 3:
			grst|=WK2XXX_UT3RST;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		case 4:
			grst|=WK2XXX_UT4RST;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GRST,grst);
			break;
		default:
			printk(KERN_ALERT ":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
	}

	wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER,dat);
	sier = dat[0];
	sier &= ~WK2XXX_TFTRIG_IEN;
	sier |= WK2XXX_RFTRIG_IEN;
	sier |= WK2XXX_RXOUT_IEN;
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER,sier);

	wk2xxx_read_reg(s->port.iobase,WK2XXX_SCR,dat);
	scr = dat[0] | WK2XXX_TXEN|WK2XXX_RXEN;
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SCR,scr);

	//initiate the fifos
	wk2xxx_write_reg(s->port.iobase,WK2XXX_FCR,0xff);//initiate the fifos
	wk2xxx_write_reg(s->port.iobase,WK2XXX_FCR,0xfc);
	//set rx/tx interrupt 
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE,1);	
	wk2xxx_write_reg(s->port.iobase,WK2XXX_RFTL,0X40);	//rx 64
	wk2xxx_write_reg(s->port.iobase,WK2XXX_TFTL,0X20);	//tx 32
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE,0);	
	//enable the sub port interrupt
	wk2xxx_read_reg(WK2XXX_GPORT,WK2XXX_GIER,dat);
	gier = dat[0];

	switch (s->port.iobase){
		case 1:
			gier|=WK2XXX_UT1IE;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		case 2:
			gier|=WK2XXX_UT2IE;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		case 3:
			gier|=WK2XXX_UT3IE;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		case 4:
			gier|=WK2XXX_UT4IE;
			wk2xxx_write_reg(WK2XXX_GPORT,WK2XXX_GIER,gier);
			break;
		default:
			printk(KERN_ALERT ": bad iobase %d\n", (uint8_t)s->port.iobase);
	}

	if (s->wk2xxx_hw_suspend){
		s->wk2xxx_hw_suspend(0);
	}
	msleep(50);


	uart_circ_clear(&s->port.state->xmit);
	wk2xxx_enable_ms(&s->port);
	// request irq 
	if(request_irq(s->port.irq, wk2xxx_irq,IRQF_SHARED|IRQF_TRIGGER_HIGH,"wk2xxxuart", s) < 0)
	{
		//dev_warn(&s->spi_wk->dev, "cannot allocate irq %d\n", s->irq);
		s->port.irq = 0;
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
		return -EBUSY;
	}       udelay(100);
	udelay(100);
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_startup------exit---\n");
#endif

	return 0;
}
//* Power down all displays on reboot, poweroff or halt *

static void wk2xxx_shutdown(struct uart_port *port)//
{

#ifdef _DEBUG_WK2XXX3
	printk(KERN_ALERT "-wk2xxx_shutdown------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	if (s->suspending)
		return;
	s->force_end_work = 1;
	if (s->workqueue) 
	{
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}

	if (s->port.irq)
	{
		//disable_irq_nosync(s->port.irq);		
		free_irq(s->port.irq,s);
	}
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-wk2xxx_shutdown-----exit---\n");
#endif

}

static void conf_wk2xxx_subport(struct uart_port *port)//i
{   
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-conf_wk2xxx_subport------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	uint8_t old_sier,lcr,scr,scr_ss,dat[1],baud0_ss,baud1_ss,pres_ss;


	lcr = s->new_lcr;
	scr_ss = s->new_scr;
	baud0_ss=s->new_baud0;
	baud1_ss=s->new_baud1;
	pres_ss=s->new_pres;
	wk2xxx_read_reg(s->port.iobase,WK2XXX_SIER ,dat);
	old_sier = dat[0];
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER ,old_sier&(~(WK2XXX_TFTRIG_IEN | WK2XXX_RFTRIG_IEN|WK2XXX_RXOUT_IEN)));
	//local_irq_restore(flags);
	do{
		wk2xxx_read_reg(s->port.iobase,WK2XXX_FSR,dat);
		//ssr = dat[0];
	} while (dat[0] & WK2XXX_TBUSY);
	// then, disable everything 
	wk2xxx_read_reg(s->port.iobase,WK2XXX_SCR,dat);
	scr = dat[0];


	wk2xxx_write_reg(s->port.iobase,WK2XXX_SCR ,scr&(~(WK2XXX_RXEN|WK2XXX_TXEN)));
	// set the parity, stop bits and data size //
	wk2xxx_write_reg(s->port.iobase,WK2XXX_LCR ,lcr);
	// set the baud rate //
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SIER ,old_sier);
	// set the baud rate //
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE ,1);
	wk2xxx_write_reg(s->port.iobase,WK2XXX_BAUD0 ,baud0_ss);
	wk2xxx_write_reg(s->port.iobase,WK2XXX_BAUD1 ,baud1_ss);
	wk2xxx_write_reg(s->port.iobase,WK2XXX_PRES ,pres_ss);
#ifdef _DEBUG_WK2XXX2
	wk2xxx_read_reg(s->port.iobase,WK2XXX_BAUD0,dat);
	printk(KERN_ALERT ":WK2XXX_BAUD0=0x%X\n", dat[0]);
	wk2xxx_read_reg(s->port.iobase,WK2XXX_BAUD1,dat);
	printk(KERN_ALERT ":WK2XXX_BAUD1=0x%X\n", dat[0]);
	wk2xxx_read_reg(s->port.iobase,WK2XXX_PRES,dat);
	printk(KERN_ALERT ":WK2XXX_PRES=0x%X\n", dat[0]);
#endif
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SPAGE ,0);

	udelay(10);
	wk2xxx_write_reg(s->port.iobase,WK2XXX_SCR ,scr|(WK2XXX_RXEN|WK2XXX_TXEN));


#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-conf_wk2xxx_subport------exit---\n");
#endif

}


// change speed
static void wk2xxx_termios( struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-vk32xx_termios------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	int baud = 0;
	uint8_t lcr,baud1,baud0,pres;
	unsigned short cflag;
	unsigned short lflag;

	cflag = termios->c_cflag;
	lflag = termios->c_lflag;
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "cflag := 0x%X  lflag : = 0x%X\n",cflag,lflag);
#endif
	baud1=0;
	baud0=0;
	pres=0;
	baud = tty_termios_baud_rate(termios);

	
	//sysclk=11.0592MHZ
	//baud =115200;
	printk(KERN_INFO"speed=%d\n",baud);
	switch (baud) {
		case 600:
			baud1=0x4;
			baud0=0x7f;
			pres=0;
			break;
		case 1200:
			baud1=0x2;
			baud0=0x3F;
			pres=0;
			break;
		case 2400:
			baud1=0x1;
			baud0=0x1f;
			pres=0;
			break;
		case 4800:
			baud1=0x00;
			baud0=0x8f;
			pres=0;
			break;
		case 9600:
			baud1=0x00;
			baud0=0x47;
			pres=0;
			break;
		case 19200:
			baud1=0x00;
			baud0=0x23;
			pres=0;
			break;
		case 38400:
			baud1=0x00;
			baud0=0x11;
			pres=0;
			break;
		case 76800:
			baud1=0x00;
			baud0=0x08;
			pres=0;
			break;	

		case 1800:
			baud1=0x01;
			baud0=0x7f;
			pres=0;
			break;
		case 3600:
			baud1=0x00;
			baud0=0xbf;
			pres=0;
			break;
		case 7200:
			baud1=0x00;
			baud0=0x5f;
			pres=0;
			break;
		case 14400:
			baud1=0x00;
			baud0=0x2f;
			pres=0;
			break;
		case 28800:
			baud1=0x00;
			baud0=0x17;
			pres=0;
			break;
		case 57600:
			baud1=0x00;
			baud0=0x0b;
			pres=0;
			break;
		case 115200:
			baud1=0x00;
			baud0=0x05;
			pres=0;
			break;
		case 230400:
			baud1=0x00;
			baud0=0x02;
			pres=0;
			break;
		default:
			baud1=0x00;
			baud0=0x00;
			pres=0;
	}
	tty_termios_encode_baud_rate(termios, baud, baud);

	/* we are sending char from a workqueue so enable */

#ifdef _DEBUG_WK2XXX

	printk(KERN_ALERT "wk2xxx_termios()----port:%d--lcr:0x%x- cflag:0x%x-CSTOPB:0x%x,PARENB:0x%x,PARODD:0x%x--\n",s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif

	lcr =0;
	if (cflag & CSTOPB)
		lcr|=WK2XXX_STPL;//two  stop_bits
	else
		lcr&=~WK2XXX_STPL;//one  stop_bits

	if (cflag & PARENB) {
		lcr|=WK2XXX_PAEN;//enbale spa
		if (!(cflag & PARODD)){

			lcr |= WK2XXX_PAM1;
			lcr &= ~WK2XXX_PAM0;
		}
		else{
			lcr |= WK2XXX_PAM0;//PAM0=1
			lcr &= ~WK2XXX_PAM1;//PAM1=0

		}
	}
	else{
		lcr&=~WK2XXX_PAEN;
	}

#ifdef _DEBUG_WK2XXX

	printk(KERN_ALERT "wk2xxx_termios()----port:%d--lcr:0x%x- cflag:0x%x-CSTOPB:0x%x,PARENB:0x%x,PARODD:0x%x--\n",s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif

	s->new_baud1=baud1;
	s->new_baud0=baud0;
	s->new_pres=pres;
	s->new_lcr = lcr;


#if 1 // simon change
	conf_wk2xxx_subport(&s->port);

#else
	if(!(s->conf_flag|| s->conf_fail))
	{
		if(wk2xxx_dowork(s))
		{
			s->conf_flag =1;
		}
		else
		{
			s->conf_fail =1;
		}

	}
#endif
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "-vk32xx_termios------exit---\n");
#endif

	return ;

}


static const char *wk2xxx_type(struct uart_port *port)
{


#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_type-------------out-------- \n");
#endif
	return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;//this is defined in serial_core.h
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void wk2xxx_release_port(struct uart_port *port)
{
	printk(KERN_ALERT "wk2xxx_release_port\n");

}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int wk2xxx_request_port(struct uart_port *port)//no such memory region needed for vk32
{
	printk(KERN_ALERT "wk2xxx_request_port\n");
	return 0;
}

/*
 * Configure/autoconfigure the port*/
static void wk2xxx_config_port(struct uart_port *port, int flags)
{
	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_config_port \n");
#endif

	if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
		s->port.type = PORT_WK2XXX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_vk32xx and PORT_UNKNOWN
 */
static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{

#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_verify_port \n");
#endif
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_PORT)
		ret = -EINVAL;
	//if (port->uartclk / 16 != ser->baud_base)//?a??2?猫路?篓
	//      ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}




static struct uart_ops wk2xxx_pops = {
tx_empty:       wk2xxx_tx_empty,
				set_mctrl:      wk2xxx_set_mctrl,
				get_mctrl:      wk2xxx_get_mctrl,
				stop_tx:        wk2xxx_stop_tx,
				start_tx:       wk2xxx_start_tx,
				stop_rx:        wk2xxx_stop_rx,
				enable_ms:      wk2xxx_enable_ms,
				break_ctl:      wk2xxx_break_ctl,
				startup:        wk2xxx_startup,
				shutdown:       wk2xxx_shutdown,
				set_termios:    wk2xxx_termios,
				type:           wk2xxx_type,
				release_port:   wk2xxx_release_port,
				request_port:   wk2xxx_request_port,
				config_port:    wk2xxx_config_port,
				verify_port:    wk2xxx_verify_port,

};
static struct uart_driver wk2xxx_uart_driver = {


owner:                  THIS_MODULE,
						major:        		 SERIAL_WK2XXX_MAJOR,
#ifdef CONFIG_DEVFS_FS
						driver_name:            "ttySWK",
						dev_name:               "ttysWK",
#else
						driver_name:            "ttySWK",
						dev_name:               "ttysWK",
#endif
						minor:                  MINOR_START,
						nr:                     NR_PORTS,
						cons:                   NULL//WK2Xxx_CONSOLE,
};

static int uart_driver_registered;


/////////////////////////////////////////////////////////////
static int __init wk2xxx_init(void)
{
	uint8_t i,dat[1];
	int status;

//#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "wk2xxx_init()-------------in-------- \n");
	printk(KERN_ALERT "gpio1_io00 irq %d\n",gpio_to_irq(0)); //gpiox_ioy  (32 * (x - 1)) + y = 0
//#endif


	wk_s5pv210uart_InitIO();/*初始化S5PV210的uart相应的端口信息*/

	mutex_lock(&wk2xxxs_lock);
	if(!uart_driver_registered)
	{
		uart_driver_registered = 1;
		status=uart_register_driver(&wk2xxx_uart_driver);
		if (status)
		{
			printk(KERN_ERR "Couldn't register wk2xxx uart driver\n");
			mutex_unlock(&wk2xxxs_lock);
			return status;
		}
	}
	printk(KERN_ALERT "wk2xxx_serial_init()\n");
	for(i =0;i<NR_PORTS;i++)
	{
		struct wk2xxx_port *s = &wk2xxxs[i];//container_of(port,struct vk32xx_port,port); 	
		s->tx_done =0;
		//s->spi_wk    = spi;
		s->port.line = i;
		s->port.ops = &wk2xxx_pops;
		s->port.uartclk = WK_CRASTAL_CLK;
		s->port.fifosize = 64;
		s->port.iobase = i+1;
		s->port.irq    = IRQ_WK2XXX;
		s->port.iotype = SERIAL_IO_PORT;
		s->port.flags  = ASYNC_BOOT_AUTOCONF;
		//s->minor       = i;
		status = uart_add_one_port(&wk2xxx_uart_driver, &s->port);
		if(status<0)
		{
			//dev_warn(&spi->dev,"uart_add_one_port failed for line i:= %d with error %d\n",i,status);
			printk(KERN_ALERT "uart_add_one_port failed for line i:= %d with error %d\n",i,status);
		}
	}
	printk(KERN_ALERT "uart_add_one_port = 0x%d\n",status);
	mutex_unlock(&wk2xxxs_lock);
	printk(KERN_ALERT "uart_add_one_port = 0x%d\n",status);
	udelay(100);

	/*发送0x55实现wk和CPU的波特率自动匹配并验证是否匹配成功*/
	i=3;
	while(i--){
		s5pv_uart_wr(0x55);
		udelay(100);
		udelay(100);
		udelay(100);
	}

#if 1
	wk2xxx_read_reg(1,WK2XXX_GENA,dat);
	printk(KERN_ALERT"WK2XXX_GENA1=:%x\n",dat[0]);

#endif
//#ifdef _DEBUG_WK2XXX1
	i=5;
	while(i--)
	{
		dat[0] = 0;
		wk2xxx_read_reg(1,WK2XXX_GENA,dat);
		printk(KERN_ALERT"WK2XXX_GENA=:%x\n",dat[0]);
		wk2xxx_write_reg(1,WK2XXX_GENA,i);
	}
//#endif	

#ifdef _DEBUG_WK2XXX1
	printk(KERN_ALERT "wk2xxx_init()-------------out-------- \n");
#endif

	return 0;
}


static void __exit wk2xxx_exit(void)
{
	uint8_t i;
	for(i =0;i<NR_PORTS;i++)
	{
		struct wk2xxx_port *s = &wk2xxxs[i];
		uart_remove_one_port(&wk2xxx_uart_driver, &s->port);
	}
	/*if(s5pv210uart_resource!=NULL)
	{
		iounmap((void*)virt);
		iounmap(gpa1con_addr);
		release_mem_region(S5PV210_UART3,0x10000);
	}*/

	//if(fd > 0) 
	//	sys_close(fd);
	if(!IS_ERR(filp))
	{
		filp_close(filp,NULL);
	}             


	printk(KERN_ALERT "removing wk2xxx ports\n");
	uart_unregister_driver(&wk2xxx_uart_driver);
	printk("TEST_REG:quit ");

}

module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR("WKIC Ltd");
MODULE_DESCRIPTION("wk2xxx generic serial port driver");
MODULE_LICENSE("GPL");






