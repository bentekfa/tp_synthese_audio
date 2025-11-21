/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */
/* =====================================SHELL SANS STRUCTURE !!!!==========================================*/
/*#include "shell.h"

#include <stdio.h>

#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"


extern uint8_t rxbuffer;
extern SemaphoreHandle_t uartRxSemaphore;

typedef struct{
	char c;
	int (* func)(int argc, char ** argv);
	char * description;
} shell_func_t;

static int shell_func_list_size = 0;
static shell_func_t shell_func_list[SHELL_FUNC_LIST_MAX_SIZE];

static char print_buffer[BUFFER_SIZE];

static char uart_read() {
	//char c;

	HAL_UART_Receive_IT(&huart2, &rxbuffer, 1);
    xSemaphoreTake(uartRxSemaphore, HAL_MAX_DELAY);

	return (char)rxbuffer;
}

static int uart_write(char * s, uint16_t size) {
	HAL_UART_Transmit(&UART_DEVICE, (uint8_t*)s, size, HAL_MAX_DELAY);
	return size;
}

static int sh_help(int argc, char ** argv) {
	int i;
	for(i = 0 ; i < shell_func_list_size ; i++) {
		int size;
		size = snprintf (print_buffer, BUFFER_SIZE, "%c: %s\r\n", shell_func_list[i].c, shell_func_list[i].description);
		uart_write(print_buffer, size);
	}

	return 0;
}

void shell_init() {
	int size = 0;

	size = snprintf (print_buffer, BUFFER_SIZE, "\r\n\r\n===== Monsieur Shell v0.2 =====\r\n");
	uart_write(print_buffer, size);

	shell_add('h', sh_help, "Help");
}

int shell_add(char c, int (* pfunc)(int argc, char ** argv), char * description) {
	if (shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		shell_func_list[shell_func_list_size].c = c;
		shell_func_list[shell_func_list_size].func = pfunc;
		shell_func_list[shell_func_list_size].description = description;
		shell_func_list_size++;
		return 0;
	}

	return -1;
}

static int shell_exec(char * buf) {
	int i;

	char c = buf[0];

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	for(i = 0 ; i < shell_func_list_size ; i++) {
		if (shell_func_list[i].c == c) {
			argc = 1;
			argv[0] = buf;

			for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return shell_func_list[i].func(argc, argv);
		}
	}

	int size;
	size = snprintf (print_buffer, BUFFER_SIZE, "%c: no such command\r\n", c);
	uart_write(print_buffer, size);
	return -1;
}

static char backspace[] = "\b \b";
static char prompt[] = "> ";

int shell_run() {
	int reading = 0;
	int pos = 0;

	static char cmd_buffer[BUFFER_SIZE];

	while (1) {
		uart_write(prompt, 2);
		reading = 1;

		while(reading) {
			char c = uart_read();
			int size;

			switch (c) {
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (print_buffer, BUFFER_SIZE, "\r\n");
				uart_write(print_buffer, size);
				cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (print_buffer, BUFFER_SIZE, ":%s\r\n", cmd_buffer);
				uart_write(print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					uart_write(backspace, 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					uart_write(&c, 1);
					cmd_buffer[pos++] = c; //store
				}
			}
		}
		shell_exec(cmd_buffer);
	}
	return 0;
}*/
/* =====================================SHELL AVEC STRUCTURE !!!!==========================================*/
/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"

#include <stdio.h>

static int sh_help(h_shell_t * h_shell, int argc, char ** argv) {
	int i;
	for(i = 0 ; i < h_shell->func_list_size ; i++) {
		int size;
		size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: %s\r\n", h_shell->func_list[i].c, h_shell->func_list[i].description);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}

	return 0;
}

void shell_init(h_shell_t * h_shell) {
	int size = 0;

	h_shell->func_list_size = 0;

	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n\r\n===== Monsieur Shell v0.2 =====\r\n");
	h_shell->drv.transmit(h_shell->print_buffer, size);

	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "TP Synthèse Audio \r\n");
	h_shell->drv.transmit(h_shell->print_buffer, size);
	shell_add(h_shell, 'h', sh_help, "Help");
}

int shell_add(h_shell_t * h_shell, char c, shell_func_pointer_t pfunc, char * description) {
	if (h_shell->func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		h_shell->func_list[h_shell->func_list_size].c = c;
		h_shell->func_list[h_shell->func_list_size].func = pfunc;
		h_shell->func_list[h_shell->func_list_size].description = description;
		h_shell->func_list_size++;
		return 0;
	}

	return -1;
}

static int shell_exec(h_shell_t * h_shell, char * buf) {
	int i;

	char c = buf[0];

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	for(i = 0 ; i < h_shell->func_list_size ; i++) {
		if (h_shell->func_list[i].c == c) {
			argc = 1;
			argv[0] = buf;

			for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return h_shell->func_list[i].func(h_shell, argc, argv);
		}
	}

	int size;
	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: no such command\r\n", c);
	h_shell->drv.transmit(h_shell->print_buffer, size);
	return -1;
}

static const char backspace[] = "\b \b";
static const char prompt[] = "> ";

int shell_run(h_shell_t * h_shell) {
	int reading = 0;
	int pos = 0;

	while (1) {
		h_shell->drv.transmit(prompt, 2);
		reading = 1;

		while(reading) {
			char c;
			h_shell->drv.receive(&c, 1);
			int size;

			switch (c) {
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n");
				h_shell->drv.transmit(h_shell->print_buffer, size);
				h_shell->cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, ":%s\r\n", h_shell->cmd_buffer);
				h_shell->drv.transmit(h_shell->print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					h_shell->drv.transmit(backspace, 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					h_shell->drv.transmit(&c, 1);
					h_shell->cmd_buffer[pos++] = c; //store
				}
			}
		}
		shell_exec(h_shell, h_shell->cmd_buffer);
	}
	return 0;
}
