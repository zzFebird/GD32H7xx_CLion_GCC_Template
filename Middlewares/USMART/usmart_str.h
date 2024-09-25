/**
 ****************************************************************************************************					 
 * @file        usmart_str.h
 * @version     V1.0
 * @brief       USMART ���ڵ������ 
 ****************************************************************************************************
 */	

#ifndef __USMART_STR_H
#define __USMART_STR_H

#include "./USMART/usmart_port.h"


uint8_t usmart_get_parmpos(uint8_t num);                /* �õ�ĳ�������ڲ������������ʼλ�� */
uint8_t usmart_strcmp(char *str1, char *str2);          /* �Ա������ַ����Ƿ���� */
uint32_t usmart_pow(uint8_t m, uint8_t n);              /* M^N�η� */
uint8_t usmart_str2num(char *str, uint32_t *res);       /* �ַ���תΪ���� */
uint8_t usmart_get_cmdname(char *str, char *cmdname, uint8_t *nlen, uint8_t maxlen); /* ��str�еõ�ָ����,������ָ��� */
uint8_t usmart_get_fname(char *str, char *fname, uint8_t *pnum, uint8_t *rval); /* ��str�еõ������� */
uint8_t usmart_get_aparm(char *str, char *fparm, uint8_t *ptype); /* ��str�еõ�һ���������� */
uint8_t usmart_get_fparam(char *str, uint8_t *parn); /* �õ�str�����еĺ�������. */

#endif











