###################################################
# Makefile for the NSS data plane driver
###################################################

obj ?= .

obj-m += qca-nss-dp.o

qca-nss-dp-objs += edma_data_plane.o \
		   nss_dp_attach.o \
		   nss_dp_main.o \
		   rumi_test.o

NSS_DP_INCLUDE = -I$(obj)/include -I$(obj)/exports

ccflags-y += $(NSS_DP_INCLUDE)
#ccflags-y += -DRUMI_BRIDGED_FLOW
