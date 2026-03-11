# Makefile for Top application
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += motorZynqApp
motorZynqApp_DEPEND_DIRS = configure
include $(TOP)/configure/RULES_TOP
