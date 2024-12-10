################################################################################
# \file common.mk
# \version 1.0
#
# \brief
# Settings shared across all projects.
#
################################################################################
# \copyright
# Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


MTB_TYPE=PROJECT

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=KIT_PSC3M5_EVK

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

SECURE_SRAM_SIZE=0x4000
SECURE_FLASH_SIZE=0x10000

ifeq ($(TOOLCHAIN),IAR)
LDFLAGS+=--config_def _size_S_SRAM=$(SECURE_SRAM_SIZE) --config_def _size_S_FLASH=$(SECURE_FLASH_SIZE)
else ifeq ($(TOOLCHAIN),GCC_ARM)
LDFLAGS+=-Wl,--defsym=_size_S_SRAM=$(SECURE_SRAM_SIZE),--defsym=_size_S_FLASH=$(SECURE_FLASH_SIZE)
else ifeq ($(TOOLCHAIN),ARM)
LDFLAGS+=--predefine="-D_size_S_SRAM=$(SECURE_SRAM_SIZE)" --predefine="-D_size_S_FLASH=$(SECURE_FLASH_SIZE)"
endif

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
# 
# If CONFIG is manually edited, ensure to update or regenerate launch configurations 
# for your IDE.
CONFIG=Debug

# Enable logging in application
# ENABLE_S_LOG -- Enable logging in secure project 
# ENABLE_NS_LOG -- Enable logging in non-secure project
# ENABLE_S_NS_LOG -- Enable logging in both secure and non-secure project. An additional miniprog is required for it.
DEFINES+=ENABLE_S_NS_LOG

include ../common_app.mk
