#**************************  M a k e f i l e ********************************
#  
#         Author: ts
#  
#    Description: makefile descriptor for 16z077/87 Ethernet IP Core,
#                 swapped Variant
#                      
#-----------------------------------------------------------------------------
#   Copyright 2007-2019, MEN Mikro Elektronik GmbH
#*****************************************************************************
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

MAK_NAME=lx_z77_sw
# the next line is updated during the MDIS installation
STAMPED_REVISION="13Z077-90_02_02-8-g8014f0c-dirty_2019-05-30"

DEF_REVISION=MAK_REVISION=$(STAMPED_REVISION)

MAK_LIBS=

MAK_INCL= $(MEN_INC_DIR)/men_typs.h   \
          $(MEN_INC_DIR)/oss.h        \
          $(MEN_INC_DIR)/dbg.h        \
          $(MEN_INC_DIR)/mdis_err.h   \
          $(MEN_INC_DIR)/men_chameleon.h \
          $(MEN_INC_DIR)/sysparam2.h \

MAK_OPTIM=$(OPT_1)

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED \
		$(SW_PREFIX)$(DEF_REVISION) \
		   $(SW_PREFIX)MAC_BYTESWAP   \
		   $(SW_PREFIX)ID_SW

MAK_INP1=men_16z077_eth$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
