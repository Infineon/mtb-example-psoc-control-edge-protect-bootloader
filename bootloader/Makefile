################################################################################
# \file Makefile
# \version 1.2
#
# \brief
# Top-level make file for the bootloader-solution project.
#
################################################################################
# \copyright
# Copyright 2019-2024, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation. All rights reserved.
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

include ../common.mk

################################################################################
# Application configuration
################################################################################

MTB_TYPE=PROJECT

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=bootloader

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

# Add additional defines to the build process (without a leading -D).
DEFINES=

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=softfloat

# Treat all compilation warnings as errors by default
WARN_AS_ERR?=1

# Additional / custom C compiler flags.
ifeq ($(WARN_AS_ERR), 1)
    ifeq ($(TOOLCHAIN), GCC_ARM)
    WERR_FLAGS=-Werror -pedantic-errors
    endif

    ifeq ($(TOOLCHAIN), ARM)
    WERR_FLAGS=-Werror
    endif

    ifeq ($(TOOLCHAIN), IAR)
    WERR_FLAGS=--warnings_are_errors
    endif

    CFLAGS += $(WERR_FLAGS)
endif

ifeq ($(TOOLCHAIN), IAR)
CFLAGS+=--vla --dlib_config normal --diag_suppress Pe550,Pe546,Pe177,Pe111,Pa082 $(IAR_FLAGS)
endif

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=

################################################################################
# Paths
################################################################################

# Folders to ignore
IFX_MBEDTLS_IGNORE=$(SEARCH_ifx-mbedtls)/3rdparty $(SEARCH_ifx-mbedtls)/configs $(SEARCH_ifx-mbedtls)/doxygen \
			          $(SEARCH_ifx-mbedtls)/programs $(SEARCH_ifx-mbedtls)/scripts $(SEARCH_ifx-mbedtls)/tests \
			          $(SEARCH_ifx-mbedtls)/visualc
			          
MBEDTLS_IGNORE=$(SEARCH_mbedtls)/3rdparty $(SEARCH_mbedtls)/configs $(SEARCH_mbedtls)/doxygen \
		          $(SEARCH_mbedtls)/programs $(SEARCH_mbedtls)/scripts $(SEARCH_mbedtls)/tests \
		          $(SEARCH_mbedtls)/visualc 

CY_IGNORE=$(IFX_MBEDTLS_IGNORE) $(MBEDTLS_IGNORE) $(SEARCH_mcuboot) $(SEARCH_cy-mbedtls-acceleration)\
          $(SEARCH_se-rt-services-utils)\
          cy_serial_flash_prog.c

CY_GETLIBS_DEPS_PATH+=../platforms/$(PLATFORM)/deps/bootloader

include ./app.mk
include $(CY_TOOLS_DIR)/make/start.mk
