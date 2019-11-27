/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes nrf52840 compile-time configuration constants
 *   for OpenThread.
 */

#ifndef OPENTHREAD_CORE_NRF52840_CONFIG_H_
#define OPENTHREAD_CORE_NRF52840_CONFIG_H_

/*
 * The GNU Autoconf system defines a PACKAGE macro which is the name
 * of the software package. This name collides with PACKAGE field in
 * the nRF52 Factory Information Configuration Registers (FICR)
 * structure.
 */
#undef PACKAGE

/**
 * @def OPENTHREAD_CONFIG_LOG_OUTPUT
 *
 * The nrf52840 platform provides an otPlatLog() function.
 */
#ifndef OPENTHREAD_CONFIG_LOG_OUTPUT
#define OPENTHREAD_CONFIG_LOG_OUTPUT OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED
#endif

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_INFO
 *
 * The platform-specific string to insert into the OpenThread version string.
 *
 */
#ifndef OPENTHREAD_CONFIG_PLATFORM_INFO
#define OPENTHREAD_CONFIG_PLATFORM_INFO "NRF52840"
#endif

/**
 * @def OPENTHREAD_CONFIG_STACK_VENDOR_OUI
 *
 * The Organizationally Unique Identifier for the vendor.
 *
 */
#ifndef OPENTHREAD_CONFIG_STACK_VENDOR_OUI
#define OPENTHREAD_CONFIG_STACK_VENDOR_OUI 0xf4ce36
#endif

/**
 * @def OPENTHREAD_CONFIG_MLE_MAX_CHILDREN
 *
 * The maximum number of children.
 *
 */
#ifndef OPENTHREAD_CONFIG_MLE_MAX_CHILDREN
#define OPENTHREAD_CONFIG_MLE_MAX_CHILDREN 32
#endif

/**
 * @def OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS
 *
 * The number of message buffers in the buffer pool.
 *
 */
#ifndef OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS
#define OPENTHREAD_CONFIG_NUM_MESSAGE_BUFFERS 160
#endif

/**
 * @def OPENTHREAD_CONFIG_MLE_IP_ADDRS_PER_CHILD
 *
 * The maximum number of supported IPv6 address registrations per child.
 *
 */
#ifndef OPENTHREAD_CONFIG_MLE_IP_ADDRS_PER_CHILD
#define OPENTHREAD_CONFIG_MLE_IP_ADDRS_PER_CHILD 6
#endif

/**
 * @def OPENTHREAD_CONFIG_MAX_STATECHANGE_HANDLERS
 *
 * The maximum number of state-changed callback handlers (set using `otSetStateChangedCallback()`).
 *
 */
#ifndef OPENTHREAD_CONFIG_MAX_STATECHANGE_HANDLERS
#define OPENTHREAD_CONFIG_MAX_STATECHANGE_HANDLERS 3
#endif

/**
 * @def OPENTHREAD_CONFIG_TMF_ADDRESS_CACHE_ENTRIES
 *
 * The number of EID-to-RLOC cache entries.
 *
 */
#ifndef OPENTHREAD_CONFIG_TMF_ADDRESS_CACHE_ENTRIES
#define OPENTHREAD_CONFIG_TMF_ADDRESS_CACHE_ENTRIES 32
#endif

/**
 * @def OPENTHREAD_CONFIG_LOG_PREPREND_LEVEL
 *
 * Define to prepend the log level to all log messages
 *
 */
#ifndef OPENTHREAD_CONFIG_LOG_PREPEND_LEVEL
#define OPENTHREAD_CONFIG_LOG_PREPEND_LEVEL 0
#endif

/**
 * @def OPENTHREAD_CONFIG_MAC_SOFTWARE_ACK_TIMEOUT_ENABLE
 *
 * Define to 1 if you want to enable software ACK timeout logic.
 *
 */
#ifndef OPENTHREAD_CONFIG_MAC_SOFTWARE_ACK_TIMEOUT_ENABLE
#define OPENTHREAD_CONFIG_MAC_SOFTWARE_ACK_TIMEOUT_ENABLE 0
#endif

/**
 * @def OPENTHREAD_CONFIG_MAC_SOFTWARE_RETRANSMIT_ENABLE
 *
 * Define to 1 if you want to enable software retransmission logic.
 *
 */
#ifndef OPENTHREAD_CONFIG_MAC_SOFTWARE_RETRANSMIT_ENABLE
#define OPENTHREAD_CONFIG_MAC_SOFTWARE_RETRANSMIT_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_MAC_SOFTWARE_CSMA_BACKOFF_ENABLE
 *
 * Define to 1 if you want to enable software CSMA-CA backoff logic.
 *
 */
#ifndef OPENTHREAD_CONFIG_MAC_SOFTWARE_CSMA_BACKOFF_ENABLE
#define OPENTHREAD_CONFIG_MAC_SOFTWARE_CSMA_BACKOFF_ENABLE 0
#endif

/**
 * @def OPENTHREAD_CONFIG_MAC_SOFTWARE_TX_SECURITY_ENABLE
 *
 * Define to 1 if you want to enable software transmission security logic.
 *
 */
#ifndef OPENTHREAD_CONFIG_MAC_SOFTWARE_TX_SECURITY_ENABLE
#define OPENTHREAD_CONFIG_MAC_SOFTWARE_TX_SECURITY_ENABLE (OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2)
#endif

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
 *
 * Define to 1 if you want to support microsecond timer in platform.
 *
 */
#ifndef OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
#define OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE
 *
 * Define to 1 to enable otPlatFlash* APIs to support non-volatile storage.
 *
 * When defined to 1, the platform MUST implement the otPlatFlash* APIs instead of the otPlatSettings* APIs.
 *
 */
#ifndef OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE
#define OPENTHREAD_CONFIG_PLATFORM_FLASH_API_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE
 *
 * The size of heap buffer when DTLS is enabled.
 *
 */
#ifndef OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE
#define OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE (4096 * sizeof(void *))
#endif

/**
 * @def OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE_NO_DTLS
 *
 * The size of heap buffer when DTLS is disabled.
 *
 */
#ifndef OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE_NO_DTLS
#define OPENTHREAD_CONFIG_HEAP_INTERNAL_SIZE_NO_DTLS 2048
#endif

/**
 * @def OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
 *
 * Define as 1 to enable the time synchronization service feature.
 *
 */
#ifndef OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
#define OPENTHREAD_CONFIG_TIME_SYNC_ENABLE 0
#endif

/**
 * @def OPENTHREAD_CONFIG_CLI_TX_BUFFER_SIZE
 *
 *  The size of CLI message buffer in bytes
 *
 */
#ifndef OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE
#define OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE 2048
#endif

/**
 * @def OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
 *
 * Define as 1 to support IEEE 802.15.4-2015 Header IE (Information Element) generation and parsing, it must be set
 * to support following features:
 *    1. Time synchronization service feature (i.e., OPENTHREAD_CONFIG_TIME_SYNC_ENABLE is set).
 *
 * @note If it's enabled, plaforms must support interrupt context and concurrent access AES.
 *
 */
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
#define OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM
 *
 * Define as 1 to support configuration of the Front-End Module management.
 *
 * This is needed on some Nordic-based boards which feature a front-end module such
 * as the Skyworks SKY66112 (present in the Fanstel BT840X modules).
 *
 * If you have such a board, change this to 1.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM
#define OPENTHREAD_CONFIG_NRF5_FEM 0
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_PA_TIME_IN_ADVANCE_US
 *
 * This sets the time delay between us signalling the PA to come on and the PA
 * actually turning on in microseconds.  The default value is taken from the Nordic
 * SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_PA_TIME_IN_ADVANCE_US
#define OPENTHREAD_CONFIG_NRF5_FEM_PA_TIME_IN_ADVANCE_US NRF_FEM_PA_TIME_IN_ADVANCE_US
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_LNA_TIME_IN_ADVANCE_US
 *
 * This sets the time delay between us signalling the LNA to come on and the LNA
 * actually turning on in microseconds.  The default value is taken from the Nordic
 * SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_LNA_TIME_IN_ADVANCE_US
#define OPENTHREAD_CONFIG_NRF5_FEM_LNA_TIME_IN_ADVANCE_US NRF_FEM_LNA_TIME_IN_ADVANCE_US
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_PDN_SETTLE_US
 *
 * This sets the time delay between the PDN signal being asserted and the signal
 * taking effect in microseconds.  The default value is taken from the Nordic
 * SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_PDN_SETTLE_US
#define OPENTHREAD_CONFIG_NRF5_FEM_PDN_SETTLE_US NRF_FEM_PDN_SETTLE_US
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_LNA_TIME_IN_ADVANCE_US
 *
 * This sets the time delay between the PDN signal being de-asserted and the signal
 * taking effect in microseconds.  The default value is taken from the Nordic
 * SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_TRX_HOLD_US
#define OPENTHREAD_CONFIG_NRF5_FEM_TRX_HOLD_US NRF_FEM_TRX_HOLD_US
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ENABLE
 *
 * This enables (1) or disables (0) control of the PA enable pin.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ENABLE
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ACTIVE_HIGH
 *
 * This defines whether the PA enable pin is active-high (1) or active-low (0).
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ACTIVE_HIGH
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_ACTIVE_HIGH 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_PIN
 *
 * This defines which GPIO pin is used to control the PA enable signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_PIN
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_PIN NRF_FEM_CONTROL_DEFAULT_PA_PIN
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_GPIOTE_CHANNEL
 *
 * This defines which GPIOTE channel is used to control the PA enable signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_GPIOTE_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PA_GPIOTE_CHANNEL NRF_FEM_CONTROL_DEFAULT_PA_GPIOTE_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ENABLE
 *
 * This enables (1) or disables (0) control of the LNA enable pin.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ENABLE
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ACTIVE_HIGH
 *
 * This defines whether the LNA enable pin is active-high (1) or active-low (0).
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ACTIVE_HIGH
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_ACTIVE_HIGH 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_PIN
 *
 * This defines which GPIO pin is used to control the LNA enable signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_PIN
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_PIN NRF_FEM_CONTROL_DEFAULT_LNA_PIN
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_GPIOTE_CHANNEL
 *
 * This defines which GPIOTE channel is used to control the LNA enable signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_GPIOTE_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_LNA_GPIOTE_CHANNEL NRF_FEM_CONTROL_DEFAULT_LNA_GPIOTE_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ENABLE
 *
 * This enables (1) or disables (0) control of the power-down pin.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ENABLE
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ENABLE 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ACTIVE_HIGH
 *
 * This defines whether the power-down pin is active-high (1) or active-low (0).
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ACTIVE_HIGH
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_ACTIVE_HIGH 1
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_PIN
 *
 * This defines which GPIO pin is used to control the power-down signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_PIN
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_PIN NRF_FEM_CONTROL_DEFAULT_PDN_PIN
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_GPIOTE_CHANNEL
 *
 * This defines which GPIOTE channel is used to control the power-down signal.
 * The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_GPIOTE_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_CONTROL_PDN_GPIOTE_CHANNEL NRF_FEM_CONTROL_DEFAULT_PDN_GPIOTE_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_SET_PPI_CHANNEL
 *
 * This defines which Programmable Peripheral Interconnect channel will be used to
 * control setting the control pins.  The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_SET_PPI_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_SET_PPI_CHANNEL NRF_FEM_CONTROL_DEFAULT_SET_PPI_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_CLR_PPI_CHANNEL
 *
 * This defines which Programmable Peripheral Interconnect channel will be used to
 * control clearing the control pins.  The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_CLR_PPI_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_CLR_PPI_CHANNEL NRF_FEM_CONTROL_DEFAULT_CLR_PPI_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_FEM_PDN_PPI_CHANNEL
 *
 * This defines which Programmable Peripheral Interconnect channel will be used to
 * manage the power-down pin.  The default value is taken from the Nordic SDK.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_FEM_PDN_PPI_CHANNEL
#define OPENTHREAD_CONFIG_NRF5_FEM_PDN_PPI_CHANNEL NRF_FEM_CONTROL_DEFAULT_PDN_PPI_CHANNEL
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_WITH_SKY66112
 *
 * Define as 1 to support control of additional signals on the SKY66112.
 *
 * This is needed on some Nordic-based boards which feature the Skyworks SKY66112
 * (present in the Fanstel BT840X modules).
 *
 * If you have such a board, change this to 1.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_WITH_SKY66112
#define OPENTHREAD_CONFIG_NRF5_WITH_SKY66112 0
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_PIN
 *
 * This controls whether the SKY66112 is in high-gain mode or not.  The default
 * is consistent with Fanstel BT840X modules -- note that OpenThread normally assigns
 * UART_TX to this same pin: this will need to be re-located elsewhere when compiling
 * for the BT840X.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_PIN
#define OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_PIN 8
#endif

/**
 * @def OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_DEFAULT_STATE
 *
 * Define the default state of the SKY66112's CHL pin.
 *
 * The default is to turn high-gain mode on.
 */
#ifndef OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_DEFAULT_STATE
#define OPENTHREAD_CONFIG_NRF5_SKY66112_CHL_DEFAULT_STATE true
#endif

/**
 * @def RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM
 *
 * The number of short source address table entries.
 *
 */
#ifndef RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM
#define RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM 0
#endif

/**
 * @def RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM
 *
 * The number of extended source address table entries.
 *
 */
#ifndef RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM
#define RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM 0
#endif

/*
 * Suppress the ARMCC warning on unreachable statement,
 * e.g. break after assert(false) or ExitNow() macro.
 */
#if defined(__CC_ARM)
_Pragma("diag_suppress=111") _Pragma("diag_suppress=128")
#endif

#endif // OPENTHREAD_CORE_NRF52840_CONFIG_H_
