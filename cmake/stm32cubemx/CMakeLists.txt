cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F767xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../LWIP/App
    ../../LWIP/Target
    ../../Middlewares/Third_Party/LwIP/src/include
    ../../Middlewares/Third_Party/LwIP/system
    ../../Drivers/STM32F7xx_HAL_Driver/Inc
    ../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
    ../../Drivers/BSP/Components/lan8742
    ../../Middlewares/Third_Party/LwIP/src/include/netif/ppp
    ../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ../../Middlewares/Third_Party/LwIP/src/include/lwip
    ../../Middlewares/Third_Party/LwIP/src/include/lwip/apps
    ../../Middlewares/Third_Party/LwIP/src/include/lwip/priv
    ../../Middlewares/Third_Party/LwIP/src/include/lwip/prot
    ../../Middlewares/Third_Party/LwIP/src/include/netif
    ../../Middlewares/Third_Party/LwIP/src/include/compat/posix
    ../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa
    ../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net
    ../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys
    ../../Middlewares/Third_Party/LwIP/src/include/compat/stdc
    ../../Middlewares/Third_Party/LwIP/system/arch
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/freertos.c
    ../../Core/Src/stm32f7xx_it.c
    ../../Core/Src/stm32f7xx_hal_msp.c
    ../../Core/Src/stm32f7xx_hal_timebase_tim.c
    ../../LWIP/App/lwip.c
    ../../LWIP/Target/ethernetif.c
    ../../Drivers/BSP/Components/lan8742/lan8742.c
    ../../Drivers/BSP/Components/lan8742/lan8742.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_eth.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rng.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
    ../../Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c
    ../../Core/Src/system_stm32f7xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/auth.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/ccp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/chap_ms.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/chap-md5.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/chap-new.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/demand.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/eap.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/eui64.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/fsm.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/ipcp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/ipv6cp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/lcp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/magic.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/mppe.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/multilink.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/ppp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/pppapi.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/pppcrypt.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/pppoe.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/pppol2tp.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/pppos.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/upap.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/utils.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/vj.c
    ../../Middlewares/Third_Party/LwIP/src/netif/bridgeif.c
    ../../Middlewares/Third_Party/LwIP/src/netif/bridgeif_fdb.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ethernet.c
    ../../Middlewares/Third_Party/LwIP/src/netif/lowpan6.c
    ../../Middlewares/Third_Party/LwIP/src/netif/lowpan6_ble.c
    ../../Middlewares/Third_Party/LwIP/src/netif/lowpan6_common.c
    ../../Middlewares/Third_Party/LwIP/src/netif/slipif.c
    ../../Middlewares/Third_Party/LwIP/src/netif/zepif.c
    ../../Middlewares/Third_Party/LwIP/src/netif/ppp/ecp.c
    ../../Middlewares/Third_Party/LwIP/src/api/api_lib.c
    ../../Middlewares/Third_Party/LwIP/src/api/netbuf.c
    ../../Middlewares/Third_Party/LwIP/src/api/err.c
    ../../Middlewares/Third_Party/LwIP/src/api/netdb.c
    ../../Middlewares/Third_Party/LwIP/src/api/api_msg.c
    ../../Middlewares/Third_Party/LwIP/src/api/tcpip.c
    ../../Middlewares/Third_Party/LwIP/src/api/netifapi.c
    ../../Middlewares/Third_Party/LwIP/src/api/if_api.c
    ../../Middlewares/Third_Party/LwIP/src/api/sockets.c
    ../../Middlewares/Third_Party/LwIP/src/core/raw.c
    ../../Middlewares/Third_Party/LwIP/src/core/altcp.c
    ../../Middlewares/Third_Party/LwIP/src/core/altcp_tcp.c
    ../../Middlewares/Third_Party/LwIP/src/core/sys.c
    ../../Middlewares/Third_Party/LwIP/src/core/timeouts.c
    ../../Middlewares/Third_Party/LwIP/src/core/dns.c
    ../../Middlewares/Third_Party/LwIP/src/core/udp.c
    ../../Middlewares/Third_Party/LwIP/src/core/def.c
    ../../Middlewares/Third_Party/LwIP/src/core/stats.c
    ../../Middlewares/Third_Party/LwIP/src/core/ip.c
    ../../Middlewares/Third_Party/LwIP/src/core/memp.c
    ../../Middlewares/Third_Party/LwIP/src/core/tcp.c
    ../../Middlewares/Third_Party/LwIP/src/core/altcp_alloc.c
    ../../Middlewares/Third_Party/LwIP/src/core/mem.c
    ../../Middlewares/Third_Party/LwIP/src/core/inet_chksum.c
    ../../Middlewares/Third_Party/LwIP/src/core/init.c
    ../../Middlewares/Third_Party/LwIP/src/core/tcp_out.c
    ../../Middlewares/Third_Party/LwIP/src/core/pbuf.c
    ../../Middlewares/Third_Party/LwIP/src/core/tcp_in.c
    ../../Middlewares/Third_Party/LwIP/src/core/netif.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.c
    ../../Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.c
    ../../Middlewares/Third_Party/LwIP/system/OS/sys_arch.c
    ../../Middlewares/Third_Party/LwIP/src/apps/mqtt/mqtt.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f767xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


