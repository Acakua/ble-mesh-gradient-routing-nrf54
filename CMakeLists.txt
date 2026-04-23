cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gradient_srv)

# Workaround for NCS 3.0.1 build failure on nRF54L15
# Ensures soc.c correctly identifies the Zephyr environment during TF-M build
if(CONFIG_TF_M)
	set(TFM_PLATFORM_EXT_CONFIG_FLAGS "-D__ZEPHYR__=1 -DMBEDTLS_PSA_CRYPTO_USER_CONFIG_FILE=\\\"nrf-psa-crypto-user-config.h\\\"" CACHE STRING "TFM extra flags" FORCE)
endif()

# NORDIC SDK APP START
target_sources(app PRIVATE
	src/main.c
	src/model_handler.c
	src/gradient_srv.c
	src/neighbor_table.c
	src/routing_policy.c
	src/led_indication.c
	src/data_forward.c
	src/gradient_work.c
	src/reverse_routing.c
	src/heartbeat.c
	src/shell_commands.c
	src/packet_stats.c
	src/sensor_manager.c
	src/sensor_shell.c
	src/storage.c
	src/teds_tlv.c)
target_include_directories(app PRIVATE include)
# NORDIC SDK APP END
