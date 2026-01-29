cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gradient_srv)

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
	src/packet_stats.c)
target_include_directories(app PRIVATE include)
# NORDIC SDK APP END
