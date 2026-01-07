/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "neighbor_table.h"
#include <limits.h>
#include <string.h>

void nt_init(neighbor_entry_t *table, size_t table_size)
{
	if (table == NULL || table_size == 0) {
		return;
	}

	for (size_t i = 0; i < table_size; i++) {
		table[i].addr = GR_ADDR_UNASSIGNED;
		table[i].rssi = INT8_MIN;
		table[i].gradient = UINT8_MAX;
		table[i].last_seen = 0;
	}
}

bool nt_update_sorted(neighbor_entry_t *table, size_t table_size,
                      uint16_t sender_addr, uint8_t sender_gradient, int8_t sender_rssi,
                      int64_t now_ms)
{
	if (table == NULL || table_size == 0) {
		return false;
	}

	int existing_pos = -1;

	/* First pass: find if sender already exists */
	for (size_t i = 0; i < table_size; i++) {
		if (table[i].addr == sender_addr) {
			existing_pos = (int)i;
			break;
		}
	}

	/* If sender exists, remove it first (we'll re-insert in sorted position) */
	if (existing_pos != -1) {
		/* Shift entries up to fill the gap */
		for (int i = existing_pos; i < (int)table_size - 1; i++) {
			table[i] = table[i + 1];
		}
		/* Clear last entry */
		table[table_size - 1].addr = GR_ADDR_UNASSIGNED;
		table[table_size - 1].rssi = INT8_MIN;
		table[table_size - 1].gradient = UINT8_MAX;
		table[table_size - 1].last_seen = 0;
	}

	/* Find the correct sorted position for insertion
	 * Sort order: smaller gradient first, then higher RSSI
	 */
	int insert_pos = -1;
	for (size_t i = 0; i < table_size; i++) {
		/* If we hit an empty slot, insert here (end of valid entries) */
		if (table[i].addr == GR_ADDR_UNASSIGNED) {
			insert_pos = (int)i;
			break;
		}

		/* Check if new entry should go before this entry */
		if (sender_gradient < table[i].gradient) {
			insert_pos = (int)i;
			break;  /* Found position, stop searching */
		} else if (sender_gradient == table[i].gradient &&
		           sender_rssi > table[i].rssi) {
			insert_pos = (int)i;
			break;  /* Found position, stop searching */
		}
		/* Otherwise, continue to next entry */
	}

	/* If no position found, table is full and new entry is worse than all */
	if (insert_pos == -1) {
		return false;
	}

	/* Shift entries down to make room (only if not inserting at empty slot) */
	if (table[insert_pos].addr != GR_ADDR_UNASSIGNED) {
		/* Find last valid entry */
		int last_valid = -1;
		for (int i = (int)table_size - 1; i >= 0; i--) {
			if (table[i].addr != GR_ADDR_UNASSIGNED) {
				last_valid = i;
				break;
			}
		}
		
		/* Shift down from last_valid to insert_pos */
		if (last_valid >= 0 && last_valid < (int)table_size - 1) {
			for (int i = last_valid; i >= insert_pos; i--) {
				table[i + 1] = table[i];
			}
		} else if (last_valid == (int)table_size - 1) {
			/* Table is full, drop the last entry */
			for (int i = last_valid - 1; i >= insert_pos; i--) {
				table[i + 1] = table[i];
			}
		}
	}

	/* Insert the new/updated entry */
	table[insert_pos].addr = sender_addr;
	table[insert_pos].rssi = sender_rssi;
	table[insert_pos].gradient = sender_gradient;
	table[insert_pos].last_seen = now_ms;

	return true;
}

const neighbor_entry_t *nt_best(const neighbor_entry_t *table, size_t table_size)
{
	if (table == NULL || table_size == 0) {
		return NULL;
	}

	if (table[0].addr == GR_ADDR_UNASSIGNED) {
		return NULL;
	}

	return &table[0];
}

const neighbor_entry_t *nt_get(const neighbor_entry_t *table, size_t table_size, size_t idx)
{
	if (table == NULL || idx >= table_size) {
		return NULL;
	}

	if (table[idx].addr == GR_ADDR_UNASSIGNED) {
		return NULL;
	}

	return &table[idx];
}

uint16_t nt_remove(neighbor_entry_t *table, size_t table_size, size_t idx)
{
	if (table == NULL || idx >= table_size) {
		return GR_ADDR_UNASSIGNED;
	}

	if (table[idx].addr == GR_ADDR_UNASSIGNED) {
		return GR_ADDR_UNASSIGNED;
	}

	/* Save address of removed entry */
	uint16_t removed_addr = table[idx].addr;

	/* Shift entries up to fill the gap */
	for (size_t i = idx; i < table_size - 1; i++) {
		table[i] = table[i + 1];
	}

	/* Reset the last entry */
	size_t last = table_size - 1;
	table[last].addr = GR_ADDR_UNASSIGNED;
	table[last].rssi = INT8_MIN;
	table[last].gradient = UINT8_MAX;
	table[last].last_seen = 0;

	return removed_addr;
}

size_t nt_count(const neighbor_entry_t *table, size_t table_size)
{
	if (table == NULL || table_size == 0) {
		return 0;
	}

	size_t count = 0;
	for (size_t i = 0; i < table_size; i++) {
		if (table[i].addr == GR_ADDR_UNASSIGNED) {
			/* Table is sorted, so first unassigned means rest are too */
			break;
		}
		count++;
	}

	return count;
}

bool nt_is_expired(const neighbor_entry_t *table, size_t table_size, size_t idx,
                   int64_t current_time_ms, int64_t timeout_ms)
{
	if (table == NULL || idx >= table_size) {
		return false;
	}

	if (table[idx].addr == GR_ADDR_UNASSIGNED) {
		return false;
	}

	int64_t time_diff = current_time_ms - table[idx].last_seen;
	return (time_diff > timeout_ms);
}
