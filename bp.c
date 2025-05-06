/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* share modes */
#define SHARE_NONE 0
#define SHARE_LSB  1
#define SHARE_MID  2

/* FSM states */
#define SNT 0  /* Strong Not Taken */
#define WNT 1  /* Weak   Not Taken */
#define WT  2  /* Weak   Taken */
#define ST  3  /* Strong Taken */

/* BTB Entry*/
typedef struct{
	int valid;
	uint32_t target;
	uint32_t tag;
	uint8_t hist;
} BTB_Entry;

/*static globals*/
static BTB_Entry *btb = NULL; //BTB
static uint8_t *FSM_Table = NULL; //FSM table

/*initializig variables*/
static unsigned btbSize_P, historySize_P, tagSize_P, fsmState_P;
static bool isGlobalHist_P, isGlobalTable_P;
static int Shared_P;

/*masks to extract from PC*/
static uint32_t  index_mask, tag_mask, hist_mask;

/*counters*/
static unsigned br_pred; // Total branch commands
static unsigned br_flush; // Total false branch commands

/*other*/
static size_t table_size; //FSM table size
static uint32_t global_hist; //global history 
static int index_bits; //Number of bits to represent an index in BTB

 
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	/*inputs*/
	btbSize_P= btbSize;
	historySize_P = historySize;
	tagSize_P = tagSize;
	fsmState_P = fsmState;
	isGlobalHist_P = isGlobalHist;
	isGlobalTable_P = isGlobalTable;
	Shared_P = Shared;
	
	/*sizes*/
	table_size = (1u << historySize);
	index_bits = 0;
	while ((1u << index_bits) < btbSize) 
		index_bits++;
	
	/*masks*/
	index_mask = btbSize - 1;
	tag_mask = (1u << tagSize) - 1;
	hist_mask = (1u <<historySize) - 1;

	/*counters and global history*/
	br_pred = 0;
	br_flush = 0;
	global_hist = 0;

	/*allocate memory for BTB*/
	btb = calloc(btbSize, sizeof(BTB_Entry));
	if(!btb)
		return -1;

	/*allocate memory for FSM*/
	size_t enrties = isGlobalTable ? table_size : (table_size * btbSize);
	FSM_Table = malloc(enrties);
	if (!FSM_Table)
	{
		return -1;
	}
	for(int i=0; i < enrties ; i++){
		FSM_Table[i] = fsmState;
	}

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	uint32_t pc2 = pc >> 2; //pc aligned to 4
	uint32_t index = pc2 & index_mask;
	uint32_t tag = (pc2 >> index_bits) & tag_mask;
	BTB_Entry *entry = &btb[index];
	/*Checking if the entry is valid and if the tag matches*/
	if(entry->valid && entry->tag == tag){
		/*Checking if the FSM table is valid*/
		uint32_t hist_calc = isGlobalHist_P ? global_hist : entry->hist;
		/*Apply sharing*/
		if(Shared_P == SHARE_LSB)
			hist_calc ^=(pc2 & hist_mask);
		else if(Shared_P == SHARE_MID)
			hist_calc ^=((pc >> 16) & hist_mask);
		size_t idx = isGlobalTable_P ? hist_calc : (index * table_size + hist_calc);

		/*Predicting the target address*/
		if(FSM_Table[idx] >= WT){
			br_pred++;
			*dst = entry->target;
			return true;
		}
	}
	/*If the entry is invalid or the FSM table is not valid, return the next instruction*/
	*dst = pc + 4;
	br_pred++;	

	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	uint32_t pc2 = pc >> 2; //pc aligned to 4
	uint32_t index = pc2 & index_mask;
	uint32_t tag = (pc2 >> index_bits) & tag_mask;
	BTB_Entry *entry = &btb[index];

	/*Checking if the prediction was correct. If incorrect, increment the flush counter*/
	bool is_taken = (pred_dst != (pc + 4));
	if(is_taken != taken) br_flush++;

	/*Checking if BTB entry is invalid or mismatched (new branch)*/
	if(entry->valid == 0 || entry->tag != tag){
		entry->valid = 1;
		entry->tag = tag;
		entry->target = targetPc;
		entry-> hist = 0;
		/*Reset the FSM table for this BTB entry (only if using local FSM tables)*/
		if(!isGlobalTable_P){
			size_t start = index * table_size;
			for(int i = 0; i < table_size; i++)
				FSM_Table[start + i] = fsmState_P;
		}
	} else {
		 /* Update existing FSM counter */
		uint32_t hist_calc = isGlobalHist_P ? global_hist : entry->hist;

		/*Apply sharing*/
		if(Shared_P == SHARE_LSB)
			hist_calc ^=(pc2 & hist_mask);
		else if(Shared_P == SHARE_MID)
			hist_calc ^=((pc >> 16) & hist_mask);
		size_t idx = isGlobalTable_P ? hist_calc : (index * table_size + hist_calc);

		/*Update the 2-bit counter*/
		if(taken){
			if( FSM_Table[idx] < ST) FSM_Table[idx]++;
		
		} else {
			if( FSM_Table[idx] > SNT) FSM_Table[idx]--;
		}
		entry->target = targetPc;
	}

	/*Update the global or local history register according to the taken/not-taken outcome*/
	uint32_t bit = taken ? 1u : 0u;
	if (isGlobalHist_P)
	{
		global_hist = ((global_hist << 1) | bit) & hist_mask;
	} else {
		entry->hist = ((entry->hist << 1) | bit) & hist_mask;
	}
	return;
}

void BP_GetStats(SIM_stats *curStats) {
    // Set number of flushes and branches
    curStats->flush_num = br_flush;
    curStats->br_num = br_pred;

    unsigned bits = 0;
    static unsigned target_bits = 30;

    // BTB: valid + tag + target per entry
    bits += btbSize_P * (1 + tagSize_P + target_bits);

    // History: global or per-entry
    bits += isGlobalHist_P ? historySize_P : historySize_P * btbSize_P;

    // FSM table: 2 bits per entry, global or per-entry
    bits += isGlobalTable_P ? table_size * 2 : table_size * 2 * btbSize_P;

    // Save total size
    curStats->size = bits;

    // Free resources
    free(btb);
    free(FSM_Table);
}
