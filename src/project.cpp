/*########################################################################################################*/
// cd <pin-path>/source/tools/BinTranslatorTutorial
// make bprofile-with-gearing.test
// pin -t obj-intel64/bprofile-with-gearing.so -- ~/workdir/tst
/*########################################################################################################*/
/*BEGIN_LEGAL
Intel Open Source License

Copyright (c) 2002-2011 Intel Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */
/* ===================================================================== */

/* ===================================================================== */
/*! @file
 * This probe pintool generates translated code of all the routines, places them 
 * in an allocated Translation Cache (TC) along with instrumentation instructions that collect 
 * profiling for each BBL and for each indirect jump target.
 *
 * The pintool generates translated code of routines, while adding a NOP7
 * instruction at the head of every translated routine, places them in an allocated
 * translation cache (TC), and patches the original code by creating jump probes from
 * the header of each original routine to the header of its translated routine in
 * the TC.
 * It then, creates a second thread that waits a pre-defined number of seconds for the
 * profiling to be gathered, and creates another translation cache called TC2 with the
 * translated instructions from TC, along with a NOP7 instr at the head of each
 * translated routine, and creates an atomic jump from the NOP7 head of each routine 
 * in TC to its correspoding NOP7 header instruction in the TC2.
 *
 * Finally, on exit, the profiling data is then printed into the output file bprofile.out.
 */

#include "pin.H"
extern "C" {
#include "xed-interface.h"
}
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <values.h>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <time.h>
#include <fstream>

using namespace std;

/*======================================================================*/
/* commandline switches                                                 */
/*======================================================================*/
KNOB<BOOL>   KnobVerbose(KNOB_MODE_WRITEONCE,    "pintool",
    "verbose", "0", "Verbose run");

KNOB<BOOL>   KnobDumpOrigCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_orig_code", "0", "Dump Original non-translated Code");

KNOB<BOOL>   KnobDumpTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_tc", "0", "Dump Translated Code");

KNOB<BOOL>   KnobDumpTranslatedCode2(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_tc2", "0", "Dump 2nd Translated Code");

KNOB<BOOL>   KnobDoNotCommitTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "no_tc_commit", "0", "Do not commit translated code");

KNOB<UINT> KnobNumSecsDuringProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "prof_time", "2", "Number of seconds for collecting BBL counters");

KNOB<BOOL> KnobDumpProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_prof", "0", "Dump profiling information");

KNOB<BOOL> KnobNoProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "no_prof", "0", "Do not collect profile information");

// Knobs to switch off single TC2 optimizations (for measurements).
KNOB<BOOL> KnobNoDevirt(KNOB_MODE_WRITEONCE,    "pintool",
    "no_devirt", "0", "Disable de-virtualization in TC2");

KNOB<BOOL> KnobNoReorder(KNOB_MODE_WRITEONCE,    "pintool",
    "no_reorder", "0", "Disable code reordering in TC2");

KNOB<BOOL> KnobNoUnroll(KNOB_MODE_WRITEONCE,    "pintool",
    "no_unroll", "0", "Disable loop unrolling in TC2");

KNOB<BOOL> KnobNoInline(KNOB_MODE_WRITEONCE,    "pintool",
    "no_inline", "0", "Disable leaf function inlining in TC2");

KNOB<BOOL> KnobNoConstProp(KNOB_MODE_WRITEONCE,    "pintool",
    "no_constprop", "0", "Disable constant propagation in TC2");

KNOB<BOOL> KnobNoRegPromo(KNOB_MODE_WRITEONCE,    "pintool",
    "no_regpromo", "0", "Disable register promotion in TC2");

KNOB<BOOL> KnobDumpPromo(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_promo", "0", "Dump the register promotion candidates");

KNOB<BOOL> KnobStats(KNOB_MODE_WRITEONCE,    "pintool",
    "stats", "0", "Print statistics of the TC2 optimizations");


/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */
std::ofstream* out = 0;

// For XED:
#if defined(TARGET_IA32E)
    xed_state_t dstate = {XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b};
#else
    xed_state_t dstate = { XED_MACHINE_MODE_LEGACY_32, XED_ADDRESS_WIDTH_32b};
#endif

//For XED: Pass in the proper length: 15 is the max. But if you do not want to
//cross pages, you can pass less than 15 bytes, of course, the
//instruction might not decode if not enough bytes are provided.
const unsigned max_inst_len = XED_MAX_INSTRUCTION_BYTES;

ADDRINT lowest_sec_addr = 0;
ADDRINT highest_sec_addr = 0;

// TC containing the new code:
char *tc = nullptr;
unsigned tc_size = 0;

// 2nd TC containing the new code after gearing:
char *tc2 = nullptr;
unsigned tc2_size = 0;

unsigned max_tc_size = 0;


// Array of original target addresses that cannot be translated in the TC.
ADDRINT *jump_to_orig_addr_map = nullptr;
unsigned jump_to_orig_addr_num = 0;

// basic instruction types.
typedef enum {
    RegularIns = 0,
    RtnHeadIns,
    ProfilingIns,

} ins_enum_t;

// instructions map with an entry for each new instruction in the code.
typedef struct {
    ADDRINT orig_ins_addr;
    ADDRINT new_ins_addr;
    ADDRINT orig_targ_addr;
    ADDRINT orig_rip_addr;
    ins_enum_t ins_type;
    char encoded_ins[XED_MAX_INSTRUCTION_BYTES];
    unsigned size;
    int targ_map_entry;
    unsigned bbl_num;
    xed_category_enum_t xed_category;
} instr_map_t;


// Instrs map:
instr_map_t *instr_map = NULL;
unsigned num_of_instr_map_entries = 0;
unsigned max_ins_count = 0;

// Is this TC entry left out of TC2 (profiling code, NOPs, empty entries)?
// The NOP at a routine head is kept: it is the target of the TC -> TC2 jump.
static bool dropped_in_tc2(const instr_map_t &e)
{
    if (e.ins_type == ProfilingIns || !e.size)
      return true;
    if (e.ins_type == RtnHeadIns && e.xed_category == XED_CATEGORY_WIDENOP)
      return false;
    return e.xed_category == XED_CATEGORY_WIDENOP || e.xed_category == XED_CATEGORY_NOP;
}

#define MAX_TARG_ADDRS 0x3

// Kind of indirect control transfer that terminates a BBL.
typedef enum {
  NoIndirect = 0,
  IndirectJump,   // jmp reg / jmp [mem]
  IndirectCall,   // call reg / call [mem]
} indirect_kind_t;

// Describes where an indirect jmp/call reads its target address from
typedef struct {
  xed_reg_enum_t targ_reg;       // XED_REG_INVALID for a memory operand
  xed_reg_enum_t base_reg;
  xed_reg_enum_t index_reg;
  xed_uint_t     scale;
  xed_int64_t    disp;
  xed_uint_t     disp_width;
  unsigned       mem_addr_width;
  ADDRINT        rip_mem_addr;   // only for base_reg == XED_REG_RIP
} indirect_target_t;

// Bbl map of all the bbl exec counters to be collected at runtime:
typedef struct {
  UINT64 counter;
  UINT64 fallthru_counter; // for BBLs that terminate with a cond branch.
  ADDRINT targ_addr[MAX_TARG_ADDRS+1];
  UINT64  targ_count[MAX_TARG_ADDRS+1];
  unsigned starting_ins_entry;
  unsigned terminating_ins_entry;
  
  //added these variables
  indirect_kind_t indirect_kind;
  ADDRINT indirect_site_addr;
} bbl_map_t;

bbl_map_t *bbl_map;
unsigned bbl_num = 0;
std::map<ADDRINT, unsigned> entry_map;

unsigned max_rtn_count = 0;
unsigned max_bbl_count = 0;   // number of entries allocated in bbl_map

struct timespec start_running_time;
struct timespec end_running_time;
struct timespec tool_start_time;




/* ============================================================= */
/* Optimization 1 (De-virtualization) Functions                  */
/* ============================================================= */

unsigned num_devirt_calls = 0;
unsigned num_devirt_jumps = 0;
unsigned num_devirt_skip_rare = 0;
unsigned num_devirt_skip_not_translated = 0;
unsigned num_devirt_skip_far_targ = 0;
unsigned num_devirt_skip_other = 0;

int add_new_encoded_instr(ADDRINT ins_addr, xed_encoder_instruction_t *enc_instr, ins_enum_t ins_type);

static bool get_indirect_target_operand(const xed_decoded_inst_t *xedd,
                                        ADDRINT ins_addr,
                                        indirect_target_t *t)
{
  memset(t, 0, sizeof(*t));
  t->targ_reg = XED_REG_INVALID;
  t->base_reg = XED_REG_INVALID;
  t->index_reg = XED_REG_INVALID;

  xed_iclass_enum_t iclass = xed_decoded_inst_get_iclass(xedd);
  if (iclass != XED_ICLASS_JMP && iclass != XED_ICLASS_CALL_NEAR)
    return false;                               // e.g. far jmp / far call
  if (xed_decoded_inst_get_operand_width(xedd) != 64)
    return false;

  const xed_inst_t *xi = xed_decoded_inst_inst(xedd);
  xed_operand_enum_t op0 = xed_operand_name(xed_inst_operand(xi, 0));

  if (op0 == XED_OPERAND_REG0) {                // jmp/call reg
    t->targ_reg = xed_decoded_inst_get_reg(xedd, XED_OPERAND_REG0);
    return true;
  }
  if (op0 != XED_OPERAND_MEM0)
    return false;

  // jmp/call [mem]: the target is memory operand 0.
  xed_reg_enum_t seg = xed_decoded_inst_get_seg_reg(xedd, 0);
  if (seg == XED_REG_FS || seg == XED_REG_GS)
    return false;           // our 'mov rax, [mem]' would miss the segment base

  t->base_reg = xed_decoded_inst_get_base_reg(xedd, 0);
  t->index_reg = xed_decoded_inst_get_index_reg(xedd, 0);
  t->scale = xed_decoded_inst_get_scale(xedd, 0);
  t->disp = xed_decoded_inst_get_memory_displacement(xedd, 0);
  t->disp_width = xed_decoded_inst_get_memory_displacement_width_bits(xedd, 0);
  t->mem_addr_width = xed_decoded_inst_get_memop_address_width(xedd, 0);
  if (t->base_reg == XED_REG_RIP)
    t->rip_mem_addr = ins_addr + xed_decoded_inst_get_length(xedd) + t->disp;
  return true;
}


// THRESHOLD for de-virtualization (frequent target):
//   A site (indirect jmp / indirect call) is de-virtualized when its most
//   frequent target was reached at least DEVIRT_MIN_COUNT times during the
//   profiling and covers at least DEVIRT_MIN_PERCENT % of all the targets
//   seen at that site. Otherwise the site is left as is.
#define DEVIRT_MIN_COUNT   1000
#define DEVIRT_MIN_PERCENT 80

// Keys (fake orig_ins_addr values) for new instrs that are branch targets
// inside TC2 only. They are never real code addresses, and chaining resolves
// branches to them like to any other entry. Unlike instr_map indices, they
// stay valid when later passes move or copy entries.
static ADDRINT next_synthetic_key = 0xFFFF800000000000ULL;
static ADDRINT new_synthetic_key() { return next_synthetic_key += 0x10; }

// BBL numbers for new cold BBLs created in TC2 (the miss path of a
// de-virtualized site). They are >= bbl_num, so their profile count is 0.
static unsigned next_synthetic_bbl = 0;

// Step 0 of create_tc2(): choose the sites to de-virtualize.
// Runs on the TC map (before the TC2 map is built), where orig_ins_addr
// still holds the original addresses.
//   orig_to_tc : original instr address -> TC address of its first instr
//                that is kept in TC2
//   sites      : TC instr_map index of the indirect jmp/call -> hot target (orig addr)
static void find_devirt_sites(const instr_map_t *tc_map, unsigned tc_entries,
                              const std::map<ADDRINT, ADDRINT> &orig_to_tc,
                              std::map<unsigned, ADDRINT> &sites)
{
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].indirect_kind == NoIndirect)
        continue;

      unsigned best = 0;
      UINT64 total = 0;
      for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++) {
        total += bbl_map[b].targ_count[j];
        if (bbl_map[b].targ_count[j] > bbl_map[b].targ_count[best])
          best = j;
      }
      if (!total)
        continue;

      ADDRINT hot_targ = bbl_map[b].targ_addr[best];
      UINT64 hot_count = bbl_map[b].targ_count[best];
      if (hot_count < DEVIRT_MIN_COUNT || hot_count * 100 < total * DEVIRT_MIN_PERCENT) {
        num_devirt_skip_rare++;
        continue;
      }
      if (KnobVerbose)
        cerr << "devirt site 0x" << hex << bbl_map[b].indirect_site_addr << " hot target 0x"
             << hot_targ << dec << " " << hot_count << "/" << total << endl;
      if (!orig_to_tc.count(hot_targ)) {
        num_devirt_skip_not_translated++;
        continue;
      }
      if (hot_targ > 0x7FFFFFFF) {           // must fit in the imm32 of the cmp
        num_devirt_skip_far_targ++;
        continue;
      }
      unsigned site = bbl_map[b].terminating_ins_entry;
      if (site >= tc_entries || tc_map[site].orig_ins_addr != bbl_map[b].indirect_site_addr) {
        num_devirt_skip_other++;
        continue;
      }
      sites[site] = hot_targ;
    }
}

static int add_devirt_instr(ADDRINT key, xed_encoder_instruction_t *enc_instr,
                            unsigned bbl, ADDRINT targ_key)
{
    if (add_new_encoded_instr(key, enc_instr, RegularIns) < 0)
      return -1;
    instr_map_t *e = &instr_map[num_of_instr_map_entries - 1];
    e->bbl_num = bbl;
    e->orig_targ_addr = targ_key;   // 0 for non-branches
    return 0;
}

// Emit the de-virtualized code of the indirect jmp/call 'site' at the end of
// the TC2 instr_map. 'site' is the TC2 entry of the indirect instr (its
// orig_ins_addr is its TC address), 'hot_targ' the original address of the
// hot target, 'hot_targ_key' its TC address (chaining turns it into TC2),
// 'next_key' the key of the instr following the site.
//
// indirect call:                          indirect jump:
//     cmp  <targ operand>, hot_targ           lea  rsp, [rsp-128]   ; skip red zone
//     jne  MISS                               push rcx
//     call hot_targ (direct, in TC2)          mov  rcx, <targ operand>
//     jmp  NEXT                               lea  rcx, [rcx-hot_targ]
//   MISS:  (cold BBL)                         jrcxz HIT              ; rcx == 0: hot target
//     call <targ operand>  (original)         pop  rcx
//   NEXT:                                     lea  rsp, [rsp+128]
//                                             jmp  <targ operand>  (original)
//                                           HIT:
//                                             pop  rcx
//                                             lea  rsp, [rsp+128]
//                                             jmp  hot_targ (direct, in TC2)
//
// At a call the flags are dead (the ABI does not preserve them across a
// call), so the call sequence may change them. At a jump they may be live:
// the jump sequence does not change them (lea, mov, push, pop and jrcxz do
// not write the flags), which is much faster than pushfq/popfq. It moves rsp
// below the 128 bytes red zone first, so that 'push rcx' cannot overwrite
// data of a leaf function. No register is changed in both sequences.
// Returns 1 if emitted, 0 if the form is not supported (the caller then
// copies the site unchanged), -1 on an encoding error.
static int emit_devirt_site(const instr_map_t *site, ADDRINT hot_targ, ADDRINT hot_targ_key,
                            ADDRINT next_key)
{
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);
    if (xed_decode(&xedd, reinterpret_cast<const UINT8*>(site->encoded_ins), max_inst_len) != XED_ERROR_NONE)
      return 0;

    indirect_target_t t;
    if (!get_indirect_target_operand(&xedd, site->orig_ins_addr, &t))
      return 0;
    if (t.mem_addr_width && t.mem_addr_width != 64)
      return 0;

    bool is_call = (xed_decoded_inst_get_category(&xedd) == XED_CATEGORY_CALL);
    if (is_call && !next_key)
      return 0;
    // The jump sequence moves rsp, so an operand based on rsp would change.
    if (!is_call && (t.targ_reg == XED_REG_RSP || t.base_reg == XED_REG_RSP ||
                     t.index_reg == XED_REG_RSP))
      return 0;
    if (next_synthetic_bbl + 1 >= max_bbl_count)
      return 0;

    ADDRINT key = site->orig_ins_addr;
    unsigned bbl = site->bbl_num;
    unsigned miss_bbl = next_synthetic_bbl++;
    ADDRINT miss_key = new_synthetic_key();
    xed_encoder_instruction_t enc_instr;

    // The target operand of the site, as the 1st operand of a 64-bit cmp.
    xed_encoder_operand_t targ_op;
    if (t.targ_reg != XED_REG_INVALID)
      targ_op = xed_reg(t.targ_reg);
    else if (t.base_reg == XED_REG_RIP)
      targ_op = xed_mem_bd(XED_REG_RIP, xed_disp(0, 32), 64);   // disp fixed later
    else
      targ_op = xed_mem_bisd(t.base_reg, t.index_reg, t.scale,
                             xed_disp(t.disp, t.disp_width ? t.disp_width : 32), 64);

    if (!is_call) {
      ADDRINT hit_key = new_synthetic_key();
      xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RSP),
                xed_mem_bd(XED_REG_RSP, xed_disp(-128, 32), 64));
      if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
      xed_inst1(&enc_instr, dstate, XED_ICLASS_PUSH, 64, xed_reg(XED_REG_RCX));
      if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
      if (t.targ_reg != XED_REG_RCX) {
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RCX), targ_op);
        if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
        if (t.base_reg == XED_REG_RIP)
          instr_map[num_of_instr_map_entries - 1].orig_rip_addr = site->orig_rip_addr;
      }
      xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RCX),
                xed_mem_bd(XED_REG_RCX, xed_disp(-(xed_int64_t)hot_targ, 32), 64));
      if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JRCXZ, 64, xed_relbr(0, 8));
      if (add_devirt_instr(key, &enc_instr, bbl, hit_key) < 0) return -1;

      // MISS: restore and run the original jump.
      xed_inst1(&enc_instr, dstate, XED_ICLASS_POP, 64, xed_reg(XED_REG_RCX));
      if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RSP),
                xed_mem_bd(XED_REG_RSP, xed_disp(128, 32), 64));
      if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
      instr_map[num_of_instr_map_entries] = *site;
      instr_map[num_of_instr_map_entries].orig_ins_addr = key;
      instr_map[num_of_instr_map_entries].ins_type = RegularIns;
      instr_map[num_of_instr_map_entries].targ_map_entry = -1;
      instr_map[num_of_instr_map_entries].bbl_num = bbl;
      num_of_instr_map_entries++;

      // HIT: restore and jump directly.
      xed_inst1(&enc_instr, dstate, XED_ICLASS_POP, 64, xed_reg(XED_REG_RCX));
      if (add_devirt_instr(hit_key, &enc_instr, bbl, 0) < 0) return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RSP),
                xed_mem_bd(XED_REG_RSP, xed_disp(128, 32), 64));
      if (add_devirt_instr(hit_key, &enc_instr, bbl, 0) < 0) return -1;
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JMP, 64, xed_relbr(0, 32));
      if (add_devirt_instr(hit_key, &enc_instr, bbl, hot_targ_key) < 0) return -1;

      num_devirt_jumps++;
      return 1;
    }

    // Indirect call.
    xed_inst2(&enc_instr, dstate, XED_ICLASS_CMP, 64, targ_op,
              xed_simm0((xed_int32_t)hot_targ, 32));
    if (add_devirt_instr(key, &enc_instr, bbl, 0) < 0) return -1;
    if (t.base_reg == XED_REG_RIP)
      instr_map[num_of_instr_map_entries - 1].orig_rip_addr = site->orig_rip_addr;
    xed_inst1(&enc_instr, dstate, XED_ICLASS_JNZ, 64, xed_relbr(0, 32));
    if (add_devirt_instr(key, &enc_instr, bbl, miss_key) < 0) return -1;
    xed_inst1(&enc_instr, dstate, XED_ICLASS_CALL_NEAR, 64, xed_relbr(0, 32));
    if (add_devirt_instr(key, &enc_instr, bbl, hot_targ_key) < 0) return -1;
    xed_inst1(&enc_instr, dstate, XED_ICLASS_JMP, 64, xed_relbr(0, 32));
    if (add_devirt_instr(key, &enc_instr, bbl, next_key) < 0) return -1;

    // MISS path (a cold BBL): the original indirect call.
    instr_map[num_of_instr_map_entries] = *site;
    instr_map[num_of_instr_map_entries].orig_ins_addr = miss_key;
    instr_map[num_of_instr_map_entries].ins_type = RegularIns;
    instr_map[num_of_instr_map_entries].targ_map_entry = -1;
    instr_map[num_of_instr_map_entries].bbl_num = miss_bbl;
    num_of_instr_map_entries++;

    if (is_call)
      num_devirt_calls++;
    else
      num_devirt_jumps++;
    return 1;
}

// Step 1b of create_tc2(): build a new instr_map in which every chosen
// site (given by its index in the TC2 map) is replaced by its de-virtualized
// code; the other entries are copied as they are.
//   hot_keys : original hot target address -> its key in the TC2 map
static int insert_devirt_sites(const std::map<unsigned, ADDRINT> &sites,
                               const std::map<ADDRINT, ADDRINT> &hot_keys)
{
    if (sites.empty())
      return 0;

    instr_map_t *old_map = instr_map;
    unsigned old_num = num_of_instr_map_entries;
    unsigned extra = 16 * sites.size() + 16;   // at most 10 new entries per site

    instr_map_t *new_map = (instr_map_t *)calloc(max_ins_count + extra, sizeof(instr_map_t));
    if (new_map == NULL) {
      perror("calloc");
      return -1;
    }
    instr_map = new_map;
    max_ins_count += extra;
    num_of_instr_map_entries = 0;
    next_synthetic_bbl = bbl_num + 1;

    for (unsigned i = 0; i < old_num; i++) {
      std::map<unsigned, ADDRINT>::const_iterator it = sites.find(i);
      if (it != sites.end()) {
        ADDRINT next_key = (i + 1 < old_num) ? old_map[i + 1].orig_ins_addr : 0;
        unsigned start = num_of_instr_map_entries;
        int rc = emit_devirt_site(&old_map[i], it->second, hot_keys.at(it->second), next_key);
        if (rc < 0) {
          // Undo the partial sequence and keep the site unchanged.
          num_of_instr_map_entries = start;
          rc = 0;
        }
        if (rc > 0)
          continue;
        num_devirt_skip_other++;             // unsupported form: copy it
      }
      instr_map[num_of_instr_map_entries++] = old_map[i];
    }

    free(old_map);
    return 0;
}




/* ============================================================= */
/* Optimization 2 (Code Reordering) Functions                    */
/* ============================================================= */

//
// THRESHOLD ("frequent" vs "rare" basic block):
//   A BBL is HOT (frequent) if it was executed at least once during
//   profiling, and COLD (rare) if its profiling counter is 0.
//
// LAYOUT: inside every routine of TC2 the BBLs are placed in this order:
//   1. the entry BBL (it must stay first: TC jumps to the routine start),
//   2. the other hot BBLs, in their original order,
//   3. the cold BBLs, in their original order (moved to the routine end).
//   So the code that really runs is packed together (fewer i-cache lines),
//   and the hot path no longer jumps over cold code.
//
// FIXING THE FALL-THROUGH: a BBL that does not end with jmp/ret continues
// ("falls through") into its original next BBL F. After reordering, for
// such a BBL:
//   - if F is still placed right after it: nothing to do;
//   - else if it ends with 'jcc T' and T is now placed right after it:
//     reverse the condition, 'jcc T' -> 'jncc F' (like the course's
//     reverse_cond_jumps example). The hot path now falls through into T
//     without a taken branch;
//   - else: add 'jmp F' after it.
//
// Routines containing LOOP/LOOPE/LOOPNE/JRCXZ are not reordered: these
// branches only have an 8-bit displacement (the target could end up too far
// away) and have no reversed form.

unsigned num_reordered_rtns = 0;
unsigned num_moved_cold_bbls = 0;
unsigned num_reversed_cond_branches = 0;
unsigned num_added_fallthru_jumps = 0;

// The reversed condition of a conditional jump, or XED_ICLASS_INVALID.
static xed_iclass_enum_t reverse_cond_iclass(xed_iclass_enum_t iclass)
{
    switch (iclass) {
      case XED_ICLASS_JB:   return XED_ICLASS_JNB;
      case XED_ICLASS_JBE:  return XED_ICLASS_JNBE;
      case XED_ICLASS_JL:   return XED_ICLASS_JNL;
      case XED_ICLASS_JLE:  return XED_ICLASS_JNLE;
      case XED_ICLASS_JNB:  return XED_ICLASS_JB;
      case XED_ICLASS_JNBE: return XED_ICLASS_JBE;
      case XED_ICLASS_JNL:  return XED_ICLASS_JL;
      case XED_ICLASS_JNLE: return XED_ICLASS_JLE;
      case XED_ICLASS_JNO:  return XED_ICLASS_JO;
      case XED_ICLASS_JNP:  return XED_ICLASS_JP;
      case XED_ICLASS_JNS:  return XED_ICLASS_JS;
      case XED_ICLASS_JNZ:  return XED_ICLASS_JZ;
      case XED_ICLASS_JO:   return XED_ICLASS_JNO;
      case XED_ICLASS_JP:   return XED_ICLASS_JNP;
      case XED_ICLASS_JS:   return XED_ICLASS_JNS;
      case XED_ICLASS_JZ:   return XED_ICLASS_JNZ;
      default:              return XED_ICLASS_INVALID;   // e.g. JRCXZ, LOOP
    }
}

static xed_iclass_enum_t entry_iclass(const instr_map_t *e)
{
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);
    if (xed_decode(&xedd, reinterpret_cast<const UINT8*>(e->encoded_ins), max_inst_len) != XED_ERROR_NONE)
      return XED_ICLASS_INVALID;
    return xed_decoded_inst_get_iclass(&xedd);
}

// A group of consecutive instr_map entries forming one BBL of TC2.
typedef struct {
    unsigned first, last;   // entry range [first, last]
    int      term;          // last entry with size > 0 (-1 if none)
    UINT64   count;         // BBL execution count during profiling
} tc2_bbl_t;

// Emit a new entry at the end of instr_map: a direct branch 'iclass'
// (JMP or a Jcc) whose target is the entry with orig_ins_addr == targ_key.
static int emit_branch_to_key(xed_iclass_enum_t iclass, ADDRINT targ_key, unsigned bbl)
{
    xed_encoder_instruction_t enc_instr;
    xed_inst1(&enc_instr, dstate, iclass, 64, xed_relbr(0, 32));
    if (add_new_encoded_instr(0, &enc_instr, RegularIns) < 0)
      return -1;
    instr_map_t *e = &instr_map[num_of_instr_map_entries - 1];
    e->orig_ins_addr = 0;           // new instr: no address of its own
    e->orig_targ_addr = targ_key;   // resolved by chaining
    e->bbl_num = bbl;
    return 0;
}

// Step 1c of create_tc2(): reorder the BBLs of every routine (see above).
// Runs after Step 1/1b (orig_ins_addr fields hold TC addresses, which chaining
// uses as keys, so moving entries does not break branch targets).
static int reorder_bbls()
{
    unsigned old_num = num_of_instr_map_entries;
    instr_map_t *old_map = instr_map;
    if (!old_num)
      return 0;

    // 1. Split instr_map into BBLs. A new BBL starts where bbl_num changes
    //    and at every routine head (a BBL can run past the end of a routine
    //    that does not end with jmp/ret).
    std::vector<tc2_bbl_t> bbls;
    std::vector<unsigned> rtn_first_bbl;       // index into bbls of each routine start
    for (unsigned i = 0; i < old_num; i++) {
      bool rtn_head = (old_map[i].ins_type == RtnHeadIns);
      if (i == 0 || rtn_head || old_map[i].bbl_num != old_map[i - 1].bbl_num) {
        tc2_bbl_t b;
        b.first = b.last = i;
        b.term = -1;
        b.count = (old_map[i].bbl_num < bbl_num) ? bbl_map[old_map[i].bbl_num].counter : 0;
        bbls.push_back(b);
        if (rtn_head || i == 0)
          rtn_first_bbl.push_back(bbls.size() - 1);
      }
      bbls.back().last = i;
      if (old_map[i].size)
        bbls.back().term = i;
    }
    rtn_first_bbl.push_back(bbls.size());      // end marker

    // Map an entry key (orig_ins_addr) to its BBL, for cond-branch targets.
    std::map<ADDRINT, unsigned> key_to_bbl;
    for (unsigned b = 0; b < bbls.size(); b++)
      for (unsigned i = bbls[b].first; i <= bbls[b].last; i++)
        if (old_map[i].orig_ins_addr)
          key_to_bbl.emplace(old_map[i].orig_ins_addr, b);

    // 2. Build the new instr_map, routine by routine.
    unsigned extra = 2 * bbls.size() + 16;        // at most 1 added jmp per BBL
    instr_map_t *new_map = (instr_map_t *)calloc(max_ins_count + extra, sizeof(instr_map_t));
    if (new_map == NULL) {
      perror("calloc");
      return -1;
    }
    instr_map = new_map;
    max_ins_count += extra;
    num_of_instr_map_entries = 0;
    std::vector<unsigned> new_index(old_num + 1, 0);

    for (unsigned r = 0; r + 1 < rtn_first_bbl.size(); r++) {
      unsigned rb = rtn_first_bbl[r], re = rtn_first_bbl[r + 1];   // BBLs [rb, re)

      // Can this routine be reordered?
      bool can_reorder = (re - rb) >= 3;
      bool has_cold = false, has_hot = false;
      UINT64 max_count = 0; // NEW: Track the peak heat of the entire routine

      for (unsigned b = rb + 1; b < re && can_reorder; b++) {
        if (bbls[b].count > max_count) max_count = bbls[b].count; // Update peak heat
        
        if (bbls[b].count) has_hot = true; else has_cold = true;
      }
      
      // CRITICAL FIX: Only reorder if the routine is definitively steady-state. 
      // If it hasn't executed heavily, we don't have enough data to safely reorder it.
     // if (max_count < 10000) {
      //    can_reorder = false;
      //}
	  
      for (unsigned b = rb; b < re && can_reorder; b++) {
        for (unsigned i = bbls[b].first; i <= bbls[b].last && can_reorder; i++) {
          if (!old_map[i].size || old_map[i].xed_category != XED_CATEGORY_COND_BR)
            continue;
          if (reverse_cond_iclass(entry_iclass(&old_map[i])) != XED_ICLASS_INVALID)
            continue;
          // LOOP / JRCXZ (or unknown): fine only inside its own BBL (like
          // the jrcxz of a de-virtualized jump), which is never split.
          std::map<ADDRINT, unsigned>::const_iterator t = key_to_bbl.find(old_map[i].orig_targ_addr);
          if (t == key_to_bbl.end() || t->second != b)
            can_reorder = false;
        }
      }
      // Nothing to gain unless a cold BBL sits before a hot one.
      bool cold_before_hot = false;
      for (unsigned b = rb + 1, seen_cold = 0; b < re && can_reorder; b++) {
        if (!bbls[b].count) seen_cold = 1;
        else if (seen_cold) cold_before_hot = true;
      }
      if (!has_cold || !has_hot || !cold_before_hot)
        can_reorder = false;

      // The layout order of this routine's BBLs.
      std::vector<unsigned> order;
      order.push_back(rb);                                 // entry BBL first
      for (unsigned b = rb + 1; b < re; b++)
        if (!can_reorder || bbls[b].count) order.push_back(b);   // hot BBLs
      if (can_reorder) {
        for (unsigned b = rb + 1; b < re; b++)
          if (!bbls[b].count) { order.push_back(b); num_moved_cold_bbls++; }  // cold BBLs
        num_reordered_rtns++;
      }

      for (unsigned k = 0; k < order.size(); k++) {
        const tc2_bbl_t &b = bbls[order[k]];
        // BBL placed right after this one (the next routine starts after the last).
        unsigned next_in_layout = (k + 1 < order.size()) ? order[k + 1] : re;
        unsigned orig_next = order[k] + 1;                 // the fall-through BBL F

        // How does this BBL end?
        bool falls_through = true, is_cond = false;
        if (b.term >= 0) {
          xed_category_enum_t cat = old_map[b.term].xed_category;
          if (cat == XED_CATEGORY_UNCOND_BR || cat == XED_CATEGORY_RET)
            falls_through = false;
          is_cond = (cat == XED_CATEGORY_COND_BR);
        }
        bool need_fix = can_reorder && falls_through && orig_next < bbls.size() &&
                        next_in_layout != orig_next;
        bool reverse = false;
        if (need_fix && is_cond) {
          std::map<ADDRINT, unsigned>::const_iterator t =
              key_to_bbl.find(old_map[b.term].orig_targ_addr);
          reverse = (t != key_to_bbl.end() && t->second == next_in_layout);
        }

        // Copy the BBL (with the cond branch reversed if needed).
        for (unsigned i = b.first; i <= b.last; i++) {
          new_index[i] = num_of_instr_map_entries;
          if (reverse && (int)i == b.term) {
            // 'jcc T' -> 'jncc F': F's first entry is the new target.
            if (emit_branch_to_key(reverse_cond_iclass(entry_iclass(&old_map[i])),
                                   old_map[bbls[orig_next].first].orig_ins_addr,
                                   old_map[i].bbl_num) < 0)
              return -1;
            instr_map[num_of_instr_map_entries - 1].orig_ins_addr = old_map[i].orig_ins_addr;
            num_reversed_cond_branches++;
            continue;
          }
          instr_map[num_of_instr_map_entries++] = old_map[i];
        }
        // Add 'jmp F' when F is no longer next and no reversal fixed it.
        if (need_fix && !reverse) {
          if (emit_branch_to_key(XED_ICLASS_JMP, old_map[bbls[orig_next].first].orig_ins_addr,
                                 old_map[b.last].bbl_num) < 0)
            return -1;
          num_added_fallthru_jumps++;
        }
        if (num_of_instr_map_entries + 2 >= max_ins_count)
          return -1;
      }
    }
    new_index[old_num] = num_of_instr_map_entries;

    // 3. Remap indices that point into instr_map.
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].starting_ins_entry <= old_num)
        bbl_map[b].starting_ins_entry = new_index[bbl_map[b].starting_ins_entry];
      if (bbl_map[b].terminating_ins_entry <= old_num)
        bbl_map[b].terminating_ins_entry = new_index[bbl_map[b].terminating_ins_entry];
    }
    free(old_map);
    return 0;
}


///The Loop Unrolling Algorithm
unsigned num_unrolled_loops = 0;

static int unroll_single_bbl_loops()
{
    unsigned old_num = num_of_instr_map_entries;
    if (!old_num) return 0;

    // Allocate a larger map to hold the duplicated instructions
    unsigned extra = max_ins_count / 2; 
    instr_map_t *new_map = (instr_map_t *)calloc(max_ins_count + extra, sizeof(instr_map_t));
    if (new_map == NULL) return -1;
    
    unsigned new_entries = 0;

    for (unsigned i = 0; i < old_num; i++) {
        // Find the start of a BBL
        unsigned bbl_start = i;
        unsigned bbl_end = i;
        while (bbl_end + 1 < old_num && instr_map[bbl_end + 1].bbl_num == instr_map[bbl_start].bbl_num) {
            bbl_end++;
        }

        // Check if the BBL ends with a conditional jump back to its own start
        if (instr_map[bbl_end].xed_category == XED_CATEGORY_COND_BR &&
            instr_map[bbl_end].orig_targ_addr == instr_map[bbl_start].orig_ins_addr) 
        {
            xed_iclass_enum_t cond_iclass = entry_iclass(&instr_map[bbl_end]);
            xed_iclass_enum_t rev_iclass = reverse_cond_iclass(cond_iclass);

            // The exit of the 1st copy jumps to the instr after the loop: it
            // needs a key (an instr added by the reordering has none).
            ADDRINT exit_key = (bbl_end + 1 < old_num) ? instr_map[bbl_end + 1].orig_ins_addr : 0;

            // Only unroll if we can reverse the condition (ignore LOOP/JRCXZ)
            if (rev_iclass != XED_ICLASS_INVALID && exit_key) {
                
                // --- UNROLL ITERATION 1 ---
                for (unsigned j = bbl_start; j < bbl_end; j++) {
                    new_map[new_entries++] = instr_map[j];
                }
                
                // Replace the jump with the reversed condition, pointing to the fall-through
                ADDRINT fallthrough_addr = exit_key;
                {
                    new_map[new_entries] = instr_map[bbl_end]; // Copy original entry
                    
                    xed_encoder_instruction_t enc_instr;
                    xed_inst1(&enc_instr, dstate, rev_iclass, 64, xed_relbr(0, 32));
                    
                    xed_encoder_request_t enc_req;
                    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
                    xed_convert_to_encoder_request(&enc_req, &enc_instr);
                    
                    unsigned int olen;
                    xed_encode(&enc_req, reinterpret_cast<UINT8*>(new_map[new_entries].encoded_ins), XED_MAX_INSTRUCTION_BYTES, &olen);
                    
                    new_map[new_entries].size = olen;
                    new_map[new_entries].orig_targ_addr = fallthrough_addr; 
                    new_map[new_entries].xed_category = XED_CATEGORY_COND_BR;
                    new_entries++;
                }

                // --- UNROLL ITERATION 2 ---
                for (unsigned j = bbl_start; j <= bbl_end; j++) {
                    new_map[new_entries] = instr_map[j];
                    // Strip the original address from the duplicate so chaining doesn't get confused
                    if (j != bbl_end) new_map[new_entries].orig_ins_addr = 0; 
                    new_entries++;
                }
                
                num_unrolled_loops++;
                i = bbl_end; // Skip to the end of the block
                continue;
            }
        }

        // If not unrolled, just copy normally
        for (unsigned j = bbl_start; j <= bbl_end; j++) {
            new_map[new_entries++] = instr_map[j];
        }
        i = bbl_end;
    }

    free(instr_map);
    instr_map = new_map;
    num_of_instr_map_entries = new_entries;
    max_ins_count += extra;

    return 0;
}


//Simple Leaf Function Inlining
unsigned num_inlined_calls = 0;

// Does the instr read or write rsp (explicitly, implicitly or in a memory operand)?
static bool uses_rsp(const instr_map_t *e)
{
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);
    if (xed_decode(&xedd, reinterpret_cast<const UINT8*>(e->encoded_ins), max_inst_len) != XED_ERROR_NONE)
      return true;
    const xed_inst_t *xi = xed_decoded_inst_inst(&xedd);
    for (unsigned k = 0; k < xed_inst_noperands(xi); k++) {
      xed_operand_enum_t n = xed_operand_name(xed_inst_operand(xi, k));
      if (xed_operand_is_register(n) || n == XED_OPERAND_BASE0 || n == XED_OPERAND_BASE1)
        if (xed_get_largest_enclosing_register(xed_decoded_inst_get_reg(&xedd, n)) == XED_REG_RSP)
          return true;
    }
    for (unsigned m = 0; m < xed_decoded_inst_number_of_memory_operands(&xedd); m++)
      if (xed_decoded_inst_get_base_reg(&xedd, m) == XED_REG_RSP ||
          xed_decoded_inst_get_index_reg(&xedd, m) == XED_REG_RSP)
        return true;
    return false;
}

static int inline_leaf_functions()
{
    unsigned old_num = num_of_instr_map_entries;
    if (!old_num) return 0;

    // 1. Build a quick lookup map for routine start addresses
    std::map<ADDRINT, unsigned> quick_entry_map;
    for (unsigned i = 0; i < old_num; i++) {
        if (instr_map[i].orig_ins_addr) {
            quick_entry_map.emplace(instr_map[i].orig_ins_addr, i);
        }
    }

    // 2. Allocate a larger map to hold the newly inlined instructions
    unsigned extra = max_ins_count / 2;
    instr_map_t *new_map = (instr_map_t *)calloc(max_ins_count + extra, sizeof(instr_map_t));
    if (new_map == NULL) return -1;
    
    unsigned new_entries = 0;

    for (unsigned i = 0; i < old_num; i++) {
        bool inlined = false;

        // Check if this is a direct CALL
        if (instr_map[i].xed_category == XED_CATEGORY_CALL && instr_map[i].orig_targ_addr) {
            auto it = quick_entry_map.find(instr_map[i].orig_targ_addr);
            if (it != quick_entry_map.end()) {
                unsigned targ_start = it->second;
                unsigned targ_end = targ_start;
                bool safe_to_inline = true;

                // Scan the target basic block to ensure it is a safe leaf function
                while (targ_end < old_num && instr_map[targ_end].bbl_num == instr_map[targ_start].bbl_num) {
                    xed_category_enum_t cat = instr_map[targ_end].xed_category;
                    
                    if (cat == XED_CATEGORY_RET) {
                        break; // End of the leaf function
                    }
                    // Abort if the function contains branches, nested calls, or touches the stack
                    if (cat == XED_CATEGORY_CALL || cat == XED_CATEGORY_UNCOND_BR || 
                        cat == XED_CATEGORY_COND_BR || cat == XED_CATEGORY_PUSH || 
                        cat == XED_CATEGORY_POP || cat == XED_CATEGORY_SYSCALL ||
                        (instr_map[targ_end].size && uses_rsp(&instr_map[targ_end]))) {
                        // Without the call, rsp is 8 bytes higher in the
                        // inlined code: stack accesses would be wrong.
                        safe_to_inline = false;
                        break;
                    }
                    targ_end++;
                }

                // If it's a small leaf function ending in RET, inline it
                if (safe_to_inline && targ_end < old_num && instr_map[targ_end].xed_category == XED_CATEGORY_RET &&
                    entry_iclass(&instr_map[targ_end]) == XED_ICLASS_RET_NEAR &&
                    instr_map[targ_end].size == 1) {      // plain 'ret' (no 'ret imm16')
                    unsigned func_length = targ_end - targ_start;
                    if (func_length <= 5) { // Threshold: Only inline functions with 5 or fewer instructions
                        for (unsigned j = targ_start; j < targ_end; j++) {
                            if (instr_map[j].xed_category == XED_CATEGORY_WIDENOP)
                                continue;          // the routine head NOP
                            new_map[new_entries] = instr_map[j];
                            // Strip the original address so the chaining logic ignores the duplicate
                            new_map[new_entries].orig_ins_addr = 0; 
                            new_entries++;
                        }
                        inlined = true;
                        num_inlined_calls++;
                    }
                }
            }
        }

        // If it wasn't a valid candidate for inlining, copy the instruction normally
        if (!inlined) {
            new_map[new_entries++] = instr_map[i];
        }
    }

    free(instr_map);
    instr_map = new_map;
    num_of_instr_map_entries = new_entries;
    max_ins_count += extra;

    return 0;
}


//Constant Propagation Algorithm
unsigned num_constant_propagations = 0;

static int apply_constant_propagation()
{
    unsigned old_num = num_of_instr_map_entries;
    if (!old_num) return 0;

    for (unsigned i = 0; i < old_num; i++) {
        if (!instr_map[i].size) continue;

        xed_decoded_inst_t xedd;
        xed_decoded_inst_zero_set_mode(&xedd, &dstate);
        if (xed_decode(&xedd, reinterpret_cast<const UINT8*>(instr_map[i].encoded_ins), max_inst_len) != XED_ERROR_NONE)
            continue;

        // 1. Identify "MOV reg, IMM"
        if (xed_decoded_inst_get_iclass(&xedd) == XED_ICLASS_MOV) {
            const xed_inst_t* xi = xed_decoded_inst_inst(&xedd);
            
            // Check if operand 0 is a register and operand 1 is an immediate
            if (xed_operand_name(xed_inst_operand(xi, 0)) == XED_OPERAND_REG0 &&
                xed_operand_name(xed_inst_operand(xi, 1)) == XED_OPERAND_IMM0) 
            {
                xed_reg_enum_t dest_reg = xed_decoded_inst_get_reg(&xedd, XED_OPERAND_REG0);
                
                // Ensure it's a standard General Purpose Register
                if (xed_reg_class(dest_reg) != XED_REG_CLASS_GPR) continue;
                
                xed_int64_t imm_val = (xed_int64_t)xed_decoded_inst_get_unsigned_immediate(&xedd);
                
                // 2. Look ahead in the SAME basic block to propagate the constant
                for (unsigned j = i + 1; j < old_num && instr_map[j].bbl_num == instr_map[i].bbl_num; j++) {
                    if (!instr_map[j].size) continue;

                    xed_decoded_inst_t next_xedd;
                    xed_decoded_inst_zero_set_mode(&next_xedd, &dstate);
                    if (xed_decode(&next_xedd, reinterpret_cast<const UINT8*>(instr_map[j].encoded_ins), max_inst_len) != XED_ERROR_NONE)
                        break;
                        
                    xed_iclass_enum_t next_iclass = xed_decoded_inst_get_iclass(&next_xedd);
                    const xed_inst_t* next_xi = xed_decoded_inst_inst(&next_xedd);
                    
                    // Check if the target instruction reads or writes our tracked register
                    bool reads_reg = false;
                    bool writes_reg = false;
                    for (unsigned k = 0; k < xed_inst_noperands(next_xi); k++) {
                        const xed_operand_t* op = xed_inst_operand(next_xi, k);
                        if (xed_operand_is_register(xed_operand_name(op))) {
                            xed_reg_enum_t op_reg = xed_decoded_inst_get_reg(&next_xedd, xed_operand_name(op));
                            // Check if the instruction touches our register (or a sub-register like AL vs RAX)
                            if (xed_get_largest_enclosing_register(op_reg) == xed_get_largest_enclosing_register(dest_reg)) {
                                if (xed_operand_read(op)) reads_reg = true;
                                if (xed_operand_written(op)) writes_reg = true;
                            }
                        }
                    }

                    // 3. Opportunity: Propagation!
                    // If it's a "MOV reg2, reg1" where reg1 is our known constant
                    if (next_iclass == XED_ICLASS_MOV && reads_reg && !writes_reg && 
                        xed_operand_name(xed_inst_operand(next_xi, 0)) == XED_OPERAND_REG0 &&
                        xed_operand_name(xed_inst_operand(next_xi, 1)) == XED_OPERAND_REG1) 
                    {
                        xed_reg_enum_t next_dest = xed_decoded_inst_get_reg(&next_xedd, XED_OPERAND_REG0);
                        xed_reg_enum_t next_src = xed_decoded_inst_get_reg(&next_xedd, XED_OPERAND_REG1);
                        
                        // If the source is exactly our tracked register and the immediate fits in 32 bits
                        if (next_src == dest_reg && imm_val >= -2147483648LL && imm_val <= 2147483647LL) {
                            
                            // RE-ENCODE AS: MOV next_dest, IMM
                            xed_encoder_instruction_t enc_instr;
                            xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 
                                      xed_decoded_inst_get_operand_width(&next_xedd),
                                      xed_reg(next_dest), xed_simm0((xed_int32_t)imm_val, 32));
                                      
                            xed_encoder_request_t enc_req;
                            xed_encoder_request_zero_set_mode(&enc_req, &dstate);
                            if (xed_convert_to_encoder_request(&enc_req, &enc_instr)) {
                                unsigned int olen = 0;
                                // Overwrite the instruction in-place
                                if (xed_encode(&enc_req, reinterpret_cast<UINT8*>(instr_map[j].encoded_ins), max_inst_len, &olen) == XED_ERROR_NONE) {
                                    instr_map[j].size = olen;
                                    num_constant_propagations++;
                                }
                            }
                        }
                    }

                    // 4. Stop tracking if the register is overwritten by anything else
                    if (writes_reg) break; 
                }
            }
        }
    }
    return 0;
}

// Remove unconditional direct jumps whose target is the next instr in the
// layout (reordering and de-virtualization can leave such jumps).
unsigned num_removed_jumps = 0;

static void remove_jumps_to_next()
{
    std::map<ADDRINT, unsigned> first_entry;       // key -> first entry (as chaining)
    for (unsigned i = 0; i < num_of_instr_map_entries; i++)
      if (instr_map[i].orig_ins_addr)
        first_entry.emplace(instr_map[i].orig_ins_addr, i);

    for (unsigned i = 0; i < num_of_instr_map_entries; i++) {
      instr_map_t &e = instr_map[i];
      if (!e.size || e.xed_category != XED_CATEGORY_UNCOND_BR || !e.orig_targ_addr)
        continue;
      if (entry_iclass(&e) != XED_ICLASS_JMP)
        continue;
      std::map<ADDRINT, unsigned>::const_iterator t = first_entry.find(e.orig_targ_addr);
      if (t == first_entry.end() || t->second <= i)
        continue;
      // Every entry between the jmp and its target must be empty.
      bool only_empty = true;
      for (unsigned j = i + 1; j < t->second; j++)
        if (instr_map[j].size) { only_empty = false; break; }
      if (!only_empty)
        continue;
      // Keep the entry (its key may be a branch target) but make it empty.
      e.size = 0;
      e.orig_targ_addr = 0;
      num_removed_jumps++;
    }
}

/* ============================================================= */
/* Service instr routines                                        */
/* ============================================================= *///{
bool isUncondJump(INS ins)
{
    const xed_decoded_inst_t* xedd = INS_XedDec(ins);
    xed_category_enum_t category_enum = xed_decoded_inst_get_category(xedd);
    if (category_enum == XED_CATEGORY_UNCOND_BR)
      return true;
    return false;
}

bool isJumpOrRet(INS ins)
{
   if (!INS_IsCall(ins) &&
       (INS_IsIndirectControlFlow(ins) ||
        INS_IsDirectControlFlow(ins) ||
        INS_IsRet(ins)))
     return true;

   return false;
}

bool isBackwardJump(INS ins)
{
  return (!INS_IsCall(ins) && INS_IsDirectControlFlow(ins) &&
          INS_DirectControlFlowTargetAddress(ins) < INS_Address(ins));
}

int create_nop7_xedd_instr(xed_decoded_inst_t *xedd)
{
  xed_encoder_instruction_t enc_instr;
  xed_encoder_request_t enc_req;
  char encoded_ins[XED_MAX_INSTRUCTION_BYTES];
  unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
  unsigned int olen = 0;
  
  xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP7, 64);
  
  xed_encoder_request_zero_set_mode(&enc_req, &dstate);
  xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
  if (!convert_ok) {
      cerr << "conversion to encode request failed" << endl;
      return -1;
  }
  xed_error_enum_t xed_error = xed_encode(&enc_req,
            reinterpret_cast<UINT8*>(encoded_ins), ilen, &olen);
  if (xed_error != XED_ERROR_NONE) {
      cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
    return -1;
  }
  xed_decoded_inst_zero_set_mode(xedd, &dstate);
  xed_error_enum_t xed_code = xed_decode(xedd, reinterpret_cast<UINT8*>(&encoded_ins), max_inst_len); // xed_decode(&xedd, nop7, max_inst_len);
  if (xed_code != XED_ERROR_NONE) {
      cerr << "DECODE ERROR: " << xed_error_enum_t2str(xed_code) << endl;
      return -1;;
  }
  return 0;
}

//}
/* ============================================================= */
/* Service dump routines                                         */
/* ============================================================= *///{

/*********************/
/* dump_image_instrs */
/*********************/
void dump_image_instrs(IMG img)
{
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {

            // Open the RTN.
            RTN_Open( rtn );

            cerr << RTN_Name(rtn) << ":" << endl;

            for( INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins) )
            {
                  cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) << endl;
            }

            // Close the RTN.
            RTN_Close( rtn );

            cerr << endl;
        }
    }
}


/*************************/
/* dump_instr_from_xedd */
/*************************/
void dump_instr_from_xedd (xed_decoded_inst_t* xedd, ADDRINT address)
{
    // debug print decoded instr:
    char disasm_buf[2048];

    xed_uint64_t runtime_address = static_cast<UINT64>(address);  // set the runtime adddress for disassembly

    xed_format_context(XED_SYNTAX_INTEL, xedd, disasm_buf, sizeof(disasm_buf), static_cast<UINT64>(runtime_address), 0, 0);

    cerr << hex << address << ": " << disasm_buf <<  endl;
}


/************************/
/* dump_instr_from_mem */
/************************/
void dump_instr_from_mem (ADDRINT *address, ADDRINT new_addr)
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;

  xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);

  xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);

  BOOL xed_ok = (xed_code == XED_ERROR_NONE);
  if (!xed_ok){
      cerr << "invalid opcode" << endl;
  }

  xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(new_addr), 0, 0);

  cerr << "0x" << hex << new_addr << ": " << disasm_buf <<  endl;

}


/****************************/
/*  dump_entire_instr_map() */
/****************************/
void dump_entire_instr_map()
{
    for (unsigned i=0; i < num_of_instr_map_entries; i++) {
      // Print the routine name if known.
      if (instr_map[i].ins_type == RtnHeadIns) {
        PIN_LockClient();
        RTN rtn = RTN_FindByAddress(instr_map[i].orig_ins_addr);
        if (rtn == RTN_Invalid()) {
            cerr << "Unknown"  << ":" << endl;
        } else {
            cerr << RTN_Name(rtn) << ":" << endl;
        }
        PIN_UnlockClient();
      }

      if (!instr_map[i].size)
        continue;


      dump_instr_from_mem ((ADDRINT *)instr_map[i].encoded_ins, instr_map[i].orig_ins_addr);
    }
}

/*******************/
/*  dump_profile() */
/*******************/
void dump_profile()
{
    for (unsigned i=0; i < num_of_instr_map_entries; i++) {
      // Do not print teh code if it has no profile counters
      if (!bbl_map[instr_map[i].bbl_num].counter &&
          !bbl_map[instr_map[i].bbl_num].fallthru_counter)
       continue;

      //Print a new line after each BBL.
      if (i > 0 && instr_map[i].bbl_num != instr_map[i - 1].bbl_num)
         *out << endl;

      // Print the routine name if known.
      if (instr_map[i].ins_type == RtnHeadIns) {
        PIN_LockClient();
        RTN rtn = RTN_FindByAddress(instr_map[i].orig_ins_addr);
        if (rtn == RTN_Invalid()) {
            *out << "Unknown"  << ":" << endl;
        } else {
            *out << RTN_Name(rtn) << ":" << endl;
        }
        PIN_UnlockClient();
      }

      if (!instr_map[i].size)
        continue;

      // Print non-empty profile info.
      if (bbl_map[instr_map[i].bbl_num].counter ||
          bbl_map[instr_map[i].bbl_num].fallthru_counter) {
        *out << " BBL heat: " << dec << bbl_map[instr_map[i].bbl_num].counter
             << " FT heat: " << dec << bbl_map[instr_map[i].bbl_num].fallthru_counter
             << " : ";
      }
      for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++) {
        if (bbl_map[instr_map[i].bbl_num].targ_addr[j] ||
            bbl_map[instr_map[i].bbl_num].targ_count[j]) {
          *out << " targ addr: 0x" << hex << bbl_map[instr_map[i].bbl_num].targ_addr[j]
               << " targ count: " << dec << bbl_map[instr_map[i].bbl_num].targ_count[j]
               << " : ";
        }
      }

      // Dump instr.
      char disasm_buf[2048];
      xed_decoded_inst_t new_xedd;
      
      xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);
      xed_error_enum_t xed_code =
        xed_decode(&new_xedd, 
                   reinterpret_cast<UINT8*>((ADDRINT *)instr_map[i].encoded_ins),
                   max_inst_len);
      BOOL xed_ok = (xed_code == XED_ERROR_NONE);
      if (!xed_ok){
          cerr << "invalid opcode" << endl;
      }
      
      xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048,
                         static_cast<UINT64>(instr_map[i].orig_ins_addr), 0, 0);
      *out << "0x" << hex << instr_map[i].orig_ins_addr << ": " << disasm_buf <<  endl;
    }
}

/**************************/
/* dump_instr_map_entry() */
/**************************/
void dump_instr_map_entry(unsigned instr_map_entry)
{
    cerr << dec << instr_map_entry << ": ";
    cerr << " orig_ins_addr: 0x" << hex << instr_map[instr_map_entry].orig_ins_addr;
    cerr << " new_ins_addr: 0x" << hex << instr_map[instr_map_entry].new_ins_addr;

    if (instr_map[instr_map_entry].orig_targ_addr) {
      cerr << " orig_targ_addr: 0x" << hex << instr_map[instr_map_entry].orig_targ_addr;
      ADDRINT new_targ_addr;
      if (instr_map[instr_map_entry].targ_map_entry >= 0)
          new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
      else
          new_targ_addr = instr_map[instr_map_entry].orig_targ_addr;
      cerr << " new_targ_addr: 0x" << hex << new_targ_addr;
    }

    cerr << "    new instr:";
    dump_instr_from_mem((ADDRINT *)instr_map[instr_map_entry].encoded_ins,
                        instr_map[instr_map_entry].new_ins_addr);
}


/*************/
/* dump_tc() */
/*************/
void dump_tc(char *tc, unsigned size_tc)
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;
  ADDRINT address = (ADDRINT)&tc[0];

  while (address < (ADDRINT)&tc[size_tc]) {

      xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);
      xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);

      BOOL xed_ok = (xed_code == XED_ERROR_NONE);
      if (!xed_ok){
          cerr << "invalid opcode" << endl;
          return;
      }

      xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(address), 0, 0);

      cerr << "0x" << hex << address << ": " << disasm_buf <<  endl;

      address += xed_decoded_inst_get_length (&new_xedd);
  }
}

//}
/* ============================================================= */
/* Translation routines                                         */
/* ============================================================= */

/****************************************************/
/* Encode a direct uncond jump from pc to targ_addr.*/
/****************************************************/
int encode_jump_instr(ADDRINT pc, ADDRINT target_addr, char *encoded_jmp_ins)
{
    xed_encoder_instruction_t enc_instr;
    xed_encoder_request_t enc_req;
    unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
    unsigned int olen = 0;
    
    xed_int64_t disp = target_addr - pc - olen;
    
    // Check if it is a short jump or a long jump.
    if (disp >= -128 && disp <= 127)
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JMP, 64, xed_relbr(disp, 8)); // Short jump.
    else        
      xed_inst1(&enc_instr, dstate,  XED_ICLASS_JMP, 64, xed_relbr(disp, 32)); // Long jump
    
    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
    xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
    if (!convert_ok) {
        cerr << "conversion to encode request failed" << endl;
        return -1;
    }           
    xed_error_enum_t xed_error = xed_encode(&enc_req,
              reinterpret_cast<UINT8*>(encoded_jmp_ins), ilen, &olen);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
      return -1;
    }
    
    disp = target_addr - pc - olen;
    
    // Check if it is a short jump or a long jump.
    if (disp >= -128 && disp <= 127)
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JMP, 64, xed_relbr(disp, 8)); // Short jump.
    else        
      xed_inst1(&enc_instr, dstate,  XED_ICLASS_JMP, 64, xed_relbr(disp, 32)); // Long jump.
    
    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
    convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
    if (!convert_ok) {
        cerr << "conversion to encode request failed" << endl;
        return -1;
    }         
    xed_error = xed_encode(&enc_req,
              reinterpret_cast<UINT8*>(encoded_jmp_ins), ilen, &olen);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
      return -1;
    }
    return olen;
}        


/********************************************/
/* Switch heads: race free patching of TC  */
/********************************************/
// The places in TC that are patched while the program runs (the head of
// every routine, and the head of every profiling stub) start with the 7
// bytes:
//     EB 05          jmp +5   ; skip the next jmp: the switch is off
//     E9 <rel32>     jmp target
// Turning the switch on: first write rel32 (not executed, as it is skipped),
// then change the single byte 05 to 00. The one byte store is atomic, so a
// thread running this code sees either the old or the new instruction,
// never a half written one. (Writing a 5 bytes jmp over a NOP7 while it is
// executed can crash the program.)
// If the target is at most 127 bytes after the 'jmp +5' (a short profiling
// stub), the single byte becomes the distance to the target instead, so a
// single short jmp skips the whole stub.
#define SWITCH_HEAD_SIZE 7

// Write a switch head over a NOP of 'size' >= 7 bytes (the bytes after the
// 7 of the switch become 1 byte NOPs).
static void write_switch_head(char *p, unsigned size)
{
    static const unsigned char head[SWITCH_HEAD_SIZE] = {0xEB, 0x05, 0xE9, 0, 0, 0, 0};
    memcpy(p, head, SWITCH_HEAD_SIZE);
    for (unsigned k = SWITCH_HEAD_SIZE; k < size; k++)
      p[k] = (char)0x90;
}

static int turn_on_switch_head(ADDRINT head, ADDRINT target)
{
    volatile unsigned char *p = (volatile unsigned char *)head;
    if (p[0] != 0xEB || p[2] != 0xE9)
      return -1;
    xed_int64_t short_rel = (xed_int64_t)target - (xed_int64_t)(head + 2);
    if (short_rel > 5 && short_rel <= 127) {
      __sync_synchronize();
      p[1] = (unsigned char)short_rel;   // jmp +5 -> jmp target
      __sync_synchronize();
      return 0;
    }
    xed_int64_t rel = (xed_int64_t)target - (xed_int64_t)(head + SWITCH_HEAD_SIZE);
    if (rel > 0x7FFFFFFF || rel < -0x7FFFFFFFLL)
      return -1;
    xed_int32_t rel32 = (xed_int32_t)rel;
    memcpy((void *)(head + 3), &rel32, 4);
    __sync_synchronize();
    p[1] = 0x00;                       // jmp +5 -> jmp +0: now jumps to target
    __sync_synchronize();
    return 0;
}

/***************************/
/* disable_profiling_in_tc */
/***************************/
int disable_profiling_in_tc(instr_map_t * instr_map, unsigned num_of_instr_map_entries)
{
    for (unsigned i = 0; i < num_of_instr_map_entries; i++) {
        // The head of a profiling code stub is a switch (see
        // write_switch_head()). Set its jump to the end of the stub and
        // turn it on.
        if (instr_map[i].ins_type == ProfilingIns &&
            instr_map[i].xed_category == XED_CATEGORY_WIDENOP &&
            instr_map[i].size >= SWITCH_HEAD_SIZE) {
            unsigned j = 1;
            ADDRINT stub_end = instr_map[i].new_ins_addr + instr_map[i].size;
            while (i + j < num_of_instr_map_entries && instr_map[i+j].ins_type == ProfilingIns) {
                stub_end += instr_map[i+j].size;
                j++;
            }
            if (turn_on_switch_head(instr_map[i].new_ins_addr, stub_end) < 0)
                return -1;
            i += (j - 1);
        }
    }
    return 0;
}

/*************************/
/* add_new_instr_entry() */
/*************************/
int add_new_instr_entry(xed_decoded_inst_t *xedd, ADDRINT pc, ins_enum_t ins_type)
{
    // copy target addr to instr map:
    ADDRINT orig_targ_addr = 0x0;

    // Check if the instruction has a branch displacement:
    xed_uint_t disp_byts = xed_decoded_inst_get_branch_displacement_width(xedd);
    xed_int32_t disp;
    if (disp_byts > 0) { // there is a branch offset.
      disp = xed_decoded_inst_get_branch_displacement(xedd);
      orig_targ_addr = pc + xed_decoded_inst_get_length (xedd) + disp;
    }

    // copy rip-relative addr to instr map:
    ADDRINT orig_rip_addr = 0x0;

    // check for a rip-relative displacement:
    unsigned memops = xed_decoded_inst_number_of_memory_operands(xedd);
    if (memops) {
      xed_reg_enum_t base_reg = xed_decoded_inst_get_base_reg(xedd, 0);
      if (base_reg == XED_REG_RIP) {
         unsigned size = xed_decoded_inst_get_length (xedd);
         xed_int64_t disp = xed_decoded_inst_get_memory_displacement(xedd, 0);
         orig_rip_addr = (ADDRINT)(pc + disp + size);
      }
    }

    // Converts the decoder request to a valid encoder request:
    xed_encoder_request_init_from_decode (xedd);

    unsigned new_size = 0;

    xed_error_enum_t xed_error =
       xed_encode (xedd, reinterpret_cast<UINT8*>(instr_map[num_of_instr_map_entries].encoded_ins),
                   max_inst_len , &new_size);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
        return -1;
    }

    // Add a new entry to instr_map:
    //
    instr_map[num_of_instr_map_entries].orig_ins_addr = pc;
    instr_map[num_of_instr_map_entries].new_ins_addr = 0x0;
    instr_map[num_of_instr_map_entries].orig_targ_addr = orig_targ_addr;
    instr_map[num_of_instr_map_entries].orig_rip_addr = orig_rip_addr;
    instr_map[num_of_instr_map_entries].targ_map_entry = -1;
    instr_map[num_of_instr_map_entries].size = new_size;
    instr_map[num_of_instr_map_entries].ins_type = ins_type;
    instr_map[num_of_instr_map_entries].bbl_num = bbl_num;
    instr_map[num_of_instr_map_entries].xed_category = xed_decoded_inst_get_category(xedd);

    num_of_instr_map_entries++;

    if (num_of_instr_map_entries >= max_ins_count) {
        cerr << "out of memory for map_instr" << endl;
        return -1;
    }

    // debug print new encoded instr:
    if (KnobVerbose) {
        cerr << "    new instr:";
        dump_instr_from_mem((ADDRINT *)instr_map[num_of_instr_map_entries-1].encoded_ins,
                            instr_map[num_of_instr_map_entries-1].new_ins_addr);
    }

    return new_size;
}

/***************************/
/* add_new_encoded_instr() */
/***************************/
int add_new_encoded_instr(ADDRINT ins_addr, xed_encoder_instruction_t *enc_instr, ins_enum_t ins_type) {
    char encoded_ins[XED_MAX_INSTRUCTION_BYTES];
    unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
    unsigned int olen = 0;
  
    // Convert the encoding instr to a valid encoder request.
    xed_encoder_request_t enc_req;    
    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
    xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, enc_instr);
    if (!convert_ok) {
        cerr << "conversion to encode request failed" << endl;
        return -1;
    }
    
    // Encode instr.
    xed_error_enum_t xed_error = xed_encode(&enc_req,
              reinterpret_cast<UINT8*>(encoded_ins), ilen, &olen);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
      return -1;
    }
  
    // Decode instr.
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd,&dstate);
    xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(&encoded_ins), max_inst_len);
    if (xed_code != XED_ERROR_NONE) {
        cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << ins_addr << endl;
        return -1;;
    }
    int rc = add_new_instr_entry(&xedd, ins_addr, ins_type);
    if (rc < 0) {
      cerr << "ERROR: failed during instructon translation." << endl;
      return -1;
    }
    return 0;
}

/**************************/
/* add_profiling_instrs() */
/**************************/
/*
int add_profiling_instrs(INS ins, ADDRINT ins_addr,
                         UINT64 *counter_addr, unsigned bbl_num)
{
  xed_encoder_instruction_t enc_instr;

  static uint64_t rax_mem = 0;
  
  // Add NOP instr (to be overwritten later on by a jmp that skips
  // the profiling, once profiling is done).
  xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP4, 64);
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // Save RAX:
  // MOV RAX into rax_mem
  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64), // Destination op.
            xed_reg(XED_REG_RAX));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // Create profiling for indirect jump targets.
  if (INS_IsIndirectControlFlow(ins) && !INS_IsRet(ins) && !INS_IsCall(ins)) {
    // Debug print.
    //cerr << " BBL terminates with indirect jump: "
    //     << " 0x" << hex << ins_addr << ": "
    //     << INS_Disassemble(ins) << "\n";

    static uint64_t rbx_mem = 0;
    static uint64_t rcx_mem = 0;

    // Retrieve the details about the mem operand.
    xed_decoded_inst_t *xedd = INS_XedDec(ins);
    xed_reg_enum_t base_reg = xed_decoded_inst_get_base_reg(xedd, 0);
    xed_reg_enum_t index_reg = xed_decoded_inst_get_index_reg(xedd, 0);
    xed_int64_t disp = xed_decoded_inst_get_memory_displacement(xedd, 0);
    xed_uint_t scale = xed_decoded_inst_get_scale(xedd, 0);
    xed_uint_t width = xed_decoded_inst_get_memory_displacement_width_bits(xedd, 0);
    unsigned mem_addr_width = xed_decoded_inst_get_memop_address_width(xedd, 0);
    
    xed_reg_enum_t targ_reg = XED_REG_INVALID;
    unsigned memops = xed_decoded_inst_number_of_memory_operands(xedd);
    if (!memops)
      targ_reg = xed_decoded_inst_get_reg(xedd, XED_OPERAND_REG0);

    // Debug print.
    //dump_instr_from_xedd(xedd, ins_addr);
    //cerr << " base reg: " << xed_reg_enum_t2str(base_reg)
    //     << " index reg " << xed_reg_enum_t2str(index_reg)
    //     << " scale: " << dec << scale
    //     << " disp: 0x" << hex << disp
    //     << " width: " << dec << width
    //     << " mem addr width: " << dec << mem_addr_width
    //     << " targ reg: " << targ_reg << xed_reg_enum_t2str(targ_reg)
    //     << "\n";
    
    // save RBX into rbx_mem in 2 steps via RAX
    // save RCX into rcx_mem in 2 steps via RAX
    // Convert jmp [base_reg + index_reg*scale] to: MOV RAX, [base_reg + index_reg*scale]
    //         Or convert jmp targ_reg to: MOV RAX, targ_reg ==> RAX holds jump targ addr
    // MOV RBX, RAX ==> Now RBX also holds targ addr
    // AND RAX, MAX_TARG_ADDR ==> RAX holds index i = 0..MAX_TARG_ADDRS
    // MOV RCX, xed_imm0((ADDRINT)&bbl_map_targ_addr[bbl_num][0])
    // MOV [RCX + 8*RAX], RBX
    // MOV RBX, xed_imm0((ADDRINT)&bbl_map_targ_count[bbl_num][0])
    // MOV RCX, [RBX + 8*RAX]
    // LEA RCX, [RCX + 1]
    // MOV [RBX + 8*RAX], RCX
    // restore RCX from rcx_mem in 2 steps via RAX
    // restore RBX from rbx_mem in 2 steps via RAX
    
    // Save RBX step 1 - MOV RBX into RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RAX),  // Destination op.
              xed_reg(XED_REG_RBX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Save RBX step 2 - MOV RAX into rbx_mem
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rbx_mem, 64), 64), // Destination op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Save RCX step 1 - MOV RCX into RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RAX),   // Destination op.
              xed_reg(XED_REG_RCX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Save RCX step 2 - MOV RAX into rcx_mem
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rcx_mem, 64), 64), // Destination op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Replace RIP reg by an absolute displacement.
    // Convert 'jmp [rax*8+0x657118]' or: 'jmp [rip+0x42513c]'
    // to: mov rax, [rax*8+0x657118] or: mov rax, [<absolute addr>]
    //
    // Check if we need to restore RAX in case  it is used as base reg or index reg,
    // e.g., jmp [RIP+8*RAX] or: jmp [RAX+8*RBX]
    
    // Check if we need to restore RAX from rax_mem.
    if (targ_reg == XED_REG_RAX || base_reg == XED_REG_RAX || index_reg == XED_REG_RAX) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX), // Destination reg op.
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;
    }
    // Check if we need to convert [RIP+disp+index*scale] to [absolute_disp + index*scale]
    if (base_reg == XED_REG_RIP) {
      unsigned int orig_size = xed_decoded_inst_get_length (xedd);
      // Modify rip displacement by an absolute displacement val.
      xed_int64_t new_disp = ins_addr + disp + orig_size;
      if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
         cerr << "Invalid rip displacement larger than 32 bits in add_profiling_instrs\n";
         return -1;
      }
      xed_int64_t new_disp_width = 32; // set maximal disp width for now.
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX),    // Destination reg op.
                xed_mem_bisd(XED_REG_INVALID, index_reg, scale, 
                             xed_disp(new_disp, new_disp_width),
                             mem_addr_width));
    } else if (targ_reg != XED_REG_RAX) { // avoid ceating the MOV RAX, RAX Nop.
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                 xed_reg(XED_REG_RAX),    // Destination reg op.
                 (targ_reg != XED_REG_INVALID ? xed_reg(targ_reg) :
                  xed_mem_bisd(base_reg, index_reg, scale, xed_disp(disp, width), mem_addr_width)));
    }
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV RBX, RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RBX),    // Destination reg op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // AND RAX, MAX_TARG_ADDRS. (NOTE: Modifies RFLAGS).
    xed_inst2(&enc_instr, dstate, XED_ICLASS_AND, 64,
              xed_reg(XED_REG_RAX),    // Destination reg op.
              xed_imm0(MAX_TARG_ADDRS, 8));  // keep only MAX_TARG_ADDRS+1 targets for profiling.
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV RCX, xed_imm0((ADDRINT)&bbl_map[bbl_num].targ_addr[0])
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RCX), // Destination reg op.
              xed_imm0((ADDRINT)&(bbl_map[bbl_num].targ_addr[0]), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV [RCX + 8*RAX], RBX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_mem_bisd(XED_REG_RCX, // base reg
                           XED_REG_RAX, //index reg
                           8, // scale
                           xed_disp(0, 32), // disp
                           64),  // Destination reg op.
              xed_reg(XED_REG_RBX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV RBX, xed_imm0((ADDRINT)&bbl_map[bbl_num].targ_count[0])
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RBX), // Destination reg op.
              xed_imm0((ADDRINT)&(bbl_map[bbl_num].targ_count[0]), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV RCX, [RBX + 8*RAX]
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RCX),   // Destination reg op.
              xed_mem_bisd(XED_REG_RBX, // base reg
                           XED_REG_RAX, //index reg
                           8, // scale
                           xed_disp(0, 32), // disp
                           64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // LEA RCX, [RCX + 1]
    xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64,
              xed_reg(XED_REG_RCX), // Destination reg op.
              xed_mem_bd(XED_REG_RCX, // base reg
                         xed_disp(1, 8), // disp
                         64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // MOV [RBX + 8*RAX], RCX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_mem_bisd(XED_REG_RBX, // base reg
                           XED_REG_RAX, //index reg
                           8, // scale
                           xed_disp(0, 32), // disp
                           64),     // Destination op.
              xed_reg(XED_REG_RCX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Restore RCX step 1- MOV from rcx_mem into RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RAX), // Destination op.
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rcx_mem, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Restore RCX step 2 - MOV RAX into RCX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RCX),  // Destination op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Restore RBX step 1 - MOV from rbx_mem into RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RAX), // Destination op.
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rbx_mem, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
    
    // Restore RBX step 2 - MOV RAX into RBX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RBX),  // Destination op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;

  } // end of: 'if bbl terminates with indirect jump'.
  
  // Create the profiling instrs for counting the BBL frequency.
  //

  // MOV from bbl_map into RAX
  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_reg(XED_REG_RAX),  // Destination reg op.
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // LEA RAX, [RAX+1]
  xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA,  64,  // operand width
            xed_reg(XED_REG_RAX), // Destination reg op.
            xed_mem_bd(XED_REG_RAX, xed_disp(1, 8), 64));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // MOV from RAX into bbl_map
  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64), // Destination op.
            xed_reg(XED_REG_RAX));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // Restore RAX:
  // MOV from rax_mem into RAX
  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_reg(XED_REG_RAX), // Destination reg op.
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;
 
  return 0;
}
*/



static UINT64 saved_counter_rax = 0;
static UINT64 saved_indirect_rax = 0;
static UINT64 saved_indirect_rbx = 0;
static UINT64 saved_indirect_rcx = 0;

static int findFastCounterIndex(const std::vector<INS>& block) {
    for (unsigned i = 0; i < block.size(); i++) {
        INS ins = block[i];
        if (INS_IsCall(ins) || INS_IsSyscall(ins) || INS_IsInterrupt(ins)) return -1;
        if (INS_Opcode(ins) == XED_ICLASS_CMP && !INS_IsPredicated(ins)) return static_cast<int>(i);
    }
    return -1;
}

int add_fast_counter_profiling_instrs(ADDRINT ins_addr, UINT64 *counter_addr) {
    xed_encoder_instruction_t enc_instr;
    xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP7, 64);
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    // The single, zero-overhead instruction
    xed_inst1(&enc_instr, dstate, XED_ICLASS_INC, 64, xed_mem_bd(XED_REG_RIP, xed_disp(0, 32), 64));
    
    const unsigned idx = num_of_instr_map_entries;
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    instr_map[idx].orig_rip_addr = reinterpret_cast<ADDRINT>(counter_addr);
    return 0;
}

int add_counter_profiling_instrs(ADDRINT ins_addr, UINT64 *counter_addr) {
    xed_encoder_instruction_t enc_instr;
    xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP7, 64);
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    // Fallback counter uses LEA to safely preserve flags without PUSHFQ
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_counter_rax, 64), 64), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_RAX, xed_disp(1, 8), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_counter_rax, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    return 0;
}

int add_indirect_target_profiling_instrs(INS ins, ADDRINT ins_addr, unsigned current_bbl_num) {
    if (!INS_IsIndirectControlFlow(ins) || INS_IsRet(ins)) return 0;
    
    xed_encoder_instruction_t enc_instr;
    xed_decoded_inst_t *xedd = INS_XedDec(ins);

    // Where does the instr take its target from? (Do not use the number of
    // memory operands: 'call reg' has one, the push of the return address.)
    indirect_target_t t;
    if (!get_indirect_target_operand(xedd, ins_addr, &t))
        return 0;                       // e.g. far jmp/call: not profiled
    xed_reg_enum_t base_reg = t.base_reg;
    xed_reg_enum_t index_reg = t.index_reg;
    xed_int64_t disp = t.disp;
    xed_uint_t scale = t.scale;
    xed_uint_t width = t.disp_width ? t.disp_width : 32;
    unsigned mem_addr_width = t.mem_addr_width ? t.mem_addr_width : 64;
    xed_reg_enum_t targ_reg = t.targ_reg;

    xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP7, 64);
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rax, 64), 64), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_reg(XED_REG_RBX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rbx, 64), 64), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_reg(XED_REG_RCX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rcx, 64), 64), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    if (targ_reg == XED_REG_RAX || base_reg == XED_REG_RAX || index_reg == XED_REG_RAX) {
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rax, 64), 64));
        if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    }

    if (base_reg == XED_REG_RIP) {
        // mov rax, [rip+disp]: the displacement is fixed like any other
        // rip-relative operand (from orig_rip_addr).
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX),
                  xed_mem_bd(XED_REG_RIP, xed_disp(0, 32), 64));
        if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
        instr_map[num_of_instr_map_entries - 1].orig_rip_addr = t.rip_mem_addr;
    } else if (targ_reg != XED_REG_RAX) {
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), (targ_reg != XED_REG_INVALID ? xed_reg(targ_reg) : xed_mem_bisd(base_reg, index_reg, scale, xed_disp(disp, width), mem_addr_width)));
        if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    }

    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RBX), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    // Save the flags (AND changes them). Move rsp below the 128 bytes red
    // zone first: a leaf function may keep its locals there, and pushfq
    // would overwrite them.
    xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RSP),
              xed_mem_bd(XED_REG_RSP, xed_disp(-128, 32), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst0(&enc_instr, dstate, XED_ICLASS_PUSHFQ, 64);
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_AND, 64, xed_reg(XED_REG_RAX), xed_imm0(MAX_TARG_ADDRS, 8));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RCX), xed_imm0((ADDRINT)&bbl_map[current_bbl_num].targ_addr[0], 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bisd(XED_REG_RCX, XED_REG_RAX, 8, xed_disp(0, 32), 64), xed_reg(XED_REG_RBX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RBX), xed_imm0((ADDRINT)&bbl_map[current_bbl_num].targ_count[0], 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RCX), xed_mem_bisd(XED_REG_RBX, XED_REG_RAX, 8, xed_disp(0, 32), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RCX), xed_mem_bd(XED_REG_RCX, xed_disp(1, 8), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_mem_bisd(XED_REG_RBX, XED_REG_RAX, 8, xed_disp(0, 32), 64), xed_reg(XED_REG_RCX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst0(&enc_instr, dstate, XED_ICLASS_POPFQ, 64);
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64, xed_reg(XED_REG_RSP),
              xed_mem_bd(XED_REG_RSP, xed_disp(128, 32), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;

    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rcx, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RCX), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rbx, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RBX), xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64, xed_reg(XED_REG_RAX), xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&saved_indirect_rax, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0) return -1;
    
    return 0;
}




/**************************************************/
/* chain_all_direct_jmp_and_call_target_entries() */
/**************************************************/
void chain_all_direct_jmp_and_call_target_entries(unsigned from_entry,
                                                 unsigned until_entry)
{
    entry_map.clear();

    for (unsigned i = from_entry; i < until_entry; i++) {
        instr_map[i].targ_map_entry = -1;
        ADDRINT orig_ins_addr = instr_map[i].orig_ins_addr;
        if (!orig_ins_addr)
          continue;
        // For instrs with same orig_addr, give precedence to the first one.
        entry_map.emplace(orig_ins_addr, i);
    }

    for (unsigned i = from_entry; i < until_entry; i++) {
        ADDRINT orig_targ_addr = instr_map[i].orig_targ_addr;
        if (orig_targ_addr == 0)
            continue;
        if (instr_map[i].targ_map_entry > 0)
            continue;
        if (!entry_map.count(orig_targ_addr))
            continue;
        if (!instr_map[i].size)
            continue;
        instr_map[i].targ_map_entry = entry_map[orig_targ_addr];
    }
}


/***********************************************/
/* set_initial_estimated_new_ins_addrs_in_tc() */
/***********************************************/
int set_initial_estimated_new_ins_addrs_in_tc(char *tc) {
  unsigned tc_cursor = 0;
  // Set initial estimated new addrs for each instruction in the tc.
  for (unsigned i=0; i < num_of_instr_map_entries; i++) {
    instr_map[i].new_ins_addr = (ADDRINT)&tc[tc_cursor];
    // update expected size of tc.
    tc_cursor += instr_map[i].size;
    // Check if we exceeded the TC size.
    if (tc_cursor >= max_tc_size)
      return -1;
  }
  return 0;
}


/**************************/
/* fix_rip_displacement() */
/**************************/
int fix_rip_displacement(unsigned instr_map_entry)
{
    // uncond jumps instructions with size=0
    // should remain with size=0 for beeing removed from tc
    if (!instr_map[instr_map_entry].size)
        return 0;

    // Check if it is a RIP-relative instr.
    if (!instr_map[instr_map_entry].orig_rip_addr)
      return 0;

    // Check if it is a direct jmp or call instruction.
    if (instr_map[instr_map_entry].orig_targ_addr != 0)
      return 0;

    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);

    xed_error_enum_t xed_code =
       xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
    if (xed_code != XED_ERROR_NONE) {
        cerr << "ERROR: xed decode failed for instr at: " << "0x"
             << hex << instr_map[instr_map_entry].new_ins_addr << endl;
        return -1;
    }

    //debug print:
    if (KnobVerbose) {
      cerr << " Before fixing rip offset\n";
      dump_instr_map_entry(instr_map_entry);
    }

    //xed_uint_t disp_byts = xed_decoded_inst_get_memory_displacement_width(xedd,i); // how many byts in disp ( disp length in byts - for example FFFFFFFF = 4
    xed_int64_t new_disp = 0;
    xed_uint_t new_disp_byts = 4;   // set maximal num of byts for now.

    // Modify rip displacement. use rip-relative direct addressing mode.
    new_disp = (xed_int64_t)(instr_map[instr_map_entry].orig_rip_addr - instr_map[instr_map_entry].new_ins_addr -
                               instr_map[instr_map_entry].size);
    // Code when using direct addressing mode.
    //xed_encoder_request_set_base0 (&xedd, XED_REG_INVALID);
    //new_disp = instr_map[instr_map_entry].orig_rip_addr;
    if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
        cerr << "Invalid rip displacement larger than 32 bits in fix_rip_displacement\n";
        dump_instr_map_entry(instr_map_entry);
        return -1;
    }

    // Set the memory displacement using a bit length.
    xed_encoder_request_set_memory_displacement (&xedd, new_disp, new_disp_byts);

    unsigned max_size = XED_MAX_INSTRUCTION_BYTES;
    unsigned new_size = 0;

    // Converts the decoder request to a valid encoder request:
    xed_encoder_request_init_from_decode (&xedd);

    xed_error_enum_t xed_error =
       xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins),
                   max_size , &new_size);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
        dump_instr_map_entry(instr_map_entry);
        return -1;
    }

    //debug print:
    if (KnobVerbose) {
      cerr << " After fixing rip offset\n";
      dump_instr_map_entry(instr_map_entry);
    }

    return new_size;
}


/**************************************/
/* fix_direct_jmp_or_call_to_orig_addr */
/**************************************/
int fix_direct_jmp_or_call_to_orig_addr(unsigned instr_map_entry)
{
    // Ignore instructions of zero size.
    if (!instr_map[instr_map_entry].size)
      return 0;

    // Debug print.
    //cerr << "jump to orig addr: 0x" << hex << instr_map[instr_map_entry].orig_targ_addr << " : ";
    //dump_instr_from_mem ((ADDRINT *)instr_map[instr_map_entry].encoded_ins,
    //                     instr_map[instr_map_entry].orig_ins_addr);

    // check for cases of direct jumps/calls back to the orginal target address:
    if (instr_map[instr_map_entry].targ_map_entry >= 0) {
        cerr << "ERROR: Invalid jump or call instruction" << endl;
        return -1;
    }

    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd,&dstate);

    xed_error_enum_t xed_code =
        xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
    if (xed_code != XED_ERROR_NONE) {
        cerr << "ERROR: xed decode failed for instr at: " << "0x"
             << hex << instr_map[instr_map_entry].new_ins_addr << endl;
        return -1;
    }

    xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);

    if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_UNCOND_BR) {
        cerr << "ERROR: Invalid direct jump from translated code to original code for:\n";
        dump_instr_map_entry(instr_map_entry);
        return -1;
    }

    unsigned ilen = XED_MAX_INSTRUCTION_BYTES;
    unsigned olen = 0;

    xed_encoder_instruction_t  enc_instr;

    // Use the heap variable instr_map[instr_map_entry].orig_targ_addr as the
    // memory container that holds the target address for the jmp/call
    // and indirectly jmp/call via that memory location.

    // search for orig_targ_addr in jump_to_orig_addr_map.
    int jump_to_orig_addr_map_entry = -1;
    for (unsigned i = 0; i < jump_to_orig_addr_num; i++) {
      if (instr_map[instr_map_entry].orig_targ_addr == jump_to_orig_addr_map[i]) {
        jump_to_orig_addr_map_entry = i;
        break;
      }
    }
    if (jump_to_orig_addr_map_entry < 0) {
      jump_to_orig_addr_num++;
      jump_to_orig_addr_map_entry = jump_to_orig_addr_num;
      if ((unsigned)jump_to_orig_addr_map_entry >= max_rtn_count) {
         cerr << "exceeded size of jump_to_orig_addr_map at fix_direct_jmp_or_call_to_orig_addr\n";
         return -1;
      }
      jump_to_orig_addr_map[jump_to_orig_addr_map_entry] = instr_map[instr_map_entry].orig_targ_addr;
    }

    xed_int64_t new_disp = (ADDRINT)&jump_to_orig_addr_map[jump_to_orig_addr_map_entry] -
                       instr_map[instr_map_entry].new_ins_addr -
                       xed_decoded_inst_get_length (&xedd);
    if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
        cerr << "Invalid rip displacement larger than 32 bits in fix_direct_jmp_or_call_to_orig_addr\n";
        cerr << "new displacement: " << dec << new_disp << "\n";
        return -1;
    }

    if (category_enum == XED_CATEGORY_CALL)
            xed_inst1(&enc_instr, dstate,
            XED_ICLASS_CALL_NEAR, 64,
            xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

    if (category_enum == XED_CATEGORY_UNCOND_BR)
            xed_inst1(&enc_instr, dstate,
            XED_ICLASS_JMP, 64,
            xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

    xed_encoder_request_t enc_req;

    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
    xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
    if (!convert_ok) {
        cerr << "conversion to encode request failed" << endl;
        return -1;
    }

    xed_error_enum_t xed_error =
       xed_encode(&enc_req, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), ilen, &olen);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
        dump_instr_map_entry(instr_map_entry);
        return -1;
    }

    // NOTE: We cannot zero the orig_targ_addr field in instr_map as follows:
    //  instr_map[instr_map_entry].orig_targ_addr = 0x0;
    // This is because the RIP displacement may become too large to fit into 4 bytes long.

    // debug prints:
    if (KnobVerbose) {
        dump_instr_map_entry(instr_map_entry);
    }

    return olen;
}


/**************************************/
/* fix_direct_jmp_or_call_displacement */
/**************************************/
int fix_direct_jmp_or_call_displacement(unsigned instr_map_entry)
{
    //uncond jumps instructions with size=0 should remain with size=0
    // for beeing removed from tc
    if (!instr_map[instr_map_entry].size)
        return 0;

    // Check if it is indeed a direct branch or a direct call instr:
    if (instr_map[instr_map_entry].orig_targ_addr == 0)
      return 0;

    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd,&dstate);

    xed_error_enum_t xed_code =
        xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
    if (xed_code != XED_ERROR_NONE) {
        cerr << "ERROR: xed decode failed for instr at: "
             << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
        return -1;
    }

    xed_int64_t  new_disp = 0;
    unsigned max_size = XED_MAX_INSTRUCTION_BYTES;
    unsigned new_size = 0;


    xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);

    if (category_enum != XED_CATEGORY_CALL &&
        category_enum != XED_CATEGORY_COND_BR &&
        category_enum != XED_CATEGORY_UNCOND_BR) {
        cerr << "ERROR: unrecognized branch displacement" << endl;
        return -1;
    }

    // fix direct branches/calls to original targ addresses or
    // indirect branches via a rip offset which had previously been
    // formed by previous calls to fix_direct_jmp_or_call_to_orig_addr()
    // in order to replace direct jumps to orig targ addrs.
    ADDRINT new_targ_addr;
    if (instr_map[instr_map_entry].targ_map_entry < 0) {
       // CRITICAL FIX: Safe fallback for unmapped conditional branches
       if (category_enum == XED_CATEGORY_COND_BR) {
           new_targ_addr = instr_map[instr_map_entry].orig_targ_addr;
       } else {
           int rc = fix_direct_jmp_or_call_to_orig_addr(instr_map_entry);
           return rc;
       }
    } else {
       new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
    }

    new_disp =
      (new_targ_addr - instr_map[instr_map_entry].new_ins_addr) - instr_map[instr_map_entry].size; // orig_size;
     if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
        cerr << "Invalid rip displacement larger than 32 bits in fix_direct_jmp_or_call_displacement\n";
        return -1;
    }

    xed_uint_t   new_disp_byts = 4; // num_of_bytes(new_disp);  ???

    // the max displacement size of loop instructions is 1 byte:
    xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(&xedd);
    if (iclass_enum == XED_ICLASS_LOOP ||
        iclass_enum == XED_ICLASS_LOOPE ||
        iclass_enum == XED_ICLASS_LOOPNE) {
      new_disp_byts = 1;
    }

    // the max displacement size of jecxz instructions is ???:
    xed_iform_enum_t iform_enum = xed_decoded_inst_get_iform_enum (&xedd);
    if (iform_enum == XED_IFORM_JRCXZ_RELBRb){
      new_disp_byts = 1;
    }

    // Converts the decoder request to a valid encoder request:
    xed_encoder_request_init_from_decode (&xedd);

    //Set the branch displacement:
    xed_encoder_request_set_branch_displacement (&xedd, new_disp, new_disp_byts);

    //xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
    //xed_error_enum_t xed_error = xed_encode (&xedd, enc_buf, max_size , &new_size);
    xed_error_enum_t xed_error =
        xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_size, &new_size);
    if (xed_error != XED_ERROR_NONE) {
        cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
        char buf[2048];
        xed_format_context(XED_SYNTAX_INTEL, &xedd, buf, 2048,
                           static_cast<UINT64>(instr_map[instr_map_entry].orig_ins_addr), 0, 0);
        cerr << " instr: " << "0x" << hex << instr_map[instr_map_entry].orig_ins_addr << " : " << buf <<  endl;
          return -1;
    }

    //debug print of new instruction in tc:
    if (KnobVerbose) {
        dump_instr_map_entry(instr_map_entry);
    }

    return new_size;
}

/************************************/
/* fix_instructions_displacements() */
/************************************/
int fix_instructions_displacements()
{
   // fix displacemnets of direct branch or call instructions:

    int size_diff = 0;
    bool is_diff = false;

    do {

        size_diff = 0;
        is_diff = false;

        if (KnobVerbose) {
            cerr << "starting a pass of fixing instructions displacements: " << endl;
        }

        for (unsigned i=0; i < num_of_instr_map_entries; i++) {

            instr_map[i].new_ins_addr += size_diff;

            // fix rip displacement:
            int new_size = fix_rip_displacement(i);
            if (new_size) {
              if (new_size < 0)
                  return -1;
              if (instr_map[i].size != (unsigned)new_size) { // this was a rip-based instruction which was fixed.
                  if (instr_map[i].size < (unsigned)new_size)
                     size_diff += (new_size - instr_map[i].size);
                  else
                     size_diff -= (instr_map[i].size - new_size);
                  instr_map[i].size = (unsigned)new_size;
                  is_diff = true;
                  continue;
              }
            }

            // fix instr displacement for direct jump or call:
            new_size = fix_direct_jmp_or_call_displacement(i);
            if (new_size) {
              if (new_size < 0)
                  return -1;
              if (instr_map[i].size != (unsigned)new_size) {
                if (instr_map[i].size < (unsigned)new_size)
                   size_diff += (new_size - instr_map[i].size);
                else
                   size_diff -= (instr_map[i].size - new_size);
                instr_map[i].size = (unsigned)new_size;
                is_diff = true;
                continue;
              }
            }

        }  // end int i=0; i ..

    } while (is_diff);

   return 0;
 }


/* ============================================================= */
/* Filter	                               			             */
/* ============================================================= */

static bool StartsWith(const string& s, const string& prefix) {
    return s.compare(0, prefix.size(), prefix) == 0;
}

static bool ShouldSkipRoutineByName(const string& name) {
    static const char* exact_skip[] = {
        "_init", "_start", "_fini", "deregister_tm_clones", "register_tm_clones",
        "__do_global_dtors_aux", "frame_dummy", "__libc_csu_init", "__libc_csu_fini",
        "_exit", "exit", "abort", "open", "open64", "__open", "__open64", "openat", "__openat",
        "close", "__close", "read", "__read", "write", "__write", "lseek", "lseek64",
        "stat", "stat64", "lstat", "lstat64", "fstat", "fstat64", "isatty", "uname",
        "mmap", "mmap64", "munmap", "mprotect", "brk", "sbrk", "malloc", "free", "calloc", "realloc",
        "fopen", "fopen64", "fclose", "fread", "fwrite", "fflush", "fseek", "ftell",
        "memcpy", "memmove", "memset", "strlen", "strcmp", "strcpy"
    };
    for (unsigned i = 0; i < sizeof(exact_skip) / sizeof(exact_skip[0]); i++) {
        if (name == exact_skip[i]) return true;
    }
    if (StartsWith(name, "_dl_") || StartsWith(name, "_IO_") || StartsWith(name, "__libc_") ||
        StartsWith(name, "__GI_") || StartsWith(name, "_Unwind_") ||
        name.find("syscall") != string::npos || name.find("freeres") != string::npos) {
        return true;
    }
    return false;
}

unsigned min_rtn_size_for_translation = 120;

static bool IsCandidateRtnForTranslation(RTN rtn, ADDRINT& last_selected_rtn_end) {
    if (!RTN_Valid(rtn)) return false;
    string rtn_name = RTN_Name(rtn);
    if (ShouldSkipRoutineByName(rtn_name)) return false;
    if (rtn_name.find(".plt") != string::npos || rtn_name.find("@plt") != string::npos) return false;
    if (!RTN_IsSafeForProbedReplacement(rtn)) return false;
    ADDRINT rtn_addr = RTN_Address(rtn);
    USIZE rtn_size = RTN_Size(rtn);
    if (rtn_size < min_rtn_size_for_translation) return false;
    if (last_selected_rtn_end != 0 && rtn_addr < last_selected_rtn_end) return false;
    last_selected_rtn_end = rtn_addr + rtn_size;
    return true;
}


/* ============================================================= */
/* Optimization: Register Promotion of stack slots               */
/* ============================================================= */
//
// Code compiled without optimizations (like bzip2) keeps every local
// variable in a stack slot [rbp+disp] and loads/stores it at every use.
// In a routine that does not call other routines, a local variable slot
// can live in a free caller-saved register instead: every access
// 'op ..., [rbp+disp]' is rewritten to 'op ..., reg' (same instr, same
// width, same flags).
//
// A routine is a candidate when (checked on the original code):
//   - it starts with 'push rbp; mov rbp, rsp' and does not change rbp later
//     (except 'pop rbp' / 'leave' at the exits),
//   - it has no call, syscall, interrupt or indirect jump (control never
//     leaves the routine, except by 'ret'), and all its direct branches
//     target the routine itself,
//   - it has no explicit rsp-based memory operand.
// A stack slot [rbp+disp] (disp < 0) is a candidate when
//   - every access to it is a non-indexed [rbp+disp] operand of the same
//     width (1/2/4/8 bytes) in an instr whose memory operand can be
//     replaced by a register,
//   - no other access overlaps it and its address is not taken
//     ('lea reg, [rbp+d]' with d <= disp).
// The free registers are the caller-saved GPRs that the routine never
// reads or writes (implicitly or explicitly). A caller-saved GPR whose only
// use is the spill of an argument 'mov [rbp+disp], reg' (in the prologue)
// holds that slot from the routine entry on, so the slot can stay in it
// (coalescing) without using a free register.
//
// THRESHOLD: at TC2 generation the slots of each routine are ordered by
// their profiled number of accesses (sum of the BBL counters of their
// accesses). The hottest slots get the free registers, as long as a slot
// was accessed at least PROMO_MIN_ACCESSES times during the profiling.

#define PROMO_MIN_ACCESSES 100

typedef struct {
    INT64 disp;
    unsigned width;                   // bytes
    std::vector<ADDRINT> accesses;    // original addresses of the accessing instrs
} promo_slot_t;

typedef struct {
    ADDRINT rtn_addr;
    std::vector<xed_reg_enum_t> free_regs;   // 64-bit names, in order of preference
    std::map<INT64, xed_reg_enum_t> coalesce; // slot disp -> the arg reg spilled into it
    std::vector<promo_slot_t> slots;
} promo_rtn_t;

typedef struct {
    UINT8 encoded_ins[XED_MAX_INSTRUCTION_BYTES];
    unsigned size;
} promoted_access_t;

static std::vector<promo_rtn_t> promo_rtns;                   // filled at TC generation
static std::map<ADDRINT, promoted_access_t> promoted_access;  // orig addr -> new encoding
unsigned num_promoted_slots = 0;
unsigned num_promoted_rtns = 0;
unsigned num_register_promotions = 0;

// Caller-saved GPRs that may hold a promoted slot, in order of preference.
static const xed_reg_enum_t promo_reg_pool[] = {
    XED_REG_R11, XED_REG_R10, XED_REG_R9, XED_REG_R8,
    XED_REG_RDI, XED_REG_RSI, XED_REG_RCX, XED_REG_RDX
};

// The 'width' bytes part of a 64-bit GPR of promo_reg_pool.
static xed_reg_enum_t promo_sub_reg(xed_reg_enum_t r64, unsigned width)
{
    static const xed_reg_enum_t tab[][4] = {
      {XED_REG_R11B, XED_REG_R11W, XED_REG_R11D, XED_REG_R11},
      {XED_REG_R10B, XED_REG_R10W, XED_REG_R10D, XED_REG_R10},
      {XED_REG_R9B,  XED_REG_R9W,  XED_REG_R9D,  XED_REG_R9},
      {XED_REG_R8B,  XED_REG_R8W,  XED_REG_R8D,  XED_REG_R8},
      {XED_REG_DIL,  XED_REG_DI,   XED_REG_EDI,  XED_REG_RDI},
      {XED_REG_SIL,  XED_REG_SI,   XED_REG_ESI,  XED_REG_RSI},
      {XED_REG_CL,   XED_REG_CX,   XED_REG_ECX,  XED_REG_RCX},
      {XED_REG_DL,   XED_REG_DX,   XED_REG_EDX,  XED_REG_RDX},
    };
    int w = (width == 1) ? 0 : (width == 2) ? 1 : (width == 4) ? 2 : (width == 8) ? 3 : -1;
    if (w < 0)
      return XED_REG_INVALID;
    for (unsigned i = 0; i < sizeof(tab) / sizeof(tab[0]); i++)
      if (tab[i][3] == r64)
        return tab[i][w];
    return XED_REG_INVALID;
}

// Instrs whose [mem] operand can be replaced by a register of the same width.
static bool promo_iclass_ok(xed_iclass_enum_t c)
{
    switch (c) {
      case XED_ICLASS_MOV:   case XED_ICLASS_ADD:  case XED_ICLASS_SUB:
      case XED_ICLASS_AND:   case XED_ICLASS_OR:   case XED_ICLASS_XOR:
      case XED_ICLASS_CMP:   case XED_ICLASS_TEST: case XED_ICLASS_ADC:
      case XED_ICLASS_SBB:   case XED_ICLASS_INC:  case XED_ICLASS_DEC:
      case XED_ICLASS_NEG:   case XED_ICLASS_NOT:  case XED_ICLASS_IMUL:
      case XED_ICLASS_SHL:   case XED_ICLASS_SHR:  case XED_ICLASS_SAR:
      case XED_ICLASS_MOVZX: case XED_ICLASS_MOVSX: case XED_ICLASS_MOVSXD:
      case XED_ICLASS_CVTSI2SD: case XED_ICLASS_CVTSI2SS:
        return true;
      default:
        return false;
    }
}

static bool is_flags_reg(xed_reg_enum_t r)
{
    return r == XED_REG_FLAGS || r == XED_REG_EFLAGS || r == XED_REG_RFLAGS;
}

// Non-memory operands of a decoded instr, in order: registers (explicit and
// implicit, except flags) and immediates. The memory operand appears as
// 'mem_as' (a register), so the operands of 'op [slot]' and of the rewritten
// 'op reg' can be compared.
static void promo_operand_sig(const xed_decoded_inst_t *xedd, xed_reg_enum_t mem_as,
                              std::vector<UINT64> &sig)
{
    const xed_inst_t *xi = xed_decoded_inst_inst(xedd);
    for (unsigned k = 0; k < xed_inst_noperands(xi); k++) {
      const xed_operand_t *op = xed_inst_operand(xi, k);
      xed_operand_enum_t n = xed_operand_name(op);
      UINT64 rw = (xed_operand_read(op) ? 1 : 0) | (xed_operand_written(op) ? 2 : 0);
      if (n == XED_OPERAND_MEM0) {
        sig.push_back(((UINT64)mem_as << 8) | rw);
      } else if (xed_operand_is_register(n)) {
        xed_reg_enum_t r = xed_decoded_inst_get_reg(xedd, n);
        if (is_flags_reg(r))
          continue;
        sig.push_back(((UINT64)r << 8) | rw);
      } else if (n == XED_OPERAND_IMM0) {
        sig.push_back(0xFFFF0000ULL | xed_decoded_inst_get_immediate_width_bits(xedd));
        sig.push_back(xed_decoded_inst_get_unsigned_immediate(xedd));
      } else if (n == XED_OPERAND_IMM1) {
        sig.push_back(0xFFFE0000ULL);
      }
    }
}

// Rewrite 'op ..., [rbp+disp]' (encoded in 'bytes') to use 'reg64' (its
// sub-register of the operand width) instead of the memory operand.
// The result is decoded again and compared with the original instr.
// Returns false if the instr cannot be rewritten.
static bool promo_rewrite(const UINT8 *bytes, xed_reg_enum_t reg64,
                          UINT8 *out, unsigned *out_size)
{
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);
    if (xed_decode(&xedd, bytes, XED_MAX_INSTRUCTION_BYTES) != XED_ERROR_NONE)
      return false;
    xed_iclass_enum_t iclass = xed_decoded_inst_get_iclass(&xedd);
    if (!promo_iclass_ok(iclass) || xed_decoded_inst_number_of_memory_operands(&xedd) != 1)
      return false;
    if (xed_decoded_inst_get_base_reg(&xedd, 0) != XED_REG_RBP ||
        xed_decoded_inst_get_index_reg(&xedd, 0) != XED_REG_INVALID)
      return false;
    xed_reg_enum_t seg = xed_decoded_inst_get_seg_reg(&xedd, 0);
    if (seg != XED_REG_INVALID && seg != XED_REG_SS && seg != XED_REG_DS)
      return false;
    if (xed_operand_values_has_lock_prefix(xed_decoded_inst_operands_const(&xedd)) ||
        xed_operand_values_has_rep_prefix(xed_decoded_inst_operands_const(&xedd)))
      return false;

    unsigned width = xed_decoded_inst_get_memory_operand_length(&xedd, 0);
    xed_reg_enum_t reg = promo_sub_reg(reg64, width);
    if (reg == XED_REG_INVALID)
      return false;

    // Build the new instr from the explicit operands.
    const xed_inst_t *xi = xed_decoded_inst_inst(&xedd);
    xed_encoder_operand_t ops[XED_ENCODER_OPERANDS_MAX];
    unsigned nops = 0;
    bool has_mem = false;
    for (unsigned k = 0; k < xed_inst_noperands(xi); k++) {
      const xed_operand_t *op = xed_inst_operand(xi, k);
      if (xed_operand_operand_visibility(op) != XED_OPVIS_EXPLICIT)
        continue;
      xed_operand_enum_t n = xed_operand_name(op);
      if (nops >= XED_ENCODER_OPERANDS_MAX)
        return false;
      if (n == XED_OPERAND_MEM0) {
        ops[nops++] = xed_reg(reg);
        has_mem = true;
      } else if (xed_operand_is_register(n)) {
        ops[nops++] = xed_reg(xed_decoded_inst_get_reg(&xedd, n));
      } else if (n == XED_OPERAND_IMM0) {
        unsigned iw = xed_decoded_inst_get_immediate_width_bits(&xedd);
        if (xed_decoded_inst_get_immediate_is_signed(&xedd))
          ops[nops++] = xed_simm0(xed_decoded_inst_get_signed_immediate(&xedd), iw);
        else
          ops[nops++] = xed_imm0(xed_decoded_inst_get_unsigned_immediate(&xedd), iw);
      } else {
        return false;
      }
    }
    if (!has_mem)
      return false;

    xed_encoder_instruction_t enc_instr;
    xed_inst(&enc_instr, dstate, iclass, xed_decoded_inst_get_operand_width(&xedd), nops, ops);
    xed_encoder_request_t enc_req;
    xed_encoder_request_zero_set_mode(&enc_req, &dstate);
    if (!xed_convert_to_encoder_request(&enc_req, &enc_instr))
      return false;
    unsigned olen = 0;
    if (xed_encode(&enc_req, out, XED_MAX_INSTRUCTION_BYTES, &olen) != XED_ERROR_NONE)
      return false;

    // Check: same instr, the memory operand replaced by 'reg'.
    xed_decoded_inst_t nxedd;
    xed_decoded_inst_zero_set_mode(&nxedd, &dstate);
    if (xed_decode(&nxedd, out, olen) != XED_ERROR_NONE)
      return false;
    if (xed_decoded_inst_get_iclass(&nxedd) != iclass ||
        xed_decoded_inst_number_of_memory_operands(&nxedd) != 0 ||
        xed_decoded_inst_get_operand_width(&nxedd) != xed_decoded_inst_get_operand_width(&xedd))
      return false;
    std::vector<UINT64> sig_old, sig_new;
    promo_operand_sig(&xedd, reg, sig_old);
    promo_operand_sig(&nxedd, XED_REG_INVALID, sig_new);
    if (sig_old != sig_new)
      return false;

    *out_size = olen;
    return true;
}

// Analyze one routine, given its decoded instrs in address order.
// Adds a promo_rtn_t to promo_rtns if it has candidate slots.
static void analyze_promotable_slots(ADDRINT rtn_addr, USIZE rtn_size,
                                     const std::vector<ADDRINT> &addrs,
                                     const std::vector<const xed_decoded_inst_t *> &insns)
{
    unsigned n = insns.size();
    if (n < 3)
      return;

    // Prologue: push rbp; mov rbp, rsp
    const xed_decoded_inst_t *p0 = insns[0], *p1 = insns[1];
    if (xed_decoded_inst_get_iclass(p0) != XED_ICLASS_PUSH ||
        xed_decoded_inst_get_reg(p0, XED_OPERAND_REG0) != XED_REG_RBP)
      return;
    if (xed_decoded_inst_get_iclass(p1) != XED_ICLASS_MOV ||
        xed_decoded_inst_get_reg(p1, XED_OPERAND_REG0) != XED_REG_RBP ||
        xed_decoded_inst_get_reg(p1, XED_OPERAND_REG1) != XED_REG_RSP)
      return;

    std::map<xed_reg_enum_t, unsigned> touches;   // full reg -> number of instrs using it
    std::map<xed_reg_enum_t, INT64> spill_of;     // full reg -> slot, if its only use is a spill
    std::map<INT64, promo_slot_t> slots;     // disp -> slot
    std::set<INT64> bad_disps;               // slots that cannot be promoted
    std::vector<std::pair<INT64, unsigned> > ranges;   // all [rbp+disp] accesses
    INT64 min_lea_disp = 1;                  // lowest 'lea reg, [rbp+d]' (1 = none)

    for (unsigned i = 0; i < n; i++) {
      const xed_decoded_inst_t *x = insns[i];
      xed_category_enum_t cat = xed_decoded_inst_get_category(x);
      xed_iclass_enum_t iclass = xed_decoded_inst_get_iclass(x);

      if (cat == XED_CATEGORY_CALL || cat == XED_CATEGORY_SYSCALL ||
          cat == XED_CATEGORY_INTERRUPT || cat == XED_CATEGORY_SYSTEM)
        return;
      if (cat == XED_CATEGORY_UNCOND_BR || cat == XED_CATEGORY_COND_BR) {
        if (!xed_decoded_inst_get_branch_displacement_width(x))
          return;                                          // indirect jump
        ADDRINT t = addrs[i] + xed_decoded_inst_get_length(x) +
                    xed_decoded_inst_get_branch_displacement(x);
        if (t < rtn_addr || t >= rtn_addr + rtn_size)
          return;                                          // leaves the routine
      }
      if (cat == XED_CATEGORY_RET && iclass != XED_ICLASS_RET_NEAR)
        return;

      // Is it 'mov [rbp+disp], reg' (a spill of reg)?
      if (iclass == XED_ICLASS_MOV && xed_decoded_inst_number_of_memory_operands(x) == 1 &&
          xed_decoded_inst_mem_written(x, 0) &&
          xed_decoded_inst_get_base_reg(x, 0) == XED_REG_RBP &&
          xed_decoded_inst_get_index_reg(x, 0) == XED_REG_INVALID &&
          xed_operand_name(xed_inst_operand(xed_decoded_inst_inst(x), 1)) == XED_OPERAND_REG0) {
        xed_reg_enum_t src = xed_decoded_inst_get_reg(x, XED_OPERAND_REG0);
        spill_of[xed_get_largest_enclosing_register(src)] = xed_decoded_inst_get_memory_displacement(x, 0);
      }

      // Registers used by the instr.
      std::set<xed_reg_enum_t> used;
      const xed_inst_t *xi = xed_decoded_inst_inst(x);
      for (unsigned k = 0; k < xed_inst_noperands(xi); k++) {
        const xed_operand_t *op = xed_inst_operand(xi, k);
        xed_operand_enum_t on = xed_operand_name(op);
        if (!xed_operand_is_register(on) && on != XED_OPERAND_BASE0 && on != XED_OPERAND_BASE1)
          continue;
        xed_reg_enum_t r = xed_decoded_inst_get_reg(x, on);
        if (r == XED_REG_INVALID)
          continue;
        xed_reg_enum_t full = xed_get_largest_enclosing_register(r);
        used.insert(full);
        if (full == XED_REG_RBP && xed_operand_written(op) && i != 1 &&
            iclass != XED_ICLASS_POP && iclass != XED_ICLASS_LEAVE)
          return;                                          // rbp changes
      }

      for (std::set<xed_reg_enum_t>::iterator u = used.begin(); u != used.end(); ++u)
        touches[*u]++;

      // Memory operands.
      unsigned memops = xed_decoded_inst_number_of_memory_operands(x);
      bool agen = (iclass == XED_ICLASS_LEA);
      for (unsigned m = 0; m < memops; m++) {
        xed_reg_enum_t base = xed_decoded_inst_get_base_reg(x, m);
        xed_reg_enum_t index = xed_decoded_inst_get_index_reg(x, m);
        if (base != XED_REG_INVALID) used.insert(xed_get_largest_enclosing_register(base));
        if (index != XED_REG_INVALID) used.insert(xed_get_largest_enclosing_register(index));
        if (base == XED_REG_RSP || base == XED_REG_ESP) {
          if (iclass == XED_ICLASS_PUSH || iclass == XED_ICLASS_POP ||
              iclass == XED_ICLASS_RET_NEAR || iclass == XED_ICLASS_LEAVE)
            continue;                                      // implicit stack access
          return;
        }
        if (base != XED_REG_RBP) {
          if (base == XED_REG_EBP || index == XED_REG_RBP || index == XED_REG_EBP)
            return;
          continue;
        }
        INT64 disp = xed_decoded_inst_get_memory_displacement(x, m);
        if (agen) {
          if (disp < min_lea_disp) min_lea_disp = disp;
          continue;
        }
        if (index != XED_REG_INVALID)
          continue;                                        // array element
        unsigned width = xed_decoded_inst_get_memory_operand_length(x, m);
        ranges.push_back(std::make_pair(disp, width));

        bool ok = (memops == 1 && disp < 0 && promo_iclass_ok(iclass) &&
                   (width == 1 || width == 2 || width == 4 || width == 8));
        promo_slot_t &s = slots[disp];
        if (s.accesses.empty()) { s.disp = disp; s.width = width; }
        if (!ok || s.width != width)
          bad_disps.insert(disp);
        s.accesses.push_back(addrs[i]);
      }
    }

    promo_rtn_t pr;
    pr.rtn_addr = rtn_addr;
    for (unsigned k = 0; k < sizeof(promo_reg_pool) / sizeof(promo_reg_pool[0]); k++) {
      xed_reg_enum_t r = promo_reg_pool[k];
      if (!touches.count(r))
        pr.free_regs.push_back(r);
      else if (touches[r] == 1 && spill_of.count(r))
        pr.coalesce[spill_of[r]] = r;
    }

    for (std::map<INT64, promo_slot_t>::iterator it = slots.begin(); it != slots.end(); ++it) {
      promo_slot_t &s = it->second;
      if (bad_disps.count(s.disp))
        continue;
      if (s.disp >= min_lea_disp)
        continue;                                          // may be reached by a pointer
      bool overlap = false;
      for (unsigned r = 0; r < ranges.size() && !overlap; r++) {
        if (ranges[r].first == s.disp && ranges[r].second == s.width)
          continue;
        overlap = (ranges[r].first < s.disp + (INT64)s.width &&
                   s.disp < ranges[r].first + (INT64)ranges[r].second);
      }
      if (!overlap)
        pr.slots.push_back(s);
    }
    // Coalesce only a slot that is a candidate and of the spilled width.
    std::map<INT64, xed_reg_enum_t> coalesce;
    for (unsigned k = 0; k < pr.slots.size(); k++)
      if (pr.coalesce.count(pr.slots[k].disp))
        coalesce[pr.slots[k].disp] = pr.coalesce[pr.slots[k].disp];
    pr.coalesce = coalesce;
    if (!pr.slots.empty() && (!pr.free_regs.empty() || !pr.coalesce.empty()))
      promo_rtns.push_back(pr);
}

// At TC2 generation (TC map in 'tc_map'): choose the promoted slots using
// the profile, and prepare the rewritten encoding of each of their
// accesses in 'promoted_access'.
static void select_promoted_slots(const instr_map_t *tc_map, unsigned tc_entries)
{
    // original addr -> TC entry of the instr itself (not its profiling code)
    std::map<ADDRINT, unsigned> entry_of;
    for (unsigned i = 0; i < tc_entries; i++)
      if (tc_map[i].ins_type != ProfilingIns && tc_map[i].size &&
          tc_map[i].xed_category != XED_CATEGORY_WIDENOP)
        entry_of.emplace(tc_map[i].orig_ins_addr, i);

    for (unsigned r = 0; r < promo_rtns.size(); r++) {
      promo_rtn_t &pr = promo_rtns[r];
      if (KnobDumpPromo)
        cerr << "promo rtn 0x" << hex << pr.rtn_addr << dec << ": " << pr.slots.size()
             << " candidate slots, " << pr.free_regs.size() << " free regs, "
             << pr.coalesce.size() << " coalescable" << endl;

      // Profiled number of accesses of every slot.
      std::vector<std::pair<UINT64, unsigned> > order;   // (weight, slot index)
      for (unsigned s = 0; s < pr.slots.size(); s++) {
        UINT64 w = 0;
        bool all_found = true;
        for (unsigned a = 0; a < pr.slots[s].accesses.size(); a++) {
          std::map<ADDRINT, unsigned>::const_iterator it = entry_of.find(pr.slots[s].accesses[a]);
          if (it == entry_of.end()) { all_found = false; break; }
          unsigned bbl = tc_map[it->second].bbl_num;
          if (bbl < bbl_num)
            w += bbl_map[bbl].counter;
        }
        if (all_found && w >= PROMO_MIN_ACCESSES)
          order.push_back(std::make_pair(w, s));
      }
      std::sort(order.begin(), order.end());
      std::reverse(order.begin(), order.end());

      unsigned next_reg = 0;
      bool any = false;
      for (unsigned o = 0; o < order.size(); o++) {
        const promo_slot_t &slot = pr.slots[order[o].second];
        bool coalesced = pr.coalesce.count(slot.disp) > 0;
        if (!coalesced && next_reg >= pr.free_regs.size())
          continue;
        xed_reg_enum_t reg = coalesced ? pr.coalesce[slot.disp] : pr.free_regs[next_reg];
        // Rewrite all the accesses first; promote only if all succeed.
        std::vector<promoted_access_t> encs(slot.accesses.size());
        bool ok = true;
        for (unsigned a = 0; a < slot.accesses.size() && ok; a++) {
          const instr_map_t &e = tc_map[entry_of[slot.accesses[a]]];
          ok = promo_rewrite(reinterpret_cast<const UINT8 *>(e.encoded_ins), reg,
                             encs[a].encoded_ins, &encs[a].size);
        }
        if (KnobDumpPromo)
          cerr << "promo rtn 0x" << hex << pr.rtn_addr << " slot [rbp" << dec << slot.disp
               << "] width " << slot.width << " weight " << order[o].first
               << (ok ? " -> " : " rewrite failed ")
               << xed_reg_enum_t2str(promo_sub_reg(reg, slot.width)) << endl;
        if (!ok)
          continue;
        for (unsigned a = 0; a < slot.accesses.size(); a++)
          promoted_access[slot.accesses[a]] = encs[a];
        if (!coalesced)
          next_reg++;
        num_promoted_slots++;
        any = true;
      }
      if (any)
        num_promoted_rtns++;
    }
}



/********************************/
/* find_candidate_rtns_for_tc() */
/********************************/
int find_candidate_rtns_for_tc(IMG img)
{
    int rc = 0;
    ADDRINT last_selected_rtn_end = 0;
    std::map<ADDRINT, bool> is_targ_map;
    
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec)) continue;

        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
            if (!IsCandidateRtnForTranslation(rtn, last_selected_rtn_end)) continue;
            
            RTN_Open(rtn);
            is_targ_map.clear();
            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
               if (INS_IsDirectControlFlow(ins)) {
                 is_targ_map[INS_DirectControlFlowTargetAddress(ins)] = true;
               }
            }

            std::vector<INS> block;
            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
                block.push_back(ins);

                INS next_ins = INS_Next(ins);
                bool isNextInsJumpTarget = (!INS_Valid(next_ins) ? false : is_targ_map[INS_Address(next_ins)]);
                bool indirect_call = INS_IsCall(ins) && INS_IsIndirectControlFlow(ins);
                bool isInsTerminatesBBL = (isJumpOrRet(ins) || indirect_call || !INS_Valid(next_ins) || isNextInsJumpTarget);

                if (isInsTerminatesBBL) {
                    const unsigned current_bbl = bbl_num;
                    int fast_index = (!KnobNoProfile ? findFastCounterIndex(block) : -1);

                    for (unsigned i = 0; i < block.size(); i++) {
                        INS b_ins = block[i];
                        ADDRINT b_ins_addr = INS_Address(b_ins);
                        bool is_first = (b_ins_addr == RTN_Address(rtn));
                        bool is_last = (i == block.size() - 1);
                        ins_enum_t ins_type = (is_first ? RtnHeadIns : RegularIns);

                        // 1. Routine Header
                        if (!KnobNoProfile && is_first) {
                            xed_decoded_inst_t nop_xedd;
                            create_nop7_xedd_instr(&nop_xedd);
                            add_new_instr_entry(&nop_xedd, b_ins_addr, ins_type);
                            ins_type = RegularIns;
                        }

                        // 2. Zero-Overhead Counter Injection
                        if (!KnobNoProfile && static_cast<int>(i) == fast_index) {
                            if (add_fast_counter_profiling_instrs(b_ins_addr, &bbl_map[current_bbl].counter) < 0) return -1;
                        }

                        // 3. Fallback Heavy Counter and Target Profiling
                        if (!KnobNoProfile && is_last) {
                            if (fast_index < 0) {
                                if (add_counter_profiling_instrs(b_ins_addr, &bbl_map[current_bbl].counter) < 0) return -1;
                            }
                            if (add_indirect_target_profiling_instrs(b_ins, b_ins_addr, current_bbl) < 0) return -1;
                        }

                        xed_decoded_inst_t xedd;
                        xed_decoded_inst_zero_set_mode(&xedd, &dstate);
                        if (xed_decode(&xedd, reinterpret_cast<UINT8*>(b_ins_addr), max_inst_len) != XED_ERROR_NONE) return -1;
                        
                        if (add_new_instr_entry(&xedd, b_ins_addr, ins_type) < 0) return -1;

                        if (i == 0) bbl_map[current_bbl].starting_ins_entry = num_of_instr_map_entries - 1;
                        if (is_last) {
                            bbl_map[current_bbl].terminating_ins_entry = num_of_instr_map_entries - 1;
                            // Remember indirect jmp/call sites for de-virtualization.
                            if (INS_IsIndirectControlFlow(b_ins) && !INS_IsRet(b_ins)) {
                                bbl_map[current_bbl].indirect_kind = INS_IsCall(b_ins) ? IndirectCall : IndirectJump;
                                bbl_map[current_bbl].indirect_site_addr = b_ins_addr;
                            }
                        }
                    }

                    bbl_num++;

                    // 4. Edge Profiling Fallback
                    if (!KnobNoProfile && INS_Category(ins) == XED_CATEGORY_COND_BR) {
                        if (add_counter_profiling_instrs(INS_Address(ins), &bbl_map[current_bbl].fallthru_counter) < 0) return -1;
                    }
                    
                    block.clear();
                }
            }
            // Register promotion analysis (see select_promoted_slots()).
            std::vector<ADDRINT> rtn_addrs;
            std::vector<const xed_decoded_inst_t *> rtn_insns;
            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
                rtn_addrs.push_back(INS_Address(ins));
                rtn_insns.push_back(INS_XedDec(ins));
            }
            analyze_promotable_slots(RTN_Address(rtn), RTN_Size(rtn), rtn_addrs, rtn_insns);
            RTN_Close(rtn);
        }
    }
    return rc;
}



/***************************/
/* int copy_instrs_to_tc() */
/***************************/
int copy_instrs_to_tc(char *tc_buf)
{
    int cursor = 0;

    for (unsigned i=0; i < num_of_instr_map_entries; i++) {

      if ((ADDRINT)&tc_buf[cursor] != instr_map[i].new_ins_addr) {
          cerr << "ERROR: Non-matching instruction addresses: "
               << hex << (ADDRINT)&tc_buf[cursor]
               << " vs. " << instr_map[i].new_ins_addr << endl;
          return -1;
      }

      memcpy(&tc_buf[cursor], (char *)instr_map[i].encoded_ins, instr_map[i].size);

      // In TC, the NOP at the head of a routine or of a profiling stub is
      // a switch that is later turned on without a race.
      if (tc_buf == tc && instr_map[i].xed_category == XED_CATEGORY_WIDENOP &&
          instr_map[i].size >= SWITCH_HEAD_SIZE &&
          (instr_map[i].ins_type == RtnHeadIns || instr_map[i].ins_type == ProfilingIns))
        write_switch_head(&tc_buf[cursor], instr_map[i].size);

      cursor += instr_map[i].size;
    }

    return cursor;
}


/***************************************/
/* void commit_translated_rtns_to_tc() */
/***************************************/
inline void commit_translated_rtns_to_tc()
{
    // Commit the translated routines:
    // Go over the routines and replace the original ones
    // by their new successfully translated ones:

    for (unsigned i=0; i < num_of_instr_map_entries; i++) {

        //replace routine by new routine in tc

        if (instr_map[i].ins_type != RtnHeadIns)
          continue;

        RTN rtn = RTN_FindByAddress(instr_map[i].orig_ins_addr);
        if (rtn == RTN_Invalid()) {
           cerr << "invalid rtN for commit for addr: 0x"
                << instr_map[i].orig_ins_addr << "\n";
           continue;
        }

        // Debug print.
        // cerr << "committing rtN: " << RTN_Name(rtn);
        // cerr << " from: 0x" << hex << RTN_Address(rtn)
        //      << " to: 0x" << hex << instr_map[i].new_ins_addr << endl;


        AFUNPTR origFptr = RTN_ReplaceProbed(rtn,  (AFUNPTR)instr_map[i].new_ins_addr);

        if (origFptr == NULL) {
            cerr << "RTN_ReplaceProbed failed.";
            cerr << " orig routine addr: 0x" << hex << RTN_Address(rtn)
                 << " translated routine addr: 0x" << hex
                 << instr_map[i].new_ins_addr << endl;
            dump_instr_from_mem ((ADDRINT *)RTN_Address(rtn), RTN_Address(rtn));
        }

        // debug print.
        //if (origFptr != NULL) {
        //  cerr << "RTN_ReplaceProbed succeeded. ";
        //  cerr << " orig routine addr: 0x" << hex << RTN_Address(rtn)
        //       << " translated routine addr: 0x" << hex
        //       << instr_map[i].new_ins_addr << endl;
        //  dump_instr_from_mem ((ADDRINT *)RTN_Address(rtn), RTN_Address(rtn));
        //}
    }
}

/**********************************************/
/* start_stop_profile_gathering_thread_func() */
/**********************************************/
void start_stop_profile_gathering_thread_func(void *v)
{
    // Wait prof_time seconds for the profiling to count
    // execution frequency for each BBL.
    if (KnobVerbose) cerr << " prof time: " << dec << KnobNumSecsDuringProfile << " sec\n";
    sleep(KnobNumSecsDuringProfile);

    if (KnobVerbose) cerr << "disabling profile gathering\n";

    // disable profiling.
    //  Add a jump at beginning of every profile stub to bypass the
	//  profiling counters in TC.
    int rc = disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
    if  (rc < 0)
      return;
}

/****************************************/
/* void commit_translated_rtns_to_tc2() */
/****************************************/
int commit_translated_rtns_to_tc2()
{
  for (unsigned i=0; i < num_of_instr_map_entries; i++) {
       // Turn on the switch at the routine head in TC: it jumps to the
       // routine in TC2 from now on.
       if (instr_map[i].ins_type != RtnHeadIns ||
           instr_map[i].xed_category != XED_CATEGORY_WIDENOP)
         continue;
       if (turn_on_switch_head(instr_map[i].orig_ins_addr, instr_map[i].new_ins_addr) < 0) {
         cerr << "failed to switch the routine at 0x" << hex << instr_map[i].orig_ins_addr
              << " to TC2 at 0x" << instr_map[i].new_ins_addr << dec << endl;
         return -1;
       }
  }
  return 0;
}




/****************************/
/* create_tc2_thread_func() */
/****************************/
void create_tc2_thread_func(void *v)
{
    // Wait prof_time seconds for the profiling to count
    // execution frequency for each BBL.
    if (KnobVerbose) cerr << " prof time: " << dec << KnobNumSecsDuringProfile << " sec\n";
    sleep(KnobNumSecsDuringProfile);

    if (KnobVerbose) cerr << "disabling profile gathering\n";

    // disable profiling.
    //  Add a jump at beginning of every profile stub to bypass the
    //  profiling counters in TC.
    int rc = disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
    if  (rc < 0)
      return;

    // Step 1: Build a new instruction map for TC2 out-of-place
    instr_map_t *old_map = instr_map;
    const unsigned old_entries = num_of_instr_map_entries;

    // Allocate a new map with extra capacity for upcoming optimizations
    const unsigned new_capacity = max_ins_count * 2;
    instr_map_t *new_map = (instr_map_t *)calloc(new_capacity, sizeof(instr_map_t));
    if (new_map == NULL) {
        cerr << "Failed to allocate new_map" << endl;
        return;
    }

    // Point global map to the new allocation
    instr_map = new_map;
    max_ins_count = new_capacity;
    num_of_instr_map_entries = 0;

    // De-virtualization, step 0: choose the sites on the TC map.
    // orig_to_tc: original address -> TC address of the first instr with
    // that original address that is kept in TC2 (its key in the TC2 map).
    std::map<ADDRINT, ADDRINT> orig_to_tc;
    for (unsigned i = 0; i < old_entries; i++) {
        if (old_map[i].orig_ins_addr && !dropped_in_tc2(old_map[i]))
            orig_to_tc.emplace(old_map[i].orig_ins_addr, old_map[i].new_ins_addr);
    }
    std::map<unsigned, ADDRINT> tc_devirt_sites;   // TC index -> hot target
    if (!KnobNoDevirt)
        find_devirt_sites(old_map, old_entries, orig_to_tc, tc_devirt_sites);

    // Register promotion: choose the promoted stack slots of every routine.
    if (!KnobNoRegPromo)
        select_promoted_slots(old_map, old_entries);

    std::map<unsigned, ADDRINT> devirt_sites;      // TC2 index -> hot target

    // Copy the instructions to the new map, without the profiling code.
    for (unsigned i = 0; i < old_entries; i++) {
        const instr_map_t &src = old_map[i];

        // 1. Keep the routine header entry (its TC2 address is the target of
        //    the TC -> TC2 jump), but empty: TC2 does not need the NOP.
        if (src.ins_type == RtnHeadIns && src.xed_category == XED_CATEGORY_WIDENOP) {
            instr_map[num_of_instr_map_entries] = src;
            instr_map[num_of_instr_map_entries].orig_ins_addr = src.new_ins_addr; // TC address becomes the key
            instr_map[num_of_instr_map_entries].size = 0;
            instr_map[num_of_instr_map_entries].targ_map_entry = -1;
            num_of_instr_map_entries++;
            continue;
        }

        // 2. Strip the profiling code and the NOPs.
        if (dropped_in_tc2(src))
            continue;

        // 3. Register promotion: access to a promoted stack slot.
        if (promoted_access.count(src.orig_ins_addr)) {
            instr_map[num_of_instr_map_entries] = src;
            instr_map_t &e = instr_map[num_of_instr_map_entries];
            const promoted_access_t &pa = promoted_access[src.orig_ins_addr];
            memcpy(e.encoded_ins, pa.encoded_ins, pa.size);
            e.size = pa.size;
            e.orig_ins_addr = src.new_ins_addr;
            e.orig_rip_addr = 0;
            e.targ_map_entry = -1;
            num_of_instr_map_entries++;
            num_register_promotions++;
            continue;
        }

        // 4. Standard instruction copy.
        if (tc_devirt_sites.count(i))
            devirt_sites[num_of_instr_map_entries] = tc_devirt_sites[i];

        instr_map[num_of_instr_map_entries] = src;

        // The TC address becomes the key ("original" address) for TC2 generation
        instr_map[num_of_instr_map_entries].orig_ins_addr = src.new_ins_addr;

        // Map targets correctly: forward past dropped instructions (like NOPs)
        if (src.targ_map_entry >= 0) {
            unsigned target_idx = src.targ_map_entry;
            while (target_idx < old_entries && dropped_in_tc2(old_map[target_idx]))
                target_idx++;
            if (target_idx < old_entries) {
                instr_map[num_of_instr_map_entries].orig_targ_addr = old_map[target_idx].new_ins_addr;
            }
        }
        instr_map[num_of_instr_map_entries].targ_map_entry = -1;

        num_of_instr_map_entries++;
    }
    if (KnobVerbose) cerr << "after modifying instr_map" << endl;

    // 1. De-virtualization
    if (!KnobNoDevirt)
      insert_devirt_sites(devirt_sites, orig_to_tc);

    // 2. Code Reordering
    if (!KnobNoReorder)
      reorder_bbls();

    // 3. Loop Unrolling
    if (!KnobNoUnroll)
      unroll_single_bbl_loops();

    // 4. Leaf Function Inlining
    if (!KnobNoInline)
      inline_leaf_functions();

    // 5. Constant Propagation
    if (!KnobNoConstProp)
      apply_constant_propagation();

    // 6. Remove jumps to the next instruction.
    remove_jumps_to_next();

    if (KnobStats) {
      cerr << dec
           << "TC2 stats: devirt calls=" << num_devirt_calls << " jumps=" << num_devirt_jumps
           << " (skipped: rare=" << num_devirt_skip_rare
           << " not_translated=" << num_devirt_skip_not_translated
           << " far=" << num_devirt_skip_far_targ << " other=" << num_devirt_skip_other << ")\n"
           << "TC2 stats: reordered rtns=" << num_reordered_rtns
           << " moved cold bbls=" << num_moved_cold_bbls
           << " reversed jcc=" << num_reversed_cond_branches
           << " added jmps=" << num_added_fallthru_jumps
           << " removed jmps=" << num_removed_jumps << "\n"
           << "TC2 stats: unrolled loops=" << num_unrolled_loops
           << " inlined calls=" << num_inlined_calls
           << " const props=" << num_constant_propagations << "\n"
           << "TC2 stats: promoted slots=" << num_promoted_slots << " in " << num_promoted_rtns
           << " rtns, rewritten accesses=" << num_register_promotions << "\n";
    }

    // Step 3: Chaining - calculate direct branch and call instructions to point
    //         to corresponding target instr entries:
    //
    chain_all_direct_jmp_and_call_target_entries(0, num_of_instr_map_entries);
    if (KnobVerbose) cerr << "after chaining all branch targets" << endl;

    // Step 4: Set initial estimated new addrs for each instruction in tc2.
    //
    set_initial_estimated_new_ins_addrs_in_tc(tc2);
    if (KnobVerbose) cerr << "after setting initial estimated new ins addrs in tc2" << endl;

    // Step 5: fix rip-based, direct branch and direct call displacements:
    //
    rc = fix_instructions_displacements();
    if (rc < 0 ) {
        cerr << "failed to fix displacments of translated instructions\n";
        return;
    }
    if (KnobVerbose) cerr << "after fixing instructions displacements" << endl;

    // Step 6: write translated instructions to tc2:
    //
    rc = copy_instrs_to_tc(tc2);
    if (rc < 0 ) {
        cerr << "failed to copy the instructions to the translation cache\n";
        return;
    }
    tc2_size = rc;
    if (KnobVerbose) cerr << "after write all new instructions to tc2" << endl;

    // Step 7: Commit the translated routines:
    //         Go over the candidate functions and replace the original ones
    //         by their new successfully translated ones:
    if (!KnobDoNotCommitTranslatedCode) {
        
      rc = commit_translated_rtns_to_tc2();
      
      if (rc < 0 ) {
          cerr << "failed to commit jump instructions from TC to TC2\n";
          return;
      }
      if (KnobVerbose) cerr << "after commit of translated routines from TC to TC2" << endl;
    }

    if (KnobDumpTranslatedCode2) {
        cerr << "Translation Cache 2 dump:" << endl;
        dump_tc(tc2, tc2_size);
    }
 
    clock_gettime(CLOCK_MONOTONIC, &start_running_time);

    PIN_ExitThread(0);
}

/****************************/
/* allocate_and_init_memory */
/****************************/
int allocate_and_init_memory(IMG img)
{
    ADDRINT highest_addr = 0;
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec)) continue;
        if (!lowest_sec_addr || lowest_sec_addr > SEC_Address(sec)) lowest_sec_addr = SEC_Address(sec);
        if (highest_sec_addr < SEC_Address(sec) + SEC_Size(sec)) highest_sec_addr = SEC_Address(sec) + SEC_Size(sec);

        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
            if (highest_addr < RTN_Address(rtn) + RTN_Size(rtn)) highest_addr = RTN_Address(rtn) + RTN_Size(rtn);
            max_rtn_count++;
            max_ins_count += RTN_NumIns(rtn);
        }
    }

    max_ins_count *= 10; 
    int pagesize = sysconf(_SC_PAGE_SIZE);
    if (pagesize == -1) return -1;

    ADDRINT text_size = (highest_sec_addr - lowest_sec_addr) * 2 + pagesize * 4;
    max_tc_size = 10 * text_size + pagesize * 4; 
    if (max_tc_size >= 0x7FFFFFFF) return -1;

    // CRITICAL FIX: Expand mem_size to include bbl_map so it stays within 2GB of the code
    const size_t mem_size =
              max_tc_size +                         
              max_rtn_count * sizeof(ADDRINT) +      
              max_ins_count * sizeof(bbl_map_t);     
              
    char *addr = nullptr;
    ADDRINT max_distance = 0x7FFFFFFF;
    const size_t step = pagesize; 
    ADDRINT aligned_target = ((ADDRINT)highest_addr) & ~(pagesize - 1);
    
    void* result = mmap((void*)aligned_target, mem_size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
    if (result != MAP_FAILED && (abs((long)((ADDRINT)result - aligned_target)) <= (long)max_distance)) {
        addr = (char *)result;
    }

    if (!addr) {
        for (size_t offset = step; offset <= max_distance; offset += step) {
            ADDRINT try_addr = aligned_target + offset;
            result = mmap((void*)try_addr, mem_size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
            if (result != MAP_FAILED && (abs((long)((ADDRINT)result - try_addr)) <= (long)max_distance)) {
                addr = (char *)result;
                break;
            }
            if (result != MAP_FAILED) munmap(result, mem_size);

            if (highest_addr >= offset) {
                try_addr = aligned_target - offset;
                result = mmap((void*)try_addr, mem_size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
                if (result != MAP_FAILED && (abs((long)((ADDRINT)result - try_addr)) <= (long)max_distance)) {
                    addr = (char *)result;
                    break;
                }
                if (result != MAP_FAILED) munmap(result, mem_size);
            }
        }
    }

    if (!addr) return -1;

    tc = (char *)addr;
    addr += max_tc_size/2;

    tc2 = (char *)addr;
    addr += max_tc_size/2;

    jump_to_orig_addr_map = (ADDRINT *)addr;
    addr += max_rtn_count * sizeof(ADDRINT);

    // CRITICAL FIX: Map BBL counters to the localized 32-bit memory block
    bbl_map = reinterpret_cast<bbl_map_t *>(addr);
    memset(bbl_map, 0, max_ins_count * sizeof(bbl_map_t));
    max_bbl_count = max_ins_count;

    instr_map = (instr_map_t *)calloc(max_ins_count, sizeof(instr_map_t));
    if (instr_map == NULL) return -1;

    return 0;
}


/* ============================================ */
/* Main translation routine                     */
/* ============================================ */
typedef VOID (*EXITFUNCPTR)(INT code);
EXITFUNCPTR origExit;

/********/
/* Fini */
/********/
VOID Fini(INT32 code, VOID* v)
{
    if (KnobVerbose) cerr << "Reached _exit." << endl;

    clock_gettime(CLOCK_MONOTONIC, &end_running_time);

    if (KnobDumpProfile && out)
      dump_profile();

    double elapsed;
    if (start_running_time.tv_sec || start_running_time.tv_nsec) {
      elapsed = (end_running_time.tv_sec - start_running_time.tv_sec) +
                (end_running_time.tv_nsec - start_running_time.tv_nsec) / 1e9 +
                KnobNumSecsDuringProfile;
    } else {
      // The program ended before TC2 was committed.
      elapsed = (end_running_time.tv_sec - tool_start_time.tv_sec) +
                (end_running_time.tv_nsec - tool_start_time.tv_nsec) / 1e9;
    }
    cerr << " Translated code run (including profiling) took: "
         << elapsed << " seconds\n";
}

/*******************/
/* ExitInProbeMode */
/*******************/
VOID ExitInProbeMode(INT code)
{
    Fini(code, 0);
    (*origExit)(code);
}

/*************/
/* create_tc */
/*************/
VOID create_tc(IMG img, VOID *v)
{
    // Insert a call to function Fini when raching the _exit routine.
    RTN exitRtn = RTN_FindByName(img, "_exit");
    if (RTN_Valid(exitRtn) && RTN_IsSafeForProbedReplacement(exitRtn)) {
      origExit = (EXITFUNCPTR)RTN_ReplaceProbed(exitRtn, AFUNPTR(ExitInProbeMode));
    }

    // Step 0: Check the image and the CPU:
    if (!IMG_IsMainExecutable(img))
      return;

    if (KnobDumpOrigCode)
      dump_image_instrs(img);

    int rc = 0;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // step 1: Check size of executable sections and allocate required memory:
    rc = allocate_and_init_memory(img);
    if (rc < 0) {
        cerr << "failed to initialize memory for translation\n";
        return;
    }
    if (KnobVerbose) cerr << "after memory allocation" << endl;

    // Step 2: go over all routines and identify candidate routines and copy
    //         their code into the instr map IR:
    rc = find_candidate_rtns_for_tc(img);
    if (rc < 0) {
        cerr << "failed to find candidates for translation\n";
        return;
    }
    if (KnobVerbose) cerr << "after identifying candidate routines" << endl;

    // Step 3: Chaining - calculate direct branch and call instructions to point
    //         to corresponding target instr entries:
    chain_all_direct_jmp_and_call_target_entries(0, num_of_instr_map_entries);
    if (KnobVerbose) cerr << "after chaining all branch targets" << endl;

    // Step 4: Set initial estimated new addrs for each instruction in the tc.
    rc = set_initial_estimated_new_ins_addrs_in_tc(tc);
    if (rc < 0 ) {
        cerr << "failed to set initial estimated new ins addrs in the TC\n";
        return;
    }
    if (KnobVerbose) cerr << "after setting initial estimated new ins addrs in the TC" << endl;

    // Step 5: fix rip-based, direct branch and direct call displacements:
    rc = fix_instructions_displacements();
    if (rc < 0 ) {
        cerr << "failed to fix displacments of translated instructions\n";
        return;
    }
    if (KnobVerbose) cerr << "after fixing instructions displacements" << endl;

    // Step 6: write translated instructions to the tc:
    rc = copy_instrs_to_tc(tc);
    if (rc < 0 ) {
        cerr << "failed to copy the instructions to the translation cache\n";
        return;
    }
    tc_size = rc;
    if (KnobVerbose) cerr << "after write all new instructions to memory tc" << endl;

    if (KnobDumpTranslatedCode) {
       cerr << "Translation Cache dump:" << endl;
       dump_tc(tc, tc_size);  // dump the entire tc

       //cerr << endl << "instructions map dump:" << endl;
       //dump_profile();     // dump all translated instructions in map_instr
    }

    // Step 7: Commit the translated routines:
    //         Go over the candidate functions and replace the original ones
    //         by their new successfully translated ones:
    if (!KnobDoNotCommitTranslatedCode) {
      commit_translated_rtns_to_tc();
      if (KnobVerbose) cerr << "after commit of translated routines from orig code to TC" << endl;
    }

    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) +
	                 (end.tv_nsec - start.tv_nsec) / 1e9;
    if (KnobVerbose) cerr << " create_tc took: " << elapsed << " seconds\n";
}



/* ===================================================================== */
/* Print Help Message                                                    */
/* ===================================================================== */
INT32 Usage()
{
    cerr << "This tool translated routines of an Intel(R) 64 binary"
         << endl;
    cerr << KNOB_BASE::StringKnobSummary();
    cerr << endl;
    return -1;
}


/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */

int main(int argc, char * argv[])
{
    // Open output profile file.
    clock_gettime(CLOCK_MONOTONIC, &tool_start_time);

    // Initialize pin & symbol manager
    if( PIN_Init(argc,argv) )
        return Usage();

    // Open output profile file (only when the profile dump is requested).
    if (KnobDumpProfile)
      out = new std::ofstream("bprofile.out");

    PIN_InitSymbols();

    // Register create_tc
    IMG_AddInstrumentFunction(create_tc, 0);

    // Create TC2 by a separate thread.
    // Note it is safe to create internal threads in the tool's main procedure and spawn new
    // internal threads from existing ones. All other places, like Pin callbacks and
    // analysis routines in application threads, are not safe for creating internal threads.
    THREADID tid = PIN_SpawnInternalThread(create_tc2_thread_func, NULL, 0, NULL);
    if (tid == INVALID_THREADID) {
        cerr << "failed to spawn a thread for commit" << endl;
    }

    // Start the program, never returns
    PIN_StartProgramProbed();

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
