/*########################################################################################################*/
// cd /nfs/iil/ptl/bt/ghaber1/pin/pin-2.10-45467-gcc.3.4.6-ia32_intel64-linux/source/tools/SimpleExamples
// make btranslate.test
//  ../../../pin -t obj-intel64/btranslate.so -- ~/workdir/tst
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
 * The profiling data is then printed on exit into the output file edge-profile.csv
 * in the following format (BBLs sorted from hottest to coldest):
 *   <bbl addr>, <bbl exec count>, <taken count>, <fallthru count>[, <targ addr, count> x up to 4]
 *
 * EX4 modifications:
 *  1. Robustness: routines that fail to decode/translate are skipped and
 *     reverted (instead of aborting the whole translation), duplicate and
 *     PLT routines are skipped, and only routines that are safe for probed
 *     replacement are committed.
 *  2. Optimization: profiling stubs avoid saving/restoring registers that
 *     are provably DEAD at the instrumentation point (forward liveness scan).
 *  3. Output: edge-profile.csv (sorted by exec count, hottest first).
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

/* ============================================================= */
/* Safety filters for Probe-mode translation                     */
/* ============================================================= */
static bool StartsWith(const string& s, const string& prefix)
{
    return s.compare(0, prefix.size(), prefix) == 0;
}

static bool ShouldSkipRoutineByName(const string& name)
{
    static const char* exact_skip[] = {
        "_init", "_start", "_fini",
        "deregister_tm_clones", "register_tm_clones",
        "__do_global_dtors_aux", "frame_dummy",
        "__libc_csu_init", "__libc_csu_fini",
        "_exit", "exit", "abort",
        "open", "open64", "__open", "__open64", "openat", "__openat",
        "close", "__close", "read", "__read", "write", "__write",
        "lseek", "lseek64", "__lseek", "__lseek64",
        "stat", "stat64", "lstat", "lstat64", "fstat", "fstat64",
        "__xstat", "__lxstat", "__fxstat", "__fxstat64",
        "__fxstatat", "__fxstatat64", "isatty", "uname",
        "mmap", "mmap64", "munmap", "mprotect", "brk", "sbrk",
        "malloc", "free", "calloc", "realloc",
        "fopen", "fopen64", "fclose", "fread", "fwrite", "fflush",
        "fseek", "ftell", "fseeko", "ftello",
        "memcpy", "memmove", "memset", "strlen", "strcmp", "strncmp",
        "strcpy", "strncpy",
        "bitmap_clear_range",
        "bitmap_intersect_compl_p",
        "fold_ignored_result.part.0",
        "strip_invariant_refs",
        "get_base_address",
        "gt_pch_nx_section",
        "gt_pch_nx_cpp_token",
        "init_object_sizes.part.0",
        "fini_object_sizes",
        "type_internals_preclude_sra_p",
        "decBiStr",
        "trim_filename",
        "tree_log2"
    };

    for (unsigned i = 0; i < sizeof(exact_skip) / sizeof(exact_skip[0]); i++) {
        if (name == exact_skip[i])
            return true;
    }

    if (StartsWith(name, "_dl_"))
        return true;
    if (StartsWith(name, "_IO_"))
        return true;
    if (StartsWith(name, "__libc_"))
        return true;
    if (StartsWith(name, "__GI_"))
        return true;
    if (StartsWith(name, "_Unwind_"))
        return true;
    if (name.find("syscall") != string::npos)
        return true;
    if (name.find("freeres") != string::npos)
        return true;

   return false;
}

// Same filters used in find_candidate_rtns_for_tc() and memory sizing.
static bool IsCandidateRtnForTranslation(RTN rtn, ADDRINT& last_selected_rtn_end)
{
   if (!RTN_Valid(rtn))
       return false;

   string rtn_name = RTN_Name(rtn);

   if (ShouldSkipRoutineByName(rtn_name))
       return false;

   if (rtn_name.find(".plt") != string::npos ||
       rtn_name.find("@plt") != string::npos)
       return false;

   if (!RTN_IsSafeForProbedReplacement(rtn))
       return false;

   ADDRINT rtn_addr = RTN_Address(rtn);
   USIZE rtn_size = RTN_Size(rtn);

   if (rtn_size < 16)
       return false;

   if (last_selected_rtn_end != 0 && rtn_addr < last_selected_rtn_end)
       return false;

   last_selected_rtn_end = rtn_addr + rtn_size;
   return true;
}

/*======================================================================*/
/* commandline switches                                                 */
/*======================================================================*/

KNOB<BOOL>   KnobVerbose(KNOB_MODE_WRITEONCE,    "pintool",
    "verbose", "0", "Verbose run");

KNOB<BOOL>   KnobDumpOrigCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_orig_code", "0", "Dump Original non-translated Code");

KNOB<BOOL>   KnobDumpTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_tc", "0", "Dump Translated Code");

KNOB<BOOL>   KnobDoNotCommitTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "no_tc_commit", "0", "Do not commit translated code");

KNOB<UINT> KnobNumSecsDuringProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "prof_time", "2", "Number of seconds for collecting BBL counters");

KNOB<BOOL> KnobDumpProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_prof", "0", "Dump profiling information");

KNOB<BOOL> KnobNoProfile(KNOB_MODE_WRITEONCE,    "pintool",
    "no_prof", "0", "Do not collect profile information");

KNOB<BOOL> KnobNoDeadRegOpt(KNOB_MODE_WRITEONCE,    "pintool",
    "no_deadreg", "0", "Disable the dead-register save/restore optimization");


/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */
std::ofstream* out = 0;

// 0 = building/not seen yet, 1 = ready, -1 = translation failed.
volatile INT32 tc_state = 0;

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

// tc containing the new code:
char *tc = nullptr;
unsigned tc_size = 0;
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

// Track up to 4 indirect-jump targets per BBL (indices 0..3).
#define MAX_TARG_ADDRS 3
#define MAX_BBL_COUNT 10000

unsigned max_instr_map_entries = 0;
bool tc_translation_succeeded = false;

// Bbl map of all the bbl exec counters to be collected at runtime:
typedef struct {
  UINT64 counter;
  UINT64 fallthru_counter; // for BBLs that terminate with a cond branch.
  ADDRINT targ_addr[MAX_TARG_ADDRS+1];
  UINT64  targ_count[MAX_TARG_ADDRS+1];
  unsigned starting_ins_entry;
  unsigned terminating_ins_entry;
} bbl_map_t;

bbl_map_t *bbl_map;
unsigned bbl_num = 0;
std::map<ADDRINT, unsigned> entry_map;

unsigned max_rtn_count = 0;

// Stats: how many save/restore instructions were eliminated by the
// dead-register analysis (for debug/README purposes).
unsigned num_eliminated_save_restore_instrs = 0;
unsigned num_profiling_stubs = 0;

struct timespec start_running_time;
struct timespec end_running_time;

/* ============================================================= */
/* Service instr routines                                        */
/* ============================================================= */
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


/* ============================================================= */
/* Service dump routines                                         */
/* ============================================================= */

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
    struct BblSortEntry {
        unsigned bbl_idx;
        UINT64   heat;
    };

    vector<BblSortEntry> sorted_bbls;
    sorted_bbls.reserve(bbl_num);

    // CRITICAL FIX: Bound the loop by MAX_BBL_COUNT to prevent heap overflows!
    for (unsigned i = 0; i < bbl_num && i < MAX_BBL_COUNT; i++) {
        if (!bbl_map[i].counter && !bbl_map[i].fallthru_counter)
            continue;

        bool has_targ = false;
        for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++) {
            if (bbl_map[i].targ_addr[j] || bbl_map[i].targ_count[j]) {
                has_targ = true;
                break;
            }
        }
        if (!bbl_map[i].counter && !bbl_map[i].fallthru_counter && !has_targ)
            continue;

        BblSortEntry entry;
        entry.bbl_idx = i;
        entry.heat = bbl_map[i].counter;
        sorted_bbls.push_back(entry);
    }

    sort(sorted_bbls.begin(), sorted_bbls.end(),
         [](const BblSortEntry& a, const BblSortEntry& b) {
             return a.heat > b.heat;
         });

    for (unsigned s = 0; s < sorted_bbls.size(); s++) {
        unsigned i = sorted_bbls[s].bbl_idx;
        ADDRINT bbl_addr = instr_map[bbl_map[i].starting_ins_entry].orig_ins_addr;
        UINT64 exec = bbl_map[i].counter;
        UINT64 fallthru = bbl_map[i].fallthru_counter;
        UINT64 taken = (exec >= fallthru) ? (exec - fallthru) : 0;

        *out << "0x" << hex << bbl_addr << dec
             << ", " << exec
             << ", " << taken
             << ", " << fallthru;

        for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++) {
            if (bbl_map[i].targ_addr[j] || bbl_map[i].targ_count[j]) {
                *out << ", 0x" << hex << bbl_map[i].targ_addr[j] << dec
                     << ", " << bbl_map[i].targ_count[j];
            }
        }
        *out << endl;
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


/* ============================================================= */
/* Translation routines                                         */
/* ============================================================= */


/***************************/
/* disable_profiling_in_tc */
/***************************/
int disable_profiling_in_tc(instr_map_t * instr_map, unsigned num_of_instr_map_entries)
{
    for (unsigned i = 0; i < num_of_instr_map_entries; i++) {
        // Check for the case of a NOP instr at the head of a
        // pofiling code stub and replace it by a jump instr that skips it.
        if (instr_map[i].ins_type == ProfilingIns &&
            instr_map[i].xed_category == XED_CATEGORY_WIDENOP) {
            // Calculate the jump displacement.
            unsigned j = 1;
            xed_int64_t disp = 0;
            while (i + j < num_of_instr_map_entries &&
                   instr_map[i+j].ins_type == ProfilingIns) {
                disp += instr_map[i+j].size;
                j++;
            }

          xed_encoder_instruction_t enc_instr;
          xed_encoder_request_t enc_req;
          unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
          char encoded_jmp_ins[XED_MAX_INSTRUCTION_BYTES];
          unsigned int olen = 5; // skip jump instr is exactly 5 bytes long.
          
          disp += (instr_map[i].size - olen);
          xed_inst1(&enc_instr, dstate,  XED_ICLASS_JMP, 64, xed_relbr(disp, 32));
          
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

          if (olen > instr_map[i].size) {
             cerr << " unable to set a relative jump to skip the profiling code stub at: "
                  << hex << "0x" << instr_map[i].new_ins_addr << "\n";
             return -1;
          }

          // Write the bypassing jump instr on the NOP instr.
          // Verify address validity before patching
          const ADDRINT patch_addr = instr_map[i].new_ins_addr;
          if (patch_addr < (ADDRINT)tc || patch_addr + sizeof(UINT64) > (ADDRINT)tc + max_tc_size) {
              continue;
          }

          // Preserve surrounding bytes and publish atomically using a 64-bit store
          UINT64 patch_word = 0;
          memcpy(&patch_word, reinterpret_cast<const void *>(patch_addr), sizeof(patch_word));
          memcpy(&patch_word, encoded_jmp_ins, olen);
          __sync_synchronize();
          *reinterpret_cast<volatile UINT64 *>(patch_addr) = patch_word;
          __sync_synchronize();
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

// --- NEW CODE
    if (num_of_instr_map_entries >= max_ins_count) {
        if (max_ins_count > UINT_MAX / 2) {
            cerr << "instruction map capacity overflow" << endl;
            return -1;
        }
        const unsigned old_capacity = max_ins_count;
        const unsigned new_capacity = old_capacity ? old_capacity * 2 : 1024;
        void *new_map = realloc(instr_map, new_capacity * sizeof(instr_map_t));
        if (new_map == NULL) {
            perror("realloc instr_map");
            return -1;
        }
        instr_map = reinterpret_cast<instr_map_t *>(new_map);
        memset(instr_map + old_capacity, 0,
               (new_capacity - old_capacity) * sizeof(instr_map_t));
        max_ins_count = new_capacity;
    }
    
    // Add a new entry to instr_map:
    instr_map[num_of_instr_map_entries].orig_ins_addr = pc;
    // ... rest of your assignment code stays exactly the same! ...
    num_of_instr_map_entries++;

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

/* ============================================================= */
/* Dead-register liveness analysis (EX4 requirement 2)           */
/* ============================================================= */
/* ============================================================= */
/* Register liveness helpers for the dead-register optimization. */
/* ============================================================= */

// Diagnostic counters (gathered statically during translation).
unsigned long g_num_bbl_stubs        = 0;
unsigned long g_num_rax_save_skipped = 0;
unsigned long g_num_rbx_save_skipped = 0;
unsigned long g_num_rcx_save_skipped = 0;
unsigned long g_num_rax_skipped_straightline = 0; // RAX skips via the straight-line rule
unsigned long g_num_rax_skipped_successor    = 0; // RAX skips via the cond-branch successor rule

// True if 'ins' reads any sub-register/alias of 'reg' (EAX/AX/AL count as RAX).
bool instructionReadsReg(INS ins, REG reg)
{
    REG full = REG_FullRegName(reg);
    UINT32 n = INS_MaxNumRRegs(ins);
    for (UINT32 i = 0; i < n; i++) {
        if (REG_FullRegName(INS_RegR(ins, i)) == full)
            return true;
    }
    return false;
}

// True if 'ins' performs a *killing* (full) write of 'reg'. On x86-64 a 64-bit
// or 32-bit destination fully defines the 64-bit register (32-bit writes
// zero-extend); writes to 16/8-bit sub-registers are partial and do NOT kill.
bool instructionFullyWritesReg(INS ins, REG reg)
{
    REG full = REG_FullRegName(reg);
    UINT32 n = INS_MaxNumWRegs(ins);
    for (UINT32 i = 0; i < n; i++) {
        REG w = INS_RegW(ins, i);
        if (REG_FullRegName(w) == full && REG_Size(w) >= 4)
            return true;
    }
    return false;
}

// True if 'ins' reads or writes any alias of 'reg'.
bool instructionReadsOrWritesReg(INS ins, REG reg)
{
    if (instructionReadsReg(ins, reg))
        return true;
    REG full = REG_FullRegName(reg);
    UINT32 n = INS_MaxNumWRegs(ins);
    for (UINT32 i = 0; i < n; i++) {
        if (REG_FullRegName(INS_RegW(ins, i)) == full)
            return true;
    }
    return false;
}

// We cannot safely continue a straight-line liveness scan past a control
// transfer (we would not know which successor actually executes).
bool isLivenessBarrier(INS ins)
{
    return INS_IsDirectControlFlow(ins)   ||
           INS_IsIndirectControlFlow(ins) ||
           INS_IsRet(ins)                 ||
           INS_IsCall(ins)                ||
           INS_IsSyscall(ins);
}

// Conservative deadness test: is 'reg' dead on the guaranteed straight-line
// path that begins at instruction 'start'?
bool regIsDeadFrom(INS start, REG reg)
{
    INS cur = start;
    int  steps = 0;
    const int MAX_STEPS = 128;
    while (INS_Valid(cur) && steps < MAX_STEPS) {
        if (instructionReadsReg(cur, reg))
            return false;                   // used -> live
        if (INS_IsPredicated(cur) && instructionReadsOrWritesReg(cur, reg))
            return false;                   // conditional def does not kill -> live
        if (instructionFullyWritesReg(cur, reg))
            return true;                    // unconditional full redefine -> dead
        if (isLivenessBarrier(cur))
            return false;                   // unknown successor -> live
        cur = INS_Next(cur);
        steps++;
    }
    return false;                           // routine boundary / scan limit -> live
}

// Successor-aware RAX deadness for a DIRECT CONDITIONAL branch terminator.
bool raxDeadOnBothCondBranchSuccessors(INS cond_branch_ins,
                                       const std::map<ADDRINT, INS>& addr2ins)
{
    if (INS_Category(cond_branch_ins) != XED_CATEGORY_COND_BR)
        return false;
    if (!INS_IsDirectControlFlow(cond_branch_ins))
        return false;

    // Fall-through successor must exist.
    INS fallthru_ins = INS_Next(cond_branch_ins);
    if (!INS_Valid(fallthru_ins))
        return false;

    // Taken target must map to an instruction in the same (open) routine.
    ADDRINT targ_addr = INS_DirectControlFlowTargetAddress(cond_branch_ins);
    std::map<ADDRINT, INS>::const_iterator it = addr2ins.find(targ_addr);
    if (it == addr2ins.end())
        return false;
    INS taken_ins = it->second;

    // RAX must be provably dead on BOTH paths.
    return regIsDeadFrom(taken_ins,    LEVEL_BASE::REG_RAX) &&
           regIsDeadFrom(fallthru_ins, LEVEL_BASE::REG_RAX);
}
/**************************/
/* add_profiling_instrs() */
/**************************/
// Insert the profiling stub for a BBL counter.
//   ins      - the instruction that terminates the BBL (the stub is placed
//              right before it), or the cond branch when inserting the
//              fallthru-counter stub (the stub is placed right after it).
//   liveness_scan_start - the first original instruction that will execute
//              AFTER the profiling stub. Used for the dead-register scan:
//              for a regular BBL-counter stub this is 'ins' itself, and for
//              a fallthru-counter stub it is INS_Next(ins).
/**************************/
/* add_profiling_instrs() */
/**************************/
int add_profiling_instrs(INS ins, ADDRINT ins_addr,
                         UINT64 *counter_addr, unsigned bbl_num,
                         bool rax_is_dead, bool rbx_is_dead, bool rcx_is_dead)
{
  xed_encoder_instruction_t enc_instr;

  static uint64_t rax_mem = 0;

  // Dead-register optimization bookkeeping. The indirect-jump profiling path
  // below relies on RAX having been saved to rax_mem, so never treat RAX as
  // dead for an indirect-jump terminator.
  g_num_bbl_stubs++;
  bool is_indirect = INS_IsIndirectControlFlow(ins) && !INS_IsRet(ins) && !INS_IsCall(ins);
  if (is_indirect)
    rax_is_dead = false;

  xed_inst0(&enc_instr, dstate, XED_ICLASS_NOP4, 64);
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // Save RAX (skipped when RAX is dead at the instrumentation point):
  if (!rax_is_dead) {
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64),
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
  } else {
    g_num_rax_save_skipped++;
  }

  if (is_indirect) {
    static uint64_t rbx_mem = 0;
    static uint64_t rcx_mem = 0;

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

    // Save RBX (skipped when RBX is dead) into rbx_mem in 2 steps via RAX:
    if (!rbx_is_dead) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX), xed_reg(XED_REG_RBX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rbx_mem, 64), 64),
                xed_reg(XED_REG_RAX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
    } else {
      g_num_rbx_save_skipped++;
    }

    // Save RCX (skipped when RCX is dead) into rcx_mem in 2 steps via RAX:
    if (!rcx_is_dead) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX), xed_reg(XED_REG_RCX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rcx_mem, 64), 64),
                xed_reg(XED_REG_RAX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
    } else {
      g_num_rcx_save_skipped++;
    }

    if (targ_reg == XED_REG_RAX || base_reg == XED_REG_RAX || index_reg == XED_REG_RAX) {
       xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                 xed_reg(XED_REG_RAX),
                 xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64));
       if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
         return -1;
     }

     if (base_reg == XED_REG_RIP) {
       unsigned int orig_size = xed_decoded_inst_get_length (xedd);
       xed_int64_t new_disp = ins_addr + disp + orig_size;
       if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
          cerr << "Invalid rip displacement larger than 32 bits in add_profiling_instrs\n";
          return -1;
       }
       xed_int64_t new_disp_width = 32;
       xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                 xed_reg(XED_REG_RAX),
                 xed_mem_bisd(XED_REG_INVALID, index_reg, scale,
                              xed_disp(new_disp, new_disp_width),
                              mem_addr_width));
     } else if (targ_reg != XED_REG_RAX) {
         xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                  xed_reg(XED_REG_RAX),
                  (targ_reg != XED_REG_INVALID ? xed_reg(targ_reg) :
                   xed_mem_bisd(base_reg, index_reg, scale, xed_disp(disp, width), mem_addr_width)));
     }
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_reg(XED_REG_RBX), xed_reg(XED_REG_RAX));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;
       
     #if defined(TARGET_IA32E)
     xed_inst0(&enc_instr, dstate, XED_ICLASS_PUSHFQ, 64);
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;
     #endif

     xed_inst2(&enc_instr, dstate, XED_ICLASS_AND, 64,
               xed_reg(XED_REG_RAX), xed_imm0(MAX_TARG_ADDRS, 8));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_reg(XED_REG_RCX),
               xed_imm0((ADDRINT)&(bbl_map[bbl_num].targ_addr[0]), 64));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_mem_bisd(XED_REG_RCX, XED_REG_RAX, 8, xed_disp(0, 32), 64),
               xed_reg(XED_REG_RBX));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_reg(XED_REG_RBX),
               xed_imm0((ADDRINT)&(bbl_map[bbl_num].targ_count[0]), 64));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_reg(XED_REG_RCX),
               xed_mem_bisd(XED_REG_RBX, XED_REG_RAX, 8, xed_disp(0, 32), 64));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64,
               xed_reg(XED_REG_RCX),
               xed_mem_bd(XED_REG_RCX, xed_disp(1, 8), 64));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
               xed_mem_bisd(XED_REG_RBX, XED_REG_RAX, 8, xed_disp(0, 32), 64),
               xed_reg(XED_REG_RCX));
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;

     #if defined(TARGET_IA32E)
     xed_inst0(&enc_instr, dstate, XED_ICLASS_POPFQ, 64);
     if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;
     #endif

    // Restore RCX (skipped when RCX is dead) from rcx_mem in 2 steps via RAX:
    if (!rcx_is_dead) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX),
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rcx_mem, 64), 64));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RCX), xed_reg(XED_REG_RAX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
    }

    // Restore RBX (skipped when RBX is dead) from rbx_mem in 2 steps via RAX:
    if (!rbx_is_dead) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX),
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rbx_mem, 64), 64));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RBX), xed_reg(XED_REG_RAX));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
    }
  }

  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_reg(XED_REG_RAX),
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64));
   if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
     return -1;

   xed_inst2(&enc_instr, dstate, XED_ICLASS_LEA, 64,
             xed_reg(XED_REG_RAX),
             xed_mem_bd(XED_REG_RAX, xed_disp(1, 8), 64));
   if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
     return -1;

  xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
            xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)counter_addr, 64), 64),
            xed_reg(XED_REG_RAX));
  if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
    return -1;

  // Restore RAX (skipped when RAX is dead at the instrumentation point):
  if (!rax_is_dead) {
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RAX),
              xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;
  }

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
    if (KnobVerbose) {
    cerr << "jump to orig addr: 0x" << hex << instr_map[instr_map_entry].orig_targ_addr << " : ";
    dump_instr_from_mem ((ADDRINT *)instr_map[instr_map_entry].encoded_ins,
                         instr_map[instr_map_entry].orig_ins_addr);
    }

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

    if (category_enum == XED_CATEGORY_COND_BR) {
        xed_int64_t new_disp =
            (xed_int64_t)(instr_map[instr_map_entry].orig_targ_addr -
                         instr_map[instr_map_entry].new_ins_addr -
                         instr_map[instr_map_entry].size);

        xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(&xedd);
        xed_iform_enum_t iform_enum = xed_decoded_inst_get_iform_enum(&xedd);
        xed_uint_t new_disp_byts = 4;

        if (iclass_enum == XED_ICLASS_LOOP ||
            iclass_enum == XED_ICLASS_LOOPE ||
            iclass_enum == XED_ICLASS_LOOPNE ||
            iform_enum == XED_IFORM_JRCXZ_RELBRb) {
            if (new_disp > 127 || new_disp < -128) {
                cerr << "Invalid 8-bit displacement for loop/jrcxz to original code\n";
                dump_instr_map_entry(instr_map_entry);
                return -1;
            }
            new_disp_byts = 1;
        } else if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
            cerr << "Invalid conditional branch displacement larger than 32 bits\n";
            dump_instr_map_entry(instr_map_entry);
            return -1;
        }

        unsigned max_size = XED_MAX_INSTRUCTION_BYTES;
        unsigned new_size = 0;

        xed_encoder_request_init_from_decode(&xedd);
        xed_encoder_request_set_branch_displacement(&xedd, new_disp, new_disp_byts);

        xed_error_enum_t xed_error =
            xed_encode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins),
                       max_size, &new_size);
        if (xed_error != XED_ERROR_NONE) {
            cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
            dump_instr_map_entry(instr_map_entry);
            return -1;
        }
        return new_size;
    }

    if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_UNCOND_BR) {
         cerr << "ERROR: Invalid direct jump from translated code to original code for:\n";
         dump_instr_map_entry(instr_map_entry);
         return -1;
     }
 
     unsigned ilen = XED_MAX_INSTRUCTION_BYTES;
     unsigned olen = 0;
 
     xed_encoder_instruction_t  enc_instr;

     // search for orig_targ_addr in jump_to_orig_addr_map.
     int jump_to_orig_addr_map_entry = -1;
     for (unsigned i = 0; i < jump_to_orig_addr_num; i++) {
       if (instr_map[instr_map_entry].orig_targ_addr == jump_to_orig_addr_map[i]) {
         jump_to_orig_addr_map_entry = i;
         break;
       }
     }
    if (jump_to_orig_addr_map_entry < 0) {
      jump_to_orig_addr_map_entry = jump_to_orig_addr_num;
      jump_to_orig_addr_num++;
      if ((unsigned)jump_to_orig_addr_map_entry >= max_ins_count) {
         cerr << "exceeded size of jump_to_orig_addr_map at fix_direct_jmp_or_call_to_orig_addr\n";
         return -1;
      }
      jump_to_orig_addr_map[jump_to_orig_addr_map_entry] = instr_map[instr_map_entry].orig_targ_addr;
    }

    const xed_uint_t indirect_jmp_call_size = 6;
    xed_int64_t new_disp = (ADDRINT)&jump_to_orig_addr_map[jump_to_orig_addr_map_entry] -
                       instr_map[instr_map_entry].new_ins_addr -
                       indirect_jmp_call_size;
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
    // formed by previouis calls to fix_direct_jmp_or_call_to_orig_addr()
    // in order to relpace direct jumps to orig targ addrs.
    if (instr_map[instr_map_entry].targ_map_entry < 0) {
       int rc = fix_direct_jmp_or_call_to_orig_addr(instr_map_entry);
       return rc;
    }

    ADDRINT new_targ_addr;
    new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;

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

std::set<ADDRINT> candidate_rtn_heads;
std::set<ADDRINT> candidate_ins_addrs;
std::set<ADDRINT> candidate_direct_targets;

static bool isTranslatableSection(SEC sec) {
    return SEC_Valid(sec) && SEC_IsExecutable(sec) &&
           !SEC_IsWriteable(sec) && SEC_Address(sec) != 0;
}

static bool routineHasBaseTranslationProblem(RTN rtn) {
    if (RTN_Name(rtn) == "_init" || !RTN_IsSafeForProbedReplacement(rtn)) return true;
    RTN_Open(rtn);
    for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
        if (INS_IsDirectControlFlow(ins) && INS_DirectControlFlowTargetAddress(ins) == 0) {
            RTN_Close(rtn); return true;
        }
    }
    RTN_Close(rtn);
    return false;
}

static int prepareCandidateRoutines(IMG img) {
    candidate_rtn_heads.clear();
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
        if (!isTranslatableSection(sec)) continue;
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
            if (!routineHasBaseTranslationProblem(rtn))
                candidate_rtn_heads.insert(RTN_Address(rtn));
        }
    }

    bool changed;
    do {
        changed = false;
        candidate_ins_addrs.clear();
        for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
            if (!isTranslatableSection(sec)) continue;
            for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
                if (!candidate_rtn_heads.count(RTN_Address(rtn))) continue;
                RTN_Open(rtn);
                for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins))
                    candidate_ins_addrs.insert(INS_Address(ins));
                RTN_Close(rtn);
            }
        }

        std::vector<ADDRINT> remove_heads;
        for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
            if (!isTranslatableSection(sec)) continue;
            for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
                if (!candidate_rtn_heads.count(RTN_Address(rtn))) continue;
                RTN_Open(rtn);
                for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
                    if (INS_Category(ins) == XED_CATEGORY_COND_BR && INS_IsDirectControlFlow(ins)) {
                        if (!candidate_ins_addrs.count(INS_DirectControlFlowTargetAddress(ins))) {
                            remove_heads.push_back(RTN_Address(rtn));
                            break;
                        }
                    }
                }
                RTN_Close(rtn);
            }
        }
        for (ADDRINT head : remove_heads) {
            if (candidate_rtn_heads.erase(head)) changed = true;
        }
    } while (changed);

    candidate_ins_addrs.clear();
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
        if (!isTranslatableSection(sec)) continue;
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
            if (!candidate_rtn_heads.count(RTN_Address(rtn))) continue;
            RTN_Open(rtn);
            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
                candidate_ins_addrs.insert(INS_Address(ins));
                if (INS_IsDirectControlFlow(ins))
                    candidate_direct_targets.insert(INS_DirectControlFlowTargetAddress(ins));
            }
            RTN_Close(rtn);
        }
    }
    return 0;
}

/********************************/
/* find_candidate_rtns_for_tc() */
/********************************/
int find_candidate_rtns_for_tc(IMG img)
{
    int rc = 0;
    ADDRINT last_selected_rtn_end = 0;
	
	if (prepareCandidateRoutines(img) < 0)
        return -1;

    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
            continue;

       for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
       {
           if (!IsCandidateRtnForTranslation(rtn, last_selected_rtn_end))
               continue;

           // Open the RTN.
           RTN_Open( rtn );

           // Map all instructions that are a target of some direct jump or call in the rtn.
           std::map<ADDRINT, bool>is_targ_map;
           is_targ_map.empty();
           // Address -> INS map for this routine, used for successor-aware liveness.
           std::map<ADDRINT, INS> addr2ins;
           for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
              addr2ins[INS_Address(ins)] = ins;
              if (INS_IsDirectControlFlow(ins)) {
                ADDRINT targ_addr = INS_DirectControlFlowTargetAddress(ins);
                is_targ_map[targ_addr] = true;
              }
           }

            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {

                //debug print of orig instruction:
                if (KnobVerbose) {
                    cerr << "old instr: ";
                    cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) <<  endl;
                }

                ADDRINT ins_addr = INS_Address(ins);

                xed_decoded_inst_t xedd;
                xed_error_enum_t xed_code;

                // Add instr into instr map:
                bool isRtnHeadIns = (RTN_Address(rtn) == ins_addr);
                ins_enum_t ins_type = (isRtnHeadIns ? RtnHeadIns : RegularIns);

                // Insert a NOP7 instr at Rtn Head to be used in order
                // to restore orig target of a cond jumps to a routine.
                //
                if (!KnobNoProfile && isRtnHeadIns) {
                  rc = create_nop7_xedd_instr(&xedd);
                  if (rc < 0) {
                    cerr << "ERROR: failed to create a NOP7 instr during translation of instr at: "
                         << "0x" << hex << ins_addr << endl;
                    return -1;
                  }
                  rc = add_new_instr_entry(&xedd, ins_addr, ins_type);
                  if (rc < 0) {
                    cerr << "ERROR: failed during instructon translation." << endl;
                    return -1;
                  }
                  ins_type = RegularIns;
                }

                // Check if ins is a control transfer instr that terminates a BBL
                // or the next instr is a target of a direct branch or call.
                INS next_ins = INS_Next(ins);
                bool isNextInsJumpTarget = 
                    (!INS_Valid(next_ins) ? false : is_targ_map[INS_Address(next_ins)]);
                bool isInsTerminatesBBL = (isJumpOrRet(ins) || isNextInsJumpTarget);
                const bool profile_this_bbl = (!KnobNoProfile && bbl_num < MAX_BBL_COUNT);

               if (profile_this_bbl && isInsTerminatesBBL) {
                   // Deadness is evaluated at the instrumentation point, which is
                   // immediately before the BBL terminator instruction 'ins'.
                   bool rax_dead = false, rbx_dead = false, rcx_dead = false;
                   if (!KnobNoDeadRegOpt) {
                     rax_dead = regIsDeadFrom(ins, LEVEL_BASE::REG_RAX);
                     if (rax_dead) {
                       g_num_rax_skipped_straightline++;
                     } else if (raxDeadOnBothCondBranchSuccessors(ins, addr2ins)) {
                       // Successor-aware rule: RAX dead on BOTH cond-branch successors.
                       rax_dead = true;
                       g_num_rax_skipped_successor++;
                     }
                     rbx_dead = regIsDeadFrom(ins, LEVEL_BASE::REG_RBX);
                     rcx_dead = regIsDeadFrom(ins, LEVEL_BASE::REG_RCX);
                   }
                   rc = add_profiling_instrs(ins, ins_addr,
                                             &bbl_map[bbl_num].counter, bbl_num,
                                             rax_dead, rbx_dead, rcx_dead);
                   if (rc < 0)
                     return -1;
               }

                // Add ins to instr_map:
                //
                xed_decoded_inst_zero_set_mode(&xedd,&dstate);
                xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(ins_addr), max_inst_len);
                if (xed_code != XED_ERROR_NONE) {
                    cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << ins_addr << endl;
                    RTN_Close(rtn);
                    return -1;
                }

                // Add the instr into the instr_map table.
                rc = add_new_instr_entry(&xedd, INS_Address(ins), ins_type);
                if (rc < 0) {
                    cerr << "ERROR: failed during instructon translation." << endl;
                    RTN_Close(rtn);
                    return -1;
                }

                if (isInsTerminatesBBL) {
                  if (profile_this_bbl) {
                    bbl_map[bbl_num].terminating_ins_entry = num_of_instr_map_entries - 1;
                    bbl_num++;
                    if (bbl_num == MAX_BBL_COUNT) {
                      cerr << "reached MAX_BBL_COUNT profiled BBLs; "
                           << "continuing translation without profiling\n";
                    }
                    if (bbl_num < MAX_BBL_COUNT)
                      bbl_map[bbl_num].starting_ins_entry = num_of_instr_map_entries;
                  } else {
                    bbl_num++;
                  }
                }

               if (profile_this_bbl && INS_Category(ins) == XED_CATEGORY_COND_BR) {
                 // The fall-through stub executes on the not-taken path, i.e. right
                 // before the fall-through instruction; evaluate deadness from there.
                 INS fallthru_ins = INS_Next(ins);
                 bool rax_dead = (!KnobNoDeadRegOpt && INS_Valid(fallthru_ins))
                                 ? regIsDeadFrom(fallthru_ins, LEVEL_BASE::REG_RAX) : false;
                 if (rax_dead)
                   g_num_rax_skipped_straightline++;
                 rc = add_profiling_instrs(ins, ins_addr,
                                           &bbl_map[bbl_num - 1].fallthru_counter, bbl_num-1,
                                           rax_dead, false, false);
                 if (rc < 0)
                   return -1;
               }

            } // end for INS...

            if (KnobVerbose) {
                cerr <<   "rtn name: " << RTN_Name(rtn) << endl;
            }

            // Close the RTN.
            RTN_Close( rtn );

         } // end for RTN..
    } // end for SEC...

    return 0;
}


/***************************/
/* int copy_instrs_to_tc() */
/***************************/
int copy_instrs_to_tc(char *tc)
{
    int cursor = 0;

    for (unsigned i=0; i < num_of_instr_map_entries; i++) {

      if ((ADDRINT)&tc[cursor] != instr_map[i].new_ins_addr) {
          cerr << "ERROR: Non-matching instruction addresses: "
               << hex << (ADDRINT)&tc[cursor]
               << " vs. " << instr_map[i].new_ins_addr << endl;
          return -1;
      }

      memcpy(&tc[cursor], (char *)instr_map[i].encoded_ins, instr_map[i].size);

      cursor += instr_map[i].size;
    }

    return cursor;
}


/***************************************/
/* void commit_translated_rtns_to_tc() */
/***************************************/
inline void commit_translated_rtns_to_tc()
{
    for (unsigned i = 0; i < num_of_instr_map_entries; ++i) {
        if (instr_map[i].ins_type != RtnHeadIns)
            continue;

        RTN rtn = RTN_FindByAddress(instr_map[i].orig_ins_addr);
        if (!RTN_Valid(rtn)) continue;

        // CRITICAL FIX: Ensure exact address match to prevent offset probing
        if (RTN_Address(rtn) != instr_map[i].orig_ins_addr) continue;
        if (!RTN_IsSafeForProbedReplacement(rtn)) continue;

        AFUNPTR original = RTN_ReplaceProbed(
            rtn,
            reinterpret_cast<AFUNPTR>(instr_map[i].new_ins_addr));

        if (original == NULL && KnobVerbose) {
            cerr << "RTN_ReplaceProbed failed for 0x" << hex << RTN_Address(rtn) << dec << endl;
        }
    }
}

/**********************************************/
/* start_stop_profile_gathering_thread_func() */
/**********************************************/
void start_stop_profile_gathering_thread_func(void *v)
{
    if (KnobNoProfile)
        return;

    // Wait until create_tc() has fully built and committed the TC
    while (tc_state == 0)
        usleep(1000);
    if (tc_state < 0)
        return;

    cerr << " prof time: " << dec << KnobNumSecsDuringProfile.Value() << " sec\n";
    sleep(KnobNumSecsDuringProfile.Value());
    cerr << "disabling profile gathering\n";

    int rc = disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
    if  (rc < 0)
        return;
}

/****************************/
/* allocate_and_init_memory */
/****************************/
int allocate_and_init_memory(IMG img)
{
    ADDRINT highest_addr = 0;
    unsigned candidate_ins_count = 0;

    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
            continue;

        if (!lowest_sec_addr || lowest_sec_addr > SEC_Address(sec))
            lowest_sec_addr = SEC_Address(sec);

        if (highest_sec_addr < SEC_Address(sec) + SEC_Size(sec))
            highest_sec_addr = SEC_Address(sec) + SEC_Size(sec);

        ADDRINT last_selected_rtn_end = 0;
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {
            if (highest_addr < RTN_Address(rtn) + RTN_Size(rtn))
                highest_addr = RTN_Address(rtn) + RTN_Size(rtn);

            if (IsCandidateRtnForTranslation(rtn, last_selected_rtn_end)) {
                max_rtn_count++;
                candidate_ins_count += RTN_NumIns(rtn);
            }
        }
    }

    if (candidate_ins_count == 0)
        candidate_ins_count = 1;

    // Original ins + profiling stubs (capped at MAX_BBL_COUNT profiled BBLs).
    max_ins_count = candidate_ins_count * 10 + MAX_BBL_COUNT * 50;
    max_instr_map_entries = 2 * max_ins_count;

    cerr << " candidate ins: " << dec << candidate_ins_count
         << ", instr_map capacity: " << max_instr_map_entries << endl;

    int pagesize = sysconf(_SC_PAGE_SIZE);
    if (pagesize == -1) {
      perror("sysconf");
      return -1;
    }

    ADDRINT text_size = (highest_sec_addr - lowest_sec_addr) * 2 + pagesize * 4;

    // Match btranslate TC sizing, plus headroom for profiling stubs.
    max_tc_size = 2 * text_size + pagesize * 4 + MAX_BBL_COUNT * 256;
    if (max_tc_size >= 0x7FFFFFFF) {
      cerr << "size of TC is beyond the range of a branch displacement" << endl;
      return -1;
    }

    const size_t mem_size =
              max_tc_size +                     // TC + TC2 size
              max_ins_count * sizeof(ADDRINT);  // jump_to_orig_addr_map size
    char *addr = nullptr;
    ADDRINT max_distance = 0x7FFFFFFF;
    const size_t step = pagesize; 
    ADDRINT aligned_target = ((ADDRINT)highest_addr) & ~(pagesize - 1);
    void* result = mmap((void*)aligned_target, mem_size,
                       PROT_READ | PROT_WRITE | PROT_EXEC,
                       MAP_PRIVATE | MAP_ANONYMOUS,
                       0, 0);
    if (result != MAP_FAILED &&
        (abs((long)((ADDRINT)result - aligned_target)) <= (long)max_distance)) {
        addr = (char *)result;
    }

    if (!addr) {
        for (size_t offset = step; offset <= max_distance; offset += step) {
            ADDRINT try_addr = aligned_target + offset;
            result = mmap((void*)try_addr, mem_size,
                         PROT_READ | PROT_WRITE | PROT_EXEC,
                         MAP_PRIVATE | MAP_ANONYMOUS,
                         0, 0);
            if (result != MAP_FAILED &&
                (abs((long)((ADDRINT)result - try_addr)) <= (long)max_distance)) {
                addr = (char *)result;
                break;
            }
            if (result != MAP_FAILED) {
                munmap(result, mem_size);
            }

            if (highest_addr >= offset) {
                try_addr = aligned_target - offset;
                result = mmap((void*)try_addr, mem_size,
                             PROT_READ | PROT_WRITE | PROT_EXEC,
                             MAP_PRIVATE | MAP_ANONYMOUS,
                             0, 0);
                if (result != MAP_FAILED &&
                    (abs((long)((ADDRINT)result - try_addr)) <= (long)max_distance)) {
                    addr = (char *)result;
                    break;
                }
                if (result != MAP_FAILED) {
                    munmap(result, mem_size);
                }
            }
        }
    }

    if (!addr) {
        cerr << "failed to allocate memory within 32-bit range. " << endl;
        return -1;
    }

    cerr << " allocated memory at: 0x" << hex << (ADDRINT)addr << "\n";

    tc = (char *)addr;
    addr += max_tc_size;

    jump_to_orig_addr_map = (ADDRINT *)addr;

    instr_map = (instr_map_t *)calloc(max_instr_map_entries, sizeof(instr_map_t));
    if (instr_map == NULL) {
        perror("calloc");
        return -1;
    }

    bbl_map = (bbl_map_t *)calloc(MAX_BBL_COUNT, sizeof(bbl_map_t));
    if (bbl_map == NULL) {
        perror("calloc");
        return -1;
    }

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
    cerr << "Reached _exit." << endl;
    dump_profile();

    clock_gettime(CLOCK_MONOTONIC, &end_running_time);
    double elapsed = (end_running_time.tv_sec - start_running_time.tv_sec) + 
                     (end_running_time.tv_nsec - start_running_time.tv_nsec) / 1e9;
	cerr << " Translated code run took: " << elapsed << " seconds\n";
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
    cerr << "after memory allocation" << endl;

    // Step 2: go over all routines and identify candidate routines and copy
    //         their code into the instr map IR:
    rc = find_candidate_rtns_for_tc(img);
    if (rc < 0) {
        cerr << "failed to find candidates for translation\n";
        return;
    }
    cerr << "after identifying candidate routines" << endl;
    cerr << " [deadreg-opt] BBL profiling stubs: " << dec << g_num_bbl_stubs
         << " | RAX save/restore skipped: " << g_num_rax_save_skipped
         << " (straight-line: " << g_num_rax_skipped_straightline
         << ", successor-aware: " << g_num_rax_skipped_successor << ")"
         << " | RBX save/restore skipped: " << g_num_rbx_save_skipped
         << " | RCX save/restore skipped: " << g_num_rcx_save_skipped << endl;

    // Step 3: Chaining - calculate direct branch and call instructions to point
    //         to corresponding target instr entries:
    chain_all_direct_jmp_and_call_target_entries(0, num_of_instr_map_entries);
    cerr << "after chaining all branch targets" << endl;

    // Step 4: Set initial estimated new addrs for each instruction in the tc.
    rc = set_initial_estimated_new_ins_addrs_in_tc(tc);
    if (rc < 0 ) {
        cerr << "failed to set initial estimated new ins addrs in the TC\n";
        return;
    }
    cerr << "after setting initial estimated new ins addrs in the TC" << endl;

    // Step 5: fix rip-based, direct branch and direct call displacements:
    rc = fix_instructions_displacements();
    if (rc < 0 ) {
        cerr << "failed to fix displacments of translated instructions\n";
        return;
    }
    cerr << "after fixing instructions displacements" << endl;

    // Step 6: write translated instructions to the tc:
    rc = copy_instrs_to_tc(tc);
    if (rc < 0 ) {
        cerr << "failed to copy the instructions to the translation cache\n";
        return;
    }
    tc_size = rc;
    cerr << "after write all new instructions to memory tc" << endl;

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
      cerr << "after commit of translated routines from orig code to TC" << endl;
    }

    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) +
	                 (end.tv_nsec - start.tv_nsec) / 1e9;
    cerr << " create_tc took: " << elapsed << " seconds\n";

	clock_gettime(CLOCK_MONOTONIC, &start_running_time);
	__sync_synchronize();
    tc_state = 1;
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
    // Open output profile file (EX4 requirement 3: edge-profile.csv).
    out = new std::ofstream("edge-profile.csv");

    // Initialize pin & symbol manager
    if( PIN_Init(argc,argv) )
        return Usage();

    PIN_InitSymbols();

    // Register create_tc
    IMG_AddInstrumentFunction(create_tc, 0);

    // Create internal thread to start and stop profile gathering.
    THREADID tid = PIN_SpawnInternalThread(start_stop_profile_gathering_thread_func, NULL, 0, NULL);
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

