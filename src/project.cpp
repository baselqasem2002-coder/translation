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
 * profiling for each BBL and for each indirect jump target and indirect call target
 * (used for de-virtualization in TC2).
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

KNOB<UINT> KnobDevirtPercent(KNOB_MODE_WRITEONCE,    "pintool",
    "devirt_pct", "90", "De-virtualize an indirect jump/call in TC2 when one target "
    "took at least this percentage of its executions during profiling");

KNOB<BOOL> KnobNoDevirt(KNOB_MODE_WRITEONCE,    "pintool",
    "no_devirt", "0", "Do not apply de-virtualization in TC2");

KNOB<BOOL> KnobNoReorder(KNOB_MODE_WRITEONCE,    "pintool",
    "no_reorder", "0", "Do not apply code reordering in TC2");


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

#define MAX_TARG_ADDRS 0x3

// Kind of indirect control transfer that terminates a BBL.
typedef enum {
  NoIndirect = 0,
  IndirectJump,   // jmp reg / jmp [mem]
  IndirectCall,   // call reg / call [mem]
} indirect_kind_t;

// Bbl map of all the bbl exec counters to be collected at runtime:
typedef struct {
  UINT64 counter;
  UINT64 fallthru_counter; // for BBLs that terminate with a cond branch.
  ADDRINT targ_addr[MAX_TARG_ADDRS+1];
  UINT64  targ_count[MAX_TARG_ADDRS+1];
  unsigned starting_ins_entry;
  unsigned terminating_ins_entry;
  // Set for BBLs that end with an indirect jump or call whose targets we
  // profile. We keep the ORIGINAL address of that jmp/call here because
  // create_tc2_thread_func() later overwrites instr_map[].orig_ins_addr.
  indirect_kind_t indirect_kind;
  ADDRINT indirect_site_addr;
} bbl_map_t;

bbl_map_t *bbl_map;
unsigned bbl_num = 0;
std::map<ADDRINT, unsigned> entry_map;

unsigned max_rtn_count = 0;

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

// Indirect call: 'call reg' or 'call [mem]'.
bool isIndirectCall(INS ins)
{
   return INS_IsCall(ins) && INS_IsIndirectControlFlow(ins);
}

// Indirect jump or indirect call (a 'ret' is not included).
bool isIndirectJumpOrCall(INS ins)
{
   return INS_IsIndirectControlFlow(ins) && !INS_IsRet(ins);
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

/****************************/
/*  dump_indirect_profile() */
/****************************/
// Print the profiled targets of every indirect jump / indirect call that
// was executed during the profiling time (enabled by the -dump_prof knob).
// Must be called after profiling stopped and BEFORE instr_map is rewritten
// for TC2. It only uses bbl_map (the original site address is saved there).
// Example output line:
//   indirect call at 0x4012a0 exec: 1000 | targ 0x401800 count: 990 | targ 0x401900 count: 10
void dump_indirect_profile()
{
    unsigned executed_sites = 0;
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].indirect_kind == NoIndirect || !bbl_map[b].counter)
        continue;
      executed_sites++;
      cerr << (bbl_map[b].indirect_kind == IndirectCall ? "indirect call" : "indirect jump")
           << " at 0x" << hex << bbl_map[b].indirect_site_addr
           << " exec: " << dec << bbl_map[b].counter;
      for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++) {
        if (!bbl_map[b].targ_count[j])
          continue;
        cerr << " | targ 0x" << hex << bbl_map[b].targ_addr[j]
             << " count: " << dec << bbl_map[b].targ_count[j];
      }
      cerr << endl;
    }
    cerr << "indirect sites executed during profiling: " << dec << executed_sites << endl;
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
            while (instr_map[i+j].ins_type == ProfilingIns) {
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
          memcpy((ADDRINT *)instr_map[i].new_ins_addr, encoded_jmp_ins, olen);
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

/************************************************/
/* Indirect jump / indirect call target operand */
/************************************************/
// Describes where an indirect jmp/call reads its target address from:
//   'jmp/call reg'          -> targ_reg
//   'jmp/call [mem]'        -> base_reg + index_reg*scale + disp
//   'jmp/call [rip+disp]'   -> the absolute address rip_mem_addr
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

// Statistics printed after the TC is built.
unsigned num_prof_indirect_jump_sites = 0;
unsigned num_prof_indirect_call_sites = 0;
unsigned num_unsupported_indirect_sites = 0;

// Fill 't' for the indirect jmp/call 'xedd' located at 'ins_addr'.
// Returns false for forms we do not profile (far jmp/call, FS/GS segment
// override, non 64-bit operand). Those sites still get their BBL counter,
// only their targets are not recorded.
//
// NOTE: we look at the FIRST EXPLICIT OPERAND to decide register vs.
// memory. We can NOT use "number of memory operands == 0" like the jump
// code in bprofile-with-gearing.cpp did, because for a CALL, XED also
// reports the implicit stack write (the push of the return address) as a
// memory operand. E.g. 'call rax' has 1 memory operand, [rsp], and using
// it would record the value at the top of the stack instead of RAX.
// For 'call [mem]' the target is memory operand 0 and the push is operand 1.
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

/**************************/
/* add_profiling_instrs() */
/**************************/
// Adds a profiling stub right before 'ins' (the instruction that ends the
// BBL), or right after it for the fallthru counter of a cond branch.
// The stub increments *counter_addr. If 'ins' is an indirect jump or an
// indirect call, the stub also records the target address in
// bbl_map[bbl_num].targ_addr[] / targ_count[] (used by de-virtualization).
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

  // Create profiling for indirect jump AND indirect call targets.
  indirect_target_t t;
  bool profile_targets = false;
  if (isIndirectJumpOrCall(ins)) {
    profile_targets = get_indirect_target_operand(INS_XedDec(ins), ins_addr, &t);
    if (!profile_targets)
      num_unsupported_indirect_sites++;
  }

  if (profile_targets) {
    // Debug print.
    //cerr << " BBL terminates with indirect jump/call: "
    //     << " 0x" << hex << ins_addr << ": "
    //     << INS_Disassemble(ins) << "\n";

    bool is_call = INS_IsCall(ins);
    bbl_map[bbl_num].indirect_kind = (is_call ? IndirectCall : IndirectJump);
    bbl_map[bbl_num].indirect_site_addr = ins_addr;
    if (is_call)
      num_prof_indirect_call_sites++;
    else
      num_prof_indirect_jump_sites++;

    static uint64_t rbx_mem = 0;
    static uint64_t rcx_mem = 0;

    // The stub (all flags-safe for jmp/call, see the AND below):
    //
    // save RBX into rbx_mem in 2 steps via RAX
    // save RCX into rcx_mem in 2 steps via RAX
    // Convert jmp/call [base_reg + index_reg*scale] to: MOV RAX, [base_reg + index_reg*scale]
    //         Or convert jmp/call targ_reg to: MOV RAX, targ_reg ==> RAX holds targ addr
    // MOV RBX, RAX ==> Now RBX also holds targ addr
    // MOV RCX, RAX / SHR RCX, 4 / XOR RAX, RCX / AND RAX, MAX_TARG_ADDRS
    //                              ==> RAX holds slot index i = 0..MAX_TARG_ADDRS
    // MOV RCX, xed_imm0((ADDRINT)&bbl_map_targ_addr[bbl_num][0])
    // MOV [RCX + 8*RAX], RBX
    // MOV RBX, xed_imm0((ADDRINT)&bbl_map_targ_count[bbl_num][0])
    // MOV RCX, [RBX + 8*RAX]
    // LEA RCX, [RCX + 1]
    // MOV [RBX + 8*RAX], RCX
    // restore RCX from rcx_mem in 2 steps via RAX
    // restore RBX from rbx_mem in 2 steps via RAX
    //
    // Saving RBX/RCX does not modify them, so a target operand that uses
    // RBX or RCX (e.g. 'call [rbx+0x10]') still sees the original values.
    // Only RAX was already overwritten, so it is reloaded from rax_mem when
    // the target operand uses it.
    
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
    
    // Load the jump/call target into RAX.
    //
    // 1. If the target operand uses RAX (e.g. 'call rax', 'jmp [rax*8+tbl]'),
    //    first restore the original RAX from rax_mem.
    if (t.targ_reg == XED_REG_RAX || t.base_reg == XED_REG_RAX || t.index_reg == XED_REG_RAX) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX), // Destination reg op.
                xed_mem_bd(XED_REG_INVALID, xed_disp((ADDRINT)&rax_mem, 64), 64));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
       return -1;
    }

    // 2. Read the target.
    if (t.base_reg == XED_REG_RIP) {
      // 'jmp/call [rip+disp]': emit 'mov rax, [rip+disp']' and let
      // fix_rip_displacement() relocate it like any other rip-relative
      // instr, both in TC and later in TC2. It has 7 bytes
      // (REX.W 8B 05 disp32), so rip after it = ins_addr + 7.
      // (bprofile-with-gearing.cpp converted this to an absolute 32-bit
      // address, which fails for PIE binaries loaded above 2GB.)
      const unsigned mov_rip_size = 7;
      xed_int64_t new_disp = (xed_int64_t)t.rip_mem_addr - (xed_int64_t)(ins_addr + mov_rip_size);
      if (new_disp > 0x7FFFFFFF || new_disp < -0x7FFFFFFF) {
        cerr << "Invalid rip displacement larger than 32 bits in add_profiling_instrs\n";
        return -1;
      }
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX),    // Destination reg op.
                xed_mem_bd(XED_REG_RIP, xed_disp(new_disp, 32), 64));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
      // Sanity check: the new instr must read exactly the same address.
      if (instr_map[num_of_instr_map_entries - 1].orig_rip_addr != t.rip_mem_addr) {
        cerr << "ERROR: bad rip-relative target load in add_profiling_instrs at 0x"
             << hex << ins_addr << endl;
        return -1;
      }
    } else if (t.targ_reg != XED_REG_INVALID) {
      if (t.targ_reg != XED_REG_RAX) {         // RAX already holds it.
        xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                  xed_reg(XED_REG_RAX),        // Destination reg op.
                  xed_reg(t.targ_reg));
        if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
          return -1;
      }
    } else {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
                xed_reg(XED_REG_RAX),          // Destination reg op.
                xed_mem_bisd(t.base_reg, t.index_reg, t.scale,
                             xed_disp(t.disp, t.disp_width), t.mem_addr_width));
      if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
        return -1;
    }
    
    // MOV RBX, RAX
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RBX),    // Destination reg op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;

    // Pick the slot for this target:  slot = (targ ^ (targ >> 4)) & MAX_TARG_ADDRS
    //
    // bprofile-with-gearing.cpp used slot = targ & MAX_TARG_ADDRS (the 2 low
    // bits). Function entries are usually 16-byte aligned, so every target
    // of an indirect CALL has low bits 00 and all of them would share slot 0.
    // Mixing in bits 4-5 spreads aligned function entries over the slots,
    // while jump-table case labels (any alignment) still differ in bits 0-1.
    // Two targets can still share a slot. The slot then keeps the LAST
    // target seen and the SUM of the counts, so it is a hint. That is fine
    // for de-virtualization, which always compares the real target against
    // the predicted one at run time.
    // (SHR, XOR and AND modify RFLAGS, like the AND in the original stub.
    // That is safe: the next instr is an indirect jmp/call, which does not
    // read the flags, and the ABI does not pass flags to a called function.)
    xed_inst2(&enc_instr, dstate, XED_ICLASS_MOV, 64,
              xed_reg(XED_REG_RCX),    // Destination reg op.
              xed_reg(XED_REG_RAX));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;

    xed_inst2(&enc_instr, dstate, XED_ICLASS_SHR, 64,
              xed_reg(XED_REG_RCX),    // Destination reg op.
              xed_imm0(4, 8));
    if (add_new_encoded_instr(ins_addr, &enc_instr, ProfilingIns) < 0)
      return -1;

    xed_inst2(&enc_instr, dstate, XED_ICLASS_XOR, 64,
              xed_reg(XED_REG_RAX),    // Destination reg op.
              xed_reg(XED_REG_RCX));
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

  } // end of: 'if bbl terminates with indirect jump or call'.
  
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
    cerr << "jump to orig addr: 0x" << hex << instr_map[instr_map_entry].orig_targ_addr << " : ";
    dump_instr_from_mem ((ADDRINT *)instr_map[instr_map_entry].encoded_ins,
                         instr_map[instr_map_entry].orig_ins_addr);

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


/********************************/
/* find_candidate_rtns_for_tc() */
/********************************/
int find_candidate_rtns_for_tc(IMG img)
{
    int rc = 0;
    // go over routines and check if they are candidates for translation and mark them for translation:

    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
            continue;

        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {
            // Keep the entry num of the rtn head in case we need to
            // revert the insertin of the instruction in rtn into the instructions
            // map due to an invalid decoding.
            //unsigned rtn_entry = num_of_instr_map_entries;

            //if (RTN_Name(rtn) == ".plt")
            //    continue;
            
            // Open the RTN.
            RTN_Open( rtn );

            // Map all instructions that are a target of some direct jump or call in the rtn.
            std::map<ADDRINT, bool>is_targ_map;
            is_targ_map.empty();
            for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
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
                    //xed_print_hex_line(reinterpret_cast<UINT8*>(INS_Address (ins)), INS_Size(ins));
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
                // An indirect call also terminates a BBL, so that the BBL's
                // profiling stub (placed right before it) records the call
                // targets in that BBL's own targ_addr[]/targ_count[] slots.
                // Direct calls do not end a BBL (their target is known).
                INS next_ins = INS_Next(ins);
                bool isNextInsJumpTarget =
                    (!INS_Valid(next_ins) ? false : is_targ_map[INS_Address(next_ins)]);
                bool isInsTerminatesBBL = (isJumpOrRet(ins) || isIndirectCall(ins) ||
                                           isNextInsJumpTarget);

                // Add profiling instructions to count each BBL exec at runtime:
                //
                if (!KnobNoProfile) {
                  // Do not insert the profiling now if there is a later instr
                  // in the BBL that kills RAX.
                  if (isInsTerminatesBBL) {
                    rc = add_profiling_instrs(ins, ins_addr, &bbl_map[bbl_num].counter, bbl_num);
                    if (rc < 0)
                      return -1;
                  }
                }
          
                // Add ins to instr_map:
                //
                xed_decoded_inst_zero_set_mode(&xedd,&dstate);
                xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(ins_addr), max_inst_len);
                if (xed_code != XED_ERROR_NONE) {
                    cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << ins_addr << endl;
                    return -1;
                }

                // Add the instr into the instr_map table.
                rc = add_new_instr_entry(&xedd, INS_Address(ins), ins_type);
                if (rc < 0) {
                    cerr << "ERROR: failed during instructon translation." << endl;
                    return -1;
                }

                if (isInsTerminatesBBL) {
                  bbl_map[bbl_num].terminating_ins_entry = num_of_instr_map_entries - 1;
                  bbl_num++;
                  bbl_map[bbl_num].starting_ins_entry = num_of_instr_map_entries;
                }

                // Apply edge Profiling: For BBLs that end with a conditional branch,
                //     insert an increment of the fallthrough counter for this BBL,
                //     immediately after the cond branch which terminates the bbl.
                //     and before the next BBL.
                if (!KnobNoProfile && INS_Category(ins) == XED_CATEGORY_COND_BR) {
                  rc = add_profiling_instrs(ins, ins_addr,
                                            &bbl_map[bbl_num - 1].fallthru_counter, bbl_num-1);
                  if (rc < 0)
                    return -1;
                }

            } // end for INS...

            // debug print of routine name:
            if (KnobVerbose) {
                cerr <<   "rtn name: " << RTN_Name(rtn) << endl;
            }

            // Close the RTN.
            RTN_Close( rtn );

            // Apply local chaining of direct calls and branches for this routine.
            //chain_all_direct_jmp_and_call_target_entries(rtn_entry, num_of_instr_map_entries);

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
    cerr << " prof time: " << dec << KnobNumSecsDuringProfile << " sec\n";
    sleep(KnobNumSecsDuringProfile);

    cerr << "disabling profile gathering\n";

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
      
       // Insert a probing jump from routine header in TC to its corresponding 
       // header in TC2, provided it is a wide NOP instr.
       if (instr_map[i].ins_type != RtnHeadIns ||
           instr_map[i].xed_category != XED_CATEGORY_WIDENOP)
         continue;
       
       // Form a probing jump instruction:
       //
      
       // Option 1: Use a direct jump for probing:
       unsigned int olen = encode_jump_instr(instr_map[i].orig_ins_addr, 
                                             instr_map[i].new_ins_addr, 
                                             instr_map[i].encoded_ins);
       if (olen < 0)
         return -1;
  
       // Option 2: Use an indirect jump for probing:
       //xed_int64_t new_disp = (ADDRINT)&instr_map[i].new_ins_addr - instr_map[i].orig_ins_addr - olen;
       //xed_inst1(&enc_instr, dstate,
       //    XED_ICLASS_JMP, 64,
       //    xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));
  
       //memcpy((ADDRINT *)instr_map[i].orig_ins_addr, instr_map[i].encoded_ins, olen);
      
       // Set the probing jump instruction atomically in 2 stages:
       //
       
       // 1st stage: set the last 4 bytes of the probe jmp instr.
       if (olen > 4)
         memcpy((char *)(instr_map[i].orig_ins_addr + 4),
                (char *)((ADDRINT)instr_map[i].encoded_ins + 4), olen - 4);
       
       // 2nd stage: set the first 4 bytes of the probe jmp instr.
       memcpy((char *)instr_map[i].orig_ins_addr, instr_map[i].encoded_ins, 4);
     
       //debug print:
       cerr << " committing rtN from: 0x" << hex << instr_map[i].orig_ins_addr
            << " to: 0x" << hex << instr_map[i].new_ins_addr 
            << " size: " << olen
            << endl;
       dump_instr_from_mem ((ADDRINT *)instr_map[i].orig_ins_addr, instr_map[i].orig_ins_addr);
  }

  return 0;
}


/* ============================================================= */
/* De-virtualization (TC2)                                       */
/* ============================================================= */
//
// THRESHOLD ("frequent" vs "rare" target):
//   A target of an indirect jump/call is FREQUENT when it took at least
//   -devirt_pct percent (default 90%) of that jump/call's executions
//   during profiling, i.e.  hot_count * 100 >= exec_count * devirt_pct.
//   Only a site with a frequent target is de-virtualized. All its other
//   targets are rare and keep using the original indirect jmp/call.
//
// WHAT IS EMITTED IN TC2 (T = the frequent target):
//
//   indirect call 'call X'            indirect jump 'jmp X'
//   ----------------------            ---------------------
//     cmp  X, T                         cmp  X, T
//     jne  miss                         je   T_in_TC2    ; direct jump
//     call T_in_TC2  ; direct call      jmp  X           ; original instr
//     jmp  done
//   miss:
//     call X         ; original instr
//   done:
//
//   X is the original target operand (a register or a memory operand), so
//   a rare target still reaches the right place. T_in_TC2 is the TC2 copy
//   of T: the direct call/jump also skips the probe jumps
//   (original code -> TC -> TC2) that the indirect call went through, and
//   for jump tables it keeps execution inside TC2 (a jump-table entry holds
//   an ORIGINAL code address).
//
// A site is NOT de-virtualized (it is copied unchanged) when:
//   - no target reaches the threshold,
//   - T is not translated (a direct jump to it would leave TC2),
//   - T does not fit in the signed 32-bit immediate of 'cmp' (a binary
//     loaded above 2GB, e.g. PIE),
//   - its form is not supported (see get_indirect_target_operand()).
//
// SAFETY NOTES:
//   - 'cmp' modifies RFLAGS. At a call this is safe: by the ABI a called
//     function does not receive flags. At an indirect jump we assume the
//     flags are not used by the target, the same assumption the profiling
//     stub in TC makes (it runs AND before the jump).
//   - The profiled hot target is only a hint (two targets may share a
//     profiling slot). Correctness never depends on it, since the 'cmp'
//     checks the real target every time.

// Statistics printed after TC2 is built.
unsigned num_devirt_calls = 0;
unsigned num_devirt_jumps = 0;
unsigned num_devirt_skip_rare = 0;
unsigned num_devirt_skip_not_translated = 0;
unsigned num_devirt_skip_far_targ = 0;
unsigned num_devirt_skip_other = 0;

// Step 0 of create_tc2(): choose the sites to de-virtualize.
// Must run BEFORE Step 1, while instr_map[].orig_ins_addr still holds the
// original addresses.
//   orig_to_tc : original instr address -> its address in TC
//   sites      : instr_map index of the indirect jmp/call -> hot target (orig addr)
static void find_devirt_sites(const std::map<ADDRINT, ADDRINT> &orig_to_tc,
                              std::map<unsigned, ADDRINT> &sites)
{
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].indirect_kind == NoIndirect || !bbl_map[b].counter)
        continue;

      // The hottest target of this site.
      unsigned best = 0;
      for (unsigned j = 1; j <= MAX_TARG_ADDRS; j++)
        if (bbl_map[b].targ_count[j] > bbl_map[b].targ_count[best])
          best = j;
      ADDRINT hot_targ = bbl_map[b].targ_addr[best];
      UINT64 hot_count = bbl_map[b].targ_count[best];
      UINT64 exec_count = bbl_map[b].counter;

      // The threshold: the hot target must be FREQUENT.
      if (hot_count * 100 < exec_count * KnobDevirtPercent.Value()) {
        num_devirt_skip_rare++;
        continue;
      }
      if (!orig_to_tc.count(hot_targ)) {
        num_devirt_skip_not_translated++;
        continue;
      }
      if (hot_targ > 0x7FFFFFFF) {
        num_devirt_skip_far_targ++;
        continue;
      }
      // The indirect jmp/call is the instr that terminates this BBL.
      unsigned site = bbl_map[b].terminating_ins_entry;
      if (site + 1 >= num_of_instr_map_entries ||
          instr_map[site].orig_ins_addr != bbl_map[b].indirect_site_addr) {
        num_devirt_skip_other++;
        continue;
      }
      sites[site] = hot_targ;

      if (KnobDumpProfile) {
        cerr << "devirt " << (bbl_map[b].indirect_kind == IndirectCall ? "call" : "jump")
             << " at 0x" << hex << bbl_map[b].indirect_site_addr
             << " -> 0x" << hot_targ << " (" << dec << hot_count << " of "
             << exec_count << " executions)" << endl;
      }
    }
}

// Emit the de-virtualized code for the indirect jmp/call 'site' (see the
// table above) at the end of the NEW instr_map.
// 'hot_targ_tc' is the TC address of the hot target: chaining turns it into
// the TC2 address later. The two branches inside a call sequence (jne miss,
// jmp done) are returned in 'internal_branches' as (branch entry, target
// entry) pairs, to be set after chaining.
// Returns 1 if emitted, 0 if the form is not supported (the caller then
// copies the site unchanged), -1 on an encoding error.
static int emit_devirt_site(const instr_map_t *site, ADDRINT hot_targ, ADDRINT hot_targ_tc,
                            std::vector<std::pair<unsigned, unsigned> > &internal_branches)
{
    xed_decoded_inst_t xedd;
    xed_decoded_inst_zero_set_mode(&xedd, &dstate);
    if (xed_decode(&xedd, reinterpret_cast<const UINT8*>(site->encoded_ins), max_inst_len) != XED_ERROR_NONE)
      return 0;
    indirect_target_t t;
    if (!get_indirect_target_operand(&xedd, site->orig_ins_addr, &t))
      return 0;
    bool is_call = (xed_decoded_inst_get_category(&xedd) == XED_CATEGORY_CALL);

    // All new instrs get the address of the site as their orig_ins_addr
    // (like the profiling stubs did in TC), so a branch that targeted the
    // site now lands on the 'cmp'.
    ADDRINT key = site->orig_ins_addr;
    xed_encoder_instruction_t enc_instr;

    // 1. cmp X, T   (T as a sign-extended 32-bit immediate)
    if (t.targ_reg != XED_REG_INVALID) {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_CMP, 64,
                xed_reg(t.targ_reg), xed_simm0((xed_int32_t)hot_targ, 32));
    } else if (t.base_reg == XED_REG_RIP) {
      // The displacement is set by fix_rip_displacement() from orig_rip_addr.
      xed_inst2(&enc_instr, dstate, XED_ICLASS_CMP, 64,
                xed_mem_bd(XED_REG_RIP, xed_disp(0, 32), 64),
                xed_simm0((xed_int32_t)hot_targ, 32));
    } else {
      xed_inst2(&enc_instr, dstate, XED_ICLASS_CMP, 64,
                xed_mem_bisd(t.base_reg, t.index_reg, t.scale,
                             xed_disp(t.disp, t.disp_width), 64),
                xed_simm0((xed_int32_t)hot_targ, 32));
    }
    if (add_new_encoded_instr(key, &enc_instr, RegularIns) < 0)
      return -1;
    if (t.base_reg == XED_REG_RIP)
      instr_map[num_of_instr_map_entries - 1].orig_rip_addr = site->orig_rip_addr;

    if (is_call) {
      // 2. jne miss
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JNZ, 64, xed_relbr(0, 32));
      if (add_new_encoded_instr(key, &enc_instr, RegularIns) < 0)
        return -1;
      unsigned jne_entry = num_of_instr_map_entries - 1;

      // 3. call T_in_TC2  (direct)
      xed_inst1(&enc_instr, dstate, XED_ICLASS_CALL_NEAR, 64, xed_relbr(0, 32));
      if (add_new_encoded_instr(key, &enc_instr, RegularIns) < 0)
        return -1;
      instr_map[num_of_instr_map_entries - 1].orig_targ_addr = hot_targ_tc;

      // 4. jmp done
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JMP, 64, xed_relbr(0, 32));
      if (add_new_encoded_instr(key, &enc_instr, RegularIns) < 0)
        return -1;
      unsigned jmp_entry = num_of_instr_map_entries - 1;

      // 5. miss: the original 'call X'
      unsigned miss_entry = num_of_instr_map_entries;
      instr_map[num_of_instr_map_entries] = *site;
      instr_map[num_of_instr_map_entries].targ_map_entry = -1;
      num_of_instr_map_entries++;

      // 'done' is the next entry, i.e. the instr after the original call.
      internal_branches.push_back(std::make_pair(jne_entry, miss_entry));
      internal_branches.push_back(std::make_pair(jmp_entry, miss_entry + 1));
      num_devirt_calls++;
    } else {
      // 2. je T_in_TC2  (direct)
      xed_inst1(&enc_instr, dstate, XED_ICLASS_JZ, 64, xed_relbr(0, 32));
      if (add_new_encoded_instr(key, &enc_instr, RegularIns) < 0)
        return -1;
      instr_map[num_of_instr_map_entries - 1].orig_targ_addr = hot_targ_tc;

      // 3. the original 'jmp X'
      instr_map[num_of_instr_map_entries] = *site;
      instr_map[num_of_instr_map_entries].targ_map_entry = -1;
      num_of_instr_map_entries++;
      num_devirt_jumps++;
    }
    if (num_of_instr_map_entries >= max_ins_count)
      return -1;
    return 1;
}

// Step 1b of create_tc2(): build a new instr_map in which every chosen
// site is replaced by its de-virtualized code (the other entries are
// copied as they are). Runs after Step 1, so all orig_ins_addr/orig_targ_addr
// fields already hold TC addresses and entry indices may change.
static int insert_devirt_sites(const std::map<unsigned, ADDRINT> &sites,
                               const std::map<ADDRINT, ADDRINT> &orig_to_tc,
                               std::vector<std::pair<unsigned, unsigned> > &internal_branches)
{
    if (sites.empty())
      return 0;

    instr_map_t *old_map = instr_map;
    unsigned old_num = num_of_instr_map_entries;
    unsigned extra = 8 * sites.size() + 16;   // at most 4 new entries per site

    instr_map_t *new_map = (instr_map_t *)calloc(max_ins_count + extra, sizeof(instr_map_t));
    if (new_map == NULL) {
      perror("calloc");
      return -1;
    }
    instr_map = new_map;
    max_ins_count += extra;
    num_of_instr_map_entries = 0;

    std::vector<unsigned> new_index(old_num + 1);
    for (unsigned i = 0; i < old_num; i++) {
      new_index[i] = num_of_instr_map_entries;
      std::map<unsigned, ADDRINT>::const_iterator it = sites.find(i);
      if (it != sites.end()) {
        int rc = emit_devirt_site(&old_map[i], it->second, orig_to_tc.at(it->second),
                                  internal_branches);
        if (rc < 0)
          return -1;
        if (rc > 0) {
          // The new entries belong to the BBL of the site.
          for (unsigned k = new_index[i]; k < num_of_instr_map_entries; k++)
            instr_map[k].bbl_num = old_map[i].bbl_num;
          continue;
        }
        num_devirt_skip_other++;             // unsupported form: copy it
      }
      instr_map[num_of_instr_map_entries++] = old_map[i];
    }
    new_index[old_num] = num_of_instr_map_entries;

    // Keep the BBL -> instr_map indices valid.
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].starting_ins_entry <= old_num)
        bbl_map[b].starting_ins_entry = new_index[bbl_map[b].starting_ins_entry];
      if (bbl_map[b].terminating_ins_entry <= old_num)
        bbl_map[b].terminating_ins_entry = new_index[bbl_map[b].terminating_ins_entry];
    }

    free(old_map);
    return 0;
}

/* ============================================================= */
/* Code reordering (TC2)                                         */
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
// 'internal_branches' (entry index pairs of de-virtualized calls) are
// remapped to the new entry indices.
static int reorder_bbls(std::vector<std::pair<unsigned, unsigned> > &internal_branches)
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
      bool can_reorder = !KnobNoReorder && (re - rb) >= 3;
      bool has_cold = false, has_hot = false;
      for (unsigned b = rb + 1; b < re && can_reorder; b++) {
        if (bbls[b].count) has_hot = true; else has_cold = true;
      }
      for (unsigned i = bbls[rb].first; i <= bbls[re - 1].last && can_reorder; i++) {
        if (!old_map[i].size || old_map[i].xed_category != XED_CATEGORY_COND_BR)
          continue;
        if (reverse_cond_iclass(entry_iclass(&old_map[i])) == XED_ICLASS_INVALID)
          can_reorder = false;                    // LOOP / JRCXZ (or unknown)
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
    for (unsigned k = 0; k < internal_branches.size(); k++) {
      internal_branches[k].first = new_index[internal_branches[k].first];
      internal_branches[k].second = new_index[internal_branches[k].second];
    }
    for (unsigned b = 0; b < bbl_num; b++) {
      if (bbl_map[b].starting_ins_entry <= old_num)
        bbl_map[b].starting_ins_entry = new_index[bbl_map[b].starting_ins_entry];
      if (bbl_map[b].terminating_ins_entry <= old_num)
        bbl_map[b].terminating_ins_entry = new_index[bbl_map[b].terminating_ins_entry];
    }
    free(old_map);
    return 0;
}

/****************/
/* create_tc2() */
/****************/
// Builds TC2 from the instructions of TC and redirects each TC routine head
// to its TC2 copy. Called by create_tc2_thread_func() once profiling stopped.
// Returns 0 on success, -1 on failure (execution then simply stays in TC).
int create_tc2()
{
    int rc = 0;

    // Step 0: De-virtualization - choose the indirect jumps/calls whose
    //         profiled hot target is frequent (see find_devirt_sites()).
    //         Must run before Step 1 overwrites the original addresses.
    std::map<ADDRINT, ADDRINT> orig_to_tc;        // orig instr addr -> TC addr
    std::map<unsigned, ADDRINT> devirt_sites;     // instr_map entry -> hot target
    if (!KnobNoDevirt) {
      for (unsigned i = 0; i < num_of_instr_map_entries; i++)
        orig_to_tc.emplace(instr_map[i].orig_ins_addr, instr_map[i].new_ins_addr); // first wins
      find_devirt_sites(orig_to_tc, devirt_sites);
    }

	// Step 1: Modify instr_map to be used for TC2.
    //
    unsigned num_removed_prof_instrs = 0;
    for (unsigned i = 0; i < num_of_instr_map_entries; i++) {    
       // Set new_ins_addr to be the orig_ins_addr.
       instr_map[i].orig_ins_addr = instr_map[i].new_ins_addr;

       // Skip the wide NOP instr at the Rtn head which was reserved
       // for the probing jump from TC to TC2.
       if (instr_map[i].ins_type == RtnHeadIns &&
           instr_map[i].xed_category == XED_CATEGORY_WIDENOP)
         instr_map[i].size = 0;
       
       // Remove unused NOPs.
       if (instr_map[i].xed_category == XED_CATEGORY_WIDENOP ||
           instr_map[i].xed_category == XED_CATEGORY_NOP)
         instr_map[i].size = 0;

       // Remove the profiling stubs: the counters are only needed in TC.
       // (The original code only removed the NOP at the head of each stub,
       // so TC2 kept executing all the counting instructions.)
       // An instr of size 0 is not written into TC2, and a branch whose
       // target is a removed instr lands on the next instr that is kept.
       // For a removed stub this is the original instr the stub was put
       // in front of, so control flow does not change.
       if (instr_map[i].ins_type == ProfilingIns) {
         if (instr_map[i].size)
           num_removed_prof_instrs++;
         instr_map[i].size = 0;
       }

       // Fix orig_targ_addr by new_ins_addr and targ_map_entry.
       if (instr_map[i].targ_map_entry >= 0) {
         ADDRINT new_targ_addr = instr_map[instr_map[i].targ_map_entry].new_ins_addr;
         instr_map[i].orig_targ_addr = new_targ_addr;
       }
    }
    
    for (unsigned i = 0; i < num_of_instr_map_entries; i++) {
       instr_map[i].targ_map_entry = -1;
    }
    
    cerr << "after modifying instr_map (removed " << dec << num_removed_prof_instrs
         << " profiling instrs)" << endl;

    // Step 1b: De-virtualization - replace each chosen site by
    //          'cmp X, T / direct jmp/call to T / original jmp/call X'.
    std::vector<std::pair<unsigned, unsigned> > devirt_internal_branches;
    rc = insert_devirt_sites(devirt_sites, orig_to_tc, devirt_internal_branches);
    if (rc < 0) {
        cerr << "failed to insert de-virtualized code\n";
        return -1;
    }
    cerr << "de-virtualization (threshold " << dec << KnobDevirtPercent.Value() << "%): "
         << num_devirt_calls << " calls, " << num_devirt_jumps << " jumps"
         << " | not done: " << num_devirt_skip_rare << " no frequent target, "
         << num_devirt_skip_not_translated << " target not translated, "
         << num_devirt_skip_far_targ << " target above 2GB, "
         << num_devirt_skip_other << " other" << endl;

    // Step 1c: Code reordering - move the cold BBLs of every routine to its
    //          end, reversing cond branches / adding jumps where needed.
    rc = reorder_bbls(devirt_internal_branches);
    if (rc < 0) {
        cerr << "failed to reorder the code for TC2\n";
        return -1;
    }
    cerr << "code reordering: " << dec << num_reordered_rtns << " routines, "
         << num_moved_cold_bbls << " cold BBLs moved to routine end, "
         << num_reversed_cond_branches << " cond branches reversed, "
         << num_added_fallthru_jumps << " jumps added" << endl;
    
    // Step 3: Chaining - calculate direct branch and call instructions to point
    //         to corresponding target instr entries:
    //
    chain_all_direct_jmp_and_call_target_entries(0, num_of_instr_map_entries);
    // The 'jne miss' / 'jmp done' inside de-virtualized calls point to entries
    // of the same sequence, not to an address, so they are set here.
    for (unsigned k = 0; k < devirt_internal_branches.size(); k++)
      instr_map[devirt_internal_branches[k].first].targ_map_entry =
          devirt_internal_branches[k].second;
    cerr << "after chaining all branch targets" << endl;

    // Step 4: Set initial estimated new addrs for each instruction in tc2.
    //
    rc = set_initial_estimated_new_ins_addrs_in_tc(tc2);
    if (rc < 0) {
        cerr << "failed to set initial estimated new ins addrs in TC2\n";
        return -1;
    }
    cerr << "after setting initial estimated new ins addrs in tc2" << endl;

    // Step 5: fix rip-based, direct branch and direct call displacements:
    //
    rc = fix_instructions_displacements();
    if (rc < 0 ) {
        cerr << "failed to fix displacments of translated instructions\n";
        return -1;
    }
    cerr << "after fixing instructions displacements" << endl;

    // Step 6: write translated instructions to tc2:
    //
    rc = copy_instrs_to_tc(tc2);
    if (rc < 0 ) {
        cerr << "failed to copy the instructions to the translation cache\n";
        return -1;
    }
    tc2_size = rc;
    cerr << "after write all new instructions to tc2 (TC size: " << dec << tc_size
         << " bytes, TC2 size: " << tc2_size << " bytes)" << endl;

    // Step 7: Commit the translated routines:
    //         Go over the candidate functions and replace the original ones
    //         by their new successfully translated ones:
    if (!KnobDoNotCommitTranslatedCode) {
      rc = commit_translated_rtns_to_tc2();
      if (rc < 0 ) {
          cerr << "failed to commit jump instructions from TC to TC2\n";
          return -1;
      }
      cerr << "after commit of translated routines from TC to TC2" << endl;
    }

    if (KnobDumpTranslatedCode2) {
        cerr << "Translation Cache 2 dump:" << endl;
        dump_tc(tc2, tc2_size);
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
    cerr << " prof time: " << dec << KnobNumSecsDuringProfile << " sec\n";
    sleep(KnobNumSecsDuringProfile);

    cerr << "disabling profile gathering\n";

    // disable profiling.
    //  Add a jump at beginning of every profile stub to bypass the
	//  profiling counters in TC.
    // (Still needed with TC2: a thread that is inside a TC routine when
    //  we switch, e.g. main()'s loop, keeps running that TC code.)
    int rc = disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
    if  (rc < 0)
      return;

    // Print the indirect jump/call target profile (debug, -dump_prof).
    if (KnobDumpProfile)
      dump_indirect_profile();

    create_tc2();
 
    clock_gettime(CLOCK_MONOTONIC, &start_running_time);

    PIN_ExitThread(0);
}

/****************************/
/* allocate_and_init_memory */
/****************************/
int allocate_and_init_memory(IMG img)
{
    // Calculate size of executable sections and allocate required memory:
    //
    ADDRINT highest_addr = 0;
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
            continue;

        if (!lowest_sec_addr || lowest_sec_addr > SEC_Address(sec))
            lowest_sec_addr = SEC_Address(sec);

        if (highest_sec_addr < SEC_Address(sec) + SEC_Size(sec))
            highest_sec_addr = SEC_Address(sec) + SEC_Size(sec);

        // need to avouid using RTN_Open as it is expensive...
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {
            if (highest_addr < RTN_Address(rtn) + RTN_Size(rtn))
                highest_addr = RTN_Address(rtn) + RTN_Size(rtn);
            max_rtn_count++;
            max_ins_count += RTN_NumIns  (rtn);
        }
    }

    max_ins_count *= 10; // estimating that the num of instrs for the profiling
                         // and for the inlined functions will not exceed
                         // the total nunmber of the entire code.


    // get a page size in the system:
    int pagesize = sysconf(_SC_PAGE_SIZE);
    if (pagesize == -1) {
      perror("sysconf");
      return -1;
    }

    ADDRINT text_size = (highest_sec_addr - lowest_sec_addr) * 2 + pagesize * 4;

    max_tc_size = 10 * text_size + pagesize * 4;   // FIXME: need a better estimate
    // Check thet max_tc_size is not larger than a 32 bit branch displacement
    if (max_tc_size >= 0x7FFFFFFF) {
      cerr << "size of TC is beyond the range of a branch displacement" << endl;
      return -1;
    }

    // Allocate the needed memory for tc and tc2 + jump orig addr map
    // with RW+EXEC permissions which is not
    // located in an address that is more than 32bits afar:
    const size_t mem_size =
              max_tc_size +                     // TC + TC2 size
              max_rtn_count * sizeof(ADDRINT);  // jump_to_orig_addr_map size
    char *addr = nullptr;
    ADDRINT max_distance = 0x7FFFFFFF;
    const size_t step = pagesize; // Try every page
    // Align target address to page boundary
    ADDRINT aligned_target = ((ADDRINT)highest_addr) & ~(pagesize - 1);
    // Try exact address first
    void* result = mmap((void*)aligned_target, mem_size,
                       PROT_READ | PROT_WRITE | PROT_EXEC,
                       MAP_PRIVATE | MAP_ANONYMOUS,
                       0, 0);
    if (result != MAP_FAILED &&
        (abs((long)((ADDRINT)result - aligned_target)) <= (long)max_distance)) {
        addr = (char *)result;
    }

    if (!addr) {
        // Search in expanding rings around target
        for (size_t offset = step; offset <= max_distance; offset += step) {
            // Try above target address
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

            // Try below target address (if not underflow)
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

    // debug print.
    cerr << " allocated memory at: 0x" << hex << (ADDRINT)addr << "\n";

    // TC is allocated first.
    tc = (char *)addr;
    addr += max_tc_size/2;

    // TC2 is allocated immeditely after TC:
    tc2 = (char *)addr;
    addr += max_tc_size/2;

    // Allocate memory to the jump map to orig addrs which cannot be relocated.
    jump_to_orig_addr_map = (ADDRINT *)addr;

    // Allocate memory for the instr_map table.
    instr_map = (instr_map_t *)calloc(max_ins_count, sizeof(instr_map_t));
    if (instr_map == NULL) {
        perror("calloc");
        return -1;
    }

    // Allocate memory for the bbl_map table.
    bbl_map = (bbl_map_t *)calloc(max_ins_count, sizeof(bbl_map_t));
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

    clock_gettime(CLOCK_MONOTONIC, &end_running_time);

    dump_profile();

    double elapsed = (end_running_time.tv_sec - start_running_time.tv_sec) + 
                     (end_running_time.tv_nsec - start_running_time.tv_nsec) / 1e9;
	cerr << " Translated code run (including profiling) took: "
	     << elapsed + KnobNumSecsDuringProfile << " seconds\n";
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
    cerr << " indirect target profiling: " << dec << num_prof_indirect_jump_sites
         << " jump sites, " << num_prof_indirect_call_sites << " call sites"
         << " (" << num_unsupported_indirect_sites << " unsupported sites not profiled)"
         << endl;

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
    out = new std::ofstream("bprofile.out");

    // Initialize pin & symbol manager
    if( PIN_Init(argc,argv) )
        return Usage();

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

