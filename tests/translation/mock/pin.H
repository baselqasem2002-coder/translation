// MOCK pin.H for testing project.cpp's code generation WITHOUT the Pin kit.
// INS_* are implemented with real XED on real machine code in memory, and
// IMG/SEC/RTN iterate over the routines a test registers in g_mock_rtns.
// RTN_ReplaceProbed only records the replacement. Anything else aborts.
#pragma once
#include <string>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <map>
extern "C" {
#include "xed-interface.h"
}
typedef void VOID; typedef bool BOOL; typedef int INT; typedef int32_t INT32;
typedef unsigned int UINT; typedef uint8_t UINT8; typedef uint32_t UINT32; typedef uint64_t UINT64;
typedef uintptr_t ADDRINT; typedef size_t USIZE; typedef UINT32 THREADID; typedef UINT64 PIN_THREAD_UID;
typedef void (*AFUNPTR)(); typedef VOID ROOT_THREAD_FUNC(VOID *);
static const THREADID INVALID_THREADID = (THREADID)-1;

// A routine registered by the test: [addr, addr+size) holds real machine code.
struct mock_rtn_t { ADDRINT addr; USIZE size; std::string name; };
std::vector<mock_rtn_t> g_mock_rtns;                 // filled by the test
std::map<ADDRINT, ADDRINT> g_mock_replaced;          // RTN_ReplaceProbed: orig -> new

// Registers (Pin's REG) are XED registers in the mock.
namespace LEVEL_BASE {
  typedef xed_reg_enum_t REG;
  static const REG REG_RAX = XED_REG_RAX, REG_RBX = XED_REG_RBX, REG_RCX = XED_REG_RCX;
}
using namespace LEVEL_BASE;

struct mock_ins_t {
  xed_decoded_inst_t xedd; ADDRINT addr; ADDRINT end;
  bool bad;                           // bytes XED cannot decode (length 1, like a data byte)
  std::vector<REG> rregs, wregs;      // registers read / written (explicit + implicit)
};
typedef mock_ins_t *INS;
struct IMG { int x; };
struct SEC { int idx; };
struct RTN { mock_rtn_t *r; };
static inline bool operator==(RTN a, RTN b) { return a.r == b.r; }
static inline bool operator!=(RTN a, RTN b) { return a.r != b.r; }

#define MOCK_ABORT { abort(); }
enum KNOB_MODE { KNOB_MODE_WRITEONCE };
template <class T> class KNOB {
public:
  T v;
  KNOB(KNOB_MODE, const std::string &, const std::string &, const std::string &def, const std::string &)
    : v((T)strtoul(def.c_str(), 0, 0)) {}
  T Value() const { return v; }
  operator T() const { return v; }
};
struct KNOB_BASE { static std::string StringKnobSummary() MOCK_ABORT };

// --- INS: decoded from real memory with XED ---
static inline void mock_add_reg(std::vector<REG> &v, REG r) {
  if (r != XED_REG_INVALID) v.push_back(r);
}
static inline INS mock_decode_ins(ADDRINT addr, ADDRINT end) {
  if (addr >= end) return 0;
  mock_ins_t *ins = new mock_ins_t;                  // leaked on purpose (tests only)
  xed_state_t st = {XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b};
  xed_decoded_inst_zero_set_mode(&ins->xedd, &st);
  ins->addr = addr; ins->end = end;
  ins->bad = (xed_decode(&ins->xedd, (const UINT8 *)addr, XED_MAX_INSTRUCTION_BYTES) != XED_ERROR_NONE);
  if (ins->bad) return ins;
  // Registers read/written, as Pin's INS_RegR/INS_RegW report them: all
  // register operands (explicit and implicit) plus the base/index registers
  // of memory and address (lea) operands, which are read.
  const xed_inst_t *xi = xed_decoded_inst_inst(&ins->xedd);
  unsigned memop = 0;
  for (unsigned i = 0; i < xed_inst_noperands(xi); i++) {
    const xed_operand_t *op = xed_inst_operand(xi, i);
    xed_operand_enum_t name = xed_operand_name(op);
    if (xed_operand_is_register(name)) {
      REG r = xed_decoded_inst_get_reg(&ins->xedd, name);
      if (xed_operand_read(op)) mock_add_reg(ins->rregs, r);
      if (xed_operand_written(op)) mock_add_reg(ins->wregs, r);
    } else if (name == XED_OPERAND_MEM0 || name == XED_OPERAND_MEM1 || name == XED_OPERAND_AGEN) {
      unsigned m = (name == XED_OPERAND_MEM1) ? 1 : (name == XED_OPERAND_AGEN ? 0 : memop);
      mock_add_reg(ins->rregs, xed_decoded_inst_get_base_reg(&ins->xedd, m));
      mock_add_reg(ins->rregs, xed_decoded_inst_get_index_reg(&ins->xedd, m));
      if (name == XED_OPERAND_MEM0) memop = 1;
    }
  }
  return ins;
}
static inline xed_decoded_inst_t *INS_XedDec(INS ins) { return &ins->xedd; }
static inline ADDRINT INS_Address(INS ins) { return ins->addr; }
static inline USIZE INS_Size(INS ins) { return ins->bad ? 1 : xed_decoded_inst_get_length(&ins->xedd); }
static inline BOOL INS_Valid(INS ins) { return ins != 0; }
static inline INS INS_Next(INS ins) { return mock_decode_ins(ins->addr + INS_Size(ins), ins->end); }
static inline xed_category_enum_t INS_Category(INS ins) { return xed_decoded_inst_get_category(&ins->xedd); }
static inline BOOL INS_IsRet(INS ins) { return INS_Category(ins) == XED_CATEGORY_RET; }
static inline BOOL INS_IsCall(INS ins) { return INS_Category(ins) == XED_CATEGORY_CALL; }
static inline BOOL INS_IsDirectControlFlow(INS ins) {
  return xed_decoded_inst_get_branch_displacement_width(&ins->xedd) > 0; }
static inline BOOL INS_IsIndirectControlFlow(INS ins) {
  xed_category_enum_t c = INS_Category(ins);
  if (c == XED_CATEGORY_RET) return true;
  return (c == XED_CATEGORY_CALL || c == XED_CATEGORY_UNCOND_BR) && !INS_IsDirectControlFlow(ins); }
static inline ADDRINT INS_DirectControlFlowTargetAddress(INS ins) {
  return ins->addr + INS_Size(ins) + xed_decoded_inst_get_branch_displacement(&ins->xedd); }
static inline UINT32 INS_MaxNumRRegs(INS ins) { return ins->rregs.size(); }
static inline UINT32 INS_MaxNumWRegs(INS ins) { return ins->wregs.size(); }
static inline REG INS_RegR(INS ins, UINT32 i) { return ins->rregs[i]; }
static inline REG INS_RegW(INS ins, UINT32 i) { return ins->wregs[i]; }
static inline REG REG_FullRegName(REG r) { return xed_get_largest_enclosing_register(r); }
static inline UINT32 REG_Size(REG r) { return xed_get_register_width_bits64(r) / 8; }
static inline BOOL INS_IsSyscall(INS ins) { return INS_Category(ins) == XED_CATEGORY_SYSCALL; }
// Pin: CMOVcc and REP-prefixed string instrs are predicated.
static inline BOOL INS_IsPredicated(INS ins) {
  return INS_Category(ins) == XED_CATEGORY_CMOV ||
         xed_operand_values_has_real_rep(xed_decoded_inst_operands_const(&ins->xedd)); }
static inline std::string INS_Disassemble(INS ins) {
  char buf[256]; xed_format_context(XED_SYNTAX_INTEL, &ins->xedd, buf, sizeof buf, ins->addr, 0, 0);
  return buf; }

// --- IMG/SEC/RTN: one executable section containing g_mock_rtns ---
static inline BOOL IMG_IsMainExecutable(IMG) { return true; }
static inline SEC IMG_SecHead(IMG) { SEC s = {0}; return s; }
static inline BOOL SEC_Valid(SEC s) { return s.idx == 0 && !g_mock_rtns.empty(); }
static inline SEC SEC_Next(SEC s) { s.idx++; return s; }
static inline BOOL SEC_IsExecutable(SEC) { return true; }
static inline BOOL SEC_IsWriteable(SEC) { return false; }
static inline ADDRINT SEC_Address(SEC) { return g_mock_rtns.front().addr; }
static inline USIZE SEC_Size(SEC) {
  return g_mock_rtns.back().addr + g_mock_rtns.back().size - g_mock_rtns.front().addr; }
static inline RTN SEC_RtnHead(SEC) { RTN r = {&g_mock_rtns[0]}; return r; }
static inline BOOL RTN_Valid(RTN r) { return r.r != 0; }
static inline RTN RTN_Invalid() { RTN r = {0}; return r; }
static inline RTN RTN_Next(RTN r) {
  RTN n = {r.r + 1 < &g_mock_rtns[0] + g_mock_rtns.size() ? r.r + 1 : 0}; return n; }
static inline VOID RTN_Open(RTN) {}
static inline VOID RTN_Close(RTN) {}
static inline INS RTN_InsHead(RTN r) { return mock_decode_ins(r.r->addr, r.r->addr + r.r->size); }
static inline ADDRINT RTN_Address(RTN r) { return r.r->addr; }
static inline USIZE RTN_Size(RTN r) { return r.r->size; }
static inline UINT32 RTN_NumIns(RTN r) {
  UINT32 n = 0; for (INS i = RTN_InsHead(r); INS_Valid(i); i = INS_Next(i)) n++; return n; }
static inline const std::string &RTN_Name(RTN r) { return r.r->name; }
static inline RTN RTN_FindByAddress(ADDRINT a) {
  for (size_t i = 0; i < g_mock_rtns.size(); i++)
    if (a >= g_mock_rtns[i].addr && a < g_mock_rtns[i].addr + g_mock_rtns[i].size) { RTN r = {&g_mock_rtns[i]}; return r; }
  return RTN_Invalid(); }
static inline RTN RTN_FindByName(IMG, const char *) { return RTN_Invalid(); }
static inline BOOL RTN_IsSafeForProbedReplacement(RTN) { return true; }
// Records the replacement instead of writing a probe; the test calls TC directly.
static inline AFUNPTR RTN_ReplaceProbed(RTN r, AFUNPTR f) {
  g_mock_replaced[r.r->addr] = (ADDRINT)f; return (AFUNPTR)r.r->addr; }
static inline VOID PIN_LockClient() {}
static inline VOID PIN_UnlockClient() {}

// --- not used by the tests: abort ---
static inline BOOL PIN_Init(INT32, char **) MOCK_ABORT
static inline VOID PIN_InitSymbols() MOCK_ABORT
static inline VOID PIN_StartProgramProbed() MOCK_ABORT
static inline VOID PIN_ExitThread(INT32) MOCK_ABORT
static inline THREADID PIN_SpawnInternalThread(ROOT_THREAD_FUNC *, VOID *, size_t, PIN_THREAD_UID *) MOCK_ABORT
typedef VOID (*IMAGECALLBACK)(IMG, VOID *);
static inline VOID IMG_AddInstrumentFunction(IMAGECALLBACK, VOID *) MOCK_ABORT
