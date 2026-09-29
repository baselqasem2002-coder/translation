// Minimal stand-in for the subset of the Pin probe-mode API used by the
// project pintool.  Backed by XED + the ELF symbol table of /proc/self/exe.
// Test harness only - NOT part of the submission.
#pragma once
extern "C" {
#include "xed-interface.h"
}
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdint.h>

typedef uint64_t ADDRINT;
typedef uint64_t UINT64;
typedef int64_t  INT64;
typedef uint32_t UINT32;
typedef int32_t  INT32;
typedef unsigned UINT;
typedef int      INT;
typedef uint8_t  UINT8;
typedef size_t   USIZE;
typedef bool     BOOL;
typedef void     VOID;
typedef void (*AFUNPTR)();
typedef uint32_t THREADID;
typedef uint32_t OPCODE;
#define INVALID_THREADID ((THREADID)-1)
#define PIN_FAST_ANALYSIS_CALL

typedef xed_reg_enum_t REG;
namespace LEVEL_BASE {
  static const REG REG_RAX = XED_REG_RAX, REG_RBX = XED_REG_RBX, REG_RCX = XED_REG_RCX,
                   REG_RDX = XED_REG_RDX, REG_RSI = XED_REG_RSI, REG_RDI = XED_REG_RDI,
                   REG_RBP = XED_REG_RBP, REG_RSP = XED_REG_RSP, REG_R8 = XED_REG_R8,
                   REG_R9 = XED_REG_R9, REG_R10 = XED_REG_R10, REG_R11 = XED_REG_R11,
                   REG_R12 = XED_REG_R12, REG_R13 = XED_REG_R13, REG_R14 = XED_REG_R14,
                   REG_R15 = XED_REG_R15, REG_INVALID_ = XED_REG_INVALID,
                   REG_RIP = XED_REG_RIP;
}
using namespace LEVEL_BASE;
#define REG_INVALID() XED_REG_INVALID
inline bool REG_valid(REG r) { return r != XED_REG_INVALID; }
inline REG REG_FullRegName(REG r) { return xed_get_largest_enclosing_register(r); }
inline bool REG_is_gr64(REG r) { return xed_reg_class(r) == XED_REG_CLASS_GPR && xed_get_register_width_bits64(r) == 64; }

struct fp_ins;  struct fp_rtn;  struct fp_sec;  struct fp_img;
typedef fp_ins* INS;
typedef fp_rtn* RTN;
typedef fp_sec* SEC;
typedef fp_img* IMG;

// ---- knobs ----
enum KNOB_MODE { KNOB_MODE_WRITEONCE, KNOB_MODE_OVERWRITE };
class KNOB_BASE {
 public:
  static std::string StringKnobSummary() { return ""; }
  static std::vector<KNOB_BASE*> &all() { static std::vector<KNOB_BASE*> v; return v; }
  std::string name;
  virtual void set(const std::string &s) = 0;
  virtual ~KNOB_BASE() {}
};
template <class T> class KNOB : public KNOB_BASE {
 public:
  T val;
  KNOB(KNOB_MODE, const char *, const char *n, const char *def, const char *) {
    name = n; set(def); all().push_back(this);
  }
  void set(const std::string &s) { std::istringstream is(s); is >> val; }
  operator T() const { return val; }
  T Value() const { return val; }
};
template <> inline void KNOB<std::string>::set(const std::string &s) { val = s; }

// ---- images / sections / routines / instructions ----
IMG  IMG_Invalid();
bool IMG_Valid(IMG);
bool IMG_IsMainExecutable(IMG);
SEC  IMG_SecHead(IMG);
std::string IMG_Name(IMG);
bool SEC_Valid(SEC);
SEC  SEC_Next(SEC);
bool SEC_IsExecutable(SEC);
bool SEC_IsWriteable(SEC);
ADDRINT SEC_Address(SEC);
USIZE SEC_Size(SEC);
RTN  SEC_RtnHead(SEC);
std::string SEC_Name(SEC);

RTN  RTN_Invalid();
bool RTN_Valid(RTN);
RTN  RTN_Next(RTN);
const std::string &RTN_Name(RTN);
ADDRINT RTN_Address(RTN);
USIZE RTN_Size(RTN);
UINT32 RTN_NumIns(RTN);
void RTN_Open(RTN);
void RTN_Close(RTN);
INS  RTN_InsHead(RTN);
bool RTN_IsSafeForProbedReplacement(RTN);
RTN  RTN_FindByAddress(ADDRINT);
RTN  RTN_FindByName(IMG, const char *);
AFUNPTR RTN_ReplaceProbed(RTN, AFUNPTR);
SEC  RTN_Sec(RTN);

bool INS_Valid(INS);
INS  INS_Next(INS);
INS  INS_Prev(INS);
ADDRINT INS_Address(INS);
USIZE INS_Size(INS);
xed_decoded_inst_t *INS_XedDec(INS);
bool INS_IsCall(INS);
bool INS_IsRet(INS);
bool INS_IsIndirectControlFlow(INS);
bool INS_IsDirectControlFlow(INS);
bool INS_IsControlFlow(INS);
bool INS_IsBranch(INS);
ADDRINT INS_DirectControlFlowTargetAddress(INS);
OPCODE INS_Opcode(INS);
bool INS_IsPredicated(INS);
bool INS_IsSyscall(INS);
bool INS_IsInterrupt(INS);
xed_category_enum_t INS_Category(INS);
std::string INS_Disassemble(INS);
UINT32 INS_MaxNumRRegs(INS);
UINT32 INS_MaxNumWRegs(INS);
REG INS_RegR(INS, UINT32);
REG INS_RegW(INS, UINT32);
UINT32 INS_OperandCount(INS);
bool INS_OperandIsMemory(INS, UINT32);
bool INS_OperandIsReg(INS, UINT32);
bool INS_OperandIsImmediate(INS, UINT32);
REG INS_OperandReg(INS, UINT32);
REG INS_OperandMemoryBaseReg(INS, UINT32);
REG INS_OperandMemoryIndexReg(INS, UINT32);
INT64 INS_OperandMemoryDisplacement(INS, UINT32);
bool INS_OperandRead(INS, UINT32);
bool INS_OperandWritten(INS, UINT32);
UINT32 INS_OperandWidth(INS, UINT32);
bool INS_IsMov(INS);
bool INS_IsLea(INS);
bool INS_IsMemoryRead(INS);
bool INS_IsMemoryWrite(INS);
bool INS_IsStackRead(INS);
bool INS_IsStackWrite(INS);

typedef void (*IMAGECALLBACK)(IMG, void *);
void IMG_AddInstrumentFunction(IMAGECALLBACK, void *);
typedef void (*FINI_CALLBACK)(INT32, void *);
void PIN_AddFiniFunction(FINI_CALLBACK, void *);
bool PIN_Init(int, char **);
void PIN_InitSymbols();
typedef void (*ROOT_THREAD_FUNC)(void *);
typedef uint64_t PIN_THREAD_UID;
THREADID PIN_SpawnInternalThread(ROOT_THREAD_FUNC, void *, size_t, PIN_THREAD_UID *);
void PIN_StartProgramProbed();
void PIN_ExitThread(INT32);
void PIN_LockClient();
void PIN_UnlockClient();
void PIN_Sleep(UINT32 ms);

#define main pintool_main
