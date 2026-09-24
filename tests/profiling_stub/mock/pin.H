// MOCK pin.H for testing project.cpp's code generation WITHOUT the Pin kit.
// Only the INS_* queries used by add_profiling_instrs() are implemented
// (on top of real XED). Every other Pin API aborts if it is ever called.
#pragma once
#include <string>
#include <stdlib.h>
#include <stdint.h>
extern "C" {
#include "xed-interface.h"
}
typedef void VOID; typedef bool BOOL; typedef int INT; typedef int32_t INT32;
typedef unsigned int UINT; typedef uint8_t UINT8; typedef uint32_t UINT32; typedef uint64_t UINT64;
typedef uintptr_t ADDRINT; typedef size_t USIZE; typedef UINT32 THREADID; typedef UINT64 PIN_THREAD_UID;
typedef void (*AFUNPTR)(); typedef VOID ROOT_THREAD_FUNC(VOID *);
static const THREADID INVALID_THREADID = (THREADID)-1;

struct mock_ins_t { xed_decoded_inst_t xedd; ADDRINT addr; };
typedef mock_ins_t *INS;
struct IMG { int x; }; struct SEC { int x; }; struct RTN { int x; };
static inline bool operator==(RTN, RTN) { abort(); } static inline bool operator!=(RTN, RTN) { abort(); }

#define MOCK_ABORT { abort(); }
enum KNOB_MODE { KNOB_MODE_WRITEONCE };
template <class T> class KNOB {
public:
  T v;
public:
  KNOB(KNOB_MODE, const std::string &, const std::string &, const std::string &def, const std::string &)
    : v((T)strtoul(def.c_str(), 0, 0)) {}
  T Value() const { return v; }
  operator T() const { return v; }
};
struct KNOB_BASE { static std::string StringKnobSummary() MOCK_ABORT };

// --- implemented (used by the stub generator) ---
static inline xed_decoded_inst_t *INS_XedDec(INS ins) { return &ins->xedd; }
static inline ADDRINT INS_Address(INS ins) { return ins->addr; }
static inline xed_category_enum_t INS_Category(INS ins) { return xed_decoded_inst_get_category(&ins->xedd); }
static inline BOOL INS_IsRet(INS ins) { return INS_Category(ins) == XED_CATEGORY_RET; }
static inline BOOL INS_IsCall(INS ins) { return INS_Category(ins) == XED_CATEGORY_CALL; }
static inline BOOL INS_IsDirectControlFlow(INS ins) {
  return xed_decoded_inst_get_branch_displacement_width(&ins->xedd) > 0; }
static inline BOOL INS_IsIndirectControlFlow(INS ins) {
  xed_category_enum_t c = INS_Category(ins);
  if (c == XED_CATEGORY_RET) return true;
  return (c == XED_CATEGORY_CALL || c == XED_CATEGORY_UNCOND_BR) && !INS_IsDirectControlFlow(ins); }

// --- not used by the test: abort ---
static inline ADDRINT INS_DirectControlFlowTargetAddress(INS) MOCK_ABORT
static inline std::string INS_Disassemble(INS) MOCK_ABORT
static inline INS INS_Next(INS) MOCK_ABORT
static inline BOOL INS_Valid(INS) MOCK_ABORT
static inline USIZE INS_Size(INS) MOCK_ABORT
static inline SEC IMG_SecHead(IMG) MOCK_ABORT
static inline BOOL IMG_IsMainExecutable(IMG) MOCK_ABORT
static inline BOOL SEC_Valid(SEC) MOCK_ABORT
static inline SEC SEC_Next(SEC) MOCK_ABORT
static inline RTN SEC_RtnHead(SEC) MOCK_ABORT
static inline ADDRINT SEC_Address(SEC) MOCK_ABORT
static inline USIZE SEC_Size(SEC) MOCK_ABORT
static inline BOOL SEC_IsExecutable(SEC) MOCK_ABORT
static inline BOOL SEC_IsWriteable(SEC) MOCK_ABORT
static inline BOOL RTN_Valid(RTN) MOCK_ABORT
static inline RTN RTN_Next(RTN) MOCK_ABORT
static inline RTN RTN_Invalid() MOCK_ABORT
static inline VOID RTN_Open(RTN) MOCK_ABORT
static inline VOID RTN_Close(RTN) MOCK_ABORT
static inline INS RTN_InsHead(RTN) MOCK_ABORT
static inline ADDRINT RTN_Address(RTN) MOCK_ABORT
static inline USIZE RTN_Size(RTN) MOCK_ABORT
static inline UINT32 RTN_NumIns(RTN) MOCK_ABORT
static inline const std::string &RTN_Name(RTN) MOCK_ABORT
static inline RTN RTN_FindByAddress(ADDRINT) MOCK_ABORT
static inline RTN RTN_FindByName(IMG, const char *) MOCK_ABORT
static inline BOOL RTN_IsSafeForProbedReplacement(RTN) MOCK_ABORT
static inline AFUNPTR RTN_ReplaceProbed(RTN, AFUNPTR) MOCK_ABORT
static inline VOID PIN_LockClient() MOCK_ABORT
static inline VOID PIN_UnlockClient() MOCK_ABORT
static inline BOOL PIN_Init(INT32, char **) MOCK_ABORT
static inline VOID PIN_InitSymbols() MOCK_ABORT
static inline VOID PIN_StartProgramProbed() MOCK_ABORT
static inline VOID PIN_ExitThread(INT32) MOCK_ABORT
static inline THREADID PIN_SpawnInternalThread(ROOT_THREAD_FUNC *, VOID *, size_t, PIN_THREAD_UID *) MOCK_ABORT
typedef VOID (*IMAGECALLBACK)(IMG, VOID *);
static inline VOID IMG_AddInstrumentFunction(IMAGECALLBACK, VOID *) MOCK_ABORT
