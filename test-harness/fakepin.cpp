// Fake Pin probe-mode runtime (test harness only).
// Loaded with LD_PRELOAD into a non-PIE x86-64 executable. A constructor
// parses /proc/self/exe, calls the tool's main (renamed pintool_main), runs
// the image callback on the main executable, and then lets the program run.
#include "pin.H"
#include <elf.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <map>
#include <set>

struct fp_ins {
  ADDRINT addr;
  xed_decoded_inst_t xedd;
  fp_ins *next, *prev;
};
struct fp_rtn {
  std::string name;
  ADDRINT addr;
  USIZE size;
  fp_sec *sec;
  fp_rtn *next;
  std::vector<fp_ins *> ins;
  bool decoded;
};
struct fp_sec {
  std::string name;
  ADDRINT addr;
  USIZE size;
  bool exec, write;
  fp_sec *next;
  fp_rtn *rtn_head;
};
struct fp_img {
  fp_sec *sec_head;
  std::vector<fp_rtn *> rtns;
};

static fp_img g_img;
static xed_state_t g_dstate;
static std::vector<std::pair<IMAGECALLBACK, void *> > g_img_cbs;
static std::vector<std::pair<FINI_CALLBACK, void *> > g_fini_cbs;

static void decode_rtn(fp_rtn *r) {
  if (r->decoded) return;
  r->decoded = true;
  ADDRINT a = r->addr, end = r->addr + r->size;
  fp_ins *prev = NULL;
  while (a < end) {
    fp_ins *i = new fp_ins();
    i->addr = a;
    xed_decoded_inst_zero_set_mode(&i->xedd, &g_dstate);
    unsigned len = std::min<ADDRINT>(15, end - a);
    if (xed_decode(&i->xedd, (const uint8_t *)a, len) != XED_ERROR_NONE) { delete i; break; }
    i->prev = prev; i->next = NULL;
    if (prev) prev->next = i;
    r->ins.push_back(i);
    prev = i;
    a += xed_decoded_inst_get_length(&i->xedd);
  }
}

static void load_image() {
  xed_tables_init();
  g_dstate.mmode = XED_MACHINE_MODE_LONG_64;
  g_dstate.stack_addr_width = XED_ADDRESS_WIDTH_64b;
  int fd = open("/proc/self/exe", O_RDONLY);
  struct stat st; fstat(fd, &st);
  char *f = (char *)mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  close(fd);
  Elf64_Ehdr *eh = (Elf64_Ehdr *)f;
  Elf64_Shdr *sh = (Elf64_Shdr *)(f + eh->e_shoff);
  const char *shstr = f + sh[eh->e_shstrndx].sh_offset;
  std::vector<fp_sec *> secs(eh->e_shnum, (fp_sec *)NULL);
  fp_sec *last = NULL;
  for (int i = 0; i < eh->e_shnum; i++) {
    if (!(sh[i].sh_flags & SHF_ALLOC)) continue;
    fp_sec *s = new fp_sec();
    s->name = shstr + sh[i].sh_name;
    s->addr = sh[i].sh_addr; s->size = sh[i].sh_size;
    s->exec = sh[i].sh_flags & SHF_EXECINSTR; s->write = sh[i].sh_flags & SHF_WRITE;
    s->next = NULL; s->rtn_head = NULL;
    if (last) last->next = s; else g_img.sec_head = s;
    last = s; secs[i] = s;
  }
  // symbols
  std::map<ADDRINT, std::pair<std::string, USIZE> > funcs;
  std::map<ADDRINT, int> fsec;
  for (int i = 0; i < eh->e_shnum; i++) {
    if (sh[i].sh_type != SHT_SYMTAB) continue;
    Elf64_Sym *sy = (Elf64_Sym *)(f + sh[i].sh_offset);
    const char *str = f + sh[sh[i].sh_link].sh_offset;
    int n = sh[i].sh_size / sizeof(Elf64_Sym);
    for (int k = 0; k < n; k++) {
      if (ELF64_ST_TYPE(sy[k].st_info) != STT_FUNC || !sy[k].st_value || !sy[k].st_shndx ||
          sy[k].st_shndx >= eh->e_shnum)
        continue;
      if (funcs.count(sy[k].st_value)) {
        if (ELF64_ST_BIND(sy[k].st_info) != STB_GLOBAL) continue;
      }
      funcs[sy[k].st_value] = std::make_pair(std::string(str + sy[k].st_name), (USIZE)sy[k].st_size);
      fsec[sy[k].st_value] = sy[k].st_shndx;
    }
  }
  std::map<fp_sec *, fp_rtn *> tail;
  for (std::map<ADDRINT, std::pair<std::string, USIZE> >::iterator it = funcs.begin(); it != funcs.end(); ++it) {
    fp_sec *s = secs[fsec[it->first]];
    if (!s) continue;
    fp_rtn *r = new fp_rtn();
    r->name = it->second.first; r->addr = it->first; r->size = it->second.second;
    std::map<ADDRINT, std::pair<std::string, USIZE> >::iterator nx = it; ++nx;
    ADDRINT lim = s->addr + s->size;
    if (nx != funcs.end() && nx->first < lim) lim = nx->first;
    if (!r->size || r->addr + r->size > lim) r->size = lim - r->addr;
    r->sec = s; r->next = NULL; r->decoded = false;
    if (tail[s]) tail[s]->next = r; else s->rtn_head = r;
    tail[s] = r;
    g_img.rtns.push_back(r);
  }
}

IMG IMG_Invalid() { return NULL; }
bool IMG_Valid(IMG i) { return i != NULL; }
bool IMG_IsMainExecutable(IMG) { return true; }
SEC IMG_SecHead(IMG i) { return i->sec_head; }
std::string IMG_Name(IMG) { return "main"; }
bool SEC_Valid(SEC s) { return s != NULL; }
SEC SEC_Next(SEC s) { return s->next; }
bool SEC_IsExecutable(SEC s) { return s->exec; }
bool SEC_IsWriteable(SEC s) { return s->write; }
ADDRINT SEC_Address(SEC s) { return s->addr; }
USIZE SEC_Size(SEC s) { return s->size; }
RTN SEC_RtnHead(SEC s) { return s->rtn_head; }
std::string SEC_Name(SEC s) { return s->name; }

RTN RTN_Invalid() { return NULL; }
bool RTN_Valid(RTN r) { return r != NULL; }
RTN RTN_Next(RTN r) { return r->next; }
const std::string &RTN_Name(RTN r) { return r->name; }
ADDRINT RTN_Address(RTN r) { return r->addr; }
USIZE RTN_Size(RTN r) { return r->size; }
UINT32 RTN_NumIns(RTN r) { decode_rtn(r); return r->ins.size(); }
void RTN_Open(RTN r) { decode_rtn(r); }
void RTN_Close(RTN) {}
INS RTN_InsHead(RTN r) { decode_rtn(r); return r->ins.empty() ? NULL : r->ins[0]; }
SEC RTN_Sec(RTN r) { return r->sec; }
bool RTN_IsSafeForProbedReplacement(RTN r) {
  decode_rtn(r);
  if (r->size < 5 || r->ins.empty()) return false;
  for (size_t k = 0; k < r->ins.size(); k++) {
    INS i = r->ins[k];
    if (INS_IsDirectControlFlow(i)) {
      ADDRINT t = INS_DirectControlFlowTargetAddress(i);
      if (t > r->addr && t < r->addr + 5) return false;
    }
  }
  return true;
}
RTN RTN_FindByAddress(ADDRINT a) {
  for (size_t k = 0; k < g_img.rtns.size(); k++)
    if (a >= g_img.rtns[k]->addr && a < g_img.rtns[k]->addr + g_img.rtns[k]->size) return g_img.rtns[k];
  return NULL;
}
RTN RTN_FindByName(IMG, const char *n) {
  for (size_t k = 0; k < g_img.rtns.size(); k++)
    if (g_img.rtns[k]->name == n) return g_img.rtns[k];
  return NULL;
}
AFUNPTR RTN_ReplaceProbed(RTN r, AFUNPTR f) {
  ADDRINT a = r->addr, t = (ADDRINT)f;
  long pg = sysconf(_SC_PAGESIZE);
  mprotect((void *)(a & ~(pg - 1)), 2 * pg, PROT_READ | PROT_WRITE | PROT_EXEC);
  INT64 d = (INT64)t - (INT64)(a + 5);
  if (d >= INT32_MIN && d <= INT32_MAX) {
    uint8_t b[5] = {0xE9};
    int32_t d32 = (int32_t)d;
    memcpy(b + 1, &d32, 4);
    memcpy((void *)a, b, 5);
  } else {
    if (r->size < 14) return NULL;
    uint8_t b[14] = {0xFF, 0x25, 0, 0, 0, 0};
    memcpy(b + 6, &t, 8);
    memcpy((void *)a, b, 14);
  }
  return (AFUNPTR)a;
}

bool INS_Valid(INS i) { return i != NULL; }
INS INS_Next(INS i) { return i->next; }
INS INS_Prev(INS i) { return i->prev; }
ADDRINT INS_Address(INS i) { return i->addr; }
USIZE INS_Size(INS i) { return xed_decoded_inst_get_length(&i->xedd); }
xed_decoded_inst_t *INS_XedDec(INS i) { return &i->xedd; }
xed_category_enum_t INS_Category(INS i) { return xed_decoded_inst_get_category(&i->xedd); }
OPCODE INS_Opcode(INS i) { return xed_decoded_inst_get_iclass(&i->xedd); }
bool INS_IsCall(INS i) { return INS_Category(i) == XED_CATEGORY_CALL; }
bool INS_IsRet(INS i) { return INS_Category(i) == XED_CATEGORY_RET; }
bool INS_IsControlFlow(INS i) {
  xed_category_enum_t c = INS_Category(i);
  return c == XED_CATEGORY_CALL || c == XED_CATEGORY_RET || c == XED_CATEGORY_UNCOND_BR ||
         c == XED_CATEGORY_COND_BR;
}
bool INS_IsBranch(INS i) {
  xed_category_enum_t c = INS_Category(i);
  return c == XED_CATEGORY_UNCOND_BR || c == XED_CATEGORY_COND_BR;
}
bool INS_IsDirectControlFlow(INS i) {
  return INS_IsControlFlow(i) && xed_decoded_inst_get_branch_displacement_width(&i->xedd) > 0;
}
bool INS_IsIndirectControlFlow(INS i) {
  return INS_IsControlFlow(i) && !INS_IsDirectControlFlow(i);
}
ADDRINT INS_DirectControlFlowTargetAddress(INS i) {
  return i->addr + xed_decoded_inst_get_length(&i->xedd) + xed_decoded_inst_get_branch_displacement(&i->xedd);
}
bool INS_IsPredicated(INS i) {
  return xed_decoded_inst_get_attribute(&i->xedd, XED_ATTRIBUTE_REP) ||
         INS_Category(i) == XED_CATEGORY_CMOV;
}
bool INS_IsSyscall(INS i) {
  xed_iclass_enum_t c = (xed_iclass_enum_t)INS_Opcode(i);
  return c == XED_ICLASS_SYSCALL || c == XED_ICLASS_SYSENTER;
}
bool INS_IsInterrupt(INS i) { return INS_Category(i) == XED_CATEGORY_INTERRUPT; }
std::string INS_Disassemble(INS i) {
  char buf[256];
  xed_format_context(XED_SYNTAX_INTEL, &i->xedd, buf, sizeof(buf), i->addr, 0, 0);
  return buf;
}

static void collect_regs(INS i, bool want_read, std::vector<REG> &v) {
  const xed_inst_t *xi = xed_decoded_inst_inst(&i->xedd);
  for (unsigned k = 0; k < xed_inst_noperands(xi); k++) {
    const xed_operand_t *op = xed_inst_operand(xi, k);
    xed_operand_enum_t n = xed_operand_name(op);
    if (xed_operand_is_register(n) || n == XED_OPERAND_BASE0 || n == XED_OPERAND_BASE1) {
      REG r = xed_decoded_inst_get_reg(&i->xedd, n);
      if (r == XED_REG_INVALID) continue;
      if (want_read ? xed_operand_read(op) : xed_operand_written(op)) v.push_back(r);
    }
  }
  if (want_read) {
    for (unsigned m = 0; m < xed_decoded_inst_number_of_memory_operands(&i->xedd); m++) {
      REG b = xed_decoded_inst_get_base_reg(&i->xedd, m), x = xed_decoded_inst_get_index_reg(&i->xedd, m);
      if (b != XED_REG_INVALID) v.push_back(b);
      if (x != XED_REG_INVALID) v.push_back(x);
    }
  }
}
UINT32 INS_MaxNumRRegs(INS i) { std::vector<REG> v; collect_regs(i, true, v); return v.size(); }
UINT32 INS_MaxNumWRegs(INS i) { std::vector<REG> v; collect_regs(i, false, v); return v.size(); }
REG INS_RegR(INS i, UINT32 k) { std::vector<REG> v; collect_regs(i, true, v); return k < v.size() ? v[k] : XED_REG_INVALID; }
REG INS_RegW(INS i, UINT32 k) { std::vector<REG> v; collect_regs(i, false, v); return k < v.size() ? v[k] : XED_REG_INVALID; }

static const xed_operand_t *opnd(INS i, UINT32 k) {
  const xed_inst_t *xi = xed_decoded_inst_inst(&i->xedd);
  return k < xed_inst_noperands(xi) ? xed_inst_operand(xi, k) : NULL;
}
UINT32 INS_OperandCount(INS i) { return xed_inst_noperands(xed_decoded_inst_inst(&i->xedd)); }
bool INS_OperandIsMemory(INS i, UINT32 k) {
  const xed_operand_t *o = opnd(i, k);
  return o && (xed_operand_name(o) == XED_OPERAND_MEM0 || xed_operand_name(o) == XED_OPERAND_MEM1);
}
bool INS_OperandIsReg(INS i, UINT32 k) { const xed_operand_t *o = opnd(i, k); return o && xed_operand_is_register(xed_operand_name(o)); }
bool INS_OperandIsImmediate(INS i, UINT32 k) {
  const xed_operand_t *o = opnd(i, k);
  return o && (xed_operand_name(o) == XED_OPERAND_IMM0 || xed_operand_name(o) == XED_OPERAND_IMM1);
}
REG INS_OperandReg(INS i, UINT32 k) { const xed_operand_t *o = opnd(i, k); return o ? xed_decoded_inst_get_reg(&i->xedd, xed_operand_name(o)) : XED_REG_INVALID; }
static unsigned memidx(INS i, UINT32 k) { const xed_operand_t *o = opnd(i, k); return (o && xed_operand_name(o) == XED_OPERAND_MEM1) ? 1 : 0; }
REG INS_OperandMemoryBaseReg(INS i, UINT32 k) { return xed_decoded_inst_get_base_reg(&i->xedd, memidx(i, k)); }
REG INS_OperandMemoryIndexReg(INS i, UINT32 k) { return xed_decoded_inst_get_index_reg(&i->xedd, memidx(i, k)); }
INT64 INS_OperandMemoryDisplacement(INS i, UINT32 k) { return xed_decoded_inst_get_memory_displacement(&i->xedd, memidx(i, k)); }
bool INS_OperandRead(INS i, UINT32 k) { const xed_operand_t *o = opnd(i, k); return o && xed_operand_read(o); }
bool INS_OperandWritten(INS i, UINT32 k) { const xed_operand_t *o = opnd(i, k); return o && xed_operand_written(o); }
UINT32 INS_OperandWidth(INS i, UINT32 k) { return xed_decoded_inst_operand_length_bits(&i->xedd, k); }
bool INS_IsMov(INS i) { return INS_Opcode(i) == XED_ICLASS_MOV; }
bool INS_IsLea(INS i) { return INS_Opcode(i) == XED_ICLASS_LEA; }
bool INS_IsMemoryRead(INS i) { return xed_decoded_inst_number_of_memory_operands(&i->xedd) && xed_decoded_inst_mem_read(&i->xedd, 0); }
bool INS_IsMemoryWrite(INS i) { return xed_decoded_inst_number_of_memory_operands(&i->xedd) && xed_decoded_inst_mem_written(&i->xedd, 0); }
bool INS_IsStackRead(INS) { return false; }
bool INS_IsStackWrite(INS) { return false; }

void IMG_AddInstrumentFunction(IMAGECALLBACK f, void *v) { g_img_cbs.push_back(std::make_pair(f, v)); }
void PIN_AddFiniFunction(FINI_CALLBACK f, void *v) { g_fini_cbs.push_back(std::make_pair(f, v)); }
bool PIN_Init(int argc, char **argv) {
  for (int k = 1; k < argc; k++) {
    if (argv[k][0] != '-') continue;
    std::string n = argv[k] + 1;
    for (size_t j = 0; j < KNOB_BASE::all().size(); j++) {
      if (KNOB_BASE::all()[j]->name != n) continue;
      if (k + 1 < argc && argv[k + 1][0] != '-') KNOB_BASE::all()[j]->set(argv[++k]);
      else KNOB_BASE::all()[j]->set("1");
    }
  }
  return false;
}
void PIN_InitSymbols() {}
struct thr_arg { ROOT_THREAD_FUNC f; void *v; };
static void *thr_tramp(void *p) { thr_arg *a = (thr_arg *)p; a->f(a->v); return NULL; }
THREADID PIN_SpawnInternalThread(ROOT_THREAD_FUNC f, void *v, size_t, PIN_THREAD_UID *) {
  pthread_t t;
  thr_arg *a = new thr_arg; a->f = f; a->v = v;
  if (pthread_create(&t, NULL, thr_tramp, a)) return INVALID_THREADID;
  pthread_detach(t);
  return 1;
}
void PIN_StartProgramProbed() {}
void PIN_ExitThread(INT32) { pthread_exit(NULL); }
static pthread_mutex_t g_client_lock = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
void PIN_LockClient() { pthread_mutex_lock(&g_client_lock); }
void PIN_UnlockClient() { pthread_mutex_unlock(&g_client_lock); }
void PIN_Sleep(UINT32 ms) { usleep(ms * 1000); }

#undef main
extern int pintool_main(int argc, char *argv[]);

static void run_fini() {
  for (size_t k = 0; k < g_fini_cbs.size(); k++) g_fini_cbs[k].first(0, g_fini_cbs[k].second);
}
// Mimic Pin's _exit probe: the tools hook _exit in libc; here we use atexit.
extern void fakepin_exit_hook();

__attribute__((constructor)) static void fakepin_init() {
  load_image();
  std::vector<std::string> args;
  args.push_back("pin");
  const char *e = getenv("FAKEPIN_ARGS");
  if (e) { std::istringstream is(e); std::string w; while (is >> w) args.push_back(w); }
  std::vector<char *> av;
  for (size_t k = 0; k < args.size(); k++) av.push_back((char *)args[k].c_str());
  av.push_back(NULL);
  pintool_main(av.size() - 1, &av[0]);
  for (size_t k = 0; k < g_img_cbs.size(); k++) g_img_cbs[k].first(&g_img, g_img_cbs[k].second);
  atexit(fakepin_exit_hook);
  atexit(run_fini);
}
