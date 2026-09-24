// Tests de-virtualization in TC2 (src/project.cpp) WITHOUT Pin, see run.sh.
//
// A translated routine 'callee(x)' returns x + 1000. After TC2 is built,
// the test patches the ORIGINAL callee to return x + 2000; the translated
// copies (TC, TC2) still return x + 1000. So a result of 1001 means the
// de-virtualized DIRECT call/jump to the TC2 copy ran, and 2001 means the
// original INDIRECT call/jump (to the original address) ran.
#define main pintool_main
#include "../../src/project.cpp"
#undef main
#include <sys/mman.h>

extern "C" { UINT64 hits_a = 0; void tgt_a(); }   // a target that is not translated
asm(".text\n.p2align 4\n.globl tgt_a\ntgt_a: incq hits_a(%rip)\n ret\n");

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("   FAIL: %s\n", msg); failures++; } \
                           else printf("   ok: %s\n", msg); } while (0)

// call_via(x, fn):  rax = x; call fn (register);           ret
static const unsigned char CALL_VIA[] = {0x48, 0x89, 0xF8, 0xFF, 0xD6, 0xC3};
// call_mem(x, pfn): rax = x; call qword ptr [rsi] (memory); ret
static const unsigned char CALL_MEM[] = {0x48, 0x89, 0xF8, 0xFF, 0x16, 0xC3};
// jump_via(x, fn):  rax = x; jmp fn (register, tail jump)
static const unsigned char JUMP_VIA[] = {0x48, 0x89, 0xF8, 0xFF, 0xE6};
// call_rip(x):      rax = x; call qword ptr [rip+0x197] (-> orig+0x200); ret
static const unsigned char CALL_RIP[] = {0x48, 0x89, 0xF8, 0xFF, 0x15, 0x97, 0x01, 0x00, 0x00, 0xC3};
// callee(x):        rax = x + 1000; ret
static const unsigned char CALLEE[] = {0x48, 0x89, 0xF8, 0x48, 0x05, 0xE8, 0x03, 0x00, 0x00, 0xC3};

typedef long (*fn2_t)(long, ADDRINT);
typedef long (*fn1_t)(long);

static unsigned slot_of(ADDRINT t) { return (t ^ (t >> 4)) & MAX_TARG_ADDRS; }

int main()
{
  setvbuf(stdout, 0, _IONBF, 0);
  xed_tables_init();

  char *orig = (char *)mmap((void *)0x10000000, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (orig == MAP_FAILED) { perror("mmap"); return 1; }
  memset(orig, 0xCC, 4096);
  // Place callee so that its profiling slot differs from tgt_a's (otherwise
  // the two targets would share one slot and the counts would merge).
  unsigned callee_off = 0x100;
  while (slot_of((ADDRINT)orig + callee_off) == slot_of((ADDRINT)tgt_a)) callee_off += 16;
  memcpy(orig + 0, CALL_VIA, sizeof CALL_VIA);
  memcpy(orig + 32, CALL_MEM, sizeof CALL_MEM);
  memcpy(orig + 64, JUMP_VIA, sizeof JUMP_VIA);
  memcpy(orig + 96, CALL_RIP, sizeof CALL_RIP);
  memcpy(orig + callee_off, CALLEE, sizeof CALLEE);
  ADDRINT *rip_ptr = (ADDRINT *)(orig + 0x200);    // read by call_rip's [rip+0x197]
  // Filler routine of plain instrs: allocate_and_init_memory() sizes
  // instr_map as 10 entries per original instr, and these test routines are
  // almost only indirect jumps/calls (~25 profiling instrs each), which is
  // far denser than real code.
  memset(orig + 0x300, 0x90, 99); orig[0x300 + 99] = (char)0xC3;   // 99 x nop; ret
  mock_rtn_t rs[] = {
    {(ADDRINT)orig + 0, sizeof CALL_VIA, "call_via"},
    {(ADDRINT)orig + 32, sizeof CALL_MEM, "call_mem"},
    {(ADDRINT)orig + 64, sizeof JUMP_VIA, "jump_via"},
    {(ADDRINT)orig + 96, sizeof CALL_RIP, "call_rip"},
    {(ADDRINT)orig + callee_off, sizeof CALLEE, "callee"},
    {(ADDRINT)orig + 0x300, 100, "filler"} };
  for (unsigned i = 0; i < 6; i++) g_mock_rtns.push_back(rs[i]);
  const ADDRINT CALLEE_ORIG = rs[4].addr, TGT_A = (ADDRINT)tgt_a;
  static ADDRINT pfn;                               // pointer used by call_mem

  printf("== create_tc() and profiling in TC\n");
  KnobDumpProfile.v = true;                         // print devirt decisions (-dump_prof)
  IMG img = {0};
  create_tc(img, 0);
  if (g_mock_replaced.size() != 6) { printf("FAIL: create_tc() did not commit the routines\n"); return 1; }
  fn2_t call_via = (fn2_t)g_mock_replaced[rs[0].addr];
  fn2_t call_mem = (fn2_t)g_mock_replaced[rs[1].addr];
  fn2_t jump_via = (fn2_t)g_mock_replaced[rs[2].addr];
  fn1_t call_rip = (fn1_t)g_mock_replaced[rs[3].addr];

  // call_via: 10 of 10 to callee (100%)       -> de-virtualized
  for (int i = 0; i < 10; i++) call_via(1, CALLEE_ORIG);
  // jump_via: 1 to tgt_a, then 9 to callee (exactly 90%) -> de-virtualized
  jump_via(1, TGT_A);
  for (int i = 0; i < 9; i++) jump_via(1, CALLEE_ORIG);
  // call_mem: 2 to tgt_a, 8 to callee (80%, below 90%)   -> NOT de-virtualized
  pfn = TGT_A; call_mem(1, (ADDRINT)&pfn); call_mem(1, (ADDRINT)&pfn);
  pfn = CALLEE_ORIG; for (int i = 0; i < 8; i++) call_mem(1, (ADDRINT)&pfn);
  // call_rip: 5 of 5 to callee                -> de-virtualized ([rip+disp] form)
  *rip_ptr = CALLEE_ORIG; for (int i = 0; i < 5; i++) call_rip(1);
  CHECK(call_via(1, CALLEE_ORIG) == 1001, "TC: call_via correct");  // 11th call
  hits_a = 0;

  printf("== stop profiling, create_tc2() with de-virtualization\n");
  disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
  CHECK(create_tc2() == 0, "create_tc2() succeeded");
  CHECK(num_devirt_calls == 2 && num_devirt_jumps == 1,
        "2 calls (call_via, call_rip) and 1 jump (jump_via) de-virtualized");
  CHECK(num_devirt_skip_rare == 1, "call_mem (80%) not de-virtualized: no frequent target");

  // Patch the ORIGINAL callee: add rax, 1000 -> add rax, 2000.
  orig[callee_off + 5] = (char)0xD0; orig[callee_off + 6] = 0x07;

  printf("== run through TC2 (1001 = direct call/jump to TC2 copy, 2001 = original indirect)\n");
  CHECK(call_via(1, CALLEE_ORIG) == 1001, "call_via hot target -> direct call to TC2 copy");
  CHECK(call_via(5, TGT_A) == 5 && hits_a == 1, "call_via other target -> original indirect call");
  CHECK(jump_via(1, CALLEE_ORIG) == 1001, "jump_via hot target -> direct jump to TC2 copy");
  CHECK(jump_via(6, TGT_A) == 6 && hits_a == 2, "jump_via other target -> original indirect jump");
  pfn = CALLEE_ORIG;
  CHECK(call_mem(1, (ADDRINT)&pfn) == 2001, "call_mem (not de-virtualized) -> original indirect call");
  *rip_ptr = CALLEE_ORIG;
  CHECK(call_rip(1) == 1001, "call_rip hot target -> direct call ([rip+disp] cmp relocated)");
  *rip_ptr = TGT_A;
  CHECK(call_rip(7) == 7 && hits_a == 3, "call_rip other target -> original indirect call");

  if (getenv("DUMP")) { printf("\nTC2:\n"); dump_tc(tc2, tc2_size); }
  printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
  return failures != 0;
}
