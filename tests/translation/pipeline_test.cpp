// End-to-end test of the TC -> TC2 pipeline of src/project.cpp WITHOUT Pin
// (see run.sh). Three small routines are placed in executable memory and
// registered with the mock pin.H. The test then calls the tool's own code:
//   create_tc()               - image-load callback: builds TC with profiling
//   disable_profiling_in_tc() - what the thread does after -prof_time
//   create_tc2()              - builds TC2 and redirects TC routine heads to it
// and runs the routines through TC and then through TC2.
#define main pintool_main
#include "../../src/project.cpp"
#undef main
#include <sys/mman.h>

extern "C" { UINT64 hits_a = 0; void tgt_a(); }
asm(".text\n.p2align 4\n.globl tgt_a\ntgt_a: incq hits_a(%rip)\n ret\n");

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("   FAIL: %s\n", msg); failures++; } \
                           else printf("   ok: %s\n", msg); } while (0)

// loop_to(n): eax = 0; do eax++ while (eax != n); return eax;
static const unsigned char LOOP_TO[] = {
  0x31, 0xC0,               // 0: xor eax, eax
  0xFF, 0xC0,               // 2: inc eax           <- target of jne
  0x39, 0xF8,               // 4: cmp eax, edi
  0x75, 0xFA,               // 6: jne 2
  0xC3 };                   // 8: ret
// inc_unless_zero(n): return n == 0 ? 0 : n + 1;
// The je targets 'ret', a BBL of a single instruction. In TC, the profiling
// stub in front of that ret is the je's target; in TC2 that stub is removed.
static const unsigned char INC_UNLESS_ZERO[] = {
  0x89, 0xF8,               // 0: mov eax, edi
  0x85, 0xFF,               // 2: test edi, edi
  0x74, 0x03,               // 4: je 9
  0x83, 0xC0, 0x01,         // 6: add eax, 1
  0xC3 };                   // 9: ret             <- target of je
// call_via(x, fn): rax = x; fn(); return rax;  (indirect call ends a BBL)
static const unsigned char CALL_VIA[] = {
  0x48, 0x89, 0xF8,         // 0: mov rax, rdi
  0xFF, 0xD6,               // 3: call rsi
  0xC3 };                   // 5: ret

typedef long (*fn1_t)(long);
typedef long (*fn2_t)(long, void (*)());

static ADDRINT tc_entry(ADDRINT orig) { return g_mock_replaced[orig]; }

// Number of instructions in [code, code+size).
static unsigned count_instrs(char *code, unsigned size)
{
  unsigned n = 0;
  for (ADDRINT a = (ADDRINT)code; a < (ADDRINT)code + size; n++) {
    xed_decoded_inst_t x; xed_decoded_inst_zero_set_mode(&x, &dstate);
    if (xed_decode(&x, (const UINT8 *)a, 15) != XED_ERROR_NONE) return 0;
    a += xed_decoded_inst_get_length(&x);
  }
  return n;
}

static UINT64 sum_bbl_counters()
{
  UINT64 s = 0;
  for (unsigned b = 0; b < bbl_num; b++) s += bbl_map[b].counter + bbl_map[b].fallthru_counter;
  return s;
}

int main()
{
  setvbuf(stdout, 0, _IONBF, 0);
  xed_tables_init();
  min_rtn_size_for_translation = 1;   // the test routines are smaller than 16 bytes

  // "Original code": 3 routines, 32 bytes apart, in one executable page.
  char *orig = (char *)mmap((void *)0x10000000, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (orig == MAP_FAILED) { perror("mmap"); return 1; }
  memset(orig, 0xCC, 4096);
  memcpy(orig + 0, LOOP_TO, sizeof LOOP_TO);
  memcpy(orig + 32, INC_UNLESS_ZERO, sizeof INC_UNLESS_ZERO);
  memcpy(orig + 64, CALL_VIA, sizeof CALL_VIA);
  mock_rtn_t r1 = {(ADDRINT)orig, sizeof LOOP_TO, "loop_to"};
  mock_rtn_t r2 = {(ADDRINT)orig + 32, sizeof INC_UNLESS_ZERO, "inc_unless_zero"};
  mock_rtn_t r3 = {(ADDRINT)orig + 64, sizeof CALL_VIA, "call_via"};
  g_mock_rtns.push_back(r1); g_mock_rtns.push_back(r2); g_mock_rtns.push_back(r3);
  const unsigned num_orig_instrs = 5 + 5 + 3;

  printf("== create_tc(): translate into TC with profiling\n");
  IMG img = {0};
  create_tc(img, 0);
  CHECK(tc_size > 0 && g_mock_replaced.size() == 3, "TC built and 3 routines committed");
  fn1_t loop_to = (fn1_t)tc_entry(r1.addr);
  fn1_t inc_unless_zero = (fn1_t)tc_entry(r2.addr);
  fn2_t call_via = (fn2_t)tc_entry(r3.addr);

  printf("== run in TC (profiling on)\n");
  CHECK(loop_to(5) == 5, "loop_to(5) == 5 in TC");
  CHECK(inc_unless_zero(0) == 0 && inc_unless_zero(7) == 8, "inc_unless_zero in TC");
  CHECK(call_via(42, tgt_a) == 42 && hits_a == 1, "call_via in TC reaches its target");
  UINT64 prof = sum_bbl_counters();
  CHECK(prof > 0, "TC profiling counted BBL executions");
  bool loop_counted = false, call_target = false;
  for (unsigned b = 0; b < bbl_num; b++) {
    if (bbl_map[b].counter == 5) loop_counted = true;
    if (bbl_map[b].indirect_kind == IndirectCall && bbl_map[b].indirect_site_addr == r3.addr + 3)
      for (unsigned j = 0; j <= MAX_TARG_ADDRS; j++)
        if (bbl_map[b].targ_addr[j] == (ADDRINT)tgt_a && bbl_map[b].targ_count[j] == 1) call_target = true;
  }
  CHECK(loop_counted, "loop body BBL counted 5 times");
  CHECK(call_target, "indirect call target recorded");

  printf("== disable_profiling_in_tc(): what the thread does after -prof_time\n");
  CHECK(disable_profiling_in_tc(instr_map, num_of_instr_map_entries) == 0, "profiling stubs bypassed");
  CHECK(loop_to(5) == 5 && inc_unless_zero(3) == 4, "TC still correct with stubs bypassed");
  CHECK(sum_bbl_counters() == prof, "no counting after profiling was disabled");

  printf("== create_tc2(): build TC2 and redirect the TC routine heads\n");
  CHECK(create_tc2() == 0, "create_tc2() succeeded");
  unsigned tc2_instrs = count_instrs(tc2, tc2_size);
  printf("   TC: %u bytes / %u instrs, TC2: %u bytes / %u instrs, original: %u instrs\n",
         tc_size, count_instrs(tc, tc_size), tc2_size, tc2_instrs, num_orig_instrs);
  CHECK(tc2_instrs == num_orig_instrs, "TC2 holds exactly the original instrs (no profiling code)");
  xed_decoded_inst_t x; xed_decoded_inst_zero_set_mode(&x, &dstate);
  xed_decode(&x, (const UINT8 *)loop_to, 15);
  ADDRINT jmp_targ = (ADDRINT)loop_to + xed_decoded_inst_get_length(&x) +
                     xed_decoded_inst_get_branch_displacement(&x);
  CHECK(xed_decoded_inst_get_iclass(&x) == XED_ICLASS_JMP &&
        jmp_targ >= (ADDRINT)tc2 && jmp_targ < (ADDRINT)tc2 + tc2_size,
        "TC head of loop_to now jumps into TC2");

  printf("== run through TC2\n");
  CHECK(loop_to(5) == 5 && loop_to(1000) == 1000, "loop_to correct in TC2");
  CHECK(inc_unless_zero(0) == 0 && inc_unless_zero(7) == 8,
        "inc_unless_zero correct in TC2 (je to a removed stub lands on the ret)");
  CHECK(call_via(9, tgt_a) == 9 && hits_a == 2, "call_via correct in TC2");
  CHECK(sum_bbl_counters() == prof, "TC2 does not touch the profiling counters");

  if (getenv("DUMP")) {
    printf("\nTC:\n"); dump_tc(tc, tc_size);
    printf("\nTC2:\n"); dump_tc(tc2, tc2_size);
  }
  printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
  return failures != 0;
}
