// Tests code reordering in TC2 (src/project.cpp) WITHOUT Pin, see run.sh.
// Each routine has a block that is never executed during profiling (cold).
// After create_tc2() the test checks the TC2 layout instruction by
// instruction, and that every path (hot and cold) still gives the right result.
#define main pintool_main
#include "../../src/project.cpp"
#undef main
#include <sys/mman.h>

extern "C" { void tgt_same(); }     // untranslated target: returns rax unchanged
asm(".text\n.p2align 4\n.globl tgt_same\ntgt_same: ret\n");

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("   FAIL: %s\n", msg); failures++; } \
                           else printf("   ok: %s\n", msg); } while (0)

// f(x) = x >= 0 ? x + 1 : -x      (x < 0 is cold)
static const unsigned char F[] = {
  0x85, 0xFF,                   //  0: test edi, edi
  0x79, 0x05,                   //  2: jns 9          -> hot, jumps over the cold block
  0x89, 0xF8,                   //  4: mov eax, edi   cold
  0xF7, 0xD8,                   //  6: neg eax        cold
  0xC3,                         //  8: ret            cold
  0x8D, 0x47, 0x01,             //  9: lea eax, [rdi+1]
  0xC3 };                       // 12: ret
// g(x) = (x == 0 ? 100 : x) + 1   (x == 0 is cold; the cold block falls through)
static const unsigned char G[] = {
  0x89, 0xF8,                   //  0: mov eax, edi
  0x85, 0xFF,                   //  2: test edi, edi
  0x75, 0x05,                   //  4: jnz 11
  0xB8, 0x64, 0x00, 0x00, 0x00, //  6: mov eax, 100   cold, falls through to 11
  0x83, 0xC0, 0x01,             // 11: add eax, 1
  0xC3 };                       // 14: ret
// n(x) = x > 5 ? x + 100 : (x == 0 ? 50 : 99)   (the 99 block is cold)
static const unsigned char N[] = {
  0x83, 0xFF, 0x05,             //  0: cmp edi, 5
  0x7F, 0x0B,                   //  3: jg 16
  0x85, 0xFF,                   //  5: test edi, edi
  0x74, 0x0C,                   //  7: jz 21          target is hot but not placed next
  0xB8, 0x63, 0x00, 0x00, 0x00, //  9: mov eax, 99    cold
  0xEB, 0x08,                   // 14: jmp 24         cold
  0x8D, 0x47, 0x64,             // 16: lea eax, [rdi+100]
  0xEB, 0x03,                   // 19: jmp 24
  0x8D, 0x47, 0x32,             // 21: lea eax, [rdi+50]  falls through to 24
  0xC3 };                       // 24: ret
// lp(x): has a LOOP instr, so it must NOT be reordered (x == 0 is cold)
static const unsigned char LP[] = {
  0x89, 0xF9,                   //  0: mov ecx, edi
  0x31, 0xC0,                   //  2: xor eax, eax
  0x85, 0xFF,                   //  4: test edi, edi
  0x75, 0x03,                   //  6: jnz 11
  0x31, 0xC0,                   //  8: xor eax, eax  cold
  0xC3,                         // 10: ret           cold
  0xFF, 0xC0,                   // 11: inc eax
  0xE2, 0xFC,                   // 13: loop 11
  0xC3 };                       // 15: ret

// cv(x, fn) = x == 0 ? 0 : fn(x)   (x == 0 is cold; 'call rsi' is de-virtualized)
// Reordering moves the de-virtualized call sequence, whose internal branches
// (jne miss / jmp done) must be remapped to the new instr_map indices.
static const unsigned char CV[] = {
  0x48, 0x89, 0xF8,             //  0: mov rax, rdi
  0x48, 0x85, 0xFF,             //  3: test rdi, rdi
  0x75, 0x03,                   //  6: jnz 11
  0x31, 0xC0,                   //  8: xor eax, eax  cold
  0xC3,                         // 10: ret           cold
  0xFF, 0xD6,                   // 11: call rsi
  0xC3 };                       // 13: ret
// callee(x) = x + 1000 (translated); the ORIGINAL is patched to x + 2000 later
static const unsigned char CALLEE[] = {0x48, 0x8D, 0x87, 0xE8, 0x03, 0x00, 0x00, 0xC3};

typedef long (*fn1_t)(long);
typedef long (*fn2_t)(long, ADDRINT);

// Address of routine 'orig' in TC2: its TC head now holds 'jmp <TC2 start>'.
static ADDRINT tc2_start(ADDRINT orig)
{
  ADDRINT tc_head = g_mock_replaced[orig];
  xed_decoded_inst_t x; xed_decoded_inst_zero_set_mode(&x, &dstate);
  xed_decode(&x, (const UINT8 *)tc_head, 15);
  return tc_head + xed_decoded_inst_get_length(&x) + xed_decoded_inst_get_branch_displacement(&x);
}

// Compare the first instrs at 'addr' with the expected iclasses.
static bool layout_is(ADDRINT addr, const xed_iclass_enum_t *want, unsigned n)
{
  for (unsigned k = 0; k < n; k++) {
    xed_decoded_inst_t x; xed_decoded_inst_zero_set_mode(&x, &dstate);
    if (xed_decode(&x, (const UINT8 *)addr, 15) != XED_ERROR_NONE) return false;
    if (xed_decoded_inst_get_iclass(&x) != want[k]) {
      printf("   instr %u: got %s, expected %s\n", k,
             xed_iclass_enum_t2str(xed_decoded_inst_get_iclass(&x)), xed_iclass_enum_t2str(want[k]));
      return false;
    }
    addr += xed_decoded_inst_get_length(&x);
  }
  return true;
}

int main()
{
  setvbuf(stdout, 0, _IONBF, 0);
  xed_tables_init();

  char *orig = (char *)mmap((void *)0x10000000, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (orig == MAP_FAILED) { perror("mmap"); return 1; }
  memset(orig, 0xCC, 4096);
  memcpy(orig + 0, F, sizeof F);
  memcpy(orig + 32, G, sizeof G);
  memcpy(orig + 64, N, sizeof N);
  memcpy(orig + 96, LP, sizeof LP);
  memcpy(orig + 128, CV, sizeof CV);
  memcpy(orig + 160, CALLEE, sizeof CALLEE);
  memset(orig + 0x100, 0x90, 199); orig[0x100 + 199] = (char)0xC3;  // filler (instr_map sizing)
  mock_rtn_t rs[] = {
    {(ADDRINT)orig + 0, sizeof F, "f"}, {(ADDRINT)orig + 32, sizeof G, "g"},
    {(ADDRINT)orig + 64, sizeof N, "n"}, {(ADDRINT)orig + 96, sizeof LP, "lp"},
    {(ADDRINT)orig + 128, sizeof CV, "cv"}, {(ADDRINT)orig + 160, sizeof CALLEE, "callee"},
    {(ADDRINT)orig + 0x100, 200, "filler"} };
  for (unsigned i = 0; i < 7; i++) g_mock_rtns.push_back(rs[i]);
  const ADDRINT CALLEE_ORIG = rs[5].addr;

  IMG img = {0};
  create_tc(img, 0);
  if (g_mock_replaced.size() != 7) { printf("FAIL: create_tc() did not commit the routines\n"); return 1; }
  fn1_t f = (fn1_t)g_mock_replaced[rs[0].addr], g = (fn1_t)g_mock_replaced[rs[1].addr];
  fn1_t n = (fn1_t)g_mock_replaced[rs[2].addr], lp = (fn1_t)g_mock_replaced[rs[3].addr];
  fn2_t cv = (fn2_t)g_mock_replaced[rs[4].addr];

  printf("== profiling in TC: only the hot paths run\n");
  for (int i = 0; i < 5; i++) { f(3); g(4); n(10); n(0); lp(4); cv(5, CALLEE_ORIG); }
  CHECK(f(3) == 4 && g(4) == 5 && n(10) == 110 && n(0) == 50 && lp(4) == 4 &&
        cv(5, CALLEE_ORIG) == 1005, "hot paths correct in TC");

  printf("== create_tc2() with code reordering\n");
  disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
  CHECK(create_tc2() == 0, "create_tc2() succeeded");
  CHECK(num_devirt_calls == 1, "cv's call de-virtualized");
  CHECK(num_reordered_rtns == 4, "f, g, n, cv reordered; lp (has LOOP) left alone");
  CHECK(num_moved_cold_bbls == 4, "4 cold BBLs moved to their routine end");
  CHECK(num_reversed_cond_branches == 3, "3 cond branches reversed (f: jns->js, g: jnz->jz, cv: jnz->jz)");
  CHECK(num_added_fallthru_jumps == 2, "2 jumps added (g's cold block, n's jz fall-through)");

  printf("== TC2 layout\n");
  const xed_iclass_enum_t f_want[] = {XED_ICLASS_TEST, XED_ICLASS_JS, XED_ICLASS_LEA, XED_ICLASS_RET_NEAR,
                                      XED_ICLASS_MOV, XED_ICLASS_NEG, XED_ICLASS_RET_NEAR};
  CHECK(layout_is(tc2_start(rs[0].addr), f_want, 7), "f: test / js cold / lea / ret / [cold: mov neg ret]");
  const xed_iclass_enum_t g_want[] = {XED_ICLASS_MOV, XED_ICLASS_TEST, XED_ICLASS_JZ, XED_ICLASS_ADD,
                                      XED_ICLASS_RET_NEAR, XED_ICLASS_MOV, XED_ICLASS_JMP};
  CHECK(layout_is(tc2_start(rs[1].addr), g_want, 7), "g: mov / test / jz cold / add / ret / [cold: mov / jmp back]");
  const xed_iclass_enum_t n_want[] = {XED_ICLASS_CMP, XED_ICLASS_JNLE, XED_ICLASS_TEST, XED_ICLASS_JZ,
                                      XED_ICLASS_JMP, XED_ICLASS_LEA, XED_ICLASS_JMP, XED_ICLASS_LEA,
                                      XED_ICLASS_RET_NEAR, XED_ICLASS_MOV, XED_ICLASS_JMP};
  CHECK(layout_is(tc2_start(rs[2].addr), n_want, 11), "n: ... jz T / jmp cold / ... / [cold: mov / jmp]");
  const xed_iclass_enum_t lp_want[] = {XED_ICLASS_MOV, XED_ICLASS_XOR, XED_ICLASS_TEST, XED_ICLASS_JNZ,
                                       XED_ICLASS_XOR, XED_ICLASS_RET_NEAR};
  CHECK(layout_is(tc2_start(rs[3].addr), lp_want, 6), "lp: original order kept");

  const xed_iclass_enum_t cv_want[] = {XED_ICLASS_MOV, XED_ICLASS_TEST, XED_ICLASS_JZ, XED_ICLASS_CMP,
                                       XED_ICLASS_JNZ, XED_ICLASS_CALL_NEAR, XED_ICLASS_JMP,
                                       XED_ICLASS_CALL_NEAR, XED_ICLASS_RET_NEAR, XED_ICLASS_XOR,
                                       XED_ICLASS_RET_NEAR};
  CHECK(layout_is(tc2_start(rs[4].addr), cv_want, 11),
        "cv: mov / test / jz cold / [devirt call seq] / ret / [cold: xor ret]");

  // Patch the ORIGINAL callee: lea rax, [rdi+1000] -> [rdi+2000].
  orig[160 + 3] = (char)0xD0; orig[160 + 4] = 0x07;

  printf("== run through TC2: hot and cold paths\n");
  CHECK(f(3) == 4 && f(-3) == 3, "f: hot and cold path");
  CHECK(g(4) == 5 && g(0) == 101, "g: hot and cold path (cold block jumps back)");
  CHECK(n(10) == 110 && n(0) == 50 && n(3) == 99 && n(-1) == 99, "n: all paths");
  CHECK(lp(4) == 4 && lp(0) == 0, "lp: hot and cold path");
  CHECK(cv(5, CALLEE_ORIG) == 1005, "cv: hot target -> direct call to the TC2 copy of callee");
  CHECK(cv(7, (ADDRINT)tgt_same) == 7, "cv: other target -> original indirect call");
  CHECK(cv(0, CALLEE_ORIG) == 0, "cv: cold path");

  if (getenv("DUMP")) { printf("\nTC2:\n"); dump_tc(tc2, tc2_size); }
  printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
  return failures != 0;
}
