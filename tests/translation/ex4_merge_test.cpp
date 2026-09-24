// Tests the exercise-4 fixes merged into src/project.cpp, WITHOUT Pin (see run.sh):
//   - routine filters (name list, minimum size 16, overlapping symbols)
//   - a routine that fails to translate is reverted, the others still work
//   - conditional branch from translated code to original code
//   - jump_to_orig_addr_map: one slot per distinct target (off-by-one fix)
//   - dead-register optimization: RAX save/restore skipped where RAX is dead
#define main pintool_main
#include "../../src/project.cpp"
#undef main
#include <sys/mman.h>

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("   FAIL: %s\n", msg); failures++; } \
                           else printf("   ok: %s\n", msg); } while (0)

// All routines are padded to >= 16 bytes with int3 (0xCC), except 'tiny'.
static const unsigned char CODE_R_OK[] = {0x8D, 0x47, 0x01, 0xC3};           // lea eax,[rdi+1]; ret
static const unsigned char CODE_MALLOC[] = {0xB8, 0x2A, 0, 0, 0, 0xC3};      // mov eax,42; ret   (skipped by name)
static const unsigned char CODE_TINY[] = {0x8D, 0x47, 0x02, 0xC3};           // lea eax,[rdi+2]; ret (8 bytes: too small)
static const unsigned char CODE_R_BAD[] = {0x06, 0xC3};                      // 0x06 is invalid in 64-bit mode
// r_jcc_out(x) = x == 0 ? malloc() : 1       'jz' jumps to ORIGINAL code (malloc is not translated)
static const unsigned char CODE_R_JCC_OUT[] = {
  0x85, 0xFF,                           // 0x80: test edi, edi
  0x0F, 0x84, 0x98, 0xFF, 0xFF, 0xFF,   // 0x82: jz 0x20 (malloc)
  0xB8, 0x01, 0, 0, 0,                  // 0x88: mov eax, 1        <- RAX dead for the fall-through stub
  0xC3 };                               // 0x8d: ret
// r_dead(x) = x == 0 ? 8 : 2
static const unsigned char CODE_R_DEAD[] = {
  0xB8, 0x01, 0, 0, 0,                  // 0xa0: mov eax, 1
  0x85, 0xFF,                           // 0xa5: test edi, edi
  0x75, 0x05,                           // 0xa7: jnz 0xae
  0xB8, 0x07, 0, 0, 0,                  // 0xa9: mov eax, 7        <- RAX dead for 2 stubs
  0x83, 0xC0, 0x01,                     // 0xae: add eax, 1
  0xC3 };                               // 0xb1: ret
// r_call2(x): malloc(); tiny(x); return   -> 2 direct calls to non-translated routines
static const unsigned char CODE_R_CALL2[] = {
  0xE8, 0x5B, 0xFF, 0xFF, 0xFF,         // 0xc0: call 0x20 (malloc)
  0xE8, 0x76, 0xFF, 0xFF, 0xFF,         // 0xc5: call 0x40 (tiny)
  0xC3 };
// r_call3(): return malloc();             -> same target as above: must reuse its slot
static const unsigned char CODE_R_CALL3[] = {0xE8, 0x3B, 0xFF, 0xFF, 0xFF, 0xC3};  // 0xe0: call 0x20

typedef long (*fn1_t)(long);

// Execution count of the BBL whose LAST instr is at 'orig_addr'. (A BBL's
// first instr_map entry can be a profiling stub of the previous BBL, e.g. the
// fall-through counter after a jcc, so we look BBLs up by their terminator.)
static UINT64 bbl_count_ending_at(ADDRINT orig_addr)
{
  for (unsigned b = 0; b < bbl_num; b++)
    if (instr_map[bbl_map[b].terminating_ins_entry].orig_ins_addr == orig_addr)
      return bbl_map[b].counter;
  return (UINT64)-1;
}

int main()
{
  setvbuf(stdout, 0, _IONBF, 0);
  xed_tables_init();
  // NOTE: min_rtn_size_for_translation keeps its default of 16 here.

  char *orig = (char *)mmap((void *)0x10000000, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (orig == MAP_FAILED) { perror("mmap"); return 1; }
  memset(orig, 0xCC, 4096);
  memcpy(orig + 0x00, CODE_R_OK, sizeof CODE_R_OK);
  memcpy(orig + 0x20, CODE_MALLOC, sizeof CODE_MALLOC);
  memcpy(orig + 0x40, CODE_TINY, sizeof CODE_TINY);
  memcpy(orig + 0x60, CODE_R_BAD, sizeof CODE_R_BAD);
  memcpy(orig + 0x80, CODE_R_JCC_OUT, sizeof CODE_R_JCC_OUT);
  memcpy(orig + 0xA0, CODE_R_DEAD, sizeof CODE_R_DEAD);
  memcpy(orig + 0xC0, CODE_R_CALL2, sizeof CODE_R_CALL2);
  memcpy(orig + 0xE0, CODE_R_CALL3, sizeof CODE_R_CALL3);
  memset(orig + 0x100, 0x90, 199); orig[0x100 + 199] = (char)0xC3;   // filler
  ADDRINT o = (ADDRINT)orig;
  mock_rtn_t rs[] = {
    {o + 0x00, 16, "r_ok"},
    {o + 0x02, 16, "r_ok_alias"},     // starts inside r_ok (duplicate symbol) -> skipped
    {o + 0x20, 16, "malloc"},         // skipped by name
    {o + 0x40, 8,  "tiny"},           // smaller than 16 bytes -> skipped
    {o + 0x60, 16, "r_bad"},          // does not decode -> reverted
    {o + 0x80, 16, "r_jcc_out"},
    {o + 0xA0, 18, "r_dead"},
    {o + 0xC0, 16, "r_call2"},
    {o + 0xE0, 16, "r_call3"},
    {o + 0x100, 200, "filler"} };
  for (unsigned i = 0; i < sizeof rs / sizeof rs[0]; i++) g_mock_rtns.push_back(rs[i]);

  printf("== create_tc()\n");
  IMG img = {0};
  create_tc(img, 0);
  CHECK(tc_state == 1, "TC built (tc_state == 1)");
  CHECK(num_translated_rtns == 6 && num_reverted_rtns == 1, "6 routines translated, r_bad reverted");
  CHECK(g_mock_replaced.size() == 6 && !g_mock_replaced.count(o + 0x60) && !g_mock_replaced.count(o + 0x20) &&
        !g_mock_replaced.count(o + 0x40) && !g_mock_replaced.count(o + 0x02),
        "malloc, tiny, r_ok_alias, r_bad not committed (they run the original code)");
  CHECK(jump_to_orig_addr_num == 2, "2 distinct targets outside TC -> 2 jump_to_orig slots");
  CHECK(jump_to_orig_addr_map[0] == o + 0x20 && jump_to_orig_addr_map[1] == o + 0x40,
        "slots 0 and 1 hold malloc and tiny (off-by-one fixed: slot 0 is used)");
  CHECK(g_num_rax_save_skipped == 3, "dead-register opt skipped 3 RAX save/restore pairs");

  fn1_t r_ok = (fn1_t)g_mock_replaced[o + 0x00], r_jcc_out = (fn1_t)g_mock_replaced[o + 0x80];
  fn1_t r_dead = (fn1_t)g_mock_replaced[o + 0xA0], r_call2 = (fn1_t)g_mock_replaced[o + 0xC0];
  fn1_t r_call3 = (fn1_t)g_mock_replaced[o + 0xE0];

  printf("== run in TC (profiling on)\n");
  CHECK(r_ok(4) == 5, "r_ok");
  CHECK(r_jcc_out(0) == 42 && r_jcc_out(5) == 1, "r_jcc_out: jz into original code, and fall-through");
  for (int i = 0; i < 4; i++) r_dead(0);
  for (int i = 0; i < 6; i++) r_dead(3);
  CHECK(r_dead(0) == 8 && r_dead(3) == 2, "r_dead correct with RAX save/restore skipped");
  CHECK(bbl_count_ending_at(o + 0xA9) == 5 && bbl_count_ending_at(o + 0xB1) == 12,
        "r_dead BBL counters right (bbl_map indices valid after the revert of r_bad)");
  CHECK(r_call2(5) == 7 && r_call3(0) == 42, "direct calls to original code via jump_to_orig_addr_map");

  printf("== TC2\n");
  disable_profiling_in_tc(instr_map, num_of_instr_map_entries);
  CHECK(create_tc2() == 0, "create_tc2() succeeded");
  CHECK(r_ok(4) == 5 && r_jcc_out(0) == 42 && r_jcc_out(5) == 1 && r_dead(0) == 8 && r_dead(3) == 2 &&
        r_call2(5) == 7 && r_call3(0) == 42, "all routines correct through TC2");
  CHECK(jump_to_orig_addr_num == 2, "TC2 reuses the same 2 jump_to_orig slots");

  printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
  return failures != 0;
}
