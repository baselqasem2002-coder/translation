#include <stdio.h>
#include <stdlib.h>
volatile unsigned long pad_sink;
#define PAD(r) do { pad_sink = r; pad_sink += 3; pad_sink ^= r >> 3; pad_sink += r * 5; pad_sink -= 11; pad_sink ^= 0x1234; pad_sink += r >> 9; } while (0)
typedef unsigned long (*fn_t)(unsigned long, unsigned long);
__attribute__((noinline)) unsigned long f0(unsigned long a, unsigned long b) { unsigned long r = a; for (int i = 0; i < 20; i++) r = r * 31 + (b ^ i); PAD(r); return r; }
__attribute__((noinline)) unsigned long f1(unsigned long a, unsigned long b) { unsigned long r = a ^ b; for (int i = 0; i < 20; i++) r = (r << 3) ^ (r >> 5) ^ i; PAD(r); return r + 1; }
__attribute__((noinline)) unsigned long f2(unsigned long a, unsigned long b) { unsigned long r = a + b; for (int i = 0; i < 20; i++) r = r * 7 - i; PAD(r); return r; }
__attribute__((noinline)) unsigned long f3(unsigned long a, unsigned long b) { unsigned long r = b - a; for (int i = 0; i < 20; i++) r ^= r * 13 + i; PAD(r); return r; }
fn_t table[4] = { f0, f1, f2, f3 };
volatile int sel_rare = 7;

__attribute__((noinline)) unsigned long sw(unsigned long x, int k) {
  switch (k) {
    case 0: x = x * 3 + 1; break;
    case 1: x = x ^ 0x5555; break;
    case 2: x = x - 17; break;
    case 3: x = (x << 1) | 1; break;
    case 4: x = x / 3 + 99; break;
    case 5: x = ~x; break;
    case 6: x = x + (x >> 7); break;
    default: x = x * 5; break;
  }
  return x;
}
/* switch in a leaf with locals in the red zone (-O0) */
__attribute__((noinline)) long leafsw(long a, int k) {
  long t1 = a + 1, t2 = a * 3, t3 = a ^ 77;
  switch (k) {
    case 0: t1 += t2; break;
    case 1: t2 -= t3; break;
    case 2: t3 ^= t1; break;
    case 3: t1 = t2 + t3; break;
    case 4: t2 = t1 * 2; break;
    case 5: t3 = t1 - 9; break;
    default: t1 = 0; break;
  }
  return t1 + t2 + t3;
}

int main(int argc, char **argv) {
  long n = argc > 1 ? atol(argv[1]) : 30000000;
  unsigned long acc = 1, acc2 = 0;
  long acc3 = 0;
  unsigned seed = 12345;
  for (long i = 0; i < n; i++) {
    seed = seed * 1103515245 + 12345;
    int k = ((seed >> 16) % 100) < 92 ? 2 : ((seed >> 16) & 3);
    acc = table[k](acc, i);
    int s = ((seed >> 20) % 100) < 90 ? 4 : ((seed >> 12) % 9);
    acc2 = sw(acc2 + i, s);
    acc3 += leafsw(i, ((seed >> 8) % 100) < 88 ? 1 : (int)((seed >> 3) % 7));
    if (sel_rare == 8) acc ^= 1;
  }
  printf("%lu %lu %ld\n", acc, acc2, acc3);
  return 0;
}
