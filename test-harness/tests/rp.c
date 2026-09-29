#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned char u8; typedef unsigned short u16; typedef unsigned int u32; typedef unsigned long u64;

__attribute__((noinline)) u32 leaf_int(u32 a, u32 b, u8 *blk, u16 *q, int n, int *budget) {
  u8 c1, c2; u16 s1, s2; int k = 0; u32 i1 = a, i2 = b; signed char sc = -3; short ss = -700;
  while (k < 8) {
    c1 = blk[i1 % n]; c2 = blk[i2 % n];
    if (c1 != c2) return c1 > c2;
    s1 = q[i1 % n]; s2 = q[i2 % n];
    if (s1 != s2) return s1 > s2;
    i1++; i2++; k++; sc += c1; ss -= s2; (*budget)--;
  }
  return (u32)(sc + ss) & 1;
}
__attribute__((noinline)) long leaf_mix(long x, unsigned long y, int z) {
  long acc = 0; unsigned long u = y; int i; short s = (short)z; signed char c = (signed char)z; u8 uc = (u8)y;
  for (i = 0; i < 16; i++) {
    acc += x * i; acc ^= (long)u >> 3; u = (u << 1) | (u >> 63); s = (short)(s * 3 + i); c = (signed char)(c - i);
    uc = (u8)(uc + 7); if (acc < 0) acc = -acc; acc -= s; acc += c; acc += uc; acc = acc >> (i & 3);
    if ((i & 1) == 0) acc = acc * 2 + (u & 0xff);
  }
  return acc + u + s + c;
}
__attribute__((noinline)) double leaf_fp(int n, long m) {
  double d = 0.0; int i; long t = m;
  for (i = 0; i < n; i++) { d += (double)i / 3.0; d += (double)t; t = t * 3 + 1; if (t > 100000) t -= 99999; }
  return d;
}
/* local array + scalar: address taken of the array */
__attribute__((noinline)) int leaf_arr(int seed) {
  int arr[16]; int i, sum = 0, v = seed;
  for (i = 0; i < 16; i++) { v = v * 1103515245 + 12345; arr[i] = v >> 16; }
  for (i = 0; i < 16; i++) sum += arr[i] * (i + 1);
  int *p = &sum; *p += 3;
  return sum;
}
int main(int argc, char **argv) {
  long n = argc > 1 ? atol(argv[1]) : 3000000;
  int N = 4096; u8 *blk = malloc(N); u16 *q = malloc(N * 2);
  for (int i = 0; i < N; i++) { blk[i] = (u8)(i * 7 % 13); q[i] = (u16)(i * 31 % 7); }
  unsigned long h = 0; int budget = 1 << 30; double fs = 0;
  for (long i = 0; i < n; i++) {
    h = h * 131 + leaf_int((u32)i, (u32)(i * 7), blk, q, N, &budget);
    h ^= (unsigned long)leaf_mix(i, h, (int)(i & 0x7fff));
    if ((i & 63) == 0) fs += leaf_fp(20, i);
    if ((i & 15) == 0) h += leaf_arr((int)i);
  }
  printf("%lu %d %.3f\n", h, budget, fs);
  return 0;
}
